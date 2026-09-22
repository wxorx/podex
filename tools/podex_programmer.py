#!/usr/bin/env python3
"""
podex_programmer.py -- flash/EEPROM programmer for the MC912DG128A via podex BDM pod.

Target: MC912DG128ACPV (CPU12/DG128A: 128K flash = 4 x 32K modules, 2K EEPROM
at $0800, 8K RAM at $2000, registers at $0000).  Pod: podex on ATTiny2313,
Kevin-Ross-BDM12-compatible protocol at 460800 8N1 on /dev/ttyUSB0.

SEE ALSO: DG128A_flash_notes.md (same directory) -- every silicon quirk,
proven timing and debugging gotcha found while building this.  Read it
before writing another flasher for this chip (e.g. over CAN/Volcano).

Flash/EEPROM erase+program is done by HC12 machine-code routines uploaded to
RAM at $2000 and started with BDM GO (see tools/asm/*.s, rebuilt by
tools/asm/build.sh).  The routines end with BGND; completion is detected by
polling the BDM status register ($FF01 bit6 BDMACT) with hardware READ_BD_BYTE
commands, which are served while user code is running.

DG128A flash facts (MC68HC912DG128 datasheet, appendix "MC68HC912DG128A"):
  - 4 x 32K modules; module m register page (FEELCK/FEEMCR/FEECTL at
    $00F4-$00F7) is visible while PPAGE ($00FF) = 2m or 2m+1; module data
    visible through the 16K window at $8000-$BFFF (page 2m = low half).
  - FEECTL: bit3 HVEN, bit1 ERAS, bit0 PGM.  Program: word only, 64-byte rows;
    erase: bulk of a whole 32K module.  FEEMCR bit0 BOOTP (reset=1) protects
    the top 8K boot block of each module.
  - EEPROM (appendix 22): EEPROG $00F3 (BULKP/AUTO/BYTE/ROW/ERASE/EELAT/EEPGM),
    EEPROT $00F1 (all blocks protected at reset), EEMCR $00F0 bit2 PROTLCK.
    EEDIV $00EE must hold round(EXTALi * 35us); it normally loads from the
    EEPROM shadow word, which is erased ($FFFF -> EEDIV=$3FF) on this unit, so
    the programmer rewrites it after measuring the bus clock.

Command line:
    ./podex_programmer.py <command> ...
    commands: probe calibrate read-flash read-eeprom verify-flash verify-eeprom
              erase-flash erase-eeprom write-flash write-eeprom restore reset
Use <command> --help for details.
"""

import argparse
import fcntl
import hashlib
import os
import struct
import sys
import termios
import time

import serial

HANDSHAKE_TIMEOUT = 0.4
PPAGE_ADDR = 0x00FF
REF_FLASH_MD5 = 'c23d85d31bafe6ca75e97fe196b2b9bd'
REF_EEPROM_MD5 = '4702072b19b5a9c0b0ba3bc192476e05'
FLASH_SIZE = 128 * 1024
EEPROM_BASE = 0x0800
EEPROM_SIZE = 2048
RAM_BASE = 0x2000

# RAM layout shared with the HC12 routines (tools/asm/common.inc)
ROUTINE_ADDR = 0x2000
ROUTINE_MAX = 384
PARAM_ADDR = 0x2180
DONE_FLAG = 0x21C0
ERR_FLAG = 0x21C1
BITMAP_ADDR = 0x21E0
BUFFER_ADDR = 0x2200     # max stream = 8 + 64*64 = 4104 bytes -> $2200-$3207
BUFFER_SIZE = 0x3FF0 - BUFFER_ADDR - 16  # keep clear of the routine stack
DONE_MARK = 0xA5

# BDM status register $FF01 bits
BDM_ENBDM = 0x80
BDM_BDMACT = 0x40

# microseconds -> HC12 dlyd loop counts.
# dlyd loop body: SUBD #imm16 (2) + BNE taken (3) = 5 cycles per iteration;
# call overhead: BSR (4) + RTS (5) - final BNE (1) - first iteration rounding = ~7
DLY_LOOP_CYCLES = 5
DLY_OVERHEAD_CYCLES = 7

FLASH_ROW = 64
FLASH_ROWS_PER_CALL = 64          # 4KB chunks; bitmap = 8 bytes
# targets in microseconds; datasheet minimums: NVS 10, PGS 5, FPGM 30 (max 40),
# NVH 5, NVHL 100, RCV 1, ERAS 8000.  The per-word CPU loop adds ~12 cycles
# around the dlyd call, so delivered t_FPGM lands ~34-35us at 4-8 MHz bus.
FLASH_TIMING_US = dict(NVS=100, PGS=2000, FPGM=33, NVH=8, RCV=8)
FLASH_ERASE_TIMING_US = dict(NVS=15, ERAS=20000, NVHL=150, RCV=8)


def asm_path(name):
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), 'asm', name)


def read_routine(name):
    path = asm_path(name + '.bin')
    with open(path, 'rb') as f:
        code = f.read()
    if len(code) > ROUTINE_MAX:
        raise RuntimeError(f'{path}: {len(code)} bytes exceeds {ROUTINE_MAX}')
    return code


class PodexBDM:
    """Kevin-Ross-BDM12-protocol podex pod, fast RTS/CTS byte handshake."""

    def __init__(self, port='/dev/ttyUSB0', baud=460800, debug=False):
        print(f'[*] Connecting to Podex BDM on {port}...')
        self.debug = debug
        self.inverted = False
        self.ser = serial.Serial(port, baud, timeout=0.6, write_timeout=1,
                                 rtscts=False)
        self.ser.rts = False
        time.sleep(1.0)  # allow serial port to settle
        self._init_tiocmget()
        self._lower_ftdi_latency()
        self.f_bus = None          # measured target bus clock [Hz]
        self.sync()

    # ------------------------------------------------------------------
    # low-level RTS/CTS handshake
    #
    # podex firmware raises its RTS (= our CTS) to accept a byte and drops
    # it once consumed; we must drop our RTS afterwards before the next byte.
    # ------------------------------------------------------------------
    def _init_tiocmget(self):
        try:
            fcntl.ioctl(self.ser.fd, termios.TIOCMGET, b'\x00' * 4)
            self._use_tio = True
        except (IOError, ValueError, OSError):
            self._use_tio = False  # fall back to read() interleaving (macOS)

    def _lower_ftdi_latency(self):
        """1ms FTDI latency timer speeds up CTS change reporting."""
        try:
            path = os.path.realpath(self.ser.port)
            lt = f'/sys/bus/usb-serial/devices/{os.path.basename(path)}/latency_timer'
            if os.path.exists(lt):
                with open(lt) as f:
                    cur = int(f.read())
                if cur > 1:
                    with open(lt, 'w') as f:
                        f.write('1')
                    print(f'[*] FTDI latency timer {cur} -> 1')
        except (OSError, IOError, PermissionError):
            pass

    def _cts(self):
        if self._use_tio:
            m = int.from_bytes(fcntl.ioctl(self.ser.fd, termios.TIOCMGET,
                                           b'\x00' * 4), 'little')
            return bool(m & termios.TIOCM_CTS) != self.inverted
        return self.ser.cts != self.inverted

    def _set_rts(self, asserted):
        self.ser.rts = asserted != self.inverted

    def _wait_cts(self, wanted, what):
        if not self._use_tio:
            # the macOS FTDI driver refreshes cached modem status lazily:
            # interleave reads so the status is re-fetched
            self.ser.timeout = 0.02
            deadline = time.monotonic() + HANDSHAKE_TIMEOUT
            while time.monotonic() < deadline:
                if self._cts() == wanted:
                    return
                self.ser.read(16)
        else:
            deadline = time.monotonic() + HANDSHAKE_TIMEOUT
            while time.monotonic() < deadline:
                if self._cts() == wanted:
                    return
        raise TimeoutError('pod did not %s (CTS stuck at %s)' %
                           (what, not wanted))

    def send_bytes(self, data, do_print=False):
        if do_print or self.debug:
            print(bytes(data).hex(), len(data))
        for b in data:
            self._set_rts(True)
            self._wait_cts(True, 'accept a byte')
            self.ser.write(bytes((b,)))
            self.ser.flush()
            self._wait_cts(False, 'consume the byte')
            self._set_rts(False)

    def resync(self):
        """Recover a pod stuck mid-handshake and put it back at the command loop."""
        for _ in range(5):
            try:
                self.ser.reset_input_buffer()
                self._set_rts(False)
                time.sleep(0.1)
                self._set_rts(True)
                self._wait_cts(True, 'accept a byte')
                self.ser.write(b'\x00')
                self.ser.flush()
                self._wait_cts(False, 'consume the byte')
                self._set_rts(False)
                time.sleep(0.05)
                return True
            except TimeoutError:
                continue
        raise TimeoutError('pod does not resync')

    def sync(self):
        """Syncs with the Podex firmware."""
        self.send_bytes(b'\x00')
        time.sleep(0.1)

    # ------------------------------------------------------------------
    # pod commands
    # ------------------------------------------------------------------
    def check_version(self):
        self.send_bytes(b'\x04\x00')
        self.ser.timeout = 1.0
        v = self.ser.read(1)
        print(f'podex ver: {v.hex()}')
        return v == b'\xc7'

    def set_clock(self):
        """Tell the pod the target runs at 4 MHz E-clock (BDM bit timing)."""
        self.send_bytes(b'\x04\x04\x02\x00\x00')

    def enter_bdm_mode(self):
        """Reset the target into active BDM (special mode) and enable firmware."""
        self.send_bytes(bytes([0x04, 0x05, 0xFC]))  # RESET + BKGD low
        time.sleep(0.020)
        self.send_bytes(bytes([0x04, 0x05, 0xFE]))  # release RESET, BKGD low
        time.sleep(0.050)
        self.send_bytes(bytes([0x04, 0x05, 0xFF]))  # release BKGD
        time.sleep(0.010)
        self.set_clock()
        # set ENBDM: WRITE_BD_BYTE $FF01 = 0x80
        self.send_bytes(bytes([0xC4, 0xFF, 0x01, 0x00, 0x80]))
        time.sleep(0.01)

    def reset_to_bdm(self):
        """Firmware command 0x01: reset the target into BDM, then re-enable."""
        self.send_bytes(b'\x01')
        time.sleep(0.3)
        self.set_clock()
        self.send_bytes(bytes([0xC4, 0xFF, 0x01, 0x00, 0x80]))
        time.sleep(0.01)

    def regdump(self):
        self.send_bytes(b'\x04\x01')
        self.ser.timeout = 1.5
        data = self.ser.read(16)
        print(data.hex(), len(data))

    # ------------------------------------------------------------------
    # memory access
    # ------------------------------------------------------------------
    def memdump(self, addr, nbytes):
        """Read memory via BDM12X_MEMDUMP (word reads).  addr must be even."""
        if addr & 1 or nbytes & 1:
            raise ValueError('memdump needs even address and byte count')
        if nbytes == 0:
            return b''
        self.ser.timeout = max(3.0, nbytes / 4000.0)
        self.send_bytes(bytes([0x04, 0x03, (addr >> 8) & 0xFF, addr & 0xFF,
                               (nbytes // 2 >> 8) & 0xFF, (nbytes // 2) & 0xFF]))
        data = self.ser.read(nbytes)
        if len(data) != nbytes:
            raise TimeoutError(f'timeout reading memory at 0x{addr:04X} '
                               f'({len(data)}/{nbytes} bytes)')
        return data

    def memput(self, addr, data):
        """Write memory via BDM12X_MEMPUT (word writes).  addr must be even."""
        if addr & 1 or len(data) & 1:
            raise ValueError('memput needs even address and byte count')
        if not data:
            return
        self.send_bytes(bytes([0x04, 0x06, (addr >> 8) & 0xFF, addr & 0xFF,
                               (len(data) // 2 >> 8) & 0xFF,
                               (len(data) // 2) & 0xFF]))
        self.send_bytes(data)

    def write_byte_raw(self, addr, data):
        """Write one byte with the BDM WRITE_BYTE hardware command."""
        if addr & 1:
            word = [data, 0x00]
        else:
            word = [0x00, data]
        self.send_bytes(bytes([0xC0, (addr >> 8) & 0xFF, addr & 0xFF,
                               word[1], word[0]]))

    def write_byte(self, addr, data):
        self.write_byte_raw(addr, data)
        time.sleep(0.002)

    def write_pc(self, pc):
        self.send_bytes(bytes([0x43, (pc >> 8) & 0xFF, pc & 0xFF]))
        time.sleep(0.02)

    def go(self):
        self.send_bytes(b'\x08')
        time.sleep(0.02)

    def halt(self):
        """BACKGROUND command: force the running CPU back into active BDM."""
        self.send_bytes(b'\x90')
        time.sleep(0.05)

    def read_pc(self):
        self.send_bytes(b'\x63')
        self.ser.timeout = 1.0
        d = self.ser.read(2)
        if len(d) != 2:
            raise TimeoutError('no READ_PC response')
        return struct.unpack('>H', d)[0]

    def bdm_status(self):
        """BDM STATUS byte (READ_BD_BYTE $FF01; byte answers in the low lane)."""
        self.send_bytes(bytes([0xE4, 0xFF, 0x01]))
        self.ser.timeout = 1.0
        d = self.ser.read(2)
        if len(d) != 2:
            raise TimeoutError('no BDM status response')
        return d[1]

    def read_flash(self, addr, length):
        return self.memdump(addr, (length + 1) // 2 * 2)[:length]

    def read_eeprom(self, addr, length):
        return self.memdump(addr, (length + 1) // 2 * 2)[:length]

    def write_ppage(self, page):
        self.write_byte(PPAGE_ADDR, page & 0x07)

    # ------------------------------------------------------------------
    # HC12 RAM routine execution
    # ------------------------------------------------------------------
    def load_routine(self, name):
        code = read_routine(name)
        if len(code) & 1:
            code += b'\xFF'  # memput streams words; padding byte is harmless
        self.memput(ROUTINE_ADDR, code)
        back = self.memdump(ROUTINE_ADDR, len(code))
        if back[:len(code)] != code:
            raise RuntimeError(f'routine {name} upload verification failed')
        return len(code)

    def run_routine(self, timeout=30.0, expect_done=True):
        """GO the routine at $2000 and wait for it to hit BGND."""
        # clear DONE/ERR first so stale flags from a previous run cannot
        # mask a crash of this one
        self.memput(DONE_FLAG, b'\x00\x00')
        self.write_pc(ROUTINE_ADDR)
        t0 = time.time()
        self.go()
        while time.time() - t0 < timeout:
            if self.bdm_status() & BDM_BDMACT:
                elapsed = time.time() - t0
                if expect_done:
                    flags = self.memdump(DONE_FLAG, 2)
                    if flags[0] != DONE_MARK:
                        pc = self.read_pc()
                        raise RuntimeError(f'routine halted without DONE flag '
                                           f'(PC=0x{pc:04X}, '
                                           f'flags={flags.hex()})')
                    if flags[1] != 0:
                        raise RuntimeError(f'routine reported error '
                                           f'0x{flags[1]:02X}')
                return elapsed
        # routine did not come back: force it into BDM for diagnosis
        self.halt()
        pc = self.read_pc()
        raise TimeoutError(f'routine did not reach BGND within {timeout}s '
                           f'(PC now 0x{pc:04X})')

    def put_params(self, data):
        self.memput(PARAM_ADDR, data)

    # ------------------------------------------------------------------
    # clock calibration + register setup
    # ------------------------------------------------------------------
    def calibrate(self):
        """Measure the target bus clock by timing the calib spin routine."""
        self.load_routine('calib')
        outer = 8
        for _ in range(2):
            self.put_params(struct.pack('>H', outer))
            elapsed = self.run_routine(timeout=60, expect_done=True)
            cycles = outer * (65536 * 4 + 6) + 17
            self.f_bus = cycles / elapsed
            # size the final run for ~2.5s wall time
            outer = min(0xFFFF, max(8, int(2.5 * self.f_bus / (65536 * 4 + 6))))
        print(f'[*] target bus clock: {self.f_bus/1e6:.3f} MHz')
        return self.f_bus

    def dly(self, us):
        """Convert microseconds to a dlyd loop count for the measured clock."""
        if self.f_bus is None:
            raise RuntimeError('bus clock not calibrated yet')
        n = (int(us * 1e-6 * self.f_bus) - DLY_OVERHEAD_CYCLES) // DLY_LOOP_CYCLES
        return max(1, min(0xFFFF, n))

    def setup_eediv(self):
        """Program EEDIV = round(EXTALi * 35us) for the EEPROM self-timer."""
        eediv = int(2 * self.f_bus * 35e-6 + 0.5) & 0x3FF
        cur = self.memdump(0x00EE, 2)
        want = struct.pack('>H', eediv)
        if cur != want:
            self.memput(0x00EE, want)
            time.sleep(0.01)
            cur = self.memdump(0x00EE, 2)
            if cur != want:
                raise RuntimeError(f'EEDIV write failed: {cur.hex()}')
        print(f'[*] EEDIV = 0x{eediv:04X} '
              f'(EXTALi = {2*self.f_bus/1e6:.3f} MHz)')
        return eediv

    # ------------------------------------------------------------------
    # session bring-up
    # ------------------------------------------------------------------
    def connect(self, calibrate=True):
        if not self.check_version():
            raise SystemExit('podex is dead')
        self.enter_bdm_mode()
        st = self.bdm_status()
        if not st & BDM_BDMACT:
            raise RuntimeError(f'target not in active BDM (status 0x{st:02X})')
        print('[*] target in active BDM')
        self.disable_cop()
        if calibrate:
            self.calibrate()
            self.setup_eediv()
        return self

    def disable_cop(self):
        """COPCTL = 0x08: set DISR (bit3) to disable the COP watchdog.

        DISR is clear-able/reset in special modes.  We also clear CME (clock
        monitor) while we are at it: bit layout is CME,FCME,FCMCOP,WCOP,
        DISR,CR2,CR1,CR0.  Do NOT use bit4 (that is WCOP, windowed COP!).
        """
        self.write_byte(0x0016, 0x08)
        if self.memdump(0x0016, 2)[0] != 0x08:
            raise RuntimeError('could not disable COP watchdog (DISR)')
        print('[*] COP watchdog disabled (COPCTL=0x08, DISR=1)')

    # ------------------------------------------------------------------
    # flash operations (DG128A, 4 x 32K modules, window at $8000)
    # ------------------------------------------------------------------
    def read_flash_all(self):
        out = bytearray()
        for page in range(8):
            self.write_ppage(page)
            out += self.memdump(0x8000, 16384)
            print(f'[*] flash page {page} read')
        return bytes(out)

    def flash_blank_check(self, modules=0xF):
        for m in range(4):
            if not modules & (1 << m):
                continue
            for page in (2 * m, 2 * m + 1):
                self.write_ppage(page)
                data = self.memdump(0x8000, 16384)
                if data != b'\xFF' * 16384:
                    first = next(i for i, b in enumerate(data) if b != 0xFF)
                    raise RuntimeError(
                        f'flash module {m} (page {page}) not blank at '
                        f'offset 0x{first:04X}')
            print(f'[*] flash module {m} blank')
        return True

    def erase_flash(self, modules=0xF):
        """Bulk-erase the selected 32K flash modules (mask bit m = module m).

        Each module gets its own routine run: four back-to-back HV erase
        cycles in a single run proved unreliable on this silicon.
        """
        self.load_routine('flash_erase')
        t = FLASH_ERASE_TIMING_US
        # $2180 MASK, $2182 NVS, $2184 ERAS, $2186 NVHL, $2188 RCV (all u16)
        for m in range(4):
            if not modules & (1 << m):
                continue
            self.put_params(struct.pack('>HHHHH', 1 << m, self.dly(t['NVS']),
                                        self.dly(t['ERAS']),
                                        self.dly(t['NVHL']),
                                        self.dly(t['RCV'])))
            elapsed = self.run_routine(timeout=10)
            print(f'[*] flash module {m} erased ({elapsed:.2f}s)')
        return self.flash_blank_check(modules)

    def program_flash(self, data, offset=0, verify=True):
        """Program raw image bytes at `offset` (64-byte row aligned).

        Fully-erased ($FF) rows are skipped and stay erased, so `restore`
        only touches rows that need it.
        """
        if offset % FLASH_ROW or len(data) % FLASH_ROW:
            raise ValueError('flash writes must be 64-byte row aligned')
        if offset + len(data) > FLASH_SIZE:
            raise ValueError('image exceeds flash size')
        self.load_routine('flash_prog')
        t = FLASH_TIMING_US
        # $2180 NVS $2182 PGS $2184 FPGM $2186 NVH $2188 RCV (u16),
        # $218A PPAGE (u8), $218B pad, $218C ROWS (u16), $218E BUF (u16),
        # $2190 BASE (u16 array base for this quarter's rows)
        def page_params(page, quarter):
            # modules 0-2: rows go through the 16K page window at $8000;
            # module 3 (pages 6/7) has a read-only window, its halves are
            # fixed at $4000-$7FFF and $C000-$FFFF
            base = {6: 0x4000, 7: 0xC000}.get(page, 0x8000)
            base += quarter * FLASH_ROWS_PER_CALL * FLASH_ROW
            return struct.pack('>HHHHHBxHHH', self.dly(t['NVS']),
                               self.dly(t['PGS']), self.dly(t['FPGM']),
                               self.dly(t['NVH']), self.dly(t['RCV']),
                               page, FLASH_ROWS_PER_CALL, BUFFER_ADDR, base)
        t0 = time.time()
        rows_done = 0
        for page in range(8):
            page_off = page * 16384
            if page_off + 16384 <= offset or page_off >= offset + len(data):
                continue
            for quarter in range(4):
                base = page_off + quarter * FLASH_ROWS_PER_CALL * FLASH_ROW
                if base + FLASH_ROWS_PER_CALL * FLASH_ROW <= offset or \
                   base >= offset + len(data):
                    continue
                chunk = data[base - offset:base - offset +
                             FLASH_ROWS_PER_CALL * FLASH_ROW]
                chunk += b'\xFF' * (FLASH_ROWS_PER_CALL * FLASH_ROW - len(chunk))
                # stream layout consumed by flash_prog.s, one group (8 rows)
                # at a time: [bitmap byte (bit0 = first row of the group)]
                # followed by the packed 64-byte content of each present row
                stream = bytearray()
                for g in range(FLASH_ROWS_PER_CALL // 8):
                    gbm = 0
                    for r in range(g * 8, g * 8 + 8):
                        if chunk[r * FLASH_ROW:(r + 1) * FLASH_ROW] != \
                                b'\xFF' * FLASH_ROW:
                            gbm |= 1 << (r % 8)
                    stream.append(gbm)
                    rows_done += bin(gbm).count('1')
                    for r in range(g * 8, g * 8 + 8):
                        if gbm & (1 << (r % 8)):
                            stream += chunk[r * FLASH_ROW:(r + 1) * FLASH_ROW]
                if len(stream) & 1:
                    stream.append(0xFF)
                self.put_params(page_params(page, quarter))
                self.memput(BUFFER_ADDR, bytes(stream))
                self.run_routine(timeout=30)
        dt = time.time() - t0
        print(f'[*] programmed {rows_done} rows ({rows_done*FLASH_ROW} bytes) '
              f'in {dt:.1f}s')
        if verify:
            return self.verify_flash(data, offset)
        return True

    def verify_flash(self, data, offset=0):
        for page in range(8):
            page_off = page * 16384
            lo = max(offset, page_off)
            hi = min(offset + len(data), page_off + 16384)
            if lo >= hi:
                continue
            n = hi - lo
            self.write_ppage(page)
            actual = self.memdump(0x8000 + (lo - page_off), n + (n & 1))[:n]
            want = data[lo - offset:hi - offset]
            if actual != want:
                for i, (a, b) in enumerate(zip(actual, want)):
                    if a != b:
                        raise RuntimeError(
                            f'verify failed at flash 0x{lo+i:05X} '
                            f'(page {page}): read 0x{a:02X}, want 0x{b:02X}')
        print(f'[*] flash verify ok ({len(data)} bytes at 0x{offset:05X})')
        return True

    # ------------------------------------------------------------------
    # EEPROM operations (2K at $0800)
    # ------------------------------------------------------------------
    def read_eeprom_all(self):
        return self.memdump(EEPROM_BASE, EEPROM_SIZE)

    def erase_eeprom(self, mode='bulk', rows=None, addr=EEPROM_BASE):
        """mode 'bulk': erase the whole array; mode 'rows': erase `rows` 32B rows."""
        self.load_routine('ee_erase')
        # $2180 MODE (u8), $2181 pad, $2182 ROWS (u16), $2184 ADDR (u16)
        if mode == 'bulk':
            params = struct.pack('>BxHH', 0, 0, addr)
        elif mode == 'rows':
            rows = rows or EEPROM_SIZE // 32
            params = struct.pack('>BxHH', 1, rows, addr)
        else:
            raise ValueError(mode)
        self.put_params(params)
        elapsed = self.run_routine(timeout=10)
        print(f'[*] EEPROM erase ({mode}) in {elapsed:.2f}s')
        return True

    def program_eeprom(self, data, addr=EEPROM_BASE, verify=True):
        if addr % 2 or len(data) % 2:
            raise ValueError('EEPROM writes must be word aligned')
        if addr + len(data) > EEPROM_BASE + EEPROM_SIZE:
            raise ValueError('image exceeds EEPROM size')
        self.load_routine('ee_prog')
        params = struct.pack('>HHH', len(data) // 2, addr, BUFFER_ADDR)
        self.put_params(params)
        self.memput(BUFFER_ADDR, data)
        t0 = time.time()
        self.run_routine(timeout=30)
        print(f'[*] EEPROM programmed {len(data)} bytes in {time.time()-t0:.1f}s')
        if verify:
            return self.verify_eeprom(data, addr)
        return True

    def verify_eeprom(self, data, addr=EEPROM_BASE):
        actual = self.memdump(addr, len(data))
        if actual != data:
            for i, (a, b) in enumerate(zip(actual, data)):
                if a != b:
                    raise RuntimeError(f'EEPROM verify failed at '
                                       f'0x{addr+i:04X}: read 0x{a:02X}, '
                                       f'want 0x{b:02X}')
        print(f'[*] EEPROM verify ok ({len(data)} bytes at 0x{addr:04X})')
        return True

    # ------------------------------------------------------------------
    # diagnostics
    # ------------------------------------------------------------------
    def probe(self):
        st = self.bdm_status()
        print(f'BDM status ($FF01): 0x{st:02X} '
              f'({"active" if st & BDM_BDMACT else "halted/inactive"})')
        mode = self.memdump(0x000A, 2)
        print(f'PEAR/MODE: 0x{mode[0]:02X}/0x{mode[1]:02X} '
              f'(SMODN={"0: special" if not mode[1] & 0x80 else "1: normal"}')
        regs = self.memdump(0x0010, 8)
        print(f'INITRM=0x{regs[0]:02X} INITRG=0x{regs[1]:02X} '
              f'INITEE=0x{regs[2]:02X} MISC=0x{regs[3]:02X}')
        print(f'COPCTL=0x{regs[6]:02X} (DISR={"set: COP off" if regs[6] & 0x10 else "clear"})')
        eediv = self.memdump(0x00EE, 2)
        print(f'EEDIV=0x{struct.unpack(">H", eediv)[0]:04X}')
        ee = self.memdump(0x00F0, 4)
        print(f'EEMCR=0x{ee[0]:02X} EEPROT=0x{ee[1]:02X} EETST=0x{ee[2]:02X} '
              f'EEPROG=0x{ee[3]:02X}')
        # probe each flash module register page (safe: FEECTL reads only)
        for m in range(4):
            self.write_ppage(2 * m)
            fe = self.memdump(0x00F4, 4)
            print(f'flash module {m}: FEELCK=0x{fe[0]:02X} '
                  f'FEEMCR=0x{fe[1]:02X} FEETST=0x{fe[2]:02X} '
                  f'FEECTL=0x{fe[3]:02X}')
        self.write_ppage(0)
        if self.f_bus:
            print(f'bus clock: {self.f_bus/1e6:.3f} MHz')
        pc = self.read_pc()
        print(f'PC=0x{pc:04X}')


# ----------------------------------------------------------------------
# command line interface
# ----------------------------------------------------------------------
def md5sum(data):
    return hashlib.md5(data).hexdigest()


def check_md5(label, data, ref):
    got = md5sum(data)
    ok = got == ref
    print(f'{"[+]" if ok else "[!]"} {label} md5 {got} '
          f'{"== REF (PASS)" if ok else "!= REF " + ref + " (FAIL)"}')
    return ok


def load_image(path, size=None):
    with open(path, 'rb') as f:
        data = f.read()
    if size is not None and len(data) > size:
        raise SystemExit(f'{path}: {len(data)} bytes exceeds {size}')
    return data


def save_image(path, data):
    with open(path, 'wb') as f:
        f.write(data)
    print(f'[*] wrote {len(data)} bytes to {path}')


def ref_path(name):
    return os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        'ref_bins', name)


def do_probe(bdm, args):
    bdm.probe()


def do_calibrate(bdm, args):
    pass  # connect() already calibrated


def do_read_flash(bdm, args):
    data = bdm.read_flash_all()
    save_image(args.file, data)
    if args.ref:
        check_md5('flash', data, REF_FLASH_MD5)


def do_read_eeprom(bdm, args):
    data = bdm.read_eeprom_all()
    save_image(args.file, data)
    if args.ref:
        check_md5('eeprom', data, REF_EEPROM_MD5)


def do_verify_flash(bdm, args):
    data = load_image(args.file, FLASH_SIZE)
    bdm.verify_flash(data, args.offset)


def do_verify_eeprom(bdm, args):
    data = load_image(args.file, EEPROM_SIZE)
    bdm.verify_eeprom(data, args.addr)


def do_erase_flash(bdm, args):
    if not args.yes:
        inp = input('Erase flash modules 0-3? This wipes the whole 128K. '
                    'Type "yes": ')
        if inp.strip().lower() != 'yes':
            raise SystemExit('aborted')
    modules = int(args.modules, 0)
    bdm.erase_flash(modules)


def do_erase_eeprom(bdm, args):
    if not args.yes:
        inp = input('Erase EEPROM? Type "yes": ')
        if inp.strip().lower() != 'yes':
            raise SystemExit('aborted')
    bdm.erase_eeprom(mode=args.mode, rows=args.rows, addr=args.addr)


def do_write_flash(bdm, args):
    data = load_image(args.file, FLASH_SIZE)
    pad = (-len(data)) % FLASH_ROW
    if pad:
        data += b'\xFF' * pad
        print(f'[*] padded image to {len(data)} bytes (row alignment)')
    if not args.yes and not args.verify_only:
        inp = input('Is the target flash range erased, or erase first? '
                    '[erase/skip/abort]: ')
        inp = inp.strip().lower()
        if inp == 'erase':
            bdm.erase_flash(0xF)
        elif inp != 'skip':
            raise SystemExit('aborted')
    bdm.program_flash(data, offset=args.offset, verify=not args.no_verify)


def do_write_eeprom(bdm, args):
    data = load_image(args.file, EEPROM_SIZE)
    if len(data) % 2:
        data += b'\xFF'
    if not args.yes:
        inp = input('Erase EEPROM before programming? [yes/skip/abort]: ')
        inp = inp.strip().lower()
        if inp == 'yes':
            bdm.erase_eeprom(mode='bulk', addr=args.addr)
        elif inp != 'skip':
            raise SystemExit('aborted')
    bdm.program_eeprom(data, addr=args.addr, verify=not args.no_verify)


def do_restore(bdm, args):
    """Erase + program + verify both memories from the reference dumps."""
    flash = load_image(ref_path('full_flash_dump.bin'), FLASH_SIZE)
    eeprom = load_image(ref_path('full_eeprom_dump.bin'), EEPROM_SIZE)
    if not args.yes:
        inp = input('Erase and restore flash (128K) + EEPROM (2K) from '
                    'ref_bins? Type "yes": ')
        if inp.strip().lower() != 'yes':
            raise SystemExit('aborted')
    print('[*] === EEPROM ===')
    bdm.erase_eeprom(mode='bulk')
    bdm.program_eeprom(eeprom, verify=True)
    check_md5('eeprom', bdm.read_eeprom_all(), REF_EEPROM_MD5)
    print('[*] === FLASH ===')
    bdm.erase_flash(0xF)
    bdm.program_flash(flash, offset=0, verify=True)
    check_md5('flash', bdm.read_flash_all(), REF_FLASH_MD5)


def do_reset(bdm, args):
    bdm.reset_to_bdm()
    st = bdm.bdm_status()
    print(f'[*] target reset into BDM (status 0x{st:02X})')


def main():
    ap = argparse.ArgumentParser(
        description='MC912DG128A flash/EEPROM programmer over podex BDM')
    ap.add_argument('--port', default='/dev/ttyUSB0')
    ap.add_argument('--debug', action='store_true')
    sub = ap.add_subparsers(dest='cmd', required=True)

    def sp(name, help_, fn):
        p = sub.add_parser(name, help=help_)
        p.set_defaults(fn=fn)
        return p

    sp('probe', 'show target registers and BDM status', do_probe)
    sp('calibrate', 'measure the target bus clock', do_calibrate)
    sp('reset', 'reset the target into BDM', do_reset)

    p = sp('read-flash', 'dump the whole 128K flash to a file', do_read_flash)
    p.add_argument('file')
    p.add_argument('--ref', action='store_true',
                   help='compare md5 against the reference dump')

    p = sp('read-eeprom', 'dump the whole 2K EEPROM to a file', do_read_eeprom)
    p.add_argument('file')
    p.add_argument('--ref', action='store_true',
                   help='compare md5 against the reference dump')

    p = sp('verify-flash', 'verify flash against a raw image', do_verify_flash)
    p.add_argument('file')
    p.add_argument('--offset', type=lambda x: int(x, 0), default=0)

    p = sp('verify-eeprom', 'verify EEPROM against a raw image',
           do_verify_eeprom)
    p.add_argument('file')
    p.add_argument('--addr', type=lambda x: int(x, 0), default=EEPROM_BASE)

    p = sp('erase-flash', 'bulk-erase whole flash modules', do_erase_flash)
    p.add_argument('--modules', default='0xF',
                   help='module bitmask 0x0-0xF (default 0xF = all)')
    p.add_argument('--yes', action='store_true')

    p = sp('erase-eeprom', 'erase EEPROM (bulk or 32-byte rows)',
           do_erase_eeprom)
    p.add_argument('--mode', choices=['bulk', 'rows'], default='bulk')
    p.add_argument('--addr', type=lambda x: int(x, 0), default=EEPROM_BASE,
                   help='start address for row mode')
    p.add_argument('--rows', type=lambda x: int(x, 0), default=None,
                   help='row count for row mode (default: all 64)')
    p.add_argument('--yes', action='store_true')

    p = sp('write-flash', 'program a raw image into (erased) flash',
           do_write_flash)
    p.add_argument('file')
    p.add_argument('--offset', type=lambda x: int(x, 0), default=0,
                   help='destination offset, 64-byte aligned (default 0)')
    p.add_argument('--no-verify', action='store_true')
    p.add_argument('--yes', action='store_true',
                   help='do not ask about erasing first')

    p = sp('write-eeprom', 'program a raw image into (erased) EEPROM',
           do_write_eeprom)
    p.add_argument('file')
    p.add_argument('--addr', type=lambda x: int(x, 0), default=EEPROM_BASE)
    p.add_argument('--no-verify', action='store_true')
    p.add_argument('--yes', action='store_true',
                   help='do not ask about erasing first')

    p = sp('restore',
           'erase + program + verify both memories from ref_bins',
           do_restore)
    p.add_argument('--yes', action='store_true')

    args = ap.parse_args()
    bdm = PodexBDM(port=args.port, debug=args.debug)
    try:
        bdm.connect()
        args.fn(bdm, args)
    finally:
        bdm.ser.close()


if __name__ == '__main__':
    main()
