import serial
import time
import hashlib

HANDSHAKE_TIMEOUT = 0.4
PPAGE_ADDR = 0x00FF
REF_FLASH = 'c23d85d31bafe6ca75e97fe196b2b9bd'
REF_EEPROM = '4702072b19b5a9c0b0ba3bc192476e05'

class PodexBDM:
    def __init__(self, port="/dev/ttyUSB0", baud=115200*4):
        """
        Initialize the Podex BDM interface.
        :param port: Serial port (e.g., 'COM3' on Windows or '/dev/ttyUSB0' on Linux)
        :param baud: Baud rate (default is 115200 for Podex)
        """
        print(f"[*] Connecting to Podex BDM on {port}...")

        self.inverted = False
        self.ser = serial.Serial(port, baud, timeout=0.6, write_timeout=1, rtscts=False)
        self.rts = False
        time.sleep(1.0) # Allow serial port to settle
        self.sync()

    def _set_rts(self, asserted):
        level = asserted != self.inverted  # XOR
        self.ser.rts = level

    def _cts(self):
        cts = self.ser.cts
        return cts != self.inverted  # XOR

    def _wait_cts(self, wanted, what):
        # interleave reads: the macOS FTDI driver refreshes cached modem
        # status lazily and polling ser.cts alone may never see a change
        self.ser.timeout = 0.02
        deadline = time.monotonic() + HANDSHAKE_TIMEOUT
        while time.monotonic() < deadline:
            if self._cts() == wanted:
                return
            self.ser.read(16)
        raise TimeoutError("pod did not %s (CTS stuck at %s)" %
                           (what, not wanted))

    def send_bytes(self, data, do_print = False):

        if do_print:
            print(bytes(data).hex(), len(data))

        for b in data:
            self._set_rts(True)
            self._wait_cts(True, "accept a byte")
            self.ser.write(bytes((b,)))
            self.ser.flush()
            self._wait_cts(False, "consume the byte")
            self._set_rts(False)

    def sync(self):
        """Syncs with the Podex firmware."""
        self.send_bytes(b'\x00') # BDM12_SYNC
        time.sleep(0.1)

    def check_version(self):
        self.send_bytes(b'\x04\x00')
        v = self.ser.read(1)
        print(f'podex ver: {v.hex()}')
        return v == b'\xc7'

    def set_clock(self):
        print('set_clock')
        self.send_bytes(b'\x04\x04\x02\x00\x00')

    def regdump(self):
        print('regdump')
        self.send_bytes(b'\x04\x01')
        self.ser.timeout = 1.5
        data = self.ser.read(16)
        print(data.hex(), len(data))

        self.send_bytes(b'\x04\x03\xff\xfe\x00\x01')
        data = self.ser.read(12)
        print(data.hex(), len(data))

    def enter_bdm_mode(self):

        # 1. Drive RESET low (0) and BKGD low (0). 
        # Binary: 1111 1100 = 0xFC
        print("[*] Asserting RESET and BKGD low...")
        self.send_bytes(bytes([0x04, 0x05, 0xFC]))
        time.sleep(0.020) # Hold for 20ms

        # 2. Release RESET (1) but KEEP BKGD low (0). 
        # Binary: 1111 1110 = 0xFE
        # This rising edge of RESET while BKGD is low triggers BDM entry!
        print("[*] Releasing RESET, holding BKGD low...")
        self.send_bytes(bytes([0x04, 0x05, 0xFE]))
        time.sleep(0.050) # Wait 50ms for oscillator to start and BDM to initialize

        # 3. Release BKGD (1). 
        # Binary: 1111 1111 = 0xFF
        print("[*] Releasing BKGD...")
        self.send_bytes(bytes([0x04, 0x05, 0xFF]))
        time.sleep(0.010)

        # set clock to 4 MHz
        self.send_bytes(bytes([0x04, 0x04, 0x02, 0x00, 0x00]))

        # enable BDM
        self.send_bytes(bytes([0xC4, 0xFF, 0x01, 0x00, 0x80]))

    def reset_mcu(self):
        """Asserts the RESET pin on the target HC12."""
        if 1:
            self.send_bytes(b'\x01') # BDM12_RESET
            time.sleep(0.2)
        else:
            self.send_bytes(b'\x02')
            time.sleep(0.2)
            self.send_bytes(b'\x03')
            time.sleep(0.2)
        print("[*] MCU Reset.")

    def write_ppage(self, page):
        # WRITE_BYTE (0xC0), addr 0x00FF, byte in low lane (odd address)
        self.send_bytes(bytes([0xC0, 0x00, 0xFF, 0x00, page & 0x07]))
        time.sleep(0.01)

    def memdump(self, addr, word_count):
        """Reads memory using Podex MEMDUMP (0x04 0x03). Returns Big-Endian bytes."""
        if word_count == 0: return b""
        cmd = bytes([0x04, 0x03, (addr >> 8) & 0xFF, addr & 0xFF, 
                     (word_count >> 8) & 0xFF, word_count & 0xFF])
        self.send_bytes(cmd)
        self.ser.timeout=2.5

        data = self.ser.read(word_count * 2)
        if len(data) != word_count * 2:
            raise Exception(f"Timeout reading memory at 0x{addr:04X}!")
        return data

    def memput(self, addr, data):
        """Writes memory using Podex MEMPUT (0x04 0x06). Data must be Big-Endian bytes."""
        if len(data) == 0: return
        if len(data) % 2 != 0:
            data += b'\xFF' # Pad to even length
        word_count = len(data) // 2
        cmd = bytes([0x04, 0x06, (addr >> 8) & 0xFF, addr & 0xFF, 
                     (word_count >> 8) & 0xFF, word_count & 0xFF])
        print(cmd.hex())
        print(data.hex())
        self.send_bytes(cmd)
        self.send_bytes(data)
        time.sleep(0.1) # Wait for Podex to finish writing

    def write_byte_raw(self, addr, data):
        """Write a single byte using WRITE_BYTE (0xC0) hardware command.
        addr: 16-bit address (can be odd or even)
        data: single byte value
        """
        # WRITE_BYTE opcode = 0xC0
        # For odd address: data in low byte
        # For even address: data in high byte
        if addr & 1:  # Odd address
            data_word = [data, 0x00]  # data in low byte
        else:  # Even address
            data_word = [0x00, data]  # data in high byte

        cmd = bytes([
            0xC0,                    # WRITE_BYTE opcode
            (addr >> 8) & 0xFF,     # Address high
            addr & 0xFF,             # Address low
            data_word[1],           
            data_word[0]          
        ])
        self.send_bytes(cmd)
        # No response from WRITE_BYTE

    def write_pc(self, pc):
        """Sets the Program Counter using BDM FW Write PC (0x43)."""
        self.send_bytes(bytes([0x43, (pc >> 8) & 0xFF, pc & 0xFF]))
        time.sleep(0.05)

    def go(self):
        """Starts CPU execution using BDM GO_GO_GO (0x08)."""
        self.send_bytes(b'\x08')
        time.sleep(0.05)

    # ----------------------------------------------------------------
    # EEPROM Operations (Handled natively by Podex Firmware)
    # ----------------------------------------------------------------
    def read_eeprom(self, addr, length):
        """Reads EEPROM memory."""
        return self.memdump(addr, (length + 1) // 2)[:length]

    def write_eeprom_word(self, addr, word):
        """NOTE: NOT TESTED AT ALL - CONSIDER BROKEN OR NON-WORKING
        Writes a single 16-bit word to EEPROM.
        Podex firmware (command 0x05) automatically handles the internal 
        erase-before-write timing cycle!
        """
        cmd = bytes([0x05, (addr >> 8) & 0xFF, addr & 0xFF, 
                     (word >> 8) & 0xFF, word & 0xFF])
        self.send_bytes(cmd)
        time.sleep(0.15) # EEPROM cycle takes ~10ms

    def write_eeprom(self, addr, data):
        """NOTE: NOT TESTED AT ALL - CONSIDER BROKEN OR NON-WORKING
        Writes a byte array to EEPROM."""
        for i in range(0, len(data), 2):
            hi = data[i]
            lo = data[i+1] if i+1 < len(data) else 0xFF
            self.write_eeprom_word(addr + i, (hi << 8) | lo)
            print(f"[*] EEPROM Write 0x{addr+i:04X}")

    # ----------------------------------------------------------------
    # Flash Operations (Requires Upload of Machine Code Routine)
    # ----------------------------------------------------------------
    
    # THE CODE BELOW IS UNTESTED - CONSIDER BROKEN AND NON-WORKING. RAM and FLASH MAPPING IS WRONG
    # Pre-assembled HC12 Machine Code for Flash Programming (Based on Freescale AN2166)
    # Assumes Bus Clock = 8MHz. 
    # Variables are mapped to RAM at 0x0800. Code is mapped to RAM at 0x0900.
    pgm_row_code = bytearray([
        0x18, 0x0B, 0x02, 0x00, 0xF7,  # movb #0x02, FEECTL  (PGM=1)
        0xFC, 0x08, 0x0E,              # ldd T_SHRT          (0x080E)
        0xC3, 0x00, 0x01,              # addd #1
        0x26, 0xFB,                    # bne wait_nvs        
        0x18, 0x0B, 0x0A, 0x00, 0xF7,  # movb #0x0A, FEECTL  (HVEN=1, PGM=1)
        0xFC, 0x08, 0x0E,              # ldd T_SHRT
        0xC3, 0x00, 0x01,              # addd #1
        0x26, 0xFB,                    # bne wait_pgs
        0xEC, 0x00,                    # ldd 2,X+            (Load from RAM buffer)
        0xED, 0x71,                    # std 2,Y+            (Write to Flash)
        0xFC, 0x08, 0x12,              # ldd T_FPGM          (0x0812)
        0xC3, 0x00, 0x01,              # addd #1
        0x26, 0xFB,                    # bne wait_fpgm
        0xFC, 0x08, 0x0A,              # ldd COUNT           (0x080A)
        0x83, 0x00, 0x02,              # subd #2
        0xFD, 0x08, 0x0A,              # std COUNT
        0x27, 0x06,                    # beq end_of_pgm
        0x18, 0x97,                    # tfr Y,D
        0xC4, 0x3F,                    # andb #0x3F          (Check end of 64-byte row)
        0x26, 0xE3,                    # bne pgm_word
        0x18, 0x0B, 0x08, 0x00, 0xF7,  # movb #0x08, FEECTL  (PGM=0, HVEN=1)
        0xFC, 0x08, 0x0E,              # ldd T_SHRT
        0xC3, 0x00, 0x01,              # addd #1
        0x26, 0xFB,                    # bne wait_nvh
        0x18, 0x0B, 0x00, 0x00, 0xF7,  # movb #0x00, FEECTL  (HVEN=0)
        0xFC, 0x08, 0x0E,              # ldd T_SHRT
        0xC3, 0x00, 0x01,              # addd #1
        0x26, 0xFB,                    # bne wait_rcv
        0xFC, 0x08, 0x0A,              # ldd COUNT
        0x26, 0xAA,                    # bne pgm_row         (Loop back to start)
        0x00                           # bgnd                (Halt CPU & Return to BDM)
    ])

    erase_code = bytearray([
    # THE CODE BELOW IS UNTESTED - CONSIDER BROKEN AND NON-WORKING. RAM and FLASH MAPPING IS WRONG
        0x18, 0x0B, 0x02, 0x00, 0xF7,  # movb #0x02, FEECTL  (ERAS=1)
        0xDE, 0x08, 0x00,              # ldx DEST_ADDR       (0x0800)
        0xED, 0x00,                    # std 0,X             (Latch address)
        0xFC, 0x08, 0x0E,              # ldd T_SHRT
        0xC3, 0x00, 0x01,              # addd #1
        0x26, 0xFB,                    # bne wait_nvs0
        0x18, 0x0B, 0x0A, 0x00, 0xF7,  # movb #0x0A, FEECTL  (HVEN=1, ERAS=1)
        0x18, 0x0B, 0x50, 0x08, 0x14,  # movb #0x50, TMP     (0x0814)
        0xFC, 0x08, 0x10,              # ldd T_NVH           (0x0810)
        0xC3, 0x00, 0x01,              # addd #1
        0x26, 0xFB,                    # bne wait_eras2
        0x7A, 0x08, 0x14,              # dec TMP
        0x26, 0xF3,                    # bne wait_eras1
        0x18, 0x0B, 0x08, 0x00, 0xF7,  # movb #0x08, FEECTL  (ERAS=0, HVEN=1)
        0xFC, 0x08, 0x10,              # ldd T_NVH
        0xC3, 0x00, 0x01,              # addd #1
        0x26, 0xFB,                    # bne wait_nvhl
        0x18, 0x0B, 0x00, 0x00, 0xF7,  # movb #0x00, FEECTL  (HVEN=0)
        0xFC, 0x08, 0x0E,              # ldd T_SHRT
        0xC3, 0x00, 0x01,              # addd #1
        0x26, 0xFB,                    # bne wait_rcv0
        0x00                           # bgnd                (Halt CPU & Return to BDM)
    ])

    def _get_timing_vars(self, bus_freq_mhz=8):
        """NOTE: NOT TESTED AT ALL - CONSIDER BROKEN OR NON-WORKING
        Calculates Flash timing variables (T_SHRT, T_NVH, T_FPGM) for the HC12 core."""
        x = bus_freq_mhz
        t_shrt = -x
        t_nvh = -10 * x
        t_fpgm = -3 * x
        return (t_shrt & 0xFFFF).to_bytes(2, 'big'), \
               (t_nvh & 0xFFFF).to_bytes(2, 'big'), \
               (t_fpgm & 0xFFFF).to_bytes(2, 'big')

    def erase_flash(self, addr, bus_freq_mhz=8):
        """NOTE: NOT TESTED AT ALL - CONSIDER BROKEN OR NON-WORKING
         Erases a block of flash memory. Ensure block is 16KB/32KB aligned."""
        t_shrt, t_nvh, t_fpgm = self._get_timing_vars(bus_freq_mhz)
        
        # 1. Initialize Variables at 0x0800
        vars_data = bytearray()
        vars_data += addr.to_bytes(2, 'big')    # DEST_ADDR
        vars_data += addr.to_bytes(2, 'big')    # END_ADDR (unused for erase)
        vars_data += b'\x00\x00'                # BUFFER_ADDR
        vars_data += b'\x00\x00'                # ERROR_FLAG + padding
        vars_data += b'\x00\x00'                # NUM_WRITTEN
        vars_data += b'\x00\x00'                # COUNT
        vars_data += b'\x00\x00'                # FLASH_LEN
        vars_data += t_shrt                     # T_SHRT
        vars_data += t_nvh                      # T_NVH
        vars_data += t_fpgm                     # T_FPGM
        vars_data += b'\x00\x50'                # TMP = 80 (Loop count for 10ms Erase delay)
        
        self.memput(0x0800, vars_data)
        self.memput(0x0900, self.erase_code)
        
        # 2. Execute
        self.write_pc(0x0900)
        self.go()
        
        # 3. Wait for BGND and check success
        time.sleep(0.2)
        res = self.memdump(0x0806, 1)
        if res[0] != 0:
            raise Exception(f"Flash Erase Failed at 0x{addr:04X}!")
        print(f"[+] Flash Erased successfully at 0x{addr:04X}")

    def write_flash(self, addr, data, bus_freq_mhz=8):
        """NOTE: NOT TESTED AT ALL - CONSIDER BROKEN OR NON-WORKING
        Writes a byte array to Flash memory. Must be word-aligned."""
        if len(data) % 2 != 0:
            data += b'\xFF' # Pad to even length
            
        t_shrt, t_nvh, t_fpgm = self._get_timing_vars(bus_freq_mhz)
        
        # 1. Initialize Variables at 0x0800
        vars_data = bytearray()
        vars_data += addr.to_bytes(2, 'big')             # DEST_ADDR
        vars_data += (addr + len(data)).to_bytes(2, 'big') # END_ADDR
        vars_data += b'\x0A\x00'                         # BUFFER_ADDR (0x0A00)
        vars_data += b'\x00\x00'                         # ERROR_FLAG + padding
        vars_data += b'\x00\x00'                         # NUM_WRITTEN
        vars_data += len(data).to_bytes(2, 'big')        # COUNT
        vars_data += len(data).to_bytes(2, 'big')        # FLASH_LEN
        vars_data += t_shrt                              # T_SHRT
        vars_data += t_nvh                               # T_NVH
        vars_data += t_fpgm                              # T_FPGM
        vars_data += b'\x00\x00'                         # TMP
        
        self.memput(0x0800, vars_data)
        self.memput(0x0900, self.pgm_row_code)
        self.memput(0x0A00, data) # Upload target data to RAM buffer
        
        # 2. Execute
        self.write_pc(0x0900)
        self.go()
        
        # 3. Wait for BGND and check success
        time.sleep(0.5) # Max programming time for reasonable blocks
        res = self.memdump(0x0806, 1)
        if res[0] != 0:
            raise Exception(f"Flash Write Failed at 0x{addr:04X}!")
        print(f"[+] Flash Written successfully at 0x{addr:04X} ({len(data)} bytes)")

    def read_flash(self, addr, length):
        """Reads flash memory."""
        return self.memdump(addr, (length + 1) // 2)[:length]

if __name__ == "__main__":
    podex = PodexBDM(port='/dev/ttyUSB0') 

    if podex.check_version():
        print('podex is alive')
    else:
        raise SystemExit('podex is dead')

    podex.enter_bdm_mode()

    podex.regdump()

    # READING WHOLE FLASH MEMORY
    if 1:
        all_flash = bytearray()
        md5 = hashlib.md5()

        for page in range(8): # 8 pages * 16KB = 128KB
            # Write the page number to the PPAGE register
            podex.write_byte_raw(PPAGE_ADDR, page)

            # Verify it was written
            verify = podex.memdump(PPAGE_ADDR ^ 1, 1)
            #print('PPAGE', verify.hex())
            
            # Read the 16KB window at 0x8000
            chunk = podex.read_flash(0x8000, 16384) 
            md5.update(chunk)
            all_flash.extend(chunk)
            print(f"Dumped Page {page}")

        hexdigest = md5.hexdigest()
        print(hexdigest, REF_FLASH == hexdigest and 'PASS' or 'FAIL')
        # Save to a binary file
        with open("full_flash_dump.bin", "wb") as f:
            f.write(all_flash)

    # READING WHOLE EEPROM MEMORY
    if 1:
        all_eeprom = bytearray()
        md5 = hashlib.md5()
        all_eeprom = podex.read_eeprom(0x0800, 2048)
        print(f'Dumped {len(all_eeprom)} EEPROM bytes')

        md5.update(all_eeprom)
        hexdigest = md5.hexdigest()
        print(hexdigest, REF_EEPROM == hexdigest and 'PASS' or 'FAIL')
        # Save to a binary file
        with open("full_eeprom_dump.bin", "wb") as f:
            f.write(all_eeprom)
    

