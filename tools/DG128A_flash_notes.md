# MC912DG128A flash/EEPROM notes — bugs, quirks, gotchas

Everything below was established empirically on this exact unit
(MC912DG128ACPV 3K91D, 8 MHz crystal, bus/E-clock 3.99 MHz, podex BDM)
while building `podex_programmer.py` + `tools/asm/*.s` (2026-09-22).
Items marked **[CAN]** matter specifically for a future on-target
flasher (e.g. Volcano over CAN); everything else is transport-independent
silicon behavior.

## 1. Variant identification — it is the "A" silicon

The chip is a DG128**A**, not a DG128. This is *not* cosmetic:

| | DG128 (non-A) | DG128A (this chip) |
|---|---|---|
| FEECTL bits | ENPE, LAT, ERAS, SVFP | HVEN, ERAS, PGM |
| program voltage | external V_FP pin | internal charge pump |
| EEPROM EEPROG | no AUTO bit | AUTO bit (self-timed ops) |
| datasheet source | main sections 7/8 | appendices 21/22 ("future" parts) |

The main-body and BC32-era docs (incl. **AN1828**, whose CAN flash code
targets the non-A module) describe the *wrong* register set. Use the
MC68HC912DG128 datasheet appendices "MC68HC912DG128A Flash/EEPROM" and
cross-check with the MC912DT128A datasheet (PNG pages 117-121), which is
real A-variant documentation.

Safe runtime probes (no HV involved):
- write EEPROG ($F3) = 0x20 → reads back 0x20 on the A part (AUTO bit exists).
- write FEECTL ($F7) = 0x01 with PPAGE=p → reads back 0x01 only for the
  module whose register page p selects; non-A would refuse ENPE-style bits.

## 2. Memory map / paging

- 4 flash modules × 32K. PPAGE ($00FF, 3 bits) selects a 16K page in the
  $8000-$BFFF window. Pages 0-5 = modules 0-2 (page 2m and 2m+1 = module m);
  pages 6/7 = module 3.
- **Module 3 is additionally fixed at $4000-$7FFF and $C000-$FFFF.**
- Module m's registers FEELCK/FEEMCR/FEETST/FEECTL ($00F4-$00F7) are visible
  only while PPAGE = 2m or 2m+1. All four register pages exist independently.
- RAM 8K at $2000 (INITRM=$20), EEPROM 2K at $0800 (INITEE=$01), registers
  at $0000.
- **[CAN]** don't execute from the $8000 window while changing PPAGE, and
  never run the flasher from the module being erased/programmed (copy the
  routine to RAM first, interrupts off).

### Quirk: module 3's window is read-only for HV operations

Erase/program latch writes to $8000-$BFFF work for modules 0-2 but are
**ignored for module 3** — its latch must be written through the fixed
decode ($4000-$7FFF / $C000-$FFFF). Symptom if you get it wrong: erase
"completes" instantly, blank check fails, nothing happened. Window *reads*
of pages 6/7 work fine.

## 3. Clock, EEDIV, watchdog

- Bus clock = f_osc/2 (PLL bypassed). Measured 3.993 MHz ± 0.003 across
  sessions → 8.000 MHz crystal. Calibrate by timing a known cycle-count
  loop, don't trust labels.
- **EEDIV ($00EE) must be programmed before any EEPROM operation.**
  Reset loads it from the EEPROM shadow word; ours is erased ($FFFF) so it
  comes up **$3FF — wrong**. Correct value: `EEDIV = round(f_EXTAL × 35 µs)`
  (≈ 0x0118 @ 8 MHz). EEDIV=0 → program/erase cycles never activate.
  **[CAN]** in normal mode EEDIV is **write-once after reset** — a CAN
  bootloader must write it (correctly) as its first EEPROM action. Special
  mode (BDM) allows rewriting anytime.
- **COPCTL ($0016) bit trap**: bit 4 is **WCOP** (windowed COP), *not* the
  disable bit. DISR (disable) is **bit 3** → write 0x08. Writing 0x10
  "to disable the watchdog" actually *enables* windowed COP: the chip then
  resets ~0.58 s into any long HV sequence, which looks like a random
  communications/clock failure. Reset default observed: 0x8F (CME=1,
  COP enabled, longest rate). We run with COPCTL=0x08 (DISR=1, CME=0).
  **[CAN]** either disable COP the same way or service it ($55,$AA to
  COPRST); verify DISR write rules in normal mode before relying on it.
- Clock monitor (CME=1 after reset on this unit): we clear it alongside
  DISR; a monitor trip during HV would limp-home the clock and change all
  timing. Untested whether it can actually trip during program/erase.

## 4. Register quick reference (A variant)

| addr | name | bits (7..0) | reset | notes |
|---|---|---|---|---|
| $00EE | EEDIVH/L | divider[9:0] | from shadow ($3FF here) | write-once (normal), 0 = ops disabled |
| $00F0 | EEMCR | NOBDML,NOSHW,-,-,EESWAI,PROTLCK,DMY | $FC | PROTLCK (bit2) gates EEPROT/BULKP writes |
| $00F1 | EEPROT | SHPROT,1,BPROT5..0 | **$FF = all protected** | clear to allow erase/program; EEMCR.PROTLCK must be 0 |
| $00F3 | EEPROG | BULKP,0,AUTO,BYTE,ROW,ERASE,EELAT,EEPGM | **$80 (BULKP=1!)** | see sequences below |
| $00F4 | FEELCK | LOCK | $00 | write-once in normal mode |
| $00F5 | FEEMCR | BOOTP | **$01 = boot-protected** | clear to erase/program boot block; blocked when FEELCK.LOCK=1 |
| $00F7 | FEECTL | -,FEESWAI,HVEN,-,ERAS,PGM | $00 | HVEN=8, ERAS=2, PGM=1 |

Per-module $F4-$F7 are selected by PPAGE (see §2). All modules came up
FEELCK=0 (unlocked) after BDM reset — **[CAN]** don't count on that in
normal mode; FEELCK.LOCK is write-once there, and the app may have set it.

## 5. Flash programming — proven sequence

**The datasheet's fast form (one row-select, 32 words under a single HVEN)
does not work on this silicon.** The first word written after HVEN=1 gets
its low byte programmed as 0x01 (the FEECTL PGM write value — cause
theorized: data-latch low byte retains state from the FEECTL write and the
first HV store doesn't reload it; not proven, but 100% reproducible across
select-data variants). Everything after word 0 programs fine, which makes
it extra sneaky.

What works, per **word** (the routine in `asm/flash_prog.s`):

```
FEECTL = PGM (0x01)                    ; byte write
word store to the row address          ; row select; data irrelevant but
                                       ;   we use the word's real data
wait t_NVS    = 100 us                 ; datasheet min 10 us
FEECTL = HVEN|PGM (0x09)               ; byte write
wait t_PGS    = 2000 us                ; datasheet min 5 us -- NOT enough,
                                       ;   charge pump needs ~ms to settle
                                       ;   (short wait => first-word corrupt)
word store (the data)                  ; programs the word
wait t_FPGM   = 33 us                  ; min 30, MAX 40 -- do not exceed
FEECTL = HVEN (0x08)
wait t_NVH    = 8 us                   ; min 5
FEECTL = 0
wait t_RCV    = 8 us                   ; min 1
```

Cost: ~2.2 ms/word → ~65 s of HV time for a full non-blank 128K image
(whole `restore` incl. host transfer ≈ 112 s over podex). Fully erased
($FF) rows/words are skipped — 54% of the reference image rows are FF.

Delay loop arithmetic: `subd #1 / bne` = 5 bus cycles per iteration
(SUBD #imm16 = 2, BNE taken = 3), ~7-10 cycles call overhead
(BSR 4 + RTS 5 + LDD 3 − 1). Host uses N = (f_bus·t − 7)/5.

More flash facts:
- programming is **aligned word only**; a byte store to the array does not
  latch program data.
- flash can only clear bits (1→0); erased = $FF. Re-programming a
  partially programmed word cannot restore bits.
- erase is **bulk per 32K module only** (no row/word erase).
- erase: FEECTL=ERAS(0x02) → word store to a valid array address → NVS
  100 µs → HVEN|ERAS(0x0A) → **t_ERAS 20 ms** (min 8) → HVEN(0x08) →
  **NVHL 150 µs** (min 100) → 0 → RCV. BOOTP must be 0 to include the
  8K boot block ($E000-$FFFF — that's where the vectors live).
- **erasing/programming several modules inside one HV session proved
  unreliable** (module 1 silently stayed programmed when 4 erases ran
  back-to-back). One module per command sequence, re-armed from scratch.
  Single-module erases: 100% reliable over many runs.
- BOOTP is per-module (register page!): clearing "BOOTP" with the wrong
  PPAGE selected silently protects the wrong module.

## 6. EEPROM — proven sequences

Chain to unlock (each session): EEMCR.PROTLCK=0 (`bclr $F0,#$04`),
EEPROT=$00, then work; afterwards restore EEPROT=$FF, PROTLCK=1,
EEPROG=$80 (BULKP back to 1).

Program (word-aligned, skip $FFFF words):
```
EEPROG = BULKP=0, AUTO=1, EELAT=1   ($22)
per word:  word store to the address
           EEPROG |= EEPGM          ($23)
           poll EEPGM until it self-clears (AUTO) -- ~tens of us/word,
              use ~25 ms timeout and count it as an error
EEPROG = $80 (EELAT=0, BULKP=1)
```
Measured: full 2K program+verify in ~0.1 s. With AUTO=0 you must wait
10 ms and clear EEPGM/EELAT manually (podex firmware does this; slow).

Erase:
- bulk: EEPROG=$26 (ERASE+EELAT), word store to $0800, EEPROG=$27, poll,
  EEPROG=$80. Erases all 2K *despite BULKP naming — BULKP must be 0*.
- row (32B): EEPROG=$2E (BYTE? no: ROW+ERASE+EELAT — $2E), word store to
  the row address, $2F, poll, $80. Verified erasing exactly one row.
- byte/word erase (BYTE=1): not exercised, same principle.

Remember EEDIV (§3) or nothing activates. Programming is by byte or
aligned word; we used words throughout.

## 7. Instruction/toolchain traps (CPU12 + m68hc11-as / any compiler)

- **MOVB/MOVW are read-modify-write** — they *read* the destination first.
  Using them for array/latch writes reads the flash while PGM latches are
  armed → the CPU derails completely (executes garbage, ends who-knows-
  where; over BDM every memory read then returns the same float value).
  Use plain stores (`std`/`staa`; in C `*addr = value;` — **check the
  generated code**, compilers can emit read-modify-write for byte RMW).
- Byte writes to FEECTL/FEEMCR ($F7/$F5) work; **word writes to $F4/$F6
  did not take effect** on this silicon. Keep register writes bytewise.
- `(n,X+)`/`(n,Y+)` post-increment forms did not increment reliably in our
  runs (`ldab 1,X+` worked, `ldd 2,X+` did not). We switched to explicit
  `inx/iny` everywhere — recommend the same in generated code, or verify
  the emitted opcodes (6C 31 = std 2,X+ etc.) actually behave.
- `ldd` clobbers B — a B loop counter around an LDD-based delay needs
  pshb/pulb. Classic, but it produced an *infinite* reprogram loop here
  (word re-programmed forever until watchdog/observer noticed).
- Watch the fall-through: a `bsr`/`pulb` path falling into the next label
  (`absent:` advancing a pointer) silently doubles pointer advancement —
  data lands rows ahead and FF gaps appear. Single-row unit tests cannot
  catch it; always verify a multi-row block.

## 8. Host-side / protocol gotchas (podex, but the *shape* recurs)

- Buffer overruns into **flash-decoded address space** ($4000+) are silent:
  writes are ignored (no bus fault), data is lost, and the routine then
  reads garbage. Our max stream (8 bitmap + 64×64 = 4104 B) overflowed a
  4K buffer by exactly 8 bytes → only the last row's tail corrupted, which
  pointed everywhere but the cause.
- Per-chunk base addresses must include the quarter offset *and* use the
  fixed decode for module 3 — every quarter programming rows 0-63 of its
  page was a real bug here (quarters overwrote each other).
- **Stale success flags mask crashes**: a "DONE" semaphore left set by the
  previous routine run makes the next run look successful even if the CPU
  died immediately. Clear DONE/ERR immediately before every GO.
- Verify with the *right* page selected (PPAGE) — verifying through a
  stale window produces confident nonsense.
- Over podex specifically: BDM hardware commands are served while user
  code runs, so BGND completion is detectable by polling STATUS ($FF01)
  bit 6 (BDMACT); ~0.68 ms/byte RTS/CTS handshake with the FTDI latency
  timer set to 1 ms (`echo 1 | sudo tee /sys/bus/usb-serial/devices/
  ttyUSB0/latency_timer`).
- After a target-side crash/reset the BDM timing desynchronizes (every
  read returns the same word, e.g. $E885): recover with a full target
  reset into BDM (podex cmd 0x01 / RESET+BKGD entry). RAM contents survive
  resets, so post-mortem dumps work after recovery.

## 9. Symptom → cause table (debug history, compressed)

| symptom | actual cause |
|---|---|
| routine "finishes" in ms, flash untouched | erase latch write ignored: module 3 needs fixed-address latch; or word store to $F6 (no effect) instead of byte to $F7 |
| first programmed word reads 0x??01 | datasheet fast form: first HV word's low byte keeps the FEECTL/PGM write value → use per-word full sequence |
| row N's data appears at row N+2, gaps elsewhere | fall-through advanced Y twice per present row |
| same word programmed forever / hangs at ~0.58 s | `ldd` clobbered the B loop counter; separately: COPCTL=0x10 enabled windowed COP (bit 4 is WCOP, DISR is bit 3) |
| CPU runs wild, memory reads all return $E885 | MOVB/MOVW read-modify-write touched the array with PGM armed; or a COP reset desynced BDM — reset target to recover |
| verify ok per-chunk, global md5 wrong | verify read through stale PPAGE / quarters all programming the same rows |
| 7-8 bad bytes at end of a dense 4K chunk | stream buffer overrun into $4000+ (writes silently ignored) |

## 10. What a CAN/Volcano flasher should reuse verbatim

1. The per-word program sequence and all timings from §5 (they are pure
   silicon behavior, independent of how code got into RAM).
2. Per-module erase with BOOTP handling and the module-3 fixed-decode
   latch (§2, §5), one module per HV session.
3. The EEPROM unlock chain + AUTO-mode sequences (§6) and the EEDIV
   write-once-in-normal-mode caveat (§3).
4. COP discipline (§3) — a CAN flash session is long; the 0.58 s windowed
   COP reset will kill it mid-image.
5. The code-generation rules from §7 (no RMW writes to array/registers,
   bytewise FEECTL access, explicit pointer increments).
6. Word-granular images, $FF skipping, verify-after-write per block —
   flash only clears bits, so a failed word must be caught before the
   block is considered done (re-erase is the only repair).

The reference image for validation is `tools/ref_bins/`
(flash md5 c23d85d31bafe6ca75e97fe196b2b9bd, EEPROM md5
4702072b19b5a9c0b0ba3bc192476e05) — `podex_programmer.py restore --yes`
reproduces both md5s from blank, repeatedly.
