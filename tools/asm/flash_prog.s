;; program flash, one full command sequence per word.
;;
;; Each word gets its own complete PGM/HVEN cycle, the exact shape that was
;; verified byte-exact on this silicon (see "silicon notes" below):
;;   PGM=1 -> word write (row select, carrying the word's own data) -> NVS
;;   -> HVEN=1 -> PGS -> same word write again -> FPGM -> PGM=0 -> NVH
;;   -> HVEN=0 -> RCV
;; The datasheet's faster form (select once per row, then 32 words under a
;; single HVEN window) mis-programs the low byte of the FIRST word after
;; HVEN with 0x01 on this MCU: the data latch evidently keeps state from
;; the FEECTL PGM write and the first HV write does not reliably reload the
;; low lane.  Repeating the whole sequence per word sidesteps the quirk at
;; ~2.2ms/word (~65s for a full non-erased 128K image).
;;
;;   param $2180 NVS  u16  dlyd count (~100us)
;;   param $2182 PGS  u16  dlyd count (~2ms charge-pump settle)
;;   param $2184 FPGM u16  dlyd count (~33us; must stay <40us incl. overhead)
;;   param $2186 NVH  u16  dlyd count (~8us)
;;   param $2188 RCV  u16  dlyd count (~8us)
;;   param $218A PPAGE byte  register page (2*module for $8000-window access)
;;   param $218B pad
;;   param $218C ROWS u16  number of 64-byte rows (multiple of 8)
;;   param $218E BUF  u16  packed row data buffer (only bitmap-set rows present)
;;   param $2190 BASE u16  array base for data writes:
;;                         $8000 for modules 0-2 (page window); module 3 has a
;;                         read-only window, use its fixed $4000 or $C000 half.
;; The data stream at BUF is consumed sequentially, one 8-row group at a
;; time: [bitmap byte (bit0 = first row of the group, set = row present)]
;; followed by the packed 64-byte content of each present row.
;;
;; Silicon notes (MC912DG128A):
;;  * FEECTL ($F7) / FEEMCR ($F5) byte stores work (STAA/CLR/MOVB);
;;    word stores to $F4/$F6 do not take effect.
;;  * array writes must be aligned WORD stores (STD).
;;  * no auto-inc addressing (n,X+): it does not post-increment reliably
;;    with this toolchain/CPU combination -- explicit INX/INY.
;;  * LDD clobbers B, so loop counters in B are pushed around LDD uses.
	.include "common.inc"
	.text
P_NVS	= P_BASE
P_PGS	= P_BASE+2
P_FPGM	= P_BASE+4
P_NVH	= P_BASE+6
P_RCV	= P_BASE+8
P_PPAGE	= P_BASE+10
P_ROWS	= P_BASE+12
P_BUF	= P_BASE+14
P_BASEA	= P_BASE+16
ROWS	= 0x21C2
start:
	lds	#0x3FF0		; above the data buffer (see common.inc)
	clr	ERR
	clr	0xF7		; FEECTL = 0
	ldab	P_PPAGE
	stab	0xFF		; select module register page (FEEMCR/FEECTL)
	movb	#0x00, 0xF5	; BOOTP = 0
	ldx	P_BUF
	ldy	P_BASEA
	ldd	P_ROWS
	std	ROWS
group:	ldab	1,X+		; bitmap byte, X -> packed data of this group
	ldaa	#8
bitl:	lsrb			; C = row present?
	bcc	absent
	pshb
	psha
	bsr	prog_row	; advances Y by 64
	pula
	pulb
	bra	bitl_next	; (do NOT fall into absent: Y would double-advance)
absent:	leay	64,Y
bitl_next:
	deca
	bne	bitl
	ldd	ROWS
	subd	#8
	std	ROWS
	bne	group
	clr	0xFF		; PPAGE = 0
	movb	#0x01, 0xF5	; restore BOOTP = 1
	clr	0xF7
	movb	#0xA5, DONE
	bgnd

;; program one 64-byte row = 32 full per-word sequences
prog_row:
	ldab	#32
wloop:	pshb
	bsr	prog_word
	pulb
	decb
	bne	wloop
	rts

;; program the word at 0,Y from 0,X, then advance both pointers by 2
prog_word:
	ldaa	#0x01
	staa	0xF7		; PGM = 1
	ldd	0,X
	std	0,Y		; row select with the word's own data
	ldd	P_NVS
	bsr	dlyd
	ldaa	#0x09
	staa	0xF7		; HVEN = 1
	ldd	P_PGS
	bsr	dlyd
	ldd	0,X
	std	0,Y		; program the word
	ldd	P_FPGM
	bsr	dlyd
	ldaa	#0x08
	staa	0xF7		; PGM = 0
	ldd	P_NVH
	bsr	dlyd
	clr	0xF7		; HVEN = 0
	ldd	P_RCV
	bsr	dlyd
	inx
	inx
	iny
	iny
	rts

dlyd:	subd	#1		; 5 cycles per iteration (SUBD #imm16 = 2 + BNE = 3)
	bne	dlyd
	rts
