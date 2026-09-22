;; mass-erase the selected DG128A flash modules (32K each, internal charge pump).
;;   param $2180 MASK u16  bit m = erase module m (module m = PPAGE pages 2m,2m+1)
;;   param $2182 NVS  u16  dlyd count (~15us)
;;   param $2184 ERAS u16  dlyd count (~20ms)
;;   param $2186 NVHL u16  dlyd count (~150us)
;;   param $2188 RCV  u16  dlyd count (~8us)
;; FEECTL: bit3 HVEN, bit1 ERAS, bit0 PGM.  FEEMCR bit0 BOOTP (reset=1).
;; Register page of module m visible at $F4-$F7 while PPAGE = 2m or 2m+1.
;; Modules 0-2 accept the erase-latch write through the $8000 page window;
;; module 3 (fixed at $4000-$7FFF / $C000-$FFFF) latches only via its fixed
;; decode, so its latch write goes to $C000.
	.include "common.inc"
	.text
P_MASK	= P_BASE
P_NVS	= P_BASE+2
P_ERAS	= P_BASE+4
P_NVHL	= P_BASE+6
P_RCV	= P_BASE+8
start:
	lds	#0x2E00
	clr	ERR
	ldab	P_MASK+1
	lsrb
	bcc	m1
	clrb			; module 0
	ldx	#0x8000
	bsr	erase_one
m1:	lsrb
	bcc	m2
	ldab	#2		; module 1
	ldx	#0x8000
	bsr	erase_one
m2:	lsrb
	bcc	m3
	ldab	#4		; module 2
	ldx	#0x8000
	bsr	erase_one
m3:	lsrb
	bcc	mdone
	ldab	#6		; module 3
	ldx	#0xC000
	bsr	erase_one
mdone:
	clr	0xFF		; PPAGE = 0 (module 0 regs)
	movb	#0x01, 0xF5	; restore BOOTP = 1
	clr	0xF7		; FEECTL = 0
	movb	#0xA5, DONE
	bgnd

erase_one:			; B = 2*module (selects reg page), X = latch address
	stab	0xFF
	movb	#0x00, 0xF5	; BOOTP = 0
	movb	#0x02, 0xF7	; ERAS = 1
	movw	#0xFFFF, 0,X	; latch: word write to a valid array address
	ldd	P_NVS
	bsr	dlyd
	movb	#0x0A, 0xF7	; HVEN = 1
	ldd	P_ERAS
	bsr	dlyd
	movb	#0x08, 0xF7	; ERAS = 0
	ldd	P_NVHL
	bsr	dlyd
	clr	0xF7		; HVEN = 0
	ldd	P_RCV
	bsr	dlyd
	rts

dlyd:	subd	#1		; 5 cycles per iteration (SUBD #imm16 = 2 + BNE = 3)
	bne	dlyd
	rts
