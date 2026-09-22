;; calib: burn (OUTER x (65536*4+6)+17) cycles, then bgnd.
;; host measures wall time -> f_bus.  param: $2180 = OUTER (u16)
	.include "common.inc"
	.text
P_OUTER	= P_BASE
start:
	lds	#0x2E00
	clr	ERR
	ldy	P_OUTER
outer:	ldx	#0
inner:	dex
	bne	inner
	dey
	bne	outer
	movb	#0xA5, DONE
	bgnd
