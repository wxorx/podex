;; program the 2K EEPROM by aligned words, skipping $FFFF words.
;;   param $2180 CNT  u16  word count
;;   param $2182 ADDR u16  destination (word aligned, in EEPROM window)
;;   param $2184 BUF  u16  source buffer (CNT*2 bytes)
	.include "common.inc"
	.text
P_CNT	= P_BASE
P_ADDR	= P_BASE+2
P_BUF	= P_BASE+4
start:
	lds	#0x2E00
	clr	ERR
	bclr	0xF0,#0x04
	clr	0xF1
	movb	#0x22, 0xF3	; BULKP=0 AUTO=1 EELAT=1
	ldx	P_BUF
	ldy	P_ADDR
	ldd	P_CNT
pw:	pshd
	ldd	2,X+
	std	2,Y+
	cpd	#0xFFFF		; erased: nothing to program
	beq	sk1
	movb	#0x23, 0xF3	; EEPGM = 1
	bsr	eepoll
sk1:	puld
	subd	#1
	bne	pw
	movb	#0x80, 0xF3	; EELAT = 0, restore BULKP default
	movb	#0xFF, 0xF1
	bset	0xF0,#0x04
	movb	#0xA5, DONE
	bgnd

eepoll:	ldd	#10000		; ~25ms timeout at 4MHz (t_prog ~10ms)
ep1:	brclr	0xF3,#0x01,epok
	subd	#1
	bne	ep1
	inc	ERR
epok:	rts
