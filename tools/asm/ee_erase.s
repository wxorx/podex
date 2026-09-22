;; erase the 2K EEPROM ($0800-$0FFF).
;;   param $2180 MODE byte   0 = bulk erase whole array, 1 = erase ROWS x 32B rows
;;   param $2182 ROWS u16    (row mode only)
;;   param $2184 ADDR u16    (row mode only, 32-byte aligned)
;; EEPROG $F3: bit7 BULKP, bit5 AUTO, bit4 BYTE, bit3 ROW, bit2 ERASE,
;;             bit1 EELAT, bit0 EEPGM.  EEPROT $F1, EEMCR $F0 (bit2 PROTLCK).
	.include "common.inc"
	.text
P_MODE	= P_BASE
P_ROWS	= P_BASE+2
P_ADDR	= P_BASE+4
RCNT	= 0x21C2
start:
	lds	#0x2E00
	clr	ERR
	bclr	0xF0,#0x04	; PROTLCK = 0 (special mode)
	clr	0xF1		; EEPROT = 0: unprotect everything
	ldab	P_MODE
	beq	bulk
	ldx	P_ADDR
	ldd	P_ROWS
	std	RCNT
erowl:	movb	#0x2E, 0xF3	; BULKP=0 AUTO=1 ROW=1 ERASE=1 EELAT=1
	movw	#0xFFFF, 0,X	; latch row address
	movb	#0x2F, 0xF3	; EEPGM = 1 (AUTO clears it when done)
	bsr	eepoll
	leax	32,X
	ldd	RCNT
	subd	#1
	std	RCNT
	bne	erowl
	bra	fin
bulk:	movb	#0x26, 0xF3	; BULKP=0 AUTO=1 ERASE=1 EELAT=1
	movw	#0xFFFF, 0x0800
	movb	#0x27, 0xF3
	bsr	eepoll
fin:	movb	#0x80, 0xF3	; EELAT = 0, restore BULKP default
	movb	#0xFF, 0xF1	; restore protection defaults
	bset	0xF0,#0x04	; PROTLCK = 1
	movb	#0xA5, DONE
	bgnd

eepoll:	ldd	#10000		; ~25ms timeout at 4MHz (t_erase ~10ms)
ep1:	brclr	0xF3,#0x01,epok
	subd	#1
	bne	ep1
	inc	ERR
epok:	rts
