;;
;; Common RAM layout for podex programming routines (loaded/GO'd via BDM)
;;
;; code     $2000-$217F  (routine binary, loaded by host)
;; params   $2180-$21BF  (written by host before GO)
;; DONE     $21C0        (set to $A5 just before BGND)
;; ERR      $21C1        (0 = ok)
;; temps    $21C2-$21DF
;; bitmap   $21E0-$21FF  (flash program: 1 byte = 8 rows of 64B)
;; buffer   $2200-$320F  (data stream area; max 8+64*64 = 4104 bytes)
;; stack    flash_prog: $3FF0 down (clear of the buffer); others: $2E00 down
;;
DONE    = 0x21C0
ERR     = 0x21C1
P_BASE  = 0x2180
