; mpy16s.inc

; ----------------------------------------------------------------------------------------------------------------------------------
; "mpy16s" - 16x16 bit signed multiplication
;
; This subroutine multiplies signed the two 16-bit register variables
; mp16sH:mp16sL and mc16sH:mc16sL.
; The result is placed in m16s3:m16s2:m16s1:m16s0.
; The routine is an implementation of Booth's algorithm. If all 32 bits
; in the result are needed, avoid calling the routine with
; -32768 (0x8000) as multiplicand
;
; Number of 2 x bytes  : 16 + return
; Number of cycles     : 210/226 (min/max) + return
; Low registers used   : None
; High registers used  : 7 (mp16sL, mp16sH, mc16sL/m16s0, mc16sH/m16s1, m16s2, m16s3, mcnt16s)
; ----------------------------------------------------------------------------------------------------------------------------------

; subroutine register variables

.def  mc16sL  = r16    ; multiplicand low byte
.def  mc16sH  = r17    ; multiplicand high byte
.def  mp16sL  = r18    ; multiplier low byte
.def  mp16sH  = r19    ; multiplier high byte
.def  m16s0   = r18    ; result byte 0 (LSB)
.def  m16s1   = r19    ; result byte 1
.def  m16s2   = r20    ; result byte 2
.def  m16s3   = r21    ; result byte 3 (MSB)
.def  mcnt16s = r22    ; loop counter

; code
mpy16s: clr   m16s3          ; clear result byte 3
        sub   m16s2, m16s2   ; clear result byte 2 and carry
        ldi   mcnt16s, 16    ; init loop counter
m16s_1: brcc  m16s_2         ; if carry (previous bit) set
        add   m16s2, mc16sL  ; add multiplicand Low to result byte 2
        adc   m16s3, mc16sH  ; add multiplicand High to result byte 3
m16s_2: sbrc  mp16sL, 0      ; if current bit set
        sub   m16s2, mc16sL  ; sub multiplicand Low from result byte 2
        sbrc  mp16sL, 0      ; if current bit set
        sbc   m16s3, mc16sH  ; sub multiplicand High from result byte 3
        asr   m16s3          ; shift right result and multiplier
        ror   m16s2
        ror   m16s1
        ror   m16s0
        dec   mcnt16s ; decrement counter
        brne  m16s_1  ; if not done, loop more
        ret

; ----------------------------------------------------------------------------------------------------------------------------------

