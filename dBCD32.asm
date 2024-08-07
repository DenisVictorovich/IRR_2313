; {источник: http://forum.easyelectronics.ru/viewtopic.php?f=4&t=14897&start=0 / Леонид Иванович (! поблагодарить !)}

.macro   ldy
   ldi   YL, byte1(@0)
   ldi   YH, byte2(@0)
.endm

.macro   ldz
   ldi   ZL, byte1(@0)
   ldi   ZH, byte2(@0)
.endm

; вход - {dbb_3:dbb_2:dbb_1:dbb_0}
; выход (BCD) - dbb_dig[10]

dis_BCD_32:
   ldy    dbb_dig
   clr    temp
   ldi    dbb_cnt, 10
dbb_clrout:
   st     Y+, temp ; output array clear
   dec    dbb_cnt
   brne   dbb_clrout

   ldi    dbb_cnt, 32 ; input bits count
   ldz    dbb_dig
dbb_hloop:
   lsl    dbb_0 ; input array shift left
   rol    dbb_1
   rol    dbb_2
   rol    dbb_3
   ldy    dbb_dig+10
dbb_sloop:
   ld     temp, -Y
   rol    temp
   subi   temp, -6 ; temp+6, C=1
   sbrs   temp,  4
   subi   temp,  6 ; temp-6, C=0
   andi   temp, 15
   st     Y, temp
   cpse   YL, ZL ; ZH:ZL = dbb_dig
   rjmp   dbb_sloop
   cpse   YH, ZH
   rjmp   dbb_sloop
   dec    dbb_cnt ; YH:YL = dbb_dig
   brne   dbb_hloop
   ret

