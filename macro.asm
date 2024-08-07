  ; macro.inc

  ; --------------------------------------------------------------------------------------------------------------------------------

  .MACRO out_i ; Начало макроопределения.
            ldi  temp, @1
            out  @0,  temp
  .ENDMACRO ; Конец макроопределения.

  .MACRO ldi_Z ; Начало макроопределения.
            ldi  ZH, high(@0<<1)
            ldi  ZL, low (@0<<1)
  .ENDMACRO ; Конец макроопределения.

  .MACRO ldi_Y ; Начало макроопределения.
            ldi  YH, high(@0)
            ldi  YL, low (@0)
  .ENDMACRO ; Конец макроопределения.

  .MACRO ldi_X ; Начало макроопределения.
            ldi  XH, high(@0)
            ldi  XL, low (@0)
  .ENDMACRO ; Конец макроопределения.

  ; --------------------------------------------------------------------------------------------------------------------------------

