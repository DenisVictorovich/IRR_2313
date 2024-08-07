
  ; --------------------------------------------------------------------------------------------------------------------------------
  ; <<система орошения комнатных растений>>
  ; модуль управления на базе "крохи-2313":                                              ________________________________
  ;  _____   ----   ----                                                                /                                |__
  ;  RESET @|1   \_/  20|= питание (2,8 .. 3,2) вольт {LCD.1} <------------------------/                                 ___НАГРУЗКА
  ; Rx PD0 =|2        19|@ PB7 `SCK`<--_-_-_-_                                                                          /
  ; Tx PD1 =|3  tiny  18|@ PB6 `DO`(`MISO`)-->                                                                       | /К
  ; К.Р.-- =|4  2313  17|@ PB5 `DI`(`MOSI`)<--                                                                _____ Б|/ <<NPN>>
  ; К.Р.-- =|5        16|= PB4 <-----------------------------------------------------------------------------[2 кОм]-|  <<BC817>>
  ; I0 PD2 =|6        15|= PB3           |   {LCD.5}---------|   |   I0 - <расходомер для воды> - п р м ч а   ^^^^^  |\ marking`6D*`
  ; I1 PD3 =|7        14|= PB2 {LCD.3}   |   {LCD.6}---------|   |   I1 - <поверочные импульсы> -  е е ы к           | \Э
  ; T0 PD4 =|8        13|= PB1 {LCD.4}   |             ||    |   |                                                      \->--- ЗЕМЛЯ
  ; T1 PD5 =|9        12|= PB0 {LCD.8}   |   {LCD.7}---||----|   |
  ;  ЗЕМЛЯ @|10       11|= PD6 -ПЕРЕМЫЧКА-             ||  ЗЕМЛЯ |
  ;          -----------                              1 uФ       |
  ; @ - выводы для подключения программатора
  ; --------------------------------------------------------------------------------------------------------------------------------

  ;                   сутки | часы | минуты | секунды | Гц
  .equ    off_delay = 4     * 24   * 60     * 60      * 50
  .equ    on_delay  =                         20      * 50

  ; градуировка расходомера (мл на импульс, 30 - не отградуирован)
  .equ    volume_per_impulse = 30

  ; --------------------------------------------------------------------------------------------------------------------------------

  .equ    DDRA  = 0x1A
  .equ    PORTA = 0x1B
  .equ    DDRB  = 0x17
  .equ    PORTB = 0x18
  .equ    DDRD  = 0x11
  .equ    PORTD = 0x12
  .equ    PIND  = 0x10
  .equ    PINB  = 0x16

  .equ    TIMSK = 0x39
  .equ    TCNT0 = 0x32
  .equ    TCCR0 = 0x33
  .equ    SREG  = 0x3f

  .equ    OCR1AL = 0x2A
  .equ    OCR1AH = 0x2B
  .equ    TCNT1L = 0x2C
  .equ    TCNT1H = 0x2D
  .equ    TCCR1B = 0x2E

  .equ    MCUCR = 0x35
  .equ    GIMSK = 0x3b

  .equ    USIDR   = 0x0F
  .equ    USISR   = 0x0E
  .equ    USICR   = 0x0D

  .equ    WDTCR   = 0x21 ; "таймер сторожевой собаки"

  ; USISR
  .equ USISIF     = 7
  .equ USIOIF     = 6
  .equ USIPF      = 5
  .equ USIDC      = 4
  .equ USICNT3    = 3
  .equ USICNT2    = 2
  .equ USICNT1    = 1
  .equ USICNT0    = 0

  ; USICR
  .equ USISIE     = 7
  .equ USIOIE     = 6
  .equ USIWM1     = 5
  .equ USIWM0     = 4
  .equ USICS1     = 3
  .equ USICS0     = 2
  .equ USICLK     = 1
  .equ USITC      = 0

  .equ    ACSR = 0x08

  ; registers:

  .def  time_delay       = r15
  .equ  time_delay_value = 25 ; x 20 ms

  .def  t_0     = r16 ; temporary variable 0
  .def  t_1     = r17 ; temporary variable 1
  .def  t_2     = r18 ; temporary variable 2
  .def  t_3     = r19 ; temporary variable 3

  .def  SREG_I  = r20 ; ячейка для хранения SREG (прерывания)
  .def  delay_c = r21 ; счётчик для формирования искусственных задержек

  ; 32-хразрядный счётчик расхода воды:
  .def    c_lo_lo = r24
  .def    c_lo_hi = r25
  .def    c_hi_lo = r22
  .def    c_hi_hi = r23

  ; time_3:time_2:time_1:time_0 - счётчик времени:
  .def  time_0  = r4
  .def  time_1  = r5
  .def  time_2  = r6
  .def  time_3  = r7

  .def    XL    = r26 ; X pointer low
  .def    XH    = r27 ; X pointer high
  .def    YL    = r28 ; Y pointer low
  .def    YH    = r29 ; Y pointer high
  .def    ZL    = r30 ; Z pointer low
  .def    ZH    = r31 ; Z pointer high

  .equ    SPL      = 0x3d
  .equ    RAMBEGIN = 0x0060
  .equ    RAMEND   = 0x00df

  ; --------------------------------------------------------------------------------------------------------------------------------

  .MACRO out_i ; Начало макроопределения.
            ldi  t_0, @1
            out  @0,  t_0
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

  .MACRO ind_i2c_wr ; Начало макроопределения.
            ldi    t_2, @0
            rcall  ind_i2c_write
  .ENDMACRO ; Конец макроопределения.

  .macro   inc_value_8
            inc   @0
            brne  inc_value_32_end_lbl
  .endm

  .macro   store_i_inc
            ldi     t_0, @0
            st      Y+, t_0
  .endm

  ; --------------------------------------------------------------------------------------------------------------------------------

  .equ   ind_DDR    = DDRB
  .equ   ind_PORT   = PORTB
  .equ   ind_PIN    = PINB

  .equ   ind_SCL    = 2 ; PB2
  .equ   ind_SDA    = 1 ; PB1
  .equ   ind_RESET  = 0 ; PB0

  .equ   digit_array    = (RAMBEGIN+0x10) ; SRAM byte-address
  .equ   digit_capacity = 5 ; количество знакомест для представления десятичных чисел

  .equ   sensor_DDR  = DDRD
  .equ   sensor_PORT = PORTD
  .equ   sensor_PIN  = PIND

  .include "hellenic.inc"

  ; --------------------------------------------------------------------------------------------------------------------------------
  ; из загрузчика `tiny_2313_bootloader`:

  .equ   check_receive = 0x0000a1 ; (void), return: if SREG.T=0, then received, result in {`temp`=r16}
  .equ   send          = 0x0000a8 ; (char b{ `par_l`=r17 })

  ; --------------------------------------------------------------------------------------------------------------------------------

  .ORG 0x100

  ; interrupts links

  ; 00   rjmp   reset        ; reset handler
  ; 01   rjmp   int0         ; external interrupt0 handler
  ; 02   rjmp   int1         ; external interrupt1 handler
  ; 03   rjmp   tim1_capt    ; timer1 capture handler
  ; 04   rjmp   tim1_compa   ; timer1 comparea handler
  ; 05   rjmp   tim1_ovf     ; timer1 overflow handler
  ; 06   rjmp   tim0_ovf     ; timer0 overflow handler
  ; 07   rjmp   usart0_rxc   ; usart0 rx complete handler
  ; 08   rjmp   usart0_dre   ; usart0,udr empty handler
  ; 09   rjmp   usart0_txc   ; usart0 tx complete handler
  ; 0a   rjmp   ana_comp     ; analog comparator handler
  ; 0b   rjmp   pcint        ; pin change interrupt
  ; 0c   rjmp   timer1_compb ; timer1 compare b handler
  ; 0d   rjmp   timer0_compa ; timer0 compare a handler
  ; 0e   rjmp   timer0_compb ; timer0 compare b handler
  ; 0f   rjmp   usi_start    ; usi start handler
  ; 10   rjmp   usi_overflow ; usi overflow handler
  ; 11   rjmp   ee_ready     ; eeprom ready handler
  ; 12   rjmp   wdt_overflow ; wdt overflow handler

  RESET:      rjmp    BEGIN          ; 00   reset handler
              rjmp    EXT_INT_0      ; 01   external interrupt0 handler
              reti                   ; 02   external interrupt1 handler
              reti                   ; 03   timer1 capture handler
              rjmp    TIMER1_INT     ; 04   timer1 compare a handler
              reti                   ; 05   timer1 overflow handler
              reti                   ; 06   timer0 overflow handler
              reti                   ; 07   usart0 rx complete handler
              reti                   ; 08   usart0,udr empty handler
              reti                   ; 09   usart0 tx complete handler
              reti                   ; 0a   analog comparator handler
              reti                   ; 0b   pin change interrupt
              reti                   ; 0c   timer1 compare b handler
              reti                   ; 0d   timer0 compare a handler
              reti                   ; 0e   timer0 compare b handler
              reti                   ; 0f   usi start handler
              reti                   ; 10   usi overflow handler
              reti                   ; 11   eeprom ready handler
              rjmp    RESET          ; 12   wdt overflow handler

  ; --------------------------------------------------------------------------------------------------------------------------------

  .include "font_5x8.inc"

  ; --------------------------------------------------------------------------------------------------------------------------------

  main_line: .db bg_CHI, bg_RHO, bg_IOTA, bg_SIGMA, bg_TAU, bg_OMICRON, bg_SIGMA, text_SPACE, bg_ALPHA, bg_NU, bg_EPSILON, bg_SIGMA, bg_TAU, bg_ETA, 255
  line01: .db bg_EPSILON, sg_MU, sg_BETA, sg_RHO, sg_UPSILON, sg_OMICRON, sg_LAMBDA, sg_OMICRON, sg_GAMMA, sg_IOTA, sg_KAPPA, sg_EPSILON, sg_SIGMA_F, 255
  line02: .db bg_SIGMA, sg_UPSILON, sg_MU, sg_BETA, sg_IOTA, sg_OMEGA, sg_TAU, sg_IOTA, sg_KAPPA, sg_ETA, 255
  line03: .db bg_NU, sg_EPSILON, sg_UPSILON, sg_RHO, sg_OMICRON, sg_DELTA, sg_IOTA, sg_CHI, sg_OMICRON, sg_TAU, sg_OMICRON, sg_MU, sg_IOTA, sg_ALPHA, 255

  .include "RIBO.INC"

  ; --------------------------------------------------------------------------------------------------------------------------------

  caption_line: .db bg_SIGMA, sg_UPSILON, sg_SIGMA, sg_TAU, sg_ETA, sg_MU, sg_ALPHA, text_SPACE, sg_ALPHA, sg_RHO, sg_DELTA, sg_EPSILON, sg_UPSILON, sg_SIGMA, sg_ETA, 255

  ; .include "mpy16s.inc"
  ; .include "div16s.inc"

  ; --------------------------------------------------------------------------------------------------------------------------------

  ; begin
  BEGIN:
         cli
            out_i  SPL, low(RAMEND) ; set stack pointer to top of ram [20.02.2014]
            out_i  ACSR, 128 ; analog comparator disable

            out_i  WDTCR, 24
            out_i  WDTCR, 15
            wdr

            out_i  TIMSK,  (1<<6)
            out_i  TCCR1B, (1<<3)|3 ; CTC | CK/64
            out_i  OCR1AH, high(2500-1)
            out_i  OCR1AL, low (2500-1) ; {8000000 Гц / 64 = 125000 Гц} / 2500 = 50 Гц ~ 20 мс

            clr     t_0
            out     TCNT1H, t_0
            out     TCNT1L, t_0

            ; c := 0000|0000h
            clr     c_lo_lo
            clr     c_lo_hi
            clr     c_hi_lo
            clr     c_hi_hi

            ; time := 0000|0000h
            clr     time_0
            clr     time_1
            clr     time_2
            clr     time_3

            ldi     t_0, time_delay_value
            mov     time_delay, t_0

            ; MCUCR: bit 7   6  5   4   3     2     1     0
            ;           PUD SM1 SE SMD ISC11 ISC10 ISC01 ISC00
            ; Table 32. Interrupt 0 sense control
            ; ISC01 ISC00 Description
            ;   0     0   The low level of INT0 generates an interrupt request.
            ;   0     1   Any logical change on INT0 generates an interrupt request.
            ;   1     0   The falling edge of INT0 generates an interrupt request.
            ;   1     1   The rising edge of INT0 generates an interrupt request.
            out_i   MCUCR, 0b01 ; Any logical change on INT0 generates an interrupt request.
            out_i   GIMSK, 1<<6 ; INT0
            out_i   sensor_PORT, (1<<3)|(0<<2)
            out_i   sensor_DDR,  (1<<3)|(0<<2)

            rcall   ind_GPM388CO_init
            SBI     PORTD, 6
            sbi     DDRB,  4
            ; <...>

         sei

            ; ПРИВЕТСТВИЕ

            ldi_Z   main_line
            ldi     t_3, 1 ; номер строки
            rcall   print_text

            ldi_Z   line01
            ldi     t_3, 3 ; номер строки
            rcall   print_text

            ldi_Z   line02
            ldi     t_3, 4 ; номер строки
            rcall   print_text

            ldi_Z   line03
            ldi     t_3, 5 ; номер строки
            rcall   print_text

            ; ЧЕРЕЗ НЕКОТОРОЕ ВРЕМЯ ПРИВЕТСТВИЕ ЗАМЕНЯЕМ РАБОЧИМ РЕЖИМОМ
            ldi     t_0, 50 ; x 20 мс = 1 с
            mov     time_delay, t_0
  title_delay_loop: ; <-------------------
            tst     time_delay       ;    |
            brne    title_delay_loop ; ---
            rcall   ind_clear_lcd

            ldi_Z   caption_line
            ldi     t_3, 0 ; номер строки
            rcall   print_text
  ; loop
  LOOP:
            rcall   check_receive ; 0x27d + (0xe2a|(~0xfff)) + 1 = 0x0000a8
            brts    check_not_received ; ------------------------------
            cpi     r16, 64            ;                               |
            brlo    codon_table_lbl    ; (r16 < 64)---                 |
            mov     r17, r16           ;              |                |
            ori     r17, 0b00100000    ; из прописных букв - строчные. |
            rjmp    serial_port_send_lbl ;            |                |
       codon_table_lbl: ; (r16) ; <-------|-----------                 |
          ; clr     r17                ;  |                            |
            ldi_Z   RIBO_codon_table   ;  |                            |
            add     ZL, r16            ;  |                            |
          ; adc     ZH, r17            ;  |                            |
            lpm     r17, Z             ;  |                            |
       serial_port_send_lbl: ; <----------                             |
            rcall   send               ;                               |
       ; окончание блока обработки принятых байтов ч/з асинхр.канал    |
       check_not_received: ; <-----------------------------------------
            ; обновление картинки раз в {time_delay_value x 20} мс
            tst     time_delay
            brne    LOOP
            ldi     t_0, time_delay_value
            mov     time_delay, t_0
            ;
            rcall   control_water_pump
            rcall   draw_counter_value
            rcall   sensor_PORT_tst_routine
            rcall   draw_time_counter_value
            ; <...>
            rjmp    LOOP

  ; ------------------------------------------------------------------------------------------------ timer interrupt {50 Гц ~ 20 мс}
  TIMER1_INT:
            in      SREG_I, SREG

            ; ----------------------------------------------------------------------------------------------------------------------

            wdr

            rcall   inc_value_32 ; time counter

            tst     time_delay ; time_delay == 0 ?
            breq    time_delay_not_eq_0
            dec     time_delay
       time_delay_not_eq_0:

            ; ----------------------------------------------------------------------------------------------------------------------

       TIMER1_END_LABEL:
            out     SREG, SREG_I
            reti

  ; ------------------------------------------------------------------------------------------- external interrupt for water counter
  EXT_INT_0:
            in      SREG_I, SREG

            ; ----------------------------------------------------------------------------------------------------------------------

            rcall   inc_c_routine

            ; ----------------------------------------------------------------------------------------------------------------------

       EXT_INT_0_END_LABEL:
            out     SREG, SREG_I
            reti

  ; --------------------------------------------------------------------------------------------------------------------------------

  .include "LCD.inc"

  ; --------------------------------------------------------------------------------------------------------------------------------

  ; print_value: ; print_value(XH:XL)
  ;    cli
  ;       push   XL
  ;       push   XH
  ;       push   YL
  ;       push   YH
  ;          bst    XH, 7 ; знак числа скопировать в SREG.T
  ;          ldi_Y  digit_array+digit_capacity
  ;       print_value_loop_lbl: ; <--------------------------
  ;          cpi    XH, 0                                  ; |
  ;          brne   print_value_symbol_lbl                 ; |
  ;          cpi    XL, 0                                  ; |
  ;          brne   print_value_symbol_lbl                 ; |
  ;          cpi    YL, low(digit_array+digit_capacity)    ; |
  ;          breq   print_value_symbol_lbl ; '0'           ; |
  ;          brtc   print_value_sign_passed ; знак учтён   ; |
  ;          ldi    t_0, 37 ; `минус`                      ; |
  ;          clt    ; SREG.T := 0                          ; |
  ;          rjmp   print_value_space_sign_lbl ; --------  ; |
  ;       print_value_sign_passed:                     ; | ; |
  ;          ldi    t_0, 36 ; `пробел`                 ; | ; |
  ;          rjmp   print_value_space_sign_lbl ; --------| ; |
  ;       print_value_symbol_lbl:                      ; | ; |
  ;          mov    dd16sH, XH                         ; | ; |
  ;          mov    dd16sL, XL                         ; | ; |
  ;          ldi    dv16sH, high(10)                   ; | ; |
  ;          ldi    dv16sL, low (10)                   ; | ; |
  ;          rcall  div16s                             ; | ; |
  ;          mov    XH, dres16sH                       ; | ; |
  ;          mov    XL, dres16sL                       ; | ; |
  ;          mov    t_0, drem16sL ; drem16sH == 0      ; | ; |
  ;          ; rcall  dig_to_SYM                       ; | ; |
  ;       print_value_space_sign_lbl: ; <----------------  ; |
  ;          st     -Y, t_0                                ; |
  ;          cpi    YL, low(digit_array) ; < 255 знакомест ; |
  ;          brne   print_value_loop_lbl ; ------------------
  ;       pop    YH
  ;       pop    YL
  ;       pop    XH
  ;       pop    XL
  ;    sei
  ;    ret

  ; --------------------------------------------------------------------------------------------------------------------------------

  draw_byte_t_1:
            ; hi {F0}
            mov     t_0, t_1
            swap    t_0
            andi    t_0, 15
            st      Y+, t_0
            ; lo {0F}
            mov     t_0, t_1
            andi    t_0, 15
            st      Y+, t_0
            ret

  ; --------------------------------------------------------------------------------------------------------------------------------

  sample_text:      .db sg_ALPHA, sg_BETA, sg_GAMMA, 255
  expense_lbl:      .db bg_UPSILON, sg_DELTA, sg_OMEGA, sg_RHO, text_SPACE, sg_DELTA, sg_ALPHA, sg_PI, sg_ALPHA, sg_NU, sg_OMEGA, sg_NU, text_COLON, 255
  time_lbl:         .db bg_CHI, sg_RHO, sg_OMICRON, sg_NU, sg_OMICRON, sg_SIGMA_F, text_COLON, 255
  water_pump_0_lbl: .db bg_ALPHA, sg_NU, sg_TAU, sg_LAMBDA, sg_IOTA, sg_ALPHA, text_SPACE, 0, 255 ; [26.10.2015]
  water_pump_1_lbl: .db bg_ALPHA, sg_NU, sg_TAU, sg_LAMBDA, sg_IOTA, sg_ALPHA, text_SPACE, 1, 255 ; [26.10.2015]

  ; --------------------------------------------------------------------------------------------------------------------------------

  load_array: ; (ZH:ZL -> YH:YL, t_1) ; <---
            lpm    t_0, Z+                ; |
            st     Y+, t_0                ; |
            dec    t_1                    ; |
            brne   load_array ; ------------
            ret

  load_text: ; (ZH:ZL -> YH:YL, t_1) ; <---
            lpm    t_0, Z+               ; |
            st     Y+, t_0               ; |
            cpi    t_0, 255              ; |
            brne   load_text ; ------------
            ret

  print_text: ; (ZH:ZL := text, t_3 := номер строки)
            ldi_Y   digit_array
            rcall   load_text
            ldi_Y   digit_array
            rcall   lcd_put_line
            ret

  ; --------------------------------------------------------------------------------------------------------------------------------

  ; dbb_3:dbb_2:dbb_1:dbb_0
  .def  dbb_0  = r10
  .def  dbb_1  = r11
  .def  dbb_2  = r12
  .def  dbb_3  = r13

  .def  temp    = r16
  .def  dbb_cnt = r17
  .equ  dbb_dig = (digit_array+3)

  .include "dBCD32.asm" ; [26.10.2015]

  ; --------------------------------------------------------------------------------------------------------------------------------

  draw_parameter: ; (ldi_Z := label, dbb_3:dbb_2:dbb_1:dbb_0 := value, t_3 := first line {second line := t_3+1})
            ; label
            push         t_3
            rcall        print_text
            ; value
            ldi_Y        digit_array
            store_i_inc  text_SPACE
            store_i_inc  text_SPACE
            store_i_inc  text_SPACE
            rcall        dis_BCD_32 ; 10 digits
            ldi_Y        digit_array+13
            store_i_inc  text_SPACE
            store_i_inc  text_SPACE
            store_i_inc  text_SPACE
            store_i_inc  text_SPACE
            store_i_inc  text_SPACE
            store_i_inc  255 ; `концевой`
            ldi_Y        digit_array
            pop          t_3
            inc          t_3 ; номер строки
            rcall        lcd_put_line
            ret

  ; --------------------------------------------------------------------------------------------------------------------------------

  draw_time_counter_value:
            ; label
            ldi_Z   time_lbl
            cli
               mov     dbb_3, time_3
               mov     dbb_2, time_2
               mov     dbb_1, time_1
               mov     dbb_0, time_0
            sei
            ldi     t_3, 6 ; номер строки
            rcall   draw_parameter
            ret

  ; --------------------------------------------------------------------------------------------------------------------------------

  draw_counter_value:
            ; label
            ldi_Z   expense_lbl
            cli
               mov     dbb_3, c_hi_hi
               mov     dbb_2, c_hi_lo
               mov     dbb_1, c_lo_hi
               mov     dbb_0, c_lo_lo
            sei
            ldi     t_3, 4 ; номер строки
            rcall   draw_parameter
            ret

  ; --------------------------------------------------------------------------------------------------------------------------------

  control_water_pump:                            ;
            ; control                            ;
            SBIS   PIND, 6                       ;
            rjmp   draw_water_pump_condition_0   ; если вывод PD6 замкнут на "землю", то - отключить насос и включать его только на время орошения.
            rjmp   draw_water_pump_condition_1   ; если вывод PD6 подтянут к питанию, то - включить  насос.
       draw_water_pump_condition_0:              ;     \
            ; auto-control                       ;     |
            sbis   PORTB, 4                      ;     |
            rjmp   auto_control_water_pump_0     ; насос отключен.
            rjmp   auto_control_water_pump_1     ; насос включен ----\
       auto_control_water_pump_0:                ;     |             |
            rcall  off_delay_copy                ;     |             |
            rcall  on_off_delay_compare          ;     |             |
            brsh   draw_water_pump_condition_on  ; {счётчик >= off_delay} - включить.
            rjmp   auto_control_water_pump_e     ; держать отключённым ---------------\
       draw_water_pump_condition_on:             ;     |             |                |
            rcall  clr_value_32                  ;     |             |                |
            rjmp   draw_water_pump_condition_1   ; ----|-------------|----\           |
       auto_control_water_pump_1:                ; <---|-------------/    |           |
            rcall  on_delay_copy                 ;     |                  |           |
            rcall  on_off_delay_compare          ;     |                  |           |
            brsh   draw_water_pump_condition_off ; {счётчик >= on_delay} - отключить. |
            rjmp   draw_water_pump_condition_1   ; держать включённым ----\           |
       draw_water_pump_condition_off:            ;     |                  |           |
            rcall  clr_value_32                  ;     |                  |           |
            ; rjmp auto_control_water_pump_e     ;     |                  |           |
       auto_control_water_pump_e:                ; <---|------------------|-----------/
            ldi_Z  water_pump_0_lbl              ;     |                  |
            cbi    PORTB, 4                      ;     |                  |
            rjmp   draw_water_pump_condition_e   ; ----|------------------|----\
       draw_water_pump_condition_1:              ; <---/------------------/    |
            ldi_Z  water_pump_1_lbl              ;                             |
            sbi    PORTB, 4                      ;                             |
            ; rjmp draw_water_pump_condition_e   ;                             |
       draw_water_pump_condition_e:              ; <---------------------------/
            ; label                              ;
            ldi    t_3, 2 ; номер строки         ;
            rcall  print_text                    ;
            ret                                  ;

  ; --------------------------------------------------------------------------------------------------------------------------------

  inc_c_routine:
       ; {ПРОВЕРИТЬ}
            adiw    c_lo_hi:c_lo_lo, volume_per_impulse ; увеличиваем на определённую порцию
            brbc    0, inc_c_routine_end_lbl ; учёт бита переноса
            inc     c_hi_lo
            brne    inc_c_routine_end_lbl
            inc     c_hi_hi
            brne    inc_c_routine_end_lbl
       inc_c_routine_end_lbl:
            ret

  ; --------------------------------------------------------------------------------------------------------------------------------

  sensor_PORT_tst_routine:
       ; {ПРОВЕРИТЬ}
            ; выдавать импульсы для поверки:
            sbis    sensor_PORT, 3 ; skip next instruction if the bit `3` in `sensor_PORT` is set
            rjmp    sensor_tst_1
            rjmp    sensor_tst_0
       sensor_tst_1:
            sbi     sensor_PORT, 3
            ret
       sensor_tst_0:
            cbi     sensor_PORT, 3
            ret

  ; --------------------------------------------------------------------------------------------------------------------------------

  inc_value_32:
        inc_value_8  time_0
        inc_value_8  time_1
        inc_value_8  time_2
        inc_value_8  time_3
     inc_value_32_end_lbl:
        ret

  ; --------------------------------------------------------------------------------------------------------------------------------

  clr_value_32:
        cli
           clr  time_0
           clr  time_1
           clr  time_2
           clr  time_3
        sei
        ret

  ; --------------------------------------------------------------------------------------------------------------------------------

  .macro  time_delay_copy
        ldi  t_0, (@0 >>  0) & 255
        ldi  t_1, (@0 >>  8) & 255
        ldi  t_2, (@0 >> 16) & 255
        ldi  t_3, (@0 >> 24) & 255
  .endm

  ; --------------------------------------------------------------------------------------------------------------------------------

  off_delay_copy:
        time_delay_copy  off_delay
        ; ldi  t_0, (off_delay >>  0) & 255
        ; ldi  t_1, (off_delay >>  8) & 255
        ; ldi  t_2, (off_delay >> 16) & 255
        ; ldi  t_3, (off_delay >> 24) & 255
        ret

  on_delay_copy:
        time_delay_copy  on_delay
        ; ldi  t_0, (on_delay >>  0) & 255
        ; ldi  t_1, (on_delay >>  8) & 255
        ; ldi  t_2, (on_delay >> 16) & 255
        ; ldi  t_3, (on_delay >> 24) & 255
        ret

  ; --------------------------------------------------------------------------------------------------------------------------------

  ; если {@3:@2:@1:@0 >= t_3:t_2:t_1:t_0} то SREG.C:=0 иначе SREG.C:=1
  ;    `BRSH – branch if same or higher (unsigned) : SREG.C == 0`
  ;    `BRLO - branch if lower (unsigned)          : SREG.C == 1`
  .macro  compare_32
        cli
           cp   @0, t_0 ; если {@0 < t_0       } то SREG.C:=1 иначе SREG.C:=0
           cpc  @1, t_1 ; если {@0 < t_0+SREG.C} то SREG.C:=1 иначе SREG.C:=0
           cpc  @2, t_2 ; если {@0 < t_0+SREG.C} то SREG.C:=1 иначе SREG.C:=0
           cpc  @3, t_3 ; если {@0 < t_0+SREG.C} то SREG.C:=1 иначе SREG.C:=0
        sei
  .endm

  ; --------------------------------------------------------------------------------------------------------------------------------

  on_off_delay_compare:
        compare_32  time_0, time_1, time_2, time_3
        ret

  ; --------------------------------------------------------------------------------------------------------------------------------

