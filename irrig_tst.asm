
  ; --------------------------------------------------------------------------------------------------------------------------------
  ; <<система орошения комнатных растений>>
  ; модуль управления на базе "крохи-2313":
  ;  _____   ----   ----
  ;  RESET @|1   \_/  20|= питание (2,8 .. 3,2) вольт {LCD.1}
  ; Rx PD0 =|2        19|@ PB7 `SCK`<--_-_-_-_
  ; Tx PD1 =|3  tiny  18|@ PB6 `DO`(`MISO`)-->
  ; К.Р.-- =|4  2313  17|@ PB5 `DI`(`MOSI`)<--
  ; К.Р.-- =|5        16|= PB4
  ; I0 PD2 =|6        15|= PB3           |   {LCD.5}---------|   |   I0 - <расходомер для воды> - п р м ч а
  ; I1 PD3 =|7        14|= PB2 {LCD.3}   |   {LCD.6}---------|   |   I1 - <поверочные импульсы> -  е е ы к
  ; T0 PD4 =|8        13|= PB1 {LCD.4}   |             ||    |   |
  ; T1 PD5 =|9        12|= PB0 {LCD.8}   |   {LCD.7}---||----|   |
  ;  ЗЕМЛЯ @|10       11|= PD6 -ПЕРЕМЫЧКА-             ||  ЗЕМЛЯ |
  ;          -----------                              1 uФ       |
  ; @ - выводы для подключения программатора
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

  ; 32-хразрядный счётчик чего-либо:
  .def    c_lo_lo = r24
  .def    c_lo_hi = r25
  .def    c_hi_lo = r22
  .def    c_hi_hi = r23

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

  ; ВНИМАНИЕ!
  ; чтобы шрифт занимал меньше места в памяти,
  ; можно разместить символы по два в каждой строке
  ; (чётное количество байтов в строке - не нужен ноль в конце строки) - сделано.
  font_lookup:
            ; -----------------------------------------------------------------------------------------
            .db 0x3E, 0x51, 0x49, 0x45, 0x3E, 0x00, 0x42, 0x7F, 0x40, 0x00 ; 0, 1
            .db 0x42, 0x61, 0x51, 0x49, 0x46, 0x21, 0x41, 0x45, 0x4B, 0x31 ; 2, 3
            .db 0x18, 0x14, 0x12, 0x7F, 0x10, 0x27, 0x45, 0x45, 0x45, 0x39 ; 4, 5
            .db 0x3C, 0x4A, 0x49, 0x49, 0x30, 0x01, 0x71, 0x09, 0x05, 0x03 ; 6, 7
            .db 0x36, 0x49, 0x49, 0x49, 0x36, 0x06, 0x49, 0x49, 0x29, 0x1E ; 8, 9
            ; -----------------------------------------------------------------------------------------
            .db 0x7E, 0x11, 0x11, 0x11, 0x7E, 0x7F, 0x49, 0x49, 0x49, 0x36 ; A (10), B (11)
            .db 0x3E, 0x41, 0x41, 0x41, 0x22, 0x7F, 0x41, 0x41, 0x22, 0x1C ; C (12), D (13)
            .db 0x7F, 0x49, 0x49, 0x49, 0x41, 0x7F, 0x09, 0x09, 0x09, 0x01 ; E (14), F (15)
            .db 0x3E, 0x41, 0x49, 0x49, 0x7A, 0x7F, 0x08, 0x08, 0x08, 0x7F ; G (16), H (17)
            .db 0x00, 0x41, 0x7F, 0x41, 0x00, 0x20, 0x40, 0x41, 0x3F, 0x01 ; I (18), J (19)
            .db 0x7F, 0x08, 0x14, 0x22, 0x41, 0x7F, 0x40, 0x40, 0x40, 0x40 ; K (20), L (21)
            .db 0x7F, 0x02, 0x0C, 0x02, 0x7F, 0x7F, 0x04, 0x08, 0x10, 0x7F ; M (22), N (23)
            .db 0x3E, 0x41, 0x41, 0x41, 0x3E, 0x7F, 0x09, 0x09, 0x09, 0x06 ; O (24), P (25)
            .db 0x3E, 0x41, 0x51, 0x21, 0x5E, 0x7F, 0x09, 0x19, 0x29, 0x46 ; Q (26), R (27)
            .db 0x46, 0x49, 0x49, 0x49, 0x31, 0x01, 0x01, 0x7F, 0x01, 0x01 ; S (28), T (29)
            .db 0x3F, 0x40, 0x40, 0x40, 0x3F, 0x1F, 0x20, 0x40, 0x20, 0x1F ; U (30), V (31)
            .db 0x3F, 0x40, 0x38, 0x40, 0x3F, 0x63, 0x14, 0x08, 0x14, 0x63 ; W (32), X (33)
            .db 0x07, 0x08, 0x70, 0x08, 0x07, 0x61, 0x51, 0x49, 0x45, 0x43 ; Y (34), Z (35)
            ; -----------------------------------------------------------------------------------------
            .db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x08, 0x08, 0x08 ; 'space' (36), 'minus' (37)
            ; -----------------------------------------------------------------------------------------
            ; прописные греческие буквы:
            .db 112,  28,  19,  28, 112,  65, 127,  73,  73,  54 ; альфа   (38), бетта   (39)
            .db  65, 127,  65,   1,   3, 112,  76,  67,  76, 112 ; гамма   (40), дельта  (41)
            .db 127,  73,  73,  73,  65,  99,  81,  73,  69,  99 ; эпсилон (42), дзета   (43)
            .db 127,   9,   8,  72, 127,  62,  93,  73,  93,  62 ; эта     (44), тета    (45)
            .db  65,  65, 127,  65,  65, 127,   8,   8,  54,  65 ; йота    (46), каппа   (47)
            .db 112,  12,   3,  12, 112, 127,   5,   8,  68, 127 ; ламбда  (48), мю      (49)
            .db 127,  68,   8,  17, 127,  99,  73,  73,  73,  99 ; ню      (50), кси     (51)
            .db  62,  65,  65,  65,  62,  65, 127,   1, 127,  65 ; омикрон (52), пи      (53)
            .db  65, 127,  73,   9,   6,  99,  85,  73,  65,  99 ; ро      (54), сигма   (55)
            .db   3,  65, 127,  65,   3,   1,  66, 124,  66,   1 ; тау     (56), ипсилон (57)
            .db   8,  85, 127,  85,   8,  65,  54,   8,  54,  65 ; фи      (58), хи      (59)
            .db   7,  72, 127,  72,   7,  94,  97,   1,  97,  94 ; пси     (60), омега   (61)
            ; -----------------------------------------------------------------------------------------
            ; строчные греческие буквы:
            .db  48,  72,  72,  56,  68, 252,  84,  84,  84,  40 ; альфа   (62), бетта   (63)
            .db  50,  76,  72,  76,  50,  32,  82,  85,  89,  34 ; гамма   (64), дельта  (65)
            .db  40,  84,  84,  68,  40,  28, 163, 162, 162,  64 ; эпсилон (66), дзета   (67)
            .db   4, 120,   4,   4, 248,  60,  74,  82,  74,  60 ; эта     (68), тета    (69)
            .db   4,  60,  64,  64,  32, 124,  16,  16,  40,  68 ; йота    (70), каппа   (71)
            .db  66,  57,   7,  56,  64, 252,  32,  32,  28,  32 ; ламбда  (72), мю      (73)
            .db   4,  56,  65,  34,  28,  21, 170, 170, 170,  64 ; ню      (74), кси     (75)
            .db  56,  68,  68,  68,  56,  72,  60,   4,  60,  68 ; омикрон (76), пи      (77)
            .db 248,  36,  36,  36,  24,  56,  68,  76,  76,  52 ; ро      (78), сигма   (79)
            .db   8,   4, 124,  68,  36,   4,  56,  64,  68,  56 ; тау     (80), ипсилон (81)
            .db  48,  72, 252,  72,  48, 152,  68, 120, 136, 100 ; фи      (82), хи      (83)
            .db  28,  32, 252,  32,  28,  56,  68,  48,  68,  56 ; пси     (84), омега   (85)
            ; -----------------------------------------------------------------------------------------
            .db  24,  36, 164, 100,   4,   0, 192,  64,   0,   0 ; сигма 2 (86), 'comma' (87)
            .db  0,   64,   0,   0,   0,   0,  68,   0,   0,   0 ; 'point' (88), 'colon' (89)
            ; -----------------------------------------------------------------------------------------

  ; --------------------------------------------------------------------------------------------------------------------------------

  line01: .db bg_EPSILON, sg_MU, sg_BETA, sg_RHO, sg_UPSILON, sg_OMICRON, sg_LAMBDA, sg_OMICRON, sg_GAMMA, sg_IOTA, sg_KAPPA, sg_EPSILON, sg_SIGMA_F, 255
  line02: .db bg_SIGMA, sg_UPSILON, sg_MU, sg_BETA, sg_IOTA, sg_OMEGA, sg_TAU, sg_IOTA, sg_KAPPA, sg_ETA, 255
  line03: .db bg_NU, sg_EPSILON, sg_UPSILON, sg_RHO, sg_OMICRON, sg_DELTA, sg_IOTA, sg_CHI, sg_OMICRON, sg_TAU, sg_OMICRON, sg_MU, sg_IOTA, sg_ALPHA, 255

  ; --------------------------------------------------------------------------------------------------------------------------------

  .include "mpy16s.inc"
  .include "div16s.inc"

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
            out_i  TCCR1B, (1<<3)|5 ; CTC | CK/1024
            out_i  OCR1AH, (0)
            out_i  OCR1AL, (216-1) ; {11059200 Гц / 1024 = 10800 Гц} / 216 = 50 Гц ~ 20 мс

            clr     t_0
            out     TCNT1H, t_0
            out     TCNT1L, t_0

            ; c := 0000|0000h
            clr     c_lo_lo
            clr     c_lo_hi
            clr     c_hi_lo
            clr     c_hi_hi

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
            ; <...>

         sei

            ; <...>
            ldi     t_0, 0 ; '0'
            rcall   fill_lcd_line_16
            ldi     t_3, 0
            rcall   lcd_put_line

            ldi     t_0, 16 ; 'G'
            rcall   fill_lcd_line_16
            ldi     t_3, 1
            rcall   lcd_put_line

            ldi     t_0, 32 ; 'W'
            rcall   fill_lcd_line_16
            ldi     t_3, 2
            rcall   lcd_put_line

            ldi     t_0, 48 ; 'ламбда'
            rcall   fill_lcd_line_16
            ldi     t_3, 3
            rcall   lcd_put_line

            ldi     t_0, 64 ; 'гамма'
            rcall   fill_lcd_line_16
            ldi     t_3, 6
            rcall   lcd_put_line

            ldi     t_0, 80 ; 'тау'
            rcall   fill_lcd_line_16_part
            ldi     t_3, 7
            rcall   lcd_put_line

  ; loop
  LOOP:
            ; обновление картинки раз в {time_delay_value x 20} мс
            tst     time_delay
            brne    LOOP
            ldi     t_0, time_delay_value
            mov     time_delay, t_0
            ;
            rcall   draw_counter_value
            rcall   sensor_PORT_tst_routine
            ; <...>
            rjmp    LOOP

  ; ------------------------------------------------------------------------------------------------ timer interrupt {50 Гц ~ 20 мс}
  TIMER1_INT:
            in      SREG_I, SREG

            ; ----------------------------------------------------------------------------------------------------------------------

            wdr

            ;;; rcall   inc_c_routine

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

            ;;; ; c := 0000|0000h
            ;;; clr     c_lo_lo
            ;;; clr     c_lo_hi
            ;;; clr     c_hi_lo
            ;;; clr     c_hi_hi
            rcall   inc_c_routine

            ; ----------------------------------------------------------------------------------------------------------------------

       EXT_INT_0_END_LABEL:
            out     SREG, SREG_I
            reti

  ; --------------------------------------------------------------------------------------------------------------------------------
  ; `GPM388CO`, `ST7558`
  ; `ST7558_motorola_C115.pdf` . page 44:
  ;  (VDD= 1.8V~3.3V, Ta=-30~85°C)
  ; SCL clock frequency   [0 .. 200] kHZ
  ; SCL clock low period  [2.5 ..  ] us
  ; SCL clock high period [2.5 ..  ] us
  ; ................................................................................................................................
  ; I2C Interface protocol
  ; The ST7558 supports command, data write addressed slaves on the bus.
  ; Before any data is transmitted on the I2C Interface, the device, which should respond, is addressed first. Four 7-bit slave
  ; addresses (0111100{0x78} to 0111111{0x7E}) are reserved for the ST7558. The R/W is assigned to 0 for Write only.
  ; The I2C Interface protocol is illustrated in Fig.7. [`ST7558_motorola_C115.pdf` . page 18]
  ; ................................................................................................................................
  .equ  ind_device_address = 0x78

  delay_10_u_sec: ; ~10 микросекунд при частоте кварцевого резонатора 11,0592 МГц
          ldi   delay_c, 40
       delay_10_u_sec_1:
          dec   delay_c
          brne  delay_10_u_sec_1
          ret

  ind_RESET_1:
            sbi  ind_PORT, ind_RESET
            ret

  ind_RESET_0:
            cbi  ind_PORT, ind_RESET
            ret

  ind_i2c_start:
            sbi    ind_PORT, ind_SDA ; SCL: -----|------  |
            sbi    ind_PORT, ind_SCL ;           |      \ |
            rcall  delay_10_u_sec    ;           |       \|
            cbi    ind_PORT, ind_SDA ;           |        ------
            rcall  delay_10_u_sec    ; SDA: ------        |
            cbi    ind_PORT, ind_SCL ;           |\       |
            rcall  delay_10_u_sec    ;           | \      |
            ret                      ;           |  ------|-----

  ind_i2c_stop:
            cbi    ind_PORT, ind_SCL ;           |  ------|-----
            cbi    ind_PORT, ind_SDA ;           | /      |
            rcall  delay_10_u_sec    ;           |/       |
            sbi    ind_PORT, ind_SCL ; SCL: -----|        |
            rcall  delay_10_u_sec    ;           |        |-----
            sbi    ind_PORT, ind_SDA ;           |       /|
            rcall  delay_10_u_sec    ;           |      / |
            ret                      ; SDA: -----|------  |

  ind_SCL_pulse:
            rcall  delay_10_u_sec    ;           |  ----  |
            sbi    ind_PORT, ind_SCL ;           | /    \ |
            rcall  delay_10_u_sec    ;           |/      \|
            cbi    ind_PORT, ind_SCL ; SCL: -----|        |-----
            rcall  delay_10_u_sec
            ret

  ind_I2C_rd_bit: ; {return t_2} - можно сделать по требованию
            cbi    ind_DDR,  ind_SDA
            sbi    ind_PORT, ind_SDA
            sbi    ind_PORT, ind_SCL
            rcall  delay_10_u_sec
            rcall  delay_10_u_sec
            rcall  delay_10_u_sec
            ; <...> - {read in t_2}
            cbi    ind_PORT, ind_SCL
            rcall  delay_10_u_sec
            sbi    ind_DDR,  ind_SDA
            ret

  ind_i2c_write: ; (t_2)
            ; MSB .. LSB
            ldi    t_0, 0x80
       ind_i2c_write_loop:            ; <--
            mov    t_1, t_2           ;    |
            and    t_1, t_0 ; t_1&=t_0;    |
            breq   ind_i2c_write_0    ;    |
            sbi    ind_PORT, ind_SDA  ;    |
            rjmp   ind_i2c_write_1_   ;    |
       ind_i2c_write_0:               ;    |
            cbi    ind_PORT, ind_SDA  ;    |
       ind_i2c_write_1_:              ;    |
            rcall  ind_SCL_pulse      ;    |
            LSR    t_0 ; t_0>>=1      ;    |
            brne   ind_i2c_write_loop ; ---
            rcall  ind_I2C_rd_bit
            ret

  ; --------------------------------------------------------------------------------------------------------------------------------

  ind_GPM388CO_init:
            out_i  ind_PORT, (1<<ind_RESET)|(1<<ind_SDA)|(1<<ind_SCL)
            out_i  ind_DDR,  (1<<ind_RESET)|(1<<ind_SDA)|(1<<ind_SCL)
            rcall  ind_RESET_0
            rcall  delay_10_u_sec
            rcall  ind_RESET_1
            rcall  delay_10_u_sec
            rcall  ind_i2c_stop
            rcall  delay_10_u_sec
            rcall  ind_i2c_start
            ldi_Z  ind_init_set_1
               ldi    t_3, 12
               rcall  ind_i2c_write_arr
            rcall  ind_i2c_start
            ldi_Z  ind_init_set_2
               ldi    t_3, 4
               rcall  ind_i2c_write_arr
            ; video_mode(1):
            rcall  ind_i2c_start
            ldi_Z  ind_init_set_3
               ldi    t_3, 4
               rcall  ind_i2c_write_arr
            rcall  ind_clear_lcd
            ret

  ind_set_row: ; (t_3 = 0..7)
            rcall  ind_i2c_start
            ind_i2c_wr  ind_device_address
            ind_i2c_wr  0x00
            ind_i2c_wr  0x20
            ind_i2c_wr  0x80
            mov    t_2, t_3
            ori    t_2, 0x40
            rcall  ind_i2c_write
            ret

  ind_clear_lcd: ; (void)
            rcall  ind_i2c_start
            ind_i2c_wr  ind_device_address
            ind_i2c_wr  0x40
            ind_i2c_wr  0x00
            ind_i2c_wr  0x00
            ldi    t_3, 229 ; x 4 + 2 = 918
       ind_clear_lcd_1:            ; <---
            ind_i2c_wr  0x00       ;     |
            ind_i2c_wr  0x00       ;     |
            ind_i2c_wr  0x00       ;     |
            ind_i2c_wr  0x00       ;     |
            dec    t_3             ;     |
            brne   ind_clear_lcd_1 ; ----
            ret

  lcd_put_line: ; (YH:YL-string{концевой символ = 255}, t_3-position{0..7})
            rcall  ind_set_row
            rcall  ind_i2c_start
            ind_i2c_wr  ind_device_address
            ind_i2c_wr  0x40
       lcd_put_line_repeat_lbl:            ; <------------------------
            ld     t_3, Y+                 ; ОЗУ -> t_3               |
            cpi    t_3, 255                ; концевой символ = 255    |
            breq   lcd_put_line_end_lbl    ; if(t_3 == 255) ----------|-
            ldi_Z  font_lookup             ;                          | |
       lcd_put_line_mul_lbl:               ; <--------------------    | |
            tst    t_3                     ; (t_3 == 0)?          |   | |
            breq   lcd_put_line_loop_end   ; ---------------------|-  | |
            ADIW   ZH:ZL, 5                ; размер одного символа| | | |
            dec    t_3                     ; (*)                  | | | |
            rjmp   lcd_put_line_mul_lbl    ; ---------------------  | | |
       lcd_put_line_loop_end:              ; <----------------------  | |
            ldi    t_3, 5                  ; (*)                      | |
            rcall  ind_i2c_write_arr       ;                          | |
            ldi    t_2, 0                  ; промежуток м/у буквами   | |
            rcall  ind_i2c_write           ;                          | |
            rjmp   lcd_put_line_repeat_lbl ; -------------------------  |
       lcd_put_line_end_lbl:               ; <--------------------------
            ret

  ; --------------------------------------------------------------------------------------------------------------------------------

  ind_i2c_write_arr: ; (ZH:ZL, t_3)  ; <---
            lpm    t_2, Z+           ;     |
            rcall  ind_i2c_write     ;     |
            dec    t_3               ;     |
            brne   ind_i2c_write_arr ; ----
            ret

  ; --------------------------------------------------------------------------------------------------------------------------------

  ind_init_set_1:
            .db ind_device_address, 0x00, 0x2E, 0x21
            .db 0x12, 0xC0, 0x0B, 0x20
            .db 0x11, 0x0C, 0x40, 0x80
  ind_init_set_2:
            .db ind_device_address, 0x00, 0x20, 0x08
  ind_init_set_3: ; video_mode(...):
            .db ind_device_address, 0x00, 0x20, 0x0C ; 0x0D

  ; --------------------------------------------------------------------------------------------------------------------------------

  print_value: ; print_value(XH:XL)
     cli
        push   XL
        push   XH
        push   YL
        push   YH
           bst    XH, 7 ; знак числа скопировать в SREG.T
           ldi_Y  digit_array+digit_capacity
        print_value_loop_lbl: ; <--------------------------
           cpi    XH, 0                                  ; |
           brne   print_value_symbol_lbl                 ; |
           cpi    XL, 0                                  ; |
           brne   print_value_symbol_lbl                 ; |
           cpi    YL, low(digit_array+digit_capacity)    ; |
           breq   print_value_symbol_lbl ; '0'           ; |
           brtc   print_value_sign_passed ; знак учтён   ; |
           ldi    t_0, 37 ; `минус`                      ; |
           clt    ; SREG.T := 0                          ; |
           rjmp   print_value_space_sign_lbl ; --------  ; |
        print_value_sign_passed:                     ; | ; |
           ldi    t_0, 36 ; `пробел`                 ; | ; |
           rjmp   print_value_space_sign_lbl ; --------| ; |
        print_value_symbol_lbl:                      ; | ; |
           mov    dd16sH, XH                         ; | ; |
           mov    dd16sL, XL                         ; | ; |
           ldi    dv16sH, high(10)                   ; | ; |
           ldi    dv16sL, low (10)                   ; | ; |
           rcall  div16s                             ; | ; |
           mov    XH, dres16sH                       ; | ; |
           mov    XL, dres16sL                       ; | ; |
           mov    t_0, drem16sL ; drem16sH == 0      ; | ; |
           ; rcall  dig_to_SYM                       ; | ; |
        print_value_space_sign_lbl: ; <----------------  ; |
           st     -Y, t_0                                ; |
           cpi    YL, low(digit_array) ; < 255 знакомест ; |
           brne   print_value_loop_lbl ; ------------------
        pop    YH
        pop    YL
        pop    XH
        pop    XL
     sei
     ret

  ; --------------------------------------------------------------------------------------------------------------------------------

  fill_lcd_line_16:
            ldi_Y   digit_array
            st      Y+, t_0 ;
            inc     t_0
            st      Y+, t_0
            inc     t_0
            st      Y+, t_0
            inc     t_0
            st      Y+, t_0
            inc     t_0
            st      Y+, t_0 ;
            inc     t_0
            st      Y+, t_0
            inc     t_0
            st      Y+, t_0
            inc     t_0
            st      Y+, t_0
            inc     t_0
            st      Y+, t_0 ;
            inc     t_0
            st      Y+, t_0
            inc     t_0
            st      Y+, t_0
            inc     t_0
            st      Y+, t_0
            inc     t_0
            st      Y+, t_0 ;
            inc     t_0
            st      Y+, t_0
            inc     t_0
            st      Y+, t_0
            inc     t_0
            st      Y+, t_0
            ldi     t_0, 255 ; `концевой`
            st      Y, t_0
            ldi_Y   digit_array
            ret

  fill_lcd_line_16_part:
            ldi_Y   digit_array
            st      Y+, t_0  ; 1
            inc     t_0
            st      Y+, t_0  ; 2
            inc     t_0
            st      Y+, t_0  ; 3
            inc     t_0
            st      Y+, t_0  ; 4
            inc     t_0
            st      Y+, t_0  ; 5
            inc     t_0
            st      Y+, t_0  ; 6
            inc     t_0
            st      Y+, t_0  ; 7
            inc     t_0
            st      Y+, t_0  ; 8
            inc     t_0
            st      Y+, t_0  ; 9
            inc     t_0
            st      Y+, t_0  ; 10
            ldi     t_0, 255 ; `концевой`
            st      Y, t_0
            ldi_Y   digit_array
            ret

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

  sample_text: .db sg_ALPHA, sg_BETA, sg_GAMMA, 255
  expense_lbl: .db bg_UPSILON, sg_DELTA, sg_OMEGA, sg_RHO, text_SPACE, sg_DELTA, sg_ALPHA, sg_PI, sg_ALPHA, sg_NU, sg_OMEGA, sg_NU, text_COLON, 255

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

  draw_counter_value:
            ; label
            ldi_Y   digit_array
            ldi_Z   expense_lbl
            rcall   load_text
            ldi_Y   digit_array
            ldi     t_3, 4 ; номер строки
            rcall   lcd_put_line
            ; value
            ldi_Y   digit_array
            ldi     t_0, text_SPACE
            st      Y+, t_0
            ldi     t_0, text_SPACE
            st      Y+, t_0
            ldi     t_0, text_SPACE
            st      Y+, t_0
            cli
               ; FF000000
               mov     t_1, c_hi_hi
               rcall   draw_byte_t_1
               ; 00FF0000
               mov     t_1, c_hi_lo
               rcall   draw_byte_t_1
               ; 0000FF00
               mov     t_1, c_lo_hi
               rcall   draw_byte_t_1
               ; 000000FF
               mov     t_1, c_lo_lo
               rcall   draw_byte_t_1
            sei
            ldi     t_0, text_SPACE
            st      Y+, t_0
            ldi     t_0, text_SPACE
            st      Y+, t_0
            ldi     t_0, text_SPACE
            st      Y+, t_0
            ldi     t_0, text_SPACE
            st      Y+, t_0
            ldi     t_0, text_SPACE
            st      Y+, t_0
            ldi     t_0, 255 ; `концевой`
            st      Y, t_0
            ldi_Y   digit_array
            ldi     t_3, 5 ; номер строки
            rcall   lcd_put_line
            ret

  ; --------------------------------------------------------------------------------------------------------------------------------

  inc_c_routine:
       ; {ПРОВЕРИТЬ}
            adiw    c_lo_hi:c_lo_lo, 30 ; увеличиваем на определённую порцию
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

