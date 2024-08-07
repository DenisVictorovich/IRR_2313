
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
            ; rcall  delay_10_u_sec [26.10.2015] {после последнего бита нужно сразу отпустить линию SDA (согласно инструкции)}
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

