
; ----------------------------------------------------------------------------------------------------------------- bl_t2313.inc

; ������� �������� � {PAGESIZE x 2} ������ � ������� p
erase_flash2: ; (ui8 p { page_l })
              rcall  t2313_page_to_Z_position
              ; �������� ��������: SPMCSR := PGERS | SPMEN
              out_i  SPMCSR, (1<<PGERS)|(1<<SPMEN)
              spm ; ���������� SPM
              ret

; -----------------------------------------------------------------------------------------------------------------------------

; �������� �������� � {PAGESIZE x 2} ������ � ������� p
write_flash2: ; (ui16* b { r_buffer+1 }, ui8 p { page_l })
              rcall  erase_flash2
              rcall  t2313_page_to_Z_position
              ldi    cap_l, PAGESIZE
            ; ������ � ����� ��������, ���������� �� `r_buffer+1`
            write_flash2_loop:
              ld     r0, Y+ ; hi
              ld     r1, Y+ ; lo
              ; ������ � ����� ��������: SPMCSR := SPMEN
              out_i  SPMCSR, (1<<SPMEN) ; ��������� ���������� (SPMEN) � ������� SPMCSR
              spm ; ���������� SPM
              adiw   ZH:ZL, 2
              dec    cap_l
              brne   write_flash2_loop
              subi   ZL, low (PAGESIZE<<1) ; restore pointer
              sbci   ZH, high(PAGESIZE<<1) ; �����������!
              ; ������ ��������: SPMCSR := (PGWRT | SPMEN)
              out_i  SPMCSR, (1<<PGWRT)|(1<<SPMEN) ; ��������� ���������� (PGWRT | SPMEN) � ������� SPMCSR
              spm ; ���������� SPM
              ret

; -----------------------------------------------------------------------------------------------------------------------------

