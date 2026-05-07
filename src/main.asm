.include "m2560def.inc"

;------------------------------- CONSTANTES ----------------------------------
.equ UMBRAL_H    = 0x02
.equ UMBRAL_L    = 0xBC

.equ VEL_RECTO   = 150
.equ VEL_LEVE_L  = 70
.equ VEL_LEVE_H  = 170
.equ VEL_CURVA_L = 0
.equ VEL_CURVA_H = 200
.equ VEL_PIVOT   = 220
.equ VEL_PIVOT_F = 230
.equ VEL_BUSQ    = 180

.equ GAP_TOLERA  = 75

; Canales ADC
.equ ADC_CH_FR   = 0
.equ ADC_CH_CR   = 1
.equ ADC_CH_CL   = 2
.equ ADC_CH_FL   = 3
.equ ADC_CH_PRES = 4
.equ ADC_CH_LL1  = 5
.equ ADC_CH_LL2  = 6
.equ ADC_CH_LL3  = 7


; Umbrales llama
.equ LLAMA_ON    = 61
.equ LLAMA_CHECK = 5

; Umbrales presión
.equ PRES_ON     = 60
.equ PRES_OFF    = 46

; Pines bomba
.equ BOMBA_ENA   = 4
.equ BOMBA_IN2   = 5
.equ BOMBA_IN1   = 6
.equ SERVO_PIN   = 3
.equ SERVO_90DEG = 2900
.equ LED_ENA_BIT = 4
.equ LED_IN1_BIT = 2
.equ LED_IN2_BIT = 0

; SRAM
.equ SRAM_LL1_L  = 0x0200
.equ SRAM_LL1_H  = 0x0201
.equ SRAM_LL2_L  = 0x0202
.equ SRAM_LL2_H  = 0x0203
.equ SRAM_LL3_L  = 0x0204
.equ SRAM_LL3_H  = 0x0205
.cseg
.org 0x0000
    jmp RESET


.org 0x001E
    jmp TIMER2_OVF_ISR

.org 0x0072

;=============================================================================
; RESET
;=============================================================================
RESET:
    ldi r16, high(RAMEND)
    out SPH, r16
    ldi r16, low(RAMEND)
    out SPL, r16

    sbi DDRB, 7
    sbi PORTB, 7

    ldi r16, 0x0F
    out DDRA, r16
    ldi r16, 0x00
    out PORTA, r16

    in  r16, DDRE
    ori r16, (1<<4)|(1<<5)
    out DDRE, r16

    ; Bomba apagada
    sbi DDRB, BOMBA_ENA
    sbi DDRB, BOMBA_IN1
    sbi DDRB, BOMBA_IN2
    cbi PORTB, BOMBA_ENA
    cbi PORTB, BOMBA_IN1
    cbi PORTB, BOMBA_IN2

    ; Servo PH3
    lds r16, DDRH
    ori r16, (1<<SERVO_PIN)
    sts DDRH, r16

    ; --- LED: PL4(ENA), PL2(IN1), PL0(IN2) como salidas ---
    lds r16, DDRL
    ori r16, (1<<LED_ENA_BIT) | (1<<LED_IN1_BIT) | (1<<LED_IN2_BIT)
    sts DDRL, r16
    ; IN1=HIGH, IN2=LOW (dirección fija)
    lds r16, PORTL
    ori r16, (1<<LED_IN1_BIT)
    sts PORTL, r16

    ; Timer3 con modo 14 fast pwm no invertido y preescaler 64
    ldi r16, (1<<COM3B1)|(1<<COM3C1)|(1<<WGM31)
    sts TCCR3A, r16
    ldi r16, (1<<WGM33)|(1<<WGM32)|(1<<CS31)|(1<<CS30)
    sts TCCR3B, r16
    ldi r16, 0x00
    sts ICR3H, r16
    ldi r16, 0xFF
    sts ICR3L, r16
    ldi r16, 0x00
    sts OCR3BH, r16
    sts OCR3BL, r16
    sts OCR3CH, r16
    sts OCR3CL, r16

    ; Timer4 con modo 14 fast pwm y preescaler 8
    ldi r16, (1<<COM4A1)|(1<<WGM41)
    sts TCCR4A, r16
    ldi r16, (1<<WGM43)|(1<<WGM42)|(1<<CS41)
    sts TCCR4B, r16
    ldi r16, high(39999)
    sts ICR4H, r16
    ldi r16, low(39999)
    sts ICR4L, r16
    ldi r16, high(SERVO_90DEG)
    sts OCR4AH, r16
    ldi r16, low(SERVO_90DEG)
    sts OCR4AL, r16

    ; --- Timer5 con fast pwm no invertido y preescaler 1
    ldi r16, (1<<COM5B1) | (1<<WGM50)
    sts TCCR5A, r16
    ldi r16, (1<<WGM52) | (1<<CS50)
    sts TCCR5B, r16
    ; LED apagado al inicio
    ldi r16, 0
    sts OCR5BH, r16
    sts OCR5BL, r16

    ; --- Timer2 en modo normal y con prescaler de 1024
    ldi r16, 0x00
    sts TCCR2A, r16
    ldi r16, (1<<CS22) | (1<<CS21) | (1<<CS20)
    sts TCCR2B, r16
    ldi r16, (1<<TOIE2)
    sts TIMSK2, r16

    ; ADC
    ldi r16, (1<<REFS0)
    sts ADMUX, r16
    ldi r16, 0x00
    sts ADCSRB, r16
    ldi r16, (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)
    sts ADCSRA, r16

    clr r7
    clr r21
    clr r13                  ; flag luz = 0
    ldi r16, LLAMA_CHECK
    mov r8, r16
    ldi r16, 90
    mov r9, r16
    clr r10

    ; Lectura inicial de presión
    ldi r22, ADC_CH_PRES
    rcall READ_ADC
    mov r16, r10
    rcall EVALUAR_PRESION
    mov r10, r16

    ; Habilitar interrupciones globales
    sei

    rcall DELAY_1S

;=============================================================================
; MAIN LOOP
;=============================================================================
MAIN_LOOP:

    ; --- Actualizar LED si Timer2 disparó la interrupción ---
    tst r13
    breq SKIP_LIGHT
    clr r13
    rcall READ_ADC8
    ; Mapear 0-1023 → 0-255 (dividir entre 4)
    mov r16, r18             
    lsr r16
    lsr r16                
    mov r17, r19             
    lsl r17
    lsl r17
    lsl r17
    lsl r17
    lsl r17
    lsl r17                  ; r17 = r19 << 6
    or r16, r17              ; r16 = valor PWM 0-255
    ; Escribir OCR5B (brillo LED)
    ldi r17, 0
    sts OCR5BH, r17
    sts OCR5BL, r16
SKIP_LIGHT:

    dec r8
    brne SKIP_SENSORS

    ldi r16, LLAMA_CHECK
    mov r8, r16

    ldi r22, ADC_CH_PRES
    rcall READ_ADC
    mov r16, r10
    rcall EVALUAR_PRESION
    mov r10, r16

    tst r10
    breq SKIP_SENSORS

    ldi r22, ADC_CH_LL1
    rcall READ_ADC
    sts SRAM_LL1_L, r18
    sts SRAM_LL1_H, r19
    tst r19
    brne LL1_NO
    cpi r18, LLAMA_ON
    brsh LL1_NO
    rjmp FUEGO_DETECTADO
LL1_NO:

    ldi r22, ADC_CH_LL2
    rcall READ_ADC
    sts SRAM_LL2_L, r18
    sts SRAM_LL2_H, r19
    tst r19
    brne LL2_NO
    cpi r18, LLAMA_ON
    brsh LL2_NO
    rjmp FUEGO_DETECTADO
LL2_NO:

    ldi r22, ADC_CH_LL3
    rcall READ_ADC
    sts SRAM_LL3_L, r18
    sts SRAM_LL3_H, r19
    tst r19
    brne SKIP_SENSORS
    cpi r18, LLAMA_ON
    brsh SKIP_SENSORS
    rjmp FUEGO_DETECTADO

SKIP_SENSORS:


    ldi r22, ADC_CH_FR
    rcall READ_ADC
    rcall ES_NEGRO
    mov r2, r16

    ldi r22, ADC_CH_CR
    rcall READ_ADC
    rcall ES_NEGRO
    mov r3, r16

    ldi r22, ADC_CH_CL
    rcall READ_ADC
    rcall ES_NEGRO
    mov r4, r16

    ldi r22, ADC_CH_FL
    rcall READ_ADC
    rcall ES_NEGRO
    mov r5, r16

    tst r5
    breq SK_PICO_IZQ
    tst r4
    breq SK_PICO_IZQ
    tst r3
    breq SK_PICO_IZQ
    rcall PIVOTE_IZQ_F
    ldi r16, 0xFF
    mov r7, r16
    clr r21
    rjmp MAIN_LOOP
SK_PICO_IZQ:

    tst r2
    breq SK_PICO_DER
    tst r3
    breq SK_PICO_DER
    tst r4
    breq SK_PICO_DER
    rcall PIVOTE_DER_F
    ldi r16, 1
    mov r7, r16
    clr r21
    rjmp MAIN_LOOP
SK_PICO_DER:

    tst r5
    breq D_CHECK_FL_SOLO
    tst r2
    breq D_FL_Y_ALGO
    tst r3
    breq D_SIGUE_LAST
    tst r4
    breq D_SIGUE_LAST
    rcall ADELANTE
    clr r21
    rjmp MAIN_LOOP

D_SIGUE_LAST:
    ldi r16, 1
    cp r7, r16
    breq D_PIVOT_DER
    tst r7
    breq D_PIVOT_IZQ
D_PIVOT_IZQ:
    rcall PIVOTE_IZQ
    clr r21
    rjmp MAIN_LOOP
D_PIVOT_DER:
    rcall PIVOTE_DER
    clr r21
    rjmp MAIN_LOOP

D_FL_Y_ALGO:
    tst r4
    brne D_PICO_IZQ_OLD
    rcall PIVOTE_IZQ
    ldi r16, 0xFF
    mov r7, r16
    clr r21
    rjmp MAIN_LOOP

D_PICO_IZQ_OLD:
    rcall PIVOTE_IZQ
    ldi r16, 0xFF
    mov r7, r16
    clr r21
    rjmp MAIN_LOOP

D_CHECK_FL_SOLO:
    tst r2
    breq D_CENTROS
    tst r3
    brne D_PICO_DER_OLD
    rcall PIVOTE_DER
    ldi r16, 1
    mov r7, r16
    clr r21
    rjmp MAIN_LOOP

D_PICO_DER_OLD:
    rcall PIVOTE_DER
    ldi r16, 1
    mov r7, r16
    clr r21
    rjmp MAIN_LOOP

D_CENTROS:
    tst r4
    breq D_CR_CHECK
    tst r3
    breq D_CL_SOLO
    rcall ADELANTE
    clr r7
    clr r21
    rjmp MAIN_LOOP

D_CL_SOLO:
    rcall CURVA_IZQ
    ldi r16, 0xFF
    mov r7, r16
    clr r21
    rjmp MAIN_LOOP

D_CR_CHECK:
    tst r3
    breq D_TODO_BLANCO
    rcall CURVA_DER
    ldi r16, 1
    mov r7, r16
    clr r21
    rjmp MAIN_LOOP

D_TODO_BLANCO:
    inc  r21
    brne TB_CHECK
    ldi  r21, 0xFF
TB_CHECK:
    cpi  r21, GAP_TOLERA
    brsh TB_BUSCAR
    tst  r7
    brne TB_BUSCAR
    rcall ADELANTE
    rjmp MAIN_LOOP

TB_BUSCAR:
    tst r7
    breq TB_RECTO
    ldi r16, 1
    cp r7, r16
    breq TB_DER
    rcall PIVOTE_IZQ
    rjmp MAIN_LOOP
TB_DER:
    rcall PIVOTE_DER
    rjmp MAIN_LOOP
TB_RECTO:
    rcall ADELANTE
    rjmp MAIN_LOOP

;=============================================================================
; FUEGO DETECTADO (IDÉNTICO)
;=============================================================================
FUEGO_DETECTADO:
    rcall DETENER

FUEGO_LOOP:
    ldi r22, ADC_CH_PRES
    rcall READ_ADC
    mov r16, r10
    rcall EVALUAR_PRESION
    mov r10, r16

    tst r10
    breq FUEGO_SIN_AGUA

    ldi r22, ADC_CH_LL1
    rcall READ_ADC
    sts SRAM_LL1_L, r18
    sts SRAM_LL1_H, r19
    rcall EVALUAR_LLAMA
    mov r2, r16

    ldi r22, ADC_CH_LL2
    rcall READ_ADC
    sts SRAM_LL2_L, r18
    sts SRAM_LL2_H, r19
    rcall EVALUAR_LLAMA
    mov r3, r16

    ldi r22, ADC_CH_LL3
    rcall READ_ADC
    sts SRAM_LL3_L, r18
    sts SRAM_LL3_H, r19
    rcall EVALUAR_LLAMA
    mov r4, r16

    mov r16, r2
    or r16, r3
    or r16, r4
    tst r16
    breq FUEGO_APAGADO

    rcall CALC_ANGULO
    rcall MOVER_SERVO
    rcall BOMBA_ON

    cbi PORTB, 7
    rcall DELAY_100MS
    rcall DELAY_100MS
    sbi PORTB, 7
    rcall DELAY_100MS
    rcall DELAY_100MS

    rjmp FUEGO_LOOP

FUEGO_SIN_AGUA:
    rcall BOMBA_OFF
    rcall SERVO_CENTRO
    sbi PORTB, 7
    clr r7
    clr r21
    ldi r16, LLAMA_CHECK
    mov r8, r16
    rjmp MAIN_LOOP

FUEGO_APAGADO:
    rcall BOMBA_OFF
    rcall SERVO_CENTRO
    sbi PORTB, 7
    clr r7
    clr r21
    ldi r16, LLAMA_CHECK
    mov r8, r16
    rjmp MAIN_LOOP


TIMER2_OVF_ISR:
    push r16
    in r16, SREG
    push r16
    ldi r16, 1
    mov r13, r16
    pop r16
    out SREG, r16
    pop r16
    reti

;#############################################################################
; MOTORES (IDÉNTICO)
;#############################################################################
SET_DIR:
    andi r16, 0x0F
    mov  r17, r16
    in   r16, PORTA
    andi r16, 0xF0
    or   r16, r17
    out  PORTA, r16
    ret

SET_PWM:
    ldi r16, 0x00
    sts OCR3BH, r16
    sts OCR3BL, r24
    sts OCR3CH, r16
    sts OCR3CL, r25
    ret

ADELANTE:
    ldi r16, 0x0A
    rcall SET_DIR
    ldi r24, VEL_RECTO
    ldi r25, VEL_RECTO
    rcall SET_PWM
    ret

CURVA_IZQ:
    ldi r16, 0x0A
    rcall SET_DIR
    ldi r24, VEL_LEVE_L
    ldi r25, VEL_LEVE_H
    rcall SET_PWM
    ret

CURVA_DER:
    ldi r16, 0x0A
    rcall SET_DIR
    ldi r24, VEL_LEVE_H
    ldi r25, VEL_LEVE_L
    rcall SET_PWM
    ret

PIVOTE_IZQ:
    ldi r16, 0x09
    rcall SET_DIR
    ldi r24, 240
    ldi r25, 220
    rcall SET_PWM
    ret

PIVOTE_DER:
    ldi r16, 0x06
    rcall SET_DIR
    ldi r24, VEL_PIVOT
    ldi r25, VEL_PIVOT
    rcall SET_PWM
    ret

PIVOTE_IZQ_F:
    ldi r16, 0x09
    rcall SET_DIR
    ldi r24, VEL_PIVOT_F
    ldi r25, VEL_PIVOT_F
    rcall SET_PWM
    ret

PIVOTE_DER_F:
    ldi r16, 0x06
    rcall SET_DIR
    ldi r24, VEL_PIVOT_F
    ldi r25, VEL_PIVOT_F
    rcall SET_PWM
    ret

DETENER:
    ldi r16, 0x00
    rcall SET_DIR
    ldi r24, 0
    ldi r25, 0
    rcall SET_PWM
    ret

;#############################################################################
; BOMBA (IDÉNTICO)
;#############################################################################
BOMBA_ON:
    sbi PORTB, BOMBA_ENA
    sbi PORTB, BOMBA_IN1
    cbi PORTB, BOMBA_IN2
    ret

BOMBA_OFF:
    cbi PORTB, BOMBA_ENA
    cbi PORTB, BOMBA_IN1
    cbi PORTB, BOMBA_IN2
    ret

;#############################################################################
; SERVO (IDÉNTICO)
;#############################################################################
MOVER_SERVO:
    push r17
    push r20
    push r22
    push r23
    push r24
    push r25
    push r26
    push r27

    mov r20, r16
    ldi r17, 21
    mul r16, r17
    movw r24, r0
    ldi r17, low(1000)
    ldi r22, high(1000)
    add r24, r17
    adc r25, r22

    mov r26, r20
    clr r27
    ldi r22, 9
    clr r23
    clr r17
MS_DIV9:
    cp  r26, r22
    cpc r27, r23
    brlo MS_DIV9_FIN
    sub r26, r22
    sbc r27, r23
    inc r17
    rjmp MS_DIV9
MS_DIV9_FIN:
    add r24, r17
    clr r16
    adc r25, r16
    sts OCR4AH, r25
    sts OCR4AL, r24

    mov r22, r20
    mov r23, r9
    sub r22, r23
    brsh MS_DIFF_POS
    neg r22
MS_DIFF_POS:
    ldi r17, 5
    mul r22, r17
    movw r24, r0
    ldi r22, 3
    clr r23
    clr r17
MS_DIV3:
    cp  r24, r22
    cpc r25, r23
    brlo MS_DIV3_FIN
    sub r24, r22
    sbc r25, r23
    inc r17
    rjmp MS_DIV3
MS_DIV3_FIN:
    ldi r22, 50
    add r17, r22
    mov r24, r17
    clr r25
    rcall DELAY_MS
    mov r9, r20
    clr r1

    pop r27
    pop r26
    pop r25
    pop r24
    pop r23
    pop r22
    pop r20
    pop r17
    ret

SERVO_CENTRO:
    ldi r16, 90
    rcall MOVER_SERVO
    ret

;#############################################################################
; CALC_ANGULO (IDÉNTICO)
;#############################################################################
CALC_ANGULO:
    push r8
    push r17
    push r20
    push r21
    push r22
    push r23
    push r24
    push r25
    push r26
    push r27

    clr r22
    clr r23
    tst r2
    breq CA_NO_I1
    lds r20, SRAM_LL1_L
    lds r21, SRAM_LL1_H
    ldi r22, low(1023)
    ldi r23, high(1023)
    sub r22, r20
    sbc r23, r21
CA_NO_I1:
    clr r24
    clr r25
    tst r4
    breq CA_NO_I3
    lds r20, SRAM_LL3_L
    lds r21, SRAM_LL3_H
    ldi r24, low(1023)
    ldi r25, high(1023)
    sub r24, r20
    sbc r25, r21
CA_NO_I3:
    clr r26
    clr r27
    tst r3
    breq CA_NO_I2
    lds r20, SRAM_LL2_L
    lds r21, SRAM_LL2_H
    ldi r26, low(1023)
    ldi r27, high(1023)
    sub r26, r20
    sbc r27, r21
CA_NO_I2:
    add r26, r22
    adc r27, r23
    add r26, r24
    adc r27, r25
    mov r16, r26
    or  r16, r27
    brne CA_NONZERO
    ldi r16, 90
    rjmp CA_DONE
CA_NONZERO:
    sub r24, r22
    sbc r25, r23
    clr r8
    sbrs r25, 7
    rjmp CA_DIFF_POS
    ldi r16, 1
    mov r8, r16
    com r24
    com r25
    ldi r16, 1
    add r24, r16
    clr r16
    adc r25, r16
CA_DIFF_POS:
    ldi r17, 50
    mul r24, r17
    mov r20, r0
    mov r21, r1
    mul r25, r17
    add r21, r0
    movw r24, r20
    clr r16
CA_DIV:
    cp  r24, r26
    cpc r25, r27
    brlo CA_DIV_FIN
    sub r24, r26
    sbc r25, r27
    inc r16
    rjmp CA_DIV
CA_DIV_FIN:
    tst r8
    breq CA_POS
    ldi r17, 90
    sub r17, r16
    mov r16, r17
    rjmp CA_CONSTRAIN
CA_POS:
    ldi r17, 90
    add r16, r17
CA_CONSTRAIN:
    cpi r16, 30
    brsh CA_MAX
    ldi r16, 30
    rjmp CA_DONE
CA_MAX:
    cpi r16, 151
    brlo CA_DONE
    ldi r16, 150
CA_DONE:
    clr r1
    pop r27
    pop r26
    pop r25
    pop r24
    pop r23
    pop r22
    pop r21
    pop r20
    pop r17
    pop r8
    ret

;#############################################################################
; EVALUAR_LLAMA / EVALUAR_PRESION (IDÉNTICO)
;#############################################################################
EVALUAR_LLAMA:
    tst r19
    brne EL_CERO
    cpi r18, LLAMA_ON
    brlo EL_UNO
EL_CERO:
    clr r16
    ret
EL_UNO:
    ldi r16, 1
    ret

EVALUAR_PRESION:
    tst r19
    brne EP_UNO
    cpi r18, PRES_ON
    brsh EP_UNO
    cpi r18, PRES_OFF
    brlo EP_CERO
    ret
EP_UNO:
    ldi r16, 1
    ret
EP_CERO:
    clr r16
    ret

READ_ADC:
    mov  r16, r22
    ori  r16, (1<<REFS0)
    sts  ADMUX, r16
    lds  r16, ADCSRA
    ori  r16, (1<<ADSC)
    sts  ADCSRA, r16
RA_W1:
    lds  r16, ADCSRA
    sbrc r16, ADSC
    rjmp RA_W1
    lds  r18, ADCL
    lds  r19, ADCH
    lds  r16, ADCSRA
    ori  r16, (1<<ADSC)
    sts  ADCSRA, r16
RA_W2:
    lds  r16, ADCSRA
    sbrc r16, ADSC
    rjmp RA_W2
    lds  r18, ADCL
    lds  r19, ADCH
    ret

READ_ADC8:
    lds r16, ADCSRB
    ori r16, (1<<MUX5)
    sts ADCSRB, r16
    ldi r16, (1<<REFS0)
    sts ADMUX, r16
    lds r16, ADCSRA
    ori r16, (1<<ADSC)
    sts ADCSRA, r16
RA8_W:
    lds r16, ADCSRA
    sbrc r16, ADSC
    rjmp RA8_W
    lds r18, ADCL
    lds r19, ADCH
    lds r16, ADCSRB
    andi r16, 0xF7            
    sts ADCSRB, r16
    ret

;#############################################################################
; ES_NEGRO (IDÉNTICO)
;#############################################################################
ES_NEGRO:
    ldi  r16, UMBRAL_L
    cp   r18, r16
    ldi  r16, UMBRAL_H
    cpc  r19, r16
    brsh EN_SI
    ldi  r16, 0
    ret
EN_SI:
    ldi  r16, 1
    ret

;#############################################################################
; DELAYS (IDÉNTICO)
;#############################################################################
DELAY_1MS:
    push r26
    push r27
    ldi  r26, low(4000)
    ldi  r27, high(4000)
D1_LP:
    sbiw r26, 1
    brne D1_LP
    pop  r27
    pop  r26
    ret

DELAY_100MS:
    push r20
    ldi  r20, 100
D100_LP:
    rcall DELAY_1MS
    dec  r20
    brne D100_LP
    pop  r20
    ret

DELAY_1S:
    push r20
    ldi  r20, 10
D1S_LP:
    rcall DELAY_100MS
    dec  r20
    brne D1S_LP
    pop  r20
    ret

DELAY_MS:
    push r26
    push r27
DMS_OUTER:
    ldi r27, high(4000)
    ldi r26, low(4000)
DMS_INNER:
    sbiw r26, 1
    brne DMS_INNER
    sbiw r24, 1
    brne DMS_OUTER
    pop r27
    pop r26
    ret