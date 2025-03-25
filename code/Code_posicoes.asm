; código exemplo que permite configurar vários PORT, o clock, o ADC, e as interrupções
; implementa também a leitura do ADC utilizando um mecanismo baseado em interrupções
;    
; configura também uma interrupção externa ligada ao switch que liga ao pino RB4
; e acende um LED em resposta a carregar no switch
;configura também o envio do valor do ADC pela porta série usando a USART (usa 
; o pino RC4 como TX da porta série) que 
;liga/desliga o envio por interrupção gerada pelo S1
; by P. M. Mendes & Hugo Dinis


; NOTE BEM
; as soluções apresentasas são meramente indicativas de como se podem implementar
; existem (quase) sempre outras alternativas e outras formas de combinar as várias 
; metodologias apresentadas 
    
; Para testar o funcionamento da INT externa em modo simulador, recomenda-se o uso
; do "windows-> Simulator->stimulus" configurando o pino desejado como trigger
; da interrupção programada
    
    
#include <pic18f47q10.inc>
#include <xc.inc>

;CONFIG CLKOUTEN=0b0	;to place clock output no RA6?
CONFIG FEXTOSC=0b100	;deactivate external oscillator (to allow write to RA7)
CONFIG CSWEN=0b1	;allow editing NDIV and NOSC for CLK config    
CONFIG WDTE=OFF		;required to avoid WDT restarting micro all the time
    
#define adc_read 0
#define count 1
#define protocolo 2
;config OSC=INTIO67, WDT=OFF
#define BAUD 9600 ; For example 9600 baud rate
#define XTAL 32 ;  for 32MHz crystal
#define X1 ((XTAL*1000000)/(64*BAUD))-1

#define X 25 ; 51 for baud 9600 for CLK at 32MHz 
  

PSECT code
ORG 0x0000
    goto start  ;the start of the code (position 0x00 in the memory) goes to function 'start'
 
ORG 0x0008
    goto CMEMS    ;position 0x08 is attributed to interruption response

    

ORG 0x0030   ;main program starts at position 0x30
start:     
    
    ;===============
    ;CONFIGURE PORTA
    ;===============
    BANKSEL LATA
    CLRF LATA,1 ; Set all LatchA bits to zero
    MOVLW 0b00000001 
    BANKSEL TRISA
    MOVWF TRISA,1 ;defines the pin direction. 0=out, 1=in. RA0 connects to potenciometer. All other are output pins
    MOVLW 0b00000001
    BANKSEL ANSELA
    MOVWF ANSELA,1 ;analog select. RA0 connects to potenciometer. The others are digital pins
   ; BTG PORTA, 4
    
    ;===============
    ;CONFIGURE CLOCK
    ;===============
    BANKSEL OSCCON1
    MOVLW 0b01100000  ;NOSC=0110 (internal high speed osc), 0101 for LF
    ;MOVLW 0b01011001
    MOVWF OSCCON1,1   ;NDIV=0000 (divider=1, clk divided by 1) NDIV 1001 div by 512
    BANKSEL OSCFRQ
    ;MOVLW 0b0000000
    MOVLW 0b0000110 ; HFFRQ 0110 -> clk= 32 MHz => 32/1<- NDIV / 0000-> clk 1MHz
    MOVWF OSCFRQ,1
    BANKSEL OSCEN
    MOVLW 0b01000000 ;internal clock @freq=OSCFRQ ativo
    MOVWF OSCEN,1
    
    ; to observe CLK at a pin, use of CLKR
    BANKSEL CLKRCON
    MOVLW  0b10010111    ; enable reference clock module, 50% dutyclycle, divide by 128
    MOVWF  CLKRCON
    BANKSEL CLKRCLK
    MOVLW  0b00000000    ; select reference clock Fosc
    MOVWF  CLKRCLK
    
    
    ;===============
    ;CONFIGURE PORTB
    ;===============
    BANKSEL LATB
    CLRF LATB,1 ; Set all LatchB bits to zero
    BANKSEL TRISB
    CLRF TRISB,1 ;All pins are output
    BSF	 TRISB,4 ; except RB4, that will be used as interrupt source - button
    BANKSEL ANSELB
    CLRF ANSELB,1 ;All digital pins
    BANKSEL RB0PPS   ;apartir daqui vai-se defenir os inputs e outputs dos respectivos perifericos
    MOVLW 0x14	    ; CLKR only conect to B/C ports
    MOVWF RB7PPS   ;place the CLKR in RB7           ->13 TMR0    para que serviu esta linha?     
    BANKSEL INT0PPS
    MOVLW 0x0C	   ; 0x0C for RB4
    MOVWF INT0PPS  ; liga o RB4 à INT0
    
    ;===============
    ;CONFIGURE PORTD
    ;===============
    BANKSEL LATD
    CLRF LATD,1 ; Set all LatchD bits to zero
    BANKSEL TRISD
    MOVLW 0b11100000
    MOVWF TRISD,1 
    BANKSEL ANSELD
    MOVLW 0b11100000
    MOVWF ANSELD,1 
    
    
    ;===============
    ;CONFIGURE PORTC
    ;===============
    BANKSEL LATC
    CLRF LATC,1 ; Set all LatchD bits to zero
    BANKSEL TRISC
    MOVLW 0b10000000
    MOVWF TRISC,1 ; todos os pinos são de output execto o RC7 que é input
    ;BSF  TRISC,4 ; ???except RC4, that will be used as TX - microchip specifies RX and TX as input pin
    BANKSEL ANSELC
    CLRF ANSELC,1 ;All digital pins
    BANKSEL RC4PPS
    MOVLW 0x09  ;09
    MOVWF RC4PPS   ;place the EUSART1 (TX/CK) in RC4  ->09
    MOVLW 0b00010111  ;escolher o pino RC7 para o RX
    MOVWF RX1PPS
    
    
    ;================
    ;CONFIGURE TIMER0
    ;================
    BANKSEL T0CON0
    MOVLW 0b00000011  ;1:2 postscaler, 8bit
    MOVWF T0CON0,1
    BANKSEL T0CON1
    MOVLW 0b01001100   ;clock=Fosc/4, ASYNC, prescaler =1011 2048   -> CLK=32M/4/32768, S bits until overflow = YY ms period between Timer interruptions
    MOVWF T0CON1,1
    BANKSEL TMR0L   
    CLRF TMR0L  ;clear the timer's lowest 8 bits (the counter)
    BANKSEL TMR0H
    MOVLW 0b00100000   ;set the timer's highest 8 bits (the comparator)
    MOVWF TMR0H
   
    
    ;=============
    ;CONFIGURE ADC
    ;=============
    ;BANKSEL ADPCH
    ;MOVLW 0b00000000   ;set RA0 as ADC input
    ;MOVWF ADPCH,1
    BANKSEL ADREF
    MOVLW 0b00000000  ;Vref set to vdd and vss
    MOVWF ADREF,1
    BANKSEL ADCLK
    MOVLW 0b0000111   ;1 us para converter 1 bit, 11.5us para 10 bits
    MOVWF ADCLK,1
    BANKSEL ADCON0
    MOVLW 0b00000000    ;results left adjusted, clock=Fosc/div, non-continuous operation
    MOVWF ADCON0,1
 
    ;========================
    ;configure serial port
    ;========================

    movlw X	    ; Move X to Baud Rate Generator -> expected a baud of 
    BANKSEL SP1BRGL
    movwf SP1BRGL
    movlw 0x00
    movwf SP1BRGH	; "1" for USART 1, since we have 2 USART available
    movlw 0b10100000	; 8 data bits, TX enabled, master clock selected
    BANKSEL TX1STA
    movwf TX1STA	; Low speed SPBRG mode
    movlw 0b10010000	; ativar  o usart, ativar a recepçao,8 bits,
    BANKSEL RC1STA
    movwf RC1STA ; Receiver enabled
    MOVLW 0b00000000
    MOVWF BAUD1CON
    ;=================
    ;ENABLE INTERRUPTS
    ;=================
   
    BANKSEL PIR0
    BCF PIR0, 5 ;clear timer interrupt flag
    BANKSEL PIR1
    BCF PIR1,0  ;clear ADC interrupt flag
    BANKSEL PIE0
    BSF PIE0,5  ;enable timer int
    BSF PIE0,0   ; enable INT0
    BCF PIR0,0   ;clear INT0 interrupt flag
    BANKSEL PIE1
    BSF PIE1,0  ;enable adc int
    BANKSEL PIE3
    BSF PIE3,5 ; ativar a interrupção de recebimento
    BANKSEL INTCON
    BSF INTCON,5 ;liga a prioridade das interrupções
    BSF INTCON,7  ;enable peripheral interruptions
    BANKSEL IPR0
    MOVLW 0b00100001 ; timer e INT0 alta prioridade
    MOVWF IPR0
    BANKSEL IPR1
    MOVLW 0b00000001 ; adc alta prioridade
    MOVWF IPR1
    BANKSEL IPR3
    MOVLW 0b00000000  ; interrupção de recebimento de baixa prioridade
    MOVWF IPR3
    BANKSEL T0CON0
   ; BSF T0CON0,7   ;start timer 0
    BANKSEL ADCON0  
    BSF ADCON0,7   ;ENABLE ADC
    ;BANKSEL PIE3
    ;BSF PIE3,4; TX1IE enable UART TX1 interrupt

    BSF INTCON,6  ;enable global interruptions - do this after 
		  ; configurations are SET
;========================
;Main code (do nothing)
;========================
main:
    
    nop
    nop
    ;CALL SENDCHAR
    BCF PORTA, 6
    
    nop
    goto main  

;===========================
;Handler for an interruption
;===========================
;baixa_int:
    ;BANKSEL PIR3
    ;BTFSC PIR3,5
    ;GOTO acender_led
    
;acender_led:
    ;MOVF RC1REG,0
    ;MOVWF count2
    ;BTG PORTA,4
    ;ver1:
       ;MOVLW 0b00000001
       ;CPFSEQ count2
       ;GOTO ver0
       ;BSF PORTA,4
       ;GOTO final
    ;ver0:
       ;MOVLW 0b00000000 
       ;CPFSEQ count2
       ;nop
       ;BCF PORTA,4
       ;GOTO final
    ;final:
       ;BSF PORTA,4
    ;GOTO INT0_int_handler
    ;RETFIE   








CMEMS:
    ;BTG  LATA, 4
    BANKSEL PIR0
    BTFSC PIR0, 5 ;(se for zero salta)check if the timer0 interrupt flag is set. If so, go to timer0_int_handler. If not, skip.
    goto timer0_int_handler
    
    BTFSC PIR0, 0 ;check if the INT0IF. If so, go to INT0_int_handler. If not, skip.
    goto INT0_int_handler
    
    BANKSEL PIR1
    BTFSC PIR1,0   ;check if the ADC interrupt flag is set. If so, go to adc_int_handler. If not, skip.
    goto adc_int_handler
    
    
    ;BANKSEL PIR3
    ;BTFSC PIR3,5
    ;GOTO acender_led
    ;RETFIE  ;return from interruption to where the code was before the int happened

    
timer0_int_handler:
    BANKSEL PIR0
    BCF PIR0,5 ;clear timer_int flag
    canal1:
         MOVLW 0b00000001
         CPFSEQ count ; se for igual ao W skip
         GOTO canal2
	 BANKSEL ADPCH
         MOVLW 0b00011101   ;set RD5 as ADC input X
         MOVWF ADPCH,1
	 MOVLW 0b00000010
	 MOVWF count
	 MOVLW 0b11111111
	 MOVWF protocolo
         CALL SENDPROTOCOLO
	 GOTO fim
    canal2:
         MOVLW 0b00000010
         CPFSEQ count ; se for igual ao W skip
         GOTO canal3
         BANKSEL ADPCH
         MOVLW 0b00011110    ;set RD6 as ADC input y
         MOVWF ADPCH,1
	 MOVLW 0b00000011
	 MOVWF count
	 MOVLW 0b11111110
	 MOVWF protocolo
	 CALL SENDPROTOCOLO
         GOTO fim
	 
    canal3:
         MOVLW 0b00000011
         CPFSEQ count ; se for igual ao W skip
         nop
         BANKSEL ADPCH
         MOVLW 0b00011111    ;set RD7 as ADC input z
         MOVWF ADPCH,1
	 MOVLW 0b00000001
	 MOVWF count
	 MOVLW 0b11111101
	 MOVWF protocolo
	 CALL SENDPROTOCOLO
         GOTO fim
    
    fim:
    BANKSEL ADCON0
    BSF ADCON0,0 ;start ADC conversion   
    BTG PORTA,4
    
    RETFIE  ;return from interruption
    
INT0_int_handler: 
    BANKSEL PIR0
    BCF PIR0, 0   ; clear the INT0 intrrupt test bit
    BANKSEL PORTA
    BTG PORTA,5    ; toggle the LED state (ON if OFF, OFF if ON)
    ;CALL SENDCHAR
    BTG T0CON0,7   ;toggle TIMER0 ON<->OFF
    MOVLW 0b00000001
    MOVWF count
    
    RETFIE  ;return from interruption
    
adc_int_handler:
    BANKSEL ADRESH
    MOVFF ADRESH, adc_read  ;copy the 8 MSBs from the ADC conversion to a variable
    CALL SENDCHAR ; The value to send is in address adc_read
    BANKSEL PIR1
    BCF PIR1,0   ;clear the ADC interrupt flag
    RETFIE  ;return from interruption

;============================
;Send a char using USART1
;============================
    
SENDCHAR:
    movlw 0x2F ; number to send by usart 1
    movwf 0x0F
    BANKSEL PIR3
    btfss PIR3,4     ;TXIF ; Check, is TX buffer full?
    bra SENDCHAR ; IF not THEN try again
    BANKSEL TX1REG
    movff adc_read,TX1REG; ELSE copy to USART TX register
    
    ;BTG PORTA, 7
    RETURN

SENDPROTOCOLO:
    movlw 0x2F ; number to send by usart 1
    movwf 0x0F
    BANKSEL PIR3
    btfss PIR3,4     ;TXIF ; Check, is TX buffer full?
    bra SENDPROTOCOLO ; IF not THEN try again
    BANKSEL TX1REG
    movff protocolo,TX1REG; ELSE copy to USART TX register
    
    RETURN   
    
end