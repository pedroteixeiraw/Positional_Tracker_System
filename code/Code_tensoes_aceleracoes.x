; Código que permite configurar vários PORT, o clock, o ADC, e as interrupções
; implementa também a leitura do ADC utilizando um mecanismo baseado em interrupções.
;    
; Configura também uma interrupção externa ligada ao switch que liga ao pino RB4
; e acende um LED em resposta a carregar no switch.
; Configura também o envio do valor do ADC pela porta série usando a USART  
; (usa o pino RC4 como TX da porta série) que liga/desliga o envio por interrupção 
; gerada pelo S1.
    
    
#include <pic18f47q10.inc>
#include <xc.inc>

CONFIG FEXTOSC=0b100	;Deactivate external oscillator (to allow write to RA7).
CONFIG CSWEN=0b1	;Allow editing NDIV and NOSC for CLK config. 
CONFIG WDTE=OFF		;Required to avoid WDT restarting micro all the time.
    
#define adc_read 0
#define count 1
#define protocolo 2
#define X 25		;25 for baud 19200 for CLK at 32MHz 


PSECT code
ORG 0x0000
    goto start		;The start of the code (position 0x00 in the memory) 
			;goes to function 'start'
 
ORG 0x0008
    goto CMEMS		;Position 0x08 is attributed to interruption response.

ORG 0x0030		;Main program starts at position 0x30
start:     
    
    ;===============
    ;CONFIGURE PORTA
    ;===============
    BANKSEL LATA	;Seleciona o Banco com o registo LATA.
    CLRF LATA,1		;Coloca todos os pinos de saída do porto A a "0".
    MOVLW 0b00000000	;Configura os RA<7:0> como saida. 
			;(Define direção dos Pinos. 1 = In, 0 = Out.)
    BANKSEL TRISA	;Seleciona o Banco com o registo TRISA.
    MOVWF TRISA,1	
    MOVLW 0b00000000	;Configura os RA<7:0> como saida digital.
    BANKSEL ANSELA	;Seleciona o Banco com o registo ANSELA.
    MOVWF ANSELA,1	
    
    ;===============
    ;CONFIGURE CLOCK
    ;===============
    BANKSEL OSCCON1	;Seleciona o Banco com o registo OSCCON1.
    MOVLW 0b01100000	;NOSC=0110 (Oscilador Interno de Alta Frequência).
			;(HFINTOSC).
			;NDIV=0000 (Divider=1, CLK divided by 1).
    MOVWF OSCCON1,1	
    BANKSEL OSCFRQ	;Seleciona o Banco com o registo OSCFRQ.
    MOVLW 0b0000110	;HFFRQ=0110 -> CLK=32 MHz => 32/1<- NDIV.
    MOVWF OSCFRQ,1
    BANKSEL OSCEN	;Seleciona o Banco com o registo OSCEN.
    MOVLW 0b01000000	;HFINTOSC Enabled @freq=OSCFRQ ativo.
    MOVWF OSCEN,1	
   
    ;===============
    ;CONFIGURE PORTB
    ;===============
    BANKSEL LATB	;Seleciona o Banco com o registo LATB.
    CLRF LATB,1		;Coloca todos os pinos de saída do porto B a "0".
    BANKSEL TRISB	;Seleciona o Banco com o registo TRISB.
    CLRF TRISB,1	;Coloca todos os pinos de saída do porto A a "0".
    BSF	 TRISB,4	;Excepto RB4, que será usado como botão de fonte de
			;interrupcão.
    BANKSEL ANSELB	;Seleciona o Banco com o registo ANSELB.
    CLRF ANSELB,1	;Configura os RB<7:0> como saida digital.
    BANKSEL RB7PPS	;Seleciona o Banco com o registo RB7PPS.
			;Selecionar os INPUTS e OUTPUTS dos respectivos perifericos.
    MOVLW 0x14		;CLKR só pode ter o OUTPUt direcionado para PORTB ou C.
    MOVWF RB7PPS	;CLKR_output no pino RB7.     
    BANKSEL INT0PPS	;Seleciona o Banco com o registo INT0PPS.
    MOVLW 0x0C		;0x0C for RB4.
    MOVWF INT0PPS	;INT0_input ligado ao pino RB4.
    
    ;===============
    ;CONFIGURE PORTD
    ;===============
    BANKSEL LATD	;Seleciona o Banco com o registo LATD.
    CLRF LATD,1		;Coloca todos os pinos de saída do porto D a "0".
    BANKSEL TRISD	;Seleciona o Banco com o registo TRISD.
    MOVLW 0b11100000	;Configura os RD<7:5> como entradas e 
			;RA<4:0> como saídas.
    MOVWF TRISD,1
    BANKSEL ANSELD	;Seleciona o Banco com o registo ANSELD.
    MOVLW 0b11100000	;Configura os RD<7:5> como entrada analógica
			;RD<4:0> como saídas digitais.
    MOVWF ANSELD,1 
    
    ;===============
    ;CONFIGURE PORTC
    ;===============
    BANKSEL LATC	;Seleciona o Banco com o registo LATC.
    CLRF LATC,1		;Coloca todos os pinos de saída do porto C a "0".
    BANKSEL TRISC	;Seleciona o Banco com o registo TRISC.
    MOVLW 0b10000000	;Configura os RC<7> como entrada e RC<6:0> como saida. 
    MOVWF TRISC,1	;Todos os pinos são de output execto o RC7 que é input.
    BANKSEL ANSELC	;Seleciona o Banco com o registo ANSELC.
    CLRF ANSELC,1	;Configura os RC<7:0> pinos digital.
    BANKSEL RC4PPS	;Seleciona o Banco com o registo RC4PPS.
    MOVLW 0x09		;0x09 for EUSART1(TX/CK).
    MOVWF RC4PPS	;EUSART1(TX/CK)_output in RC4.
    MOVLW 0x17		;EUSART1_Receive_input ligado ao RC7.
    MOVWF RX1PPS
    
    ;================
    ;CONFIGURE TIMER0
    ;================
    BANKSEL T0CON0	;Seleciona o Banco com o registo T0CON0.
    MOVLW 0b00000001	;Modulo desativo, TMR0 é 8bit, 1:2 PostScaler.
    MOVWF T0CON0,1
    BANKSEL T0CON1	;Seleciona o Banco com o registo T0CON1.
    MOVLW 0b01001011	;CLK=Fosc/4, TMRO sincrono em Fosc/4. 
			;PreScaler =1011 (2048).
			;CLK=32M/4/2048/2/7=279.01Hz.
			;4 ms period between Timer interruptions.
    MOVWF T0CON1,1
    BANKSEL TMR0L	;Seleciona o Banco com o registo TMR0L.
    CLRF TMR0L		;Coloca os 8 bits menos significativos do contador 
			;do TIMER0 a 0 (Limpa o nosso contador.)
    BANKSEL TMR0H	;Seleciona o Banco com o registo TMR0H. (Counter)
    MOVLW 0b0000111	;Seleciona 7 como valor a comparar. (Comparater)
    MOVWF TMR0H
   
    ;=============
    ;CONFIGURE ADC
    ;=============
    BANKSEL ADREF	;Seleciona o Banco com o registo ADREF.
    MOVLW 0b00000000	;V_ref(-) = AV_ss, V_ref(+) = V_dd.
    MOVWF ADREF,1	
    BANKSEL ADCLK	;Seleciona o Banco com o registo ADCLK.
    MOVLW 0b00001111	;ADCS=001111, ADC_CLK = 32 MHz/32 = 1 MHz. 
			;1us para converter 1 bit, 11.5us para 10 bits.
    MOVWF ADCLK,1
    BANKSEL ADCON0	;Seleciona o Banco com o registo ADCON0.
    MOVLW 0b00000000    ;ADC desativo, ADC_CLK=Fosc/div, results left adjusted
			;Conversion not in progress.
    MOVWF ADCON0,1
 
    ;========================
    ;configure serial port
    ;========================
    movlw X		;Move X to Baud Rate Generator.
			;Expected a baud of 19200.  
    BANKSEL SP1BRGL	;Seleciona o Banco com o registo SP1BRGL.
    movwf SP1BRGL	;Passa o valor de X(25) para o registo SP1BRGL.
    movlw 0x00		;Passa o valor de 0x00 para o registo SP1BRGH.
    movwf SP1BRGH	; "1" for USART 1, since we have 2 USART available.
    movlw 0b10100000	;8 data bits, TX enabled, master clock selected.
    BANKSEL TX1STA
    movwf TX1STA	;Low speed SPBRG mode.
    movlw 0b10010000	;Ativar  o usart, ativar a recepçao,8 bits.
    BANKSEL RC1STA
    movwf RC1STA	;Receiver enabled
    MOVLW 0b00000000
    MOVWF BAUD1CON
    
    ;=================
    ;ENABLE INTERRUPTS
    ;=================
    BANKSEL PIR0
    BCF PIR0, 5		;clear timer interrupt flag
    BANKSEL PIR1
    BCF PIR1,0		;clear ADC interrupt flag
    BANKSEL PIE0
    BSF PIE0,5		;enable timer int
    BSF PIE0,0		;enable INT0
    BCF PIR0,0		;clear INT0 interrupt flag
    BANKSEL PIE1
    BSF PIE1,0		;enable adc int
    BANKSEL PIE3
    BSF PIE3,5		;ativar a interrupção de recebimento
    BANKSEL INTCON
    BSF INTCON,5	;liga a prioridade das interrupções
    BSF INTCON,7	;enable peripheral interruptions
    BANKSEL IPR0
    MOVLW 0b00100001	;timer e INT0 alta prioridade
    MOVWF IPR0
    BANKSEL IPR1
    MOVLW 0b00000001	;adc alta prioridade
    MOVWF IPR1
    BANKSEL IPR3
    MOVLW 0b00000000	;interrupção de recebimento de baixa prioridade
    MOVWF IPR3
    BANKSEL T0CON0
    BANKSEL ADCON0  
    BSF ADCON0,7	;ENABLE ADC
    BSF INTCON,6	;enable global interruptions - do this after 
			;configurations are SET

;========================
;Main code (do nothing)
;========================
main:
    nop
    nop
    BCF PORTA, 6
    nop
    goto main  

CMEMS:
    BANKSEL PIR0
    BTFSC PIR0, 5	;(se for zero salta)
			;check if the timer0 interrupt flag is set. 
			;If so, go to timer0_int_handler. If not, skip.
    goto timer0_int_handler
    
    BTFSC PIR0, 0	;check if the INT0IF. 
			;If so, go to INT0_int_handler. 
			;If not, skip.
    goto INT0_int_handler
    
    BANKSEL PIR1
    BTFSC PIR1,0	;check if the ADC interrupt flag is set. 
			;If so, go to adc_int_handler. 
			;If not, skip.
    goto adc_int_handler
    
timer0_int_handler:
    BANKSEL PIR0
    BCF PIR0,5		    ;clear timer_int flag.
    canal1:
         MOVLW 0b00000001
         CPFSEQ count	    ;se for igual ao W skip.
         GOTO canal2	    ;Vai para o canal Y.
	 BANKSEL ADPCH	
         MOVLW 0b00011101   ;Set RD5 as ADC, input X.
         MOVWF ADPCH,1
	 MOVLW 0b00000010   ;Passa o canal Y para count.
	 MOVWF count
	 MOVLW 0b11111111   ;Identificador do canal X.
	 MOVWF protocolo    ;Coloca identificador no protocolo.
         CALL SENDPROTOCOLO ;Envia protocolo.
	 GOTO fim
    canal2:
         MOVLW 0b00000010
         CPFSEQ count	    ;se for igual ao W skip.
         GOTO canal3	    ;Vai para o canal Z.
         BANKSEL ADPCH	    
         MOVLW 0b00011110   ;Set RD6 as ADC,input Y.
         MOVWF ADPCH,1	    
	 MOVLW 0b00000011   ;Passa o canal Z para count.
	 MOVWF count
	 MOVLW 0b11111110   ;Identificador do canal Y.
	 MOVWF protocolo    ;Coloca identificador no protocolo.
	 CALL SENDPROTOCOLO ;Envia protocolo.
         GOTO fim 
    canal3:
         MOVLW 0b00000011
         CPFSEQ count	    ;Se for igual ao W skip.
         nop
         BANKSEL ADPCH
         MOVLW 0b00011111   ;Set RD7 as ADC input z.
         MOVWF ADPCH,1	    
	 MOVLW 0b00000001   ;Passa o canal X para count.
	 MOVWF count
	 MOVLW 0b11111101   ;Identificador do canal Z.
	 MOVWF protocolo    ;Coloca identificador no protocolo.
	 CALL SENDPROTOCOLO ;Envia protocolo.
         GOTO fim
    fim:
    BANKSEL ADCON0
    BSF ADCON0,0	    ;Inicia conversao do ADC.   
    BTG PORTA,4
    RETFIE		    ;return from interruption
    
INT0_int_handler: 
    BANKSEL PIR0
    BCF PIR0, 0		    ;Clear the INT0 intrrupt test bit
    BANKSEL PORTA
    BTG PORTA,5		    ;Toggle the LED state (ON if OFF, OFF if ON).
    BTG T0CON0,7	    ;Toggle TIMER0 ON<->OFF.
    MOVLW 0b00000001	    ;Passa o canal X para count.
    MOVWF count		    
    RETFIE		    ;Return from interruption.
    
adc_int_handler:
    BANKSEL ADRESH
    MOVFF ADRESH, adc_read  ;Copy the 8 MSBs from the ADC conversion to a variable.
    CALL SENDCHAR	    ;The value to send is in address adc_read
    BANKSEL PIR1
    BCF PIR1,0		    ;clear the ADC interrupt flag.
    RETFIE		    ;Return from interruption.

;============================
;Send a char using USART1
;============================
SENDCHAR:
    movlw 0x2F		    ;number to send by usart 1
    movwf 0x0F
    BANKSEL PIR3
    btfss PIR3,4	    ;TXIF| Check, is TX buffer full?
    bra SENDCHAR	    ;IF not THEN try again.
    BANKSEL TX1REG
    movff adc_read,TX1REG   ;ELSE copy to USART TX register.
    RETURN

SENDPROTOCOLO:
    movlw 0x2F		    ;number to send by usart 1
    movwf 0x0F
    BANKSEL PIR3
    btfss PIR3,4	    ;TXIF ; Check, is TX buffer full?
    bra SENDPROTOCOLO	    ;IF not THEN try again
    BANKSEL TX1REG
    movff protocolo,TX1REG  ;ELSE copy to USART TX register
    RETURN      
end