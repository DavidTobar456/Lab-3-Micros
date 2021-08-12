; Archivo: main.s
; Autor: David Antonio Tobar López
    
; Programa: Contador de 10ms con TIMER0, Contador por PORTB con display
;	    Contador de 1s, se reinicia cuando el valor es el mismo que en disp
; Compilador: pic-as(v2.32)
; Hardware: PIC16F887
    
; Fecha de creación: 11 de agosto 2021
; Última fecha de modificación: 12 de agosto de 2021

; PIC16F887 Configuration Bit Settings

; Assembly source line config statements

PROCESSOR   16F887
#include <xc.inc>

; CONFIG1
  CONFIG  FOSC = INTRC_NOCLKOUT ; Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
  CONFIG  WDTE = OFF            ; Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
  CONFIG  PWRTE = OFF           ; Power-up Timer Enable bit (PWRT disabled)
  CONFIG  MCLRE = OFF           ; RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
  CONFIG  CP = OFF              ; Code Protection bit (Program memory code protection is disabled)
  CONFIG  CPD = OFF             ; Data Code Protection bit (Data memory code protection is disabled)
  CONFIG  BOREN = OFF           ; Brown Out Reset Selection bits (BOR disabled)
  CONFIG  IESO = OFF            ; Internal External Switchover bit (Internal/External Switchover mode is disabled)
  CONFIG  FCMEN = OFF           ; Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
  CONFIG  LVP = OFF             ; Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

; CONFIG2
  CONFIG  BOR4V = BOR40V        ; Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
  CONFIG  WRT = OFF             ; Flash Program Memory Self Write Enable bits (Write protection off)

PSECT udata_bank0
    contT:	DS	1;  Se asigna un byte a la variable contador
    contT0:	DS	1;  Se asigna un byte a la variable contador de Timer0
    contd:	DS	2;  Se asignan dos bytes al contador del delay
    sumaComp:	DS	1;  Se asigna un byte a la variable sumaComp
    resta:	DS	1;  Se asigna un byte a la variable resta

PSECT resVet, class = CODE, abs, delta = 2
    ORG 00h
    resetVec:
	PAGESEL main
	goto main

PSECT code, delta = 2, abs
    ORG 100h
    ; ---------------Tabla de valores--------------
    tabla:
	clrf	PCLATH
	bsf	PCLATH, 0
	andlw	0x0F;
	addwf	PCL;
	retlw	00111111B; cero
	retlw	00000110B; uno
	retlw	01011011B; dos
	retlw	01001111B; tres
	retlw	01100110B; cuatro
	retlw	01101101B; cinco
	retlw	01111101B; seis
	retlw	00000111B; siete
	retlw	01111111B; ocho
	retlw	01100111B; nueve
	retlw	01110111B; A
	retlw	01111100B; b
	retlw	00111001B; C
	retlw	01011110B; D
	retlw	01111001B; E
	retlw	01110001B; F
	
    ; ---------------Configuración-----------------
    main:
	call	config_timer0
	call	config_io
	call	config_osc
	BANKSEL PORTA
	
    ; ---------------Loop--------------------------
    loop:
	btfsc	T0IF
	call	inc_portc
	btfsc	PORTB,0
	call	inc_contador
	btfsc	PORTB,1
	call	dec_contador
	call	ver_seg
	call	ver_igualdad
    goto loop
	
    ; ---------------Sub Rutinas-------------------
    config_io:
	BANKSEL ANSEL
	clrf	ANSEL;	Se configuran los pines de ANSEL como I/O digitales
	clrf	ANSELH;	Se configuran los pines de ANSELH como I/O digitales
	
	BANKSEL TRISA
	clrf	TRISA;	Se configuran los pines RA como salidas
	clrf	TRISC;	Se configuran los pines RB como salidas
	clrf	TRISD;	Se configuran los pines RD como salidas
	clrf	TRISE;	Se configuran los pines RE como salidas
	bsf	TRISB,0;Se configura RB0 como entrada
	bsf	TRISB,1;Se configura RB1 como entrada
	
	BANKSEL PORTA
	clrf	PORTA;	Se limpian los valores en PORTA
	clrf	PORTC;	Se limpian los valores en PORTC
	clrf	PORTD;	Se limpian los valores en PORTD
	clrf	PORTE;	Se limpian los valores en PORTE
    return
    
    config_osc:
	BANKSEL OSCCON
	bsf	IRCF2
	bcf	IRCF1
	bcf	IRCF0;	Se configura el oscilador a 1MHz
	bsf	SCS;	Se selecciona el oscilador interno
    return
    
    config_timer0:
	banksel	TRISA
	bcf	T0CS;	El módulo TIMER0 corre con el reloj interno
	bcf	PSA;	El pre escalador se asigna a TIMER0
	bsf	PS2
	bsf	PS1
	bsf	PS0;	Se asignó  el prescaler de 256 al timer
	banksel PORTA
	call	reset_timer0
    return
    
    reset_timer0:
	movlw	159 ;	El valor que se asignará a TIMER0 para lograr 100ms
	movwf	TMR0;
	bcf	T0IF
    return
    
    inc_portc:
	incf	PORTC;	Se incrementa el valor en PORTA
	incf	contT0;	Se incrementa el valor en contT0
	call	reset_timer0
	btfsc	PORTC,4
	clrf	PORTC
    return
    
    inc_contador:
	call	delay
	btfsc	PORTB,0;    Valida si ya se dejó de apachar el push button
	goto	$-1;	    Regresa a la instrucción anterior
	incf	contT
	btfsc	contT,4;    Se verifica que no haya pasado los 4 bits
	clrf	contT;	    Si hay overflow, se resetea el contador
	movf	contT,w
	call	tabla
	movwf	PORTA
    return
    
    dec_contador:
	call	delay
	btfsc	PORTB,1;    Valida si ya se dejó de apachar el push button
	goto	$-1;	    Regresa a la instrucción anterior
	decf	contT
	btfsc	contT,4;    Se verifica que no haya pasado los 4 bits
	call	res_contT;  Si hay overflow, se resetea el contador
	movf	contT,w
	call	tabla
	movwf	PORTA
    return
    
    res_contT:
	clrf	contT
	bsf	contT,0
	bsf	contT,1
	bsf	contT,2
	bsf	contT,3
    return
    
    ver_seg:
	movlw	00000110B
	addwf	contT0,w;	    Se suma el valor en comp a contT0
	movwf	sumaComp;   Se guarda el valor de la suma en sumaComp
	btfsc	sumaComp,4; Se verifica si ya han pasado 10 conteos de 100ms
	call	inc_seg;    Se llama a la función inc_seg
    return
    
    inc_seg:
	clrf	contT0;	    Se resetea el valor en contT0
	incf	PORTD;	    Se incrementa PORTD
	btfsc	PORTD,4;    Se verifica que no haya habido overflow en PORTD
	clrf	PORTD;	    Si ha habido overflow se resetea PORTD
    return
    
    ver_igualdad:
	movf	contT,w
	subwf	PORTD,w
	movwf	resta;	    Se asigna el valor de la resta a la variable
	btfsc	resta,0
	goto	$+17
	btfsc	resta,1
	goto	$+15
	btfsc	resta,2
	goto	$+13
	btfsc	resta,3
	goto	$+11
	btfsc	resta,4
	goto	$+9
	btfsc	resta,5
	goto	$+7
	btfsc	resta,6
	goto	$+5
	btfsc	resta,7
	goto	$+3
	call	res_TimerSeg
	call	cambiar_estado
    return
    
    res_TimerSeg:
	clrf	PORTD
	clrf	contT0
    return
    
    cambiar_estado:
	movlw	001B
	xorwf	PORTE,1
    return
    
    delay:
	movlw	150;	Se asigna un valor inicial de 150 al contador del delay
	movwf	contd;	Se asigna el valor a la variable contd
	decfsz	contd;	Se decrementa contd hasta llegar a cero, en cero skipea
	goto	$-1;	Se regresa a la instrucción anterior
    return
    
END


