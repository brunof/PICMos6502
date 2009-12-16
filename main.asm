;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;made by Bruno Fascendini(bipus24hs@hotmail.com) for uControl(http://www.ucontrol.com.ar)
;I started programming this lines in: 12/12/2009
;project main URLs: 
;http://www.ucontrol.com.ar/forosmf/proyectos-con-pic/ordenador-basado-en-un-pic-de-40-pines-(en-limpio)/
;discuss:
;http://www.ucontrol.com.ar/forosmf/proyectos-con-pic/ordenador-basado-en-un-pic-de-40-pines/
;PLEASE, DO NOT DELETE THIS CREDITS!!
;******************************************************************************
;******************************************************************************
;******************************************************************************
;NAME: emul6502
;PART OF: 
;DESCRIPTION:
;EMULATES A MOS6502 uP with a PIC18F4550
;
;
;******************************************************************************
;******************************************************************************
;******************************************************************************

	LIST P=18F4550				;Directiva para definir el procesador
	#include <P18F4550.INC>		;Definicion de SFRs para el procesador

;******************************************************************************
;Bits de Configuracion 
;********	Configuracion del Oscilador	**********
    ;CONFIG	FOSC = INTOSC_HS
	;CONFIG	PLLDIV = 5
	;CONFIG	CPUDIV = OSC1_PLL2
;********	Otros bits de configuracion	**********
	CONFIG	PWRT = ON					;PWRT habilitado
	CONFIG	BOR  = OFF					;Brown out resete deshabilitado 
	CONFIG	WDT	 = OFF					;Watch dog deshabilitado	
	CONFIG  MCLRE = OFF					;MCLR como entrada
	CONFIG	PBADEN = ON					;Todos los pines como entradas analogicas
	CONFIG	LVP	= OFF					;Programacion en bajo voltaje apagado

;*********	Bits de proteccion	******************
	CONFIG	CP0	= OFF					;los bloques del codigo de programa
	CONFIG	CP1	= OFF					;no estan protegidos
	CONFIG	CP2	= OFF
	CONFIG	CP3	= OFF
	CONFIG	CPB	= OFF					;Sector Boot no esta protegido
	CONFIG	CPD	= OFF					;La EEPROM no esta protegida
;******************************************************************************
; Definicion de variables
	UDATA
;emul fisical registers
emulPCL res 1		;Stores Program Counter Low Byte
emulPCH res 1		;Stores Program Counter High Byte

emulAC	res 1		;Acumulator
emulX	res 1		;X register
emulY	res 1		;Y register
emulSR	res 1		;Status Register
emulSP	res 1		;Stack Register

;emul aux registers
emulOP	res 3		;stores actual OPCODE+CMD readed(3 bytes maximum for 6502)
counter	res 1		;a counter to read X bytes of OPT+CMD :)

aux1	res	1		;aux variables for addressing modes and instruction  
auxOPL	res 1		;auxiliar table calculation
auxOPH	res 1		;auxiliar table calculation

;aux for writing and reading routines
ramL	res	1
ramH	res	1
data8	res	1

;RS232
aux232	res 1
cont232	res 1

;DelayUs
del1	res 1

emulTempPC 	res 2	;stores aux PC counter

;auxiliar variables
auxDir		res	2
WREG_TEMP 	res 1			;variable usada para salvar contexto
STATUS_TEMP res 1		;variable usada para salvar contexto
BSR_TEMP 	res 1			;variable usada para salvar contexto

;******************************************************************************
; Definicion de bits constantes
;******************************************************************************
#define	BAUDRATE		.19200								;indicate 9600 baudrate for RS-232
#define	DELAY232		(.800000/BAUDRATE)					;get quantity of uS for each Baud

; Definicion de PINES!!!
	;#define	emul_PIN_NC1	PORTA,4	
	;#define	emul_PIN_NC2	PORTE,0
#define	emul_PIN_MODE	PORTA,5		
#define	emul_PIN_WR		PORTE,1
#define	emul_PIN_RD		PORTE,2
#define	emul_PIN_WAIT	PORTC,0
#define	emul_PIN_RAMROM	PORTC,1
#define	emul_PIN_INT	PORTC,2

#define	emul_PIN_A0		PORTA,0
#define	emul_PIN_A1		PORTA,1
#define	emul_PIN_A2		PORTA,2
#define	emul_PIN_A3		PORTA,3
#define	emul_PIN_A4		PORTC,4
#define	emul_PIN_A5		PORTC,5
#define	emul_PIN_A6		PORTC,6
#define	emul_PIN_A7		PORTC,7
#define	emul_PIN_A8		PORTD,0
#define	emul_PIN_A9		PORTD,1
#define	emul_PIN_A10	PORTD,2
#define	emul_PIN_A11	PORTD,3
#define	emul_PIN_A12	PORTD,4
#define	emul_PIN_A13	PORTD,5
#define	emul_PIN_A14	PORTD,6
#define	emul_PIN_A15	PORTD,7

#define	emul_PIN_D0		PORTB,0
#define	emul_PIN_D1		PORTB,1
#define	emul_PIN_D2		PORTB,2
#define	emul_PIN_D3		PORTB,3
#define	emul_PIN_D4		PORTB,4
#define	emul_PIN_D5		PORTB,5
#define	emul_PIN_D6		PORTB,6
#define	emul_PIN_D7		PORTB,7

;RS232 Pins
#define	emul_RX			PORTA,4
#define	emul_TX			PORTE,0
;emulSR bits(STATUS REGISTER)
#define	emul_N		emulSR,7
#define	emul_V		emulSR,6
;#define	emul_NULL	emulSR,5
#define	emul_B		emulSR,4
#define	emul_D		emulSR,3
#define	emul_I		emulSR,2
#define	emul_Z		emulSR,1
#define	emul_C		emulSR,0

;FLAG VALUES FOR FUNCTIONSS!! Shouldnt be used outside that!
#define		FLAG_N		0x80
#define		FLAG_V		0x40
#define		FLAG_X		0x20
#define		FLAG_B		0x10
#define		FLAG_D		0x08
#define		FLAG_I		0x04
#define		FLAG_Z		0x02
#define		FLAG_C		0x01

;******************************************************************************
;MACROS:
;******************************************************************************
MODIFY_FLAGS	MACRO	FLAGS
		if(FLAGS & FLAG_C)
			MODIFY_C_FLAG
		endif
		if(FLAGS & FLAG_Z)
			MODIFY_Z_FLAG
		endif
		if(FLAGS & FLAG_N)
			MODIFY_N_FLAG
		endif
		if(FLAGS & FLAG_B)
			MODIFY_B_FLAG
		endif
		if(FLAGS & FLAG_D)
			MODIFY_D_FLAG
		endif
		if(FLAGS & FLAG_I)
			MODIFY_I_FLAG
		endif
		if(FLAGS & FLAG_V)
			MODIFY_V_FLAG
		endif
	ENDM


;////////////
MODIFY_C_FLAG	MACRO
		bcf		emul_C				;modify C flag
		btfsc	STATUS,C			;carry from prev operation?
		bsf		emul_C				;set emul Carry
	ENDM

;////////////
MODIFY_Z_FLAG	MACRO
		bcf		emul_Z
		btfsc	STATUS,Z			;STATUS,Z set?
		bsf		emul_Z				;yes
	ENDM

;////////////
MODIFY_N_FLAG	MACRO
		bcf		emul_N				;modify Negative flag
		btfsc	STATUS,N			;negative bit set?
		bsf		emul_N				;yes	
	ENDM

;////////////
MODIFY_B_FLAG	MACRO
	ENDM

;////////////
MODIFY_D_FLAG	MACRO
	ENDM

;////////////
MODIFY_I_FLAG	MACRO
	ENDM

;////////////
MODIFY_V_FLAG	MACRO
		bcf		emul_V
		btfsc	STATUS,OV
		bsf		emul_V
	ENDM

;/////////////////////////////////
;PUSH PC TO SOFT STACK
;/////////////////////////////////
PUSHPC2STACK	MACRO
		;lets push PC+2 value on software STACK...
		movlw	0x01
		movwf	ramH
	
		movff	emulPCH,aux1	;store high emulPC byte in aux var
		movff	emulSP,ramL	;load 0x0100 + emulSP address were to write...
	
		movlw	0x02		;calculate PC+2 position...
		addwf	emulPCL,W
		btfss	STATUS,C	;Overflow?
		incf	aux1		;indicate it in high byte emulPC var
	
		call	writeRAM	;Write low emulPC byte in STACK address
		
		incf	emulSP,F	;Increment Stack Pointer

		movff	emulSP,ramL	;load 0x0100 + emulSR address were to write...(remember that ramH shoul continue beeing 0x01)
		movf	aux1,W
		call	writeRAM	;Write High emulPC byte in STACK address	
	
		incf	emulSP,F	;Increment Stack Pointer
	ENDM

;/////////////////////////////////
;PULL PC FROM SOFT STACK
;/////////////////////////////////
PULLPCFROMSTACK	MACRO
		decf	emulSP,F	;decrement SP
		
		;lets pull PC value from software STACK...
		movlw	0x01
		movwf	ramH		;address 0x01XX

		movf	emulSP,W	;address 0x0100+emulSP
		call	readRAM

		movwf	emulPCH		;save readed value to PCH

		decf	emulSP,F	;decrement SP

		movf	emulSP,W	;address 0x0100+emulSP
		call	readRAM

		movwf	emulPCL		;save readed value to PCL
	ENDM

;/////////////////////////////////
;PUSH SR TO SOFT STACK
;/////////////////////////////////
PUSHSR2STACK	MACRO
		;push emulSR
		movlw	0x01
		movwf	ramH
		movff	emulSP,ramL	;load 0x0100 + emulSR address were to write...
		movf	emulSR,W	;load Status Register on W
		call	writeRAM	;Write emulSR in STACK address		
	
		incf	emulSP,F	;Increment Stack Pointer
	ENDM

;******************************************************************************
; Vector de Reset.
; Este codigo comenzara a ejecutarse cuando suceda un reset

		ORG	0x0000
		bra	Main		;Se va al inicio del codigo principal

;******************************************************************************
; Vector de interrupcion de alta prioridad
; Este codigo comenzara a ejecutarse cuando una interrupcion de alta prioridad ocurra
; o cuando cualquier interrupcion ocurra si es que las prioridades de las interrupciones
; no estan habilitadas

		ORG	0x0008

		bra	HighInt						;Va a la rutina de interrupcion de alta prioridad

;******************************************************************************
; Vector de interrupcion de baja prioridad y rutina
; Este codigo comenzara a ejecutrase cuando una interrupcion de baja prioridad ocurra
; Este codigo puede ser eliminado si las interrupciones de baja prioridad no son usadas

		ORG	0x0018

		movff	STATUS,STATUS_TEMP		;Salva el registro STATUS
		movff	WREG,WREG_TEMP			;Salva el registro de trabajo
		movff	BSR,BSR_TEMP			;Salva el registro BSR

;	*** El codigo de la interrupcion de baja prioridad va aqui ***


		movff	BSR_TEMP,BSR			;recupera el registro BSR
		movff	WREG_TEMP,WREG			;recupera el registro de trabajo
		movff	STATUS_TEMP,STATUS		;recupera el registro STATUS
		retfie

;******************************************************************************
; Rutina de interrupcion de alta prioridad
; El codigo para la rutina de interrupcion de alta prioridad es colocado aqui para
; evitar conflictos con el vector de interrupciones de baja prioridad

HighInt:

;	*** El codigo para las interrupciones de alta prioridad va aqui ***

		retfie	FAST

;******************************************************************************
Main:
;********************	Modules Init	*********************
	movlw	B'01100000'
	movwf	OSCCON				;Ajustamos el oscilador interno a 4 MHz(CHANGE)
	movlw	B'00001111'
	movwf	ADCON1,0			;All pins as Digital

	movlw	b'00010000'			;Set PORTA to outputs except for RX pin
	movwf	TRISA
	clrf	TRISB,0				;PORTB is now OUTPUT
	clrf	TRISC,0				;PORTC is now OUTPUT
	clrf	TRISD,0				;PORTD is now OUTPUT
	clrf	TRISE,0				;PORTE is now OUTPUT	

;********************	emulator initial values		*********************
	;clear emulator Registers...
	clrf	emulPCL			;emulator Program Counter...
	clrf	emulPCH
	clrf	emulAC			;emulator Acumulator
	clrf	emulX			;emulator X pointer
	clrf	emulY			;emulator Y pointer
	movlw	b'00110000'		;D and I flags begin Set
	movwf	emulSR			;emulator STATUS REGISTER
	clrf	emulSP			;emulator STACK POINTER

	clrf	emulOP			;emulator Actual OPERATOR 

;RS232test:
;	call	GetC
;	movf	WREG,F
;	btfsc	STATUS,Z
;	bra		RS232test

;	movff	aux232,PORTB
;	bra		RS232test

	bra		EXECUTE			;eternal loop begins...

EXECUTE
	lfsr	FSR0, emulOP	;load indirect data register FSR0 with start of data array were command will be stored...
	movlw	.1				;indicate that we will read 1 byte...(only OP)
	call	ReadOP
	
	movff	emulOP,auxOPL				;[BEGIN]calculate Goto ADDRESS
	clrf	auxOPH					;...

	movlw	HIGH BEGIN_OPLIST		;...
	movwf	PCLATH
	bcf		STATUS,C
	rlcf	auxOPL,F
	rlcf	auxOPH,F
	bcf		STATUS,C
	rlcf	auxOPL,F
	rlcf	auxOPH,W
	addwf	PCLATH
	movlw	LOW	BEGIN_OPLIST
	addwf	auxOPL,F
	btfsc	STATUS,C
	incf	PCLATH,F				;...[/BEGIN]

	movf	auxOPL,W
	movwf	PCL				;JUMP To calculated position inside OPLIST
BEGIN_OPLIST
	goto	OP00			;OP = 
	goto	OP01			;OP = 
	goto	EXECUTE
	goto	EXECUTE
	goto	EXECUTE
	goto	OP05			;OP =
	goto	OP06			;OP =
	goto	EXECUTE
	goto	OP08			;OP =
	goto	OP09			;OP =
	goto	OP0A			;OP =
	goto	EXECUTE
	goto	EXECUTE
	goto	OP0D			;OP =
	goto	OP0E			;OP =
	goto	EXECUTE

	goto	OP10			;OP = 
	goto	OP11			;OP = 
	goto	EXECUTE
	goto	EXECUTE
	goto	EXECUTE
	goto	OP15			;OP =
	goto	OP16			;OP =
	goto	EXECUTE
	goto	OP18			;OP =
	goto	OP19			;OP =
	goto	EXECUTE
	goto	EXECUTE
	goto	EXECUTE
	goto	OP1D			;OP =
	goto	OP1E			;OP =
	goto	EXECUTE

	goto	OP20			;OP = 
	goto	OP21			;OP = 
	goto	EXECUTE
	goto	EXECUTE
	goto	OP24			;OP =
	goto	OP25			;OP =
	goto	OP26			;OP =
	goto	EXECUTE
	goto	OP28			;OP =
	goto	OP29			;OP =
	goto	OP2A			;OP =
	goto	EXECUTE
	goto	OP2C			;OP =
	goto	OP2D			;OP =
	goto	OP2E			;OP =
	goto	EXECUTE

	goto	OP30			;OP = 
	goto	OP31			;OP = 
	goto	EXECUTE
	goto	EXECUTE
	goto	EXECUTE
	goto	OP35			;OP =
	goto	OP36			;OP =
	goto	EXECUTE
	goto	OP38			;OP =
	goto	OP39			;OP =
	goto	EXECUTE
	goto	EXECUTE
	goto	EXECUTE
	goto	OP3D			;OP =
	goto	OP3E			;OP =
	goto	EXECUTE

	goto	OP40			;OP = 
	goto	OP41			;OP = 
	goto	EXECUTE
	goto	EXECUTE
	goto	EXECUTE
	goto	OP45			;OP =
	goto	OP46			;OP =
	goto	EXECUTE
	goto	OP48			;OP =
	goto	OP49			;OP =
	goto	OP4A			;OP =
	goto	EXECUTE
	goto	OP4C			;OP =
	goto	OP4D			;OP =
	goto	OP4E			;OP =
	goto	EXECUTE

	goto	OP50			;OP = 
	goto	OP51			;OP = 
	goto	EXECUTE
	goto	EXECUTE
	goto	EXECUTE
	goto	OP55			;OP =
	goto	OP56			;OP =
	goto	EXECUTE
	goto	OP58			;OP =
	goto	OP59			;OP =
	goto	EXECUTE
	goto	EXECUTE
	goto	EXECUTE
	goto	OP5D			;OP =
	goto	OP5E			;OP =
	goto	EXECUTE

	goto	OP60			;OP = 
	goto	OP61			;OP = 
	goto	EXECUTE
	goto	EXECUTE
	goto	EXECUTE
	goto	OP65			;OP =
	goto	OP66			;OP =
	goto	EXECUTE
	goto	OP68			;OP =
	goto	OP69			;OP =
	goto	OP6A			;OP =
	goto	EXECUTE
	goto	OP6C			;OP =
	goto	OP6D			;OP =
	goto	OP6E			;OP =
	goto	EXECUTE

	goto	OP70			;OP = 
	goto	OP71			;OP = 
	goto	EXECUTE
	goto	EXECUTE
	goto	EXECUTE
	goto	OP75			;OP =
	goto	OP76			;OP =
	goto	EXECUTE
	goto	OP78			;OP =
	goto	OP79			;OP =
	goto	EXECUTE
	goto	EXECUTE
	goto	EXECUTE
	goto	OP7D			;OP =
	goto	OP7E			;OP =
	goto	EXECUTE


							;OPCODE [0x80,0xFF]
	goto	EXECUTE
	goto	OP81			;OP = 
	goto	EXECUTE
	goto	EXECUTE
	goto	OP84			;OP =
	goto	OP85			;OP =
	goto	OP86			;OP =
	goto	EXECUTE
	goto	OP88			;OP =
	goto	EXECUTE
	goto	OP8A			;OP =
	goto	EXECUTE
	goto	OP8C			;OP =
	goto	OP8D			;OP =
	goto	OP8E			;OP =
	goto	EXECUTE

	goto	OP90			;OP = 
	goto	OP91			;OP = 
	goto	EXECUTE
	goto	EXECUTE
	goto	OP94			;OP =
	goto	OP95			;OP =
	goto	OP96			;OP =
	goto	EXECUTE
	goto	OP98			;OP =
	goto	OP99			;OP =
	goto	OP9A			;OP =
	goto	EXECUTE
	goto	EXECUTE
	goto	OP9D			;OP =
	goto	EXECUTE
	goto	EXECUTE

	goto	OPA0			;OP = 
	goto	OPA1			;OP = 
	goto	OPA2			;OP =
	goto	EXECUTE
	goto	OPA4			;OP =
	goto	OPA5			;OP =
	goto	OPA6			;OP =
	goto	EXECUTE
	goto	OPA8			;OP =
	goto	OPA9			;OP =
	goto	OPAA			;OP =
	goto	EXECUTE
	goto	EXECUTE
	goto	OPAD			;OP =
	goto	OPAE			;OP =
	goto	EXECUTE

	goto	OPB0			;OP = 
	goto	OPB1			;OP = 
	goto	EXECUTE
	goto	EXECUTE
	goto	OPB4			;OP =
	goto	OPB5			;OP =
	goto	OPB6			;OP =
	goto	EXECUTE
	goto	OPB8			;OP =
	goto	OPB9			;OP =
	goto	OPBA			;OP =
	goto	EXECUTE
	goto	OPBC			;OP =
	goto	OPBD			;OP =
	goto	OPBE			;OP =
	goto	EXECUTE

	goto	OPC0			;OP = 
	goto	OPC1			;OP = 
	goto	EXECUTE
	goto	EXECUTE
	goto	OPC4			;OP =
	goto	OPC5			;OP =
	goto	OPC6			;OP =
	goto	EXECUTE
	goto	OPC8			;OP =
	goto	OPC9			;OP =
	goto	OPCA			;OP =
	goto	EXECUTE
	goto	OPCC			;OP =
	goto	OPCD			;OP =
	goto	OPCE			;OP =
	goto	EXECUTE

	goto	OPD0			;OP = 
	goto	OPD1			;OP = 
	goto	EXECUTE
	goto	EXECUTE
	goto	EXECUTE
	goto	OPD5			;OP =
	goto	OPD6			;OP =
	goto	EXECUTE
	goto	OPD8			;OP =
	goto	OPD9			;OP =
	goto	EXECUTE
	goto	EXECUTE
	goto	EXECUTE
	goto	OPDD			;OP =
	goto	OPDE			;OP =
	goto	EXECUTE


	goto	OPE0			;OP = 
	goto	OPE1			;OP = 
	goto	EXECUTE
	goto	EXECUTE
	goto	OPE4			;OP =
	goto	OPE5			;OP =
	goto	OPE6			;OP =
	goto	EXECUTE
	goto	OPE8			;OP =
	goto	OPE9			;OP =
	goto	OPEA			;OP =
	goto	EXECUTE
	goto	OPEC			;OP =
	goto	OPED			;OP =
	goto	OPEE			;OP =
	goto	EXECUTE

	goto	OPF0			;OP = 
	goto	OPF1			;OP = 
	goto	EXECUTE
	goto	EXECUTE
	goto	EXECUTE
	goto	OPF5			;OP =
	goto	OPF6			;OP =
	goto	EXECUTE
	goto	OPF8			;OP =
	goto	OPF9			;OP =
	goto	EXECUTE
	goto	EXECUTE
	goto	EXECUTE
	goto	OPFD			;OP =
	goto	OPFE			;OP =
	goto	EXECUTE


;///////////////////////////////////*********;///////////////////////////////////
;///////////////////////////////////*********;///////////////////////////////////
;///////////////////////////////////*********;///////////////////////////////////
;///////////////////////////////////*********;///////////////////////////////////
;S E T   O F    I N S T R U C T I O N S:
;///////////////////////////////////*********;///////////////////////////////////
;///////////////////////////////////*********;///////////////////////////////////
;///////////////////////////////////*********;///////////////////////////////////
;///////////////////////////////////*********;///////////////////////////////////

;**********************************************************************
;ADC INSTRUCTIONS!!!
;**********************************************************************
;ADC 
;Add Memory to Accumulator with Carry
 ;A + M + C -> A, C 
;N Z C I D V 
;+ + + - - + 

;addressing assembler 		opc bytes cyles
;----------------------------------------- 
;(indirect,X) ADC (oper,X) 	61 2 6 
;zeropage ADC oper 			65 2 3 
;immidiate ADC #oper 		69 2 2 
;absolute ADC oper 			6D 3 4 
;(indirect),Y ADC (oper),Y 	71 2 5* 
;zeropage,X ADC oper,X 		75 2 4 
;absolute,Y ADC oper,Y 		79 3 4* 
;absolute,X ADC oper,X 		7D 3 4* 
;**********************************************************************
;///////////////////////////////////
;///////////////////////////////////
ADC_COMMON
	movwf	aux1				;store in temp var
	btfsc	emul_C				;add carry?
	incf	aux1,W				;yes...	
	addwf	emulAC,F			;and store result in Acumulator

	;set flags...	
	MODIFY_FLAGS	(FLAG_N | FLAG_Z | FLAG_C | FLAG_V)
	goto	EXECUTE
;///////////////////////////////////
;///////////////////////////////////


;///////////////////////////////////
;OPCODE 0x61:	ADC
;(indirect,X) ADC (oper,X)
;///////////////////////////////////
OP61
	call	BPX
	bra		ADC_COMMON

;///////////////////////////////////
;OPCODE 0x65:	ADC
;zeropage ADC oper
;///////////////////////////////////
OP65
	call	ZPG
	bra		ADC_COMMON

;///////////////////////////////////
;OPCODE 0x69:	ADC
;immidiate ADC #oper
;///////////////////////////////////
OP69
	call	IMM
	bra		ADC_COMMON

;///////////////////////////////////
;OPCODE 0x6D:	ADC
;absolute ADC oper
;///////////////////////////////////
OP6D
	call	ABS
	bra		ADC_COMMON

;///////////////////////////////////
;OPCODE 0x71:	ADC
;(indirect),Y ADC (oper),Y
;///////////////////////////////////
OP71
	call	INDY
	bra		ADC_COMMON

;///////////////////////////////////
;OPCODE 0x75:	ADC
;zeropage,X ADC oper,X
;///////////////////////////////////
OP75
	call	ZPGX
	bra		ADC_COMMON

;///////////////////////////////////
;OPCODE 0x79:	ADC
;absolute,Y ADC oper,Y
;///////////////////////////////////
OP79
	call	ABSY
	bra		ADC_COMMON

;///////////////////////////////////
;OPCODE 0x7D:	ADC
;absolute,X ADC oper,X
;///////////////////////////////////
OP7D
	call	ABSX
	bra		ADC_COMMON



;**********************************************************************
;AND INSTRUCTIONS!!!
;**********************************************************************
;AND 
;AND Memory with Accumulator A AND M
 ;M -> A
;N Z C I D V 
;+ + - - - -

;addressing assembler opc bytes cyles
;-- ---------------------------------------
;(indirect,X) AND (oper,X) 21 2 6 
;zeropage AND oper 25 2 3 
;immidiate AND #oper 29 2 2 
;absolute AND oper 2D 3 4 
;(indirect),Y AND (oper),Y 31 2 5* 
;zeropage,X AND oper,X 35 2 4 
;absolute,Y AND oper,Y 39 3 4* 
;absolute,X AND oper,X 3D 3 4* 
;**********************************************************************
;///////////////////////////////////
;///////////////////////////////////
AND_COMMON
	andwf	emulAC,F

	;set flags...	
	MODIFY_FLAGS	(FLAG_N | FLAG_Z)
	goto	EXECUTE
;///////////////////////////////////
;///////////////////////////////////


;///////////////////////////////////
;OPCODE 0x21:	AND
;(indirect,X) AND (oper,X)
;///////////////////////////////////
OP21
	call	BPX
	bra		AND_COMMON

;///////////////////////////////////
;OPCODE 0x25:	AND
;zeropage AND oper
;///////////////////////////////////
OP25
	call	ZPG
	bra		AND_COMMON

;///////////////////////////////////
;OPCODE 0x29:	AND
;immidiate AND #oper
;///////////////////////////////////
OP29
	call	IMM
	bra		AND_COMMON

;///////////////////////////////////
;OPCODE 0x2D:	AND
;absolute AND oper
;///////////////////////////////////
OP2D
	call	ABS
	bra		AND_COMMON

;///////////////////////////////////
;OPCODE 0x31:	AND
;(indirect),Y AND (oper),Y
;///////////////////////////////////
OP31
	call	INDY
	bra		AND_COMMON

;///////////////////////////////////
;OPCODE 0x35:	AND
;zeropage,X AND oper,X
;///////////////////////////////////
OP35
	call	ZPGX
	bra		AND_COMMON

;///////////////////////////////////
;OPCODE 0x39:	AND
;absolute,Y AND oper,Y
;///////////////////////////////////
OP39
	call	ABSY
	bra		AND_COMMON

;///////////////////////////////////
;OPCODE 0x3D:	AND
;absolute,X AND oper,X
;///////////////////////////////////
OP3D
	call	ABSX
	bra		AND_COMMON


;**********************************************************************
;ASL INSTRUCTIONS!!!
;**********************************************************************
;ASL 
;Shift Left One Bit (Memory or Accumulator)
 ;C <- [76543210] <- 0 
;N Z C I D V 
;+ + + - - -

;addressing assembler opc bytes cyles
;-- ---------------------------------------
;zeropage ASL oper 06 2 5 
;accumulator ASL A 0A 1 2 
;absolute ASL oper 0E 3 6 
;zeropage,X ASL oper,X 16 2 6 
;absolute,X ASL oper,X 1E 3 7 
;**********************************************************************
;///////////////////////////////////
;///////////////////////////////////
ASL_SEMICOMMON
	movwf	aux1		;store value in auxiliar variable
	bcf		STATUS,C	;clean carry 
	rlcf	aux1,F		;CANT USE RLNCF because it wont affect C flag
	
	;set flags...	
	MODIFY_FLAGS	(FLAG_N | FLAG_Z | FLAG_C)
	
	movf	aux1,W		;ramL and ramH previously set(when reading M)
	call	writeRAM	;finally, write new value in address
	goto	EXECUTE
;///////////////////////////////////
;///////////////////////////////////




;///////////////////////////////////
;OPCODE 0x06:		ASL
;zeropage ASL oper
;///////////////////////////////////
OP06
	call	ZPG
	bra		ASL_SEMICOMMON

;///////////////////////////////////
;OPCODE 0x0A:		ASL
;accumulator ASL A
;///////////////////////////////////
OP0A
	bcf		STATUS,C		;clean carry 
	rlcf	emulAC,F		;CANT USE RLNCF because it wont affect C flag
	
	;set flags...	
	MODIFY_FLAGS	(FLAG_N | FLAG_Z | FLAG_C)
	goto	EXECUTE

;///////////////////////////////////
;OPCODE 0x0E:		ASL
;absolute ASL oper
;///////////////////////////////////
OP0E
	call	ABS
	bra		ASL_SEMICOMMON

;///////////////////////////////////
;OPCODE 0x16:		ASL
;zeropage,X ASL oper,X
;///////////////////////////////////
OP16
	call	ZPGX
	bra		ASL_SEMICOMMON

;///////////////////////////////////
;OPCODE 0x1E:		ASL
;absolute,X ASL oper,X
;///////////////////////////////////
OP1E
	call	ABSX
	bra		ASL_SEMICOMMON


;**********************************************************************
;BCC INSTRUCTIONS!!!
;**********************************************************************
;BCC 
;Branch on Carry Clear branch on C = 0 
;N Z C I D V
;- - - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;relative BCC oper 90 2 2** 
;**********************************************************************
;///////////////////////////////////
;OPCODE 0x90:		BCC
;relative BCC oper
;///////////////////////////////////
OP90
	btfsc	emul_C			;Carry clear?
	goto	EXECUTE

	movf	emulOP+1,W		;Yes. Branch!
	addwf	emulPCL,F
	btfsc	STATUS,C		;page overflow?
	incf	emulPCH,F		;yes, increase page number

	MODIFY_FLAGS	(0)		;no flags affected
	goto	EXECUTE			;done!

;**********************************************************************
;BCS INSTRUCTIONS!!!
;**********************************************************************
;BCS 
;Branch on Carry Set branch on C = 1 
;N Z C I D V 
;- - - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;relative BCS oper B0 2 2** 
;**********************************************************************
;///////////////////////////////////
;OPCODE 0xB0:		BCS
;relative BCS oper
;///////////////////////////////////
OPB0
	btfss	emul_C			;Carry set?
	goto	EXECUTE

	movf	emulOP+1,W		;Yes. Branch!
	addwf	emulPCL,F
	btfsc	STATUS,C		;page overflow?
	incf	emulPCH,F		;yes, increase page number

	MODIFY_FLAGS	(0)		;no flags affected
	goto	EXECUTE			;done!


;**********************************************************************
;BEQ INSTRUCTIONS!!!
;**********************************************************************
;BEQ 
;Branch on Result Zero branch on Z = 1 
;N Z C I D V 
;- - - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;relative BEQ oper F0 2 2** 
;**********************************************************************
;///////////////////////////////////
;OPCODE 0xF0:		BEQ
;relative BEQ oper
;///////////////////////////////////
OPF0
	btfss	emul_Z			;Zero set?
	goto	EXECUTE

	movf	emulOP+1,W		;Yes. Branch!
	addwf	emulPCL,F
	btfsc	STATUS,C		;page overflow?
	incf	emulPCH,F		;yes, increase page number	

	MODIFY_FLAGS	(0)		;no flags affected
	goto	EXECUTE

;**********************************************************************
;BIT INSTRUCTIONS!!!
;**********************************************************************
;BIT 
;Test Bits in Memory with Accumulator bits 7 and 6 of operand are transfered to bit 7 and 6 of SR (N,V); 
;the zeroflag is set to the result of operand AND accumulator. 
;A AND M, M7 -> N, M6 -> V 
;N Z C I D V 
;M7 + - - - M6 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;zeropage BIT oper 24 2 3 
;absolute BIT oper 2C 3 4 
;**********************************************************************
;///////////////////////////////////
;///////////////////////////////////
BIT_COMMON
	movwf	aux1			;store result in aux var
	andwf	emulAC,W		;and with acumulator

	MODIFY_FLAGS	(FLAG_Z) ;modify Z flag

	bcf		emul_N
	btfsc	aux1,7			;transfer memory bits 7 and 6 to STATUS Reg
	bsf		emul_N

	bcf		emul_V
	btfsc	aux1,6
	bsf		emul_V

	goto	EXECUTE
;///////////////////////////////////
;///////////////////////////////////



;///////////////////////////////////
;OPCODE 0x24:		BIT
;zeropage BIT oper
;///////////////////////////////////
OP24
	call	ZPG
	bra		BIT_COMMON

;///////////////////////////////////
;OPCODE 0x2C:		BIT
;absolute BIT oper
;///////////////////////////////////
OP2C
	call	ABS
	bra		BIT_COMMON

;**********************************************************************
;BMI INSTRUCTIONS!!!
;**********************************************************************
;BMI 
;Branch on Result Minus 
;branch on N = 1 
;N Z C I D V 
;- - - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;relative BMI oper 30 2 2** 
;**********************************************************************
;///////////////////////////////////
;OPCODE 0x30:		BMI		
;relative BMI oper
;///////////////////////////////////
OP30	
	btfss	emul_N			;Negative set?
	goto	EXECUTE

	movf	emulOP+1,W		;Yes. Branch!
	addwf	emulPCL,F
	btfsc	STATUS,C		;page overflow?
	incf	emulPCH,F		;yes, increase page number	

	MODIFY_FLAGS	(0)		;no flags affected
	goto	EXECUTE


;**********************************************************************
;BNE INSTRUCTIONS!!!
;**********************************************************************
;BNE 
;Branch on Result not Zero 
;branch on Z = 0 
;N Z C I D V 
;- - - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;relative BNE oper D0 2 2** 
;**********************************************************************
;///////////////////////////////////
;OPCODE 0xD0:		BNE		
;relative BNE oper
;///////////////////////////////////
OPD0	
	btfsc	emul_Z			;Zero clear?
	goto	EXECUTE

	movf	emulOP+1,W		;Yes. Branch!
	addwf	emulPCL,F
	btfsc	STATUS,C		;page overflow?
	incf	emulPCH,F		;yes, increase page number	

	MODIFY_FLAGS	(0)		;no flags affected
	goto	EXECUTE


;**********************************************************************
;BPL INSTRUCTIONS!!!
;**********************************************************************
;BPL 
;Branch on Result Plus 
;branch on N = 0 
;N Z C I D V 
;- - - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;relative BPL oper 10 2 2** 
;**********************************************************************
;///////////////////////////////////
;OPCODE 0x10:		BPL		
;relative BPL oper
;///////////////////////////////////
OP10	
	btfsc	emul_N			;Negative clear?
	goto	EXECUTE

	movf	emulOP+1,W		;Yes. Branch!
	addwf	emulPCL,F
	btfsc	STATUS,C		;page overflow?
	incf	emulPCH,F		;yes, increase page number	

	MODIFY_FLAGS	(0)		;no flags affected
	goto	EXECUTE


;**********************************************************************
;BRK INSTRUCTIONS!!!
;**********************************************************************
;BRK 
;Force Break 
;interrupt, 			N Z C I D V 
;push PC+2, push SR 	- - - 1 - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;implied BRK 00 1 7 
;**********************************************************************
;///////////////////////////////////
;OPCODE 0x00:		BRK
;implied BRK
;///////////////////////////////////
OP00
	PUSHPC2STACK		;MACRO THAT PUSHs PC TO SOFTWARE STACK
	PUSHSR2STACK		;MACRO THAT PUSHs SR TO SOFTWARE STACK

	;load Software interrupt address in PC
	movlw	0xFF
	movwf	emulPCH		;load 0xFFXX address

	movlw	0xFE
	movwf	emulPCL		;load 0xFFFE address

	bsf		emul_I		;indicate Software Interrupt occur
	
	goto	EXECUTE		;done


;**********************************************************************
;BVC INSTRUCTIONS!!!
;**********************************************************************
;BVC 
;Branch on Overflow Clear 
;branch on V = 0 
;N Z C I D V 
;- - - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;relative BVC oper 50 2 2** 
;///////////////////////////////////
;OPCODE 0x50:		BVC
;relative BVC oper
;///////////////////////////////////
OP50	
	btfsc	emul_V			;Overflow clear?
	goto	EXECUTE

	movf	emulOP+1,W		;Yes. Branch!
	addwf	emulPCL,F
	btfsc	STATUS,C		;page overflow?
	incf	emulPCH,F		;yes, increase page number	

	MODIFY_FLAGS	(0)		;no flags affected
	goto	EXECUTE


;**********************************************************************
;BVS INSTRUCTIONS!!!
;**********************************************************************
;BVS 
;Branch on Overflow 
;Set branch on V = 1 
;N Z C I D V 
;- - - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;relative BVC oper 70 2 2** 
;///////////////////////////////////
;OPCODE 0x70:		BVS		
;relative BVC oper
;///////////////////////////////////
OP70
	btfss	emul_V			;Overflow set?
	goto	EXECUTE

	movf	emulOP+1,W		;Yes. Branch!
	addwf	emulPCL,F
	btfsc	STATUS,C		;page overflow?
	incf	emulPCH,F		;yes, increase page number	

	MODIFY_FLAGS	(0)		;no flags affected
	goto	EXECUTE


;**********************************************************************
;CLC INSTRUCTIONS!!!
;**********************************************************************
;CLC 
;Clear Carry Flag 0 -> C 
;N Z C I D V 
;- - 0 - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;implied CLC 18 1 2 
;///////////////////////////////////
;OPCODE 0x18:		CLC
;implied CLC
;///////////////////////////////////
OP18
	bcf		emul_C
	goto	EXECUTE
	

;**********************************************************************
;CLD INSTRUCTIONS!!!
;**********************************************************************
;CLD 
;Clear Decimal Mode 
;0 -> D 
;N Z C I D V 
;- - - - 0 - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;implied CLD D8 1 2 
;///////////////////////////////////
;OPCODE 0xD8:		CLD	
;implied CLD
;///////////////////////////////////
OPD8
	bcf		emul_D
	goto	EXECUTE


;**********************************************************************
;CLI INSTRUCTIONS!!!
;**********************************************************************
;CLI 
;Clear Interrupt Disable Bit 0 -> I 
;N Z C I D V 
;- - - 0 - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;implied CLI 58 1 2 
;///////////////////////////////////
;OPCODE 0x58:		CLI
;implied CLI
;///////////////////////////////////
OP58
	bcf		emul_I
	goto	EXECUTE


;**********************************************************************
;CLV INSTRUCTIONS!!!
;**********************************************************************
;CLV 
;Clear Overflow Flag 0 -> V 
;N Z C I D V 
;- - - - - 0 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;implied CLV B8 1 2 
;///////////////////////////////////
;OPCODE 0xB8:		CLV
;implied CLV
;///////////////////////////////////
OPB8
	bcf		emul_V
	goto	EXECUTE


;**********************************************************************
;CMP INSTRUCTIONS!!!
;**********************************************************************
;CMP 
;Compare Memory with Accumulator 
;A - M 
;N Z C I D V 
;+ + + - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;(indirect,X) CMP (oper,X) C1 2 6 
;zeropage CMP oper C5 2 3 
;immidiate CMP #oper C9 2 2 
;absolute CMP oper CD 3 4 
;(indirect),Y CMP (oper),Y D1 2 5* 
;zeropage,X CMP oper,X D5 2 4 
;absolute,Y CMP oper,Y D9 3 4* 
;absolute,X CMP oper,X DD 3 4* 
;///////////////////////////////////
;///////////////////////////////////
CMP_COMMON
	subwf	emulAC,W		;substract M from AC
	MODIFY_FLAGS	(FLAG_Z | FLAG_C | FLAG_N) ;modify flags
	goto	EXECUTE
;///////////////////////////////////
;///////////////////////////////////


;///////////////////////////////////
;OPCODE 0xC1:		CMP
;(indirect,X) CMP (oper,X)
;///////////////////////////////////
OPC1
	call	BPX
	bra		CMP_COMMON

;///////////////////////////////////
;OPCODE 0xC5:		CMP
;zeropage CMP oper
;///////////////////////////////////
OPC5
	call	ZPG
	bra		CMP_COMMON

;///////////////////////////////////
;OPCODE 0xC9:		CMP
;immidiate CMP #oper
;///////////////////////////////////
OPC9
	call	IMM
	bra		CMP_COMMON

;///////////////////////////////////
;OPCODE 0xCD:		CMP
;absolute CMP oper
;///////////////////////////////////
OPCD
	call	ABS
	bra		CMP_COMMON

;///////////////////////////////////
;OPCODE 0xD1:		CMP
;(indirect),Y CMP (oper),Y
;///////////////////////////////////
OPD1
	call	INDY
	bra		CMP_COMMON

;///////////////////////////////////
;OPCODE 0xD5:		CMP
;zeropage,X CMP oper,X
;///////////////////////////////////
OPD5
	call	ZPGX
	bra		CMP_COMMON

;///////////////////////////////////
;OPCODE 0xD9:		CMP
;absolute,Y CMP oper,Y
;///////////////////////////////////
OPD9
	call	ABSY
	bra		CMP_COMMON

;///////////////////////////////////
;OPCODE 0xDD:		CMP
;absolute,X CMP oper,X
;///////////////////////////////////
OPDD
	call	ABSX
	bra		CMP_COMMON


;**********************************************************************
;CPX INSTRUCTIONS!!!
;**********************************************************************
;CPX 
;Compare Memory and Index X 
;X - M 
;N Z C I D V 
;+ + + - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;immidiate CPX #oper E0 2 2 
;zeropage CPX oper E4 2 3 
;absolute CPX oper EC 3 4 
;///////////////////////////////////
;///////////////////////////////////
CPX_COMMON
	subwf	emulX,W		;substract M from X
	MODIFY_FLAGS	(FLAG_Z | FLAG_C | FLAG_N) ;modify flags
	goto	EXECUTE
;///////////////////////////////////
;///////////////////////////////////



;///////////////////////////////////
;OPCODE 0xE0:		CPX
;immidiate CPX #oper
;///////////////////////////////////
OPE0
	call	IMM
	bra		CPX_COMMON

;///////////////////////////////////
;OPCODE 0xE4:		CPX
;zeropage CPX oper
;///////////////////////////////////
OPE4
	call	ZPG
	bra		CPX_COMMON

;///////////////////////////////////
;OPCODE 0xEC:		CPX
;absolute CPX oper
;///////////////////////////////////
OPEC
	call	ABS
	bra		CPX_COMMON


;**********************************************************************
;CPY INSTRUCTIONS!!!
;**********************************************************************
;CPY 
;Compare Memory and Index Y 
;Y - M 
;N Z C I D V 
;+ + + - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;immidiate CPY #oper C0 2 2 
;zeropage CPY oper C4 2 3 
;absolute CPY oper CC 3 4 
;///////////////////////////////////
;///////////////////////////////////
CPY_COMMON
	subwf	emulY,W		;substract M from Y
	MODIFY_FLAGS	(FLAG_Z | FLAG_C | FLAG_N) ;modify flags
	goto	EXECUTE
;///////////////////////////////////
;///////////////////////////////////



;///////////////////////////////////
;OPCODE 0xE0:		CPY
;immidiate CPY #oper
;///////////////////////////////////
OPC0
	call	IMM
	bra		CPY_COMMON

;///////////////////////////////////
;OPCODE 0xE4:		CPY
;zeropage CPY oper
;///////////////////////////////////
OPC4
	call	ZPG
	bra		CPY_COMMON

;///////////////////////////////////
;OPCODE 0xEC:		CPY
;absolute CPY oper
;///////////////////////////////////
OPCC
	call	ABS
	bra		CPY_COMMON


;**********************************************************************
;DEC INSTRUCTIONS!!!
;**********************************************************************
;DEC 
;Decrement Memory by One 
;M - 1 -> M 
;N Z C I D V 
;+ + - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;zeropage DEC oper C6 2 5 
;absolute DEC oper CE 3 3 
;zeropage,X DEC oper,X D6 2 6 
;absolute,X DEC oper,X DE 3 7 
;///////////////////////////////////
;///////////////////////////////////
DEC_COMMON
	movwf	aux1
	decf	aux1,W
	MODIFY_FLAGS	(FLAG_N | FLAG_Z) ;modify flags

	call	writeRAM		;write Memory position with new value
	goto	EXECUTE
;///////////////////////////////////
;///////////////////////////////////
;///////////////////////////////////
;OPCODE 0xC6:		DEC
;zeropage DEC oper
;///////////////////////////////////
OPC6
	call	ZPG
	bra		DEC_COMMON

;///////////////////////////////////
;OPCODE 0xCE:		DEC
;absolute DEC oper 
;///////////////////////////////////
OPCE
	call	ABS
	bra		DEC_COMMON

;///////////////////////////////////
;OPCODE 0xD6:		DEC
;zeropage,X DEC oper,X 
;///////////////////////////////////
OPD6
	call	ZPGX
	bra		DEC_COMMON

;///////////////////////////////////
;OPCODE 0xDE:		DEC
;absolute,X DEC oper,X 
;///////////////////////////////////
OPDE
	call	ABSY
	bra		DEC_COMMON


;**********************************************************************
;DEX INSTRUCTIONS!!!
;**********************************************************************
;DEX 
;Decrement Index X by One 
;X - 1 -> X 
;N Z C I D V 
;+ + - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;implied DEX CA 1 2 
;///////////////////////////////////
;OPCODE 0xCA:		DEX
;implied DEX 
;///////////////////////////////////
OPCA
	decf	emulX,F		;decrement X
	MODIFY_FLAGS	(FLAG_N | FLAG_Z) ;modify flags
	goto	EXECUTE


;**********************************************************************
;DEY INSTRUCTIONS!!!
;**********************************************************************
;DEY 
;Decrement Index Y by One 
;Y - 1 -> Y 
;N Z C I D V 
;+ + - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;implied DEY 88 1 2 
;///////////////////////////////////
;OPCODE 0x88:		DEY
;implied DEY
;///////////////////////////////////
OP88
	decf	emulY,F		;decrement Y
	MODIFY_FLAGS	(FLAG_N | FLAG_Z) ;modify flags
	goto	EXECUTE


;**********************************************************************
;EOR INSTRUCTIONS!!!
;**********************************************************************
;EOR 
;Exclusive-OR Memory with Accumulator 
;A EOR M -> A 
;N Z C I D V 
;+ + - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;(indirect,X) EOR (oper,X) 41 2 6 
;zeropage EOR oper 45 2 3 
;immidiate EOR #oper 49 2 2 
;absolute EOR oper 4D 3 4 
;(indirect),Y EOR (oper),Y 51 2 5*
;zeropage,X EOR oper,X 55 2 4 
;absolute,Y EOR oper,Y 59 3 4* 
;absolute,X EOR oper,X 5D 3 4
;///////////////////////////////////
;///////////////////////////////////
EOR_COMMON
	xorwf	emulAC,F

	;set flags...	
	MODIFY_FLAGS	(FLAG_N | FLAG_Z)
	goto	EXECUTE
;///////////////////////////////////
;///////////////////////////////////


;///////////////////////////////////
;OPCODE 0x41:	EOR
;(indirect,X) EOR (oper,X) 
;///////////////////////////////////
OP41
	call	BPX
	bra		EOR_COMMON

;///////////////////////////////////
;OPCODE 0x45:	EOR
;zeropage EOR oper
;///////////////////////////////////
OP45
	call	ZPG
	bra		EOR_COMMON

;///////////////////////////////////
;OPCODE 0x49:	EOR
;immidiate EOR #oper
;///////////////////////////////////
OP49
	call	IMM
	bra		EOR_COMMON

;///////////////////////////////////
;OPCODE 0x4D:	EOR
;absolute EOR oper 
;///////////////////////////////////
OP4D
	call	ABS
	bra		EOR_COMMON

;///////////////////////////////////
;OPCODE 0x51:	EOR
;(indirect),Y EOR (oper),Y
;///////////////////////////////////
OP51
	call	INDY
	bra		EOR_COMMON

;///////////////////////////////////
;OPCODE 0x55:	EOR
;zeropage,X EOR oper,X 
;///////////////////////////////////
OP55
	call	ZPGX
	bra		EOR_COMMON

;///////////////////////////////////
;OPCODE 0x59:	EOR
;absolute,Y EOR oper,Y 
;///////////////////////////////////
OP59
	call	ABSY
	bra		EOR_COMMON

;///////////////////////////////////
;OPCODE 0x5D:	EOR
;absolute,X EOR oper,X 
;///////////////////////////////////
OP5D
	call	ABSX
	bra		EOR_COMMON


;**********************************************************************
;INC INSTRUCTIONS!!!
;**********************************************************************
;INC 
;Increment Memory by One 
;M + 1 -> M 
;N Z C I D V 
;+ + - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;zeropage INC oper E6 2 5 
;absolute INC oper EE 3 6 
;zeropage,X INC oper,X F6 2 6 
;absolute,X INC oper,X FE 3 7 
;///////////////////////////////////
;///////////////////////////////////
INC_COMMON
	movwf	aux1
	incf	aux1,W
	MODIFY_FLAGS	(FLAG_N | FLAG_Z) ;modify flags

	call	writeRAM		;write Memory position with new value
	goto	EXECUTE
;///////////////////////////////////
;///////////////////////////////////
;///////////////////////////////////
;OPCODE 0xE6:		INC
;zeropage INC oper
;///////////////////////////////////
OPE6
	call	ZPG
	bra		INC_COMMON

;///////////////////////////////////
;OPCODE 0xE:		INC
;absolute INC oper
;///////////////////////////////////
OPEE
	call	ABS
	bra		INC_COMMON

;///////////////////////////////////
;OPCODE 0xF6:		INC
;zeropage,X INC oper,X
;///////////////////////////////////
OPF6
	call	ZPGX
	bra		INC_COMMON

;///////////////////////////////////
;OPCODE 0xFE:		INC
;absolute,X INC oper,X
;///////////////////////////////////
OPFE
	call	ABSY
	bra		INC_COMMON


;**********************************************************************
;INX INSTRUCTIONS!!!
;**********************************************************************
;INX 
;Increment Index X by One 
;X + 1 -> X 
;N Z C I D V 
;+ + - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;implied INX E8 1 2 
;///////////////////////////////////
;OPCODE 0xE8:		INX
;implied INX E8 1 2
;///////////////////////////////////
OPE8
	incf	emulX,F		;increment X
	MODIFY_FLAGS	(FLAG_N | FLAG_Z) ;modify flags
	goto	EXECUTE

;**********************************************************************
;INY INSTRUCTIONS!!!
;**********************************************************************
;INY 
;Increment Index Y by One 
;Y + 1 -> Y 
;N Z C I D V 
;+ + - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;implied INY C8 1 2 
;///////////////////////////////////
;OPCODE 0xC8:		INY
;implied INY C8 1 2 
;///////////////////////////////////
OPC8
	incf	emulY,F		;increment Y
	MODIFY_FLAGS	(FLAG_N | FLAG_Z) ;modify flags
	goto	EXECUTE

;**********************************************************************
;JMP INSTRUCTIONS!!!
;**********************************************************************
;JMP 
;Jump to New Location 
;(PC+1) -> PCL N Z C I D V     
;(PC+2) -> PCH - - - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;absolute JMP oper 4C 3 3 
;indirect JMP (oper) 6C 3 5 
;///////////////////////////////////
;///////////////////////////////////
JMP_COMMON
	movff	emulOP+1,emulPCL
	movff	emulOP+2,emulPCH		;set emulPC with address in instruction
	goto	EXECUTE

;///////////////////////////////////
;///////////////////////////////////


;///////////////////////////////////
;OPCODE 0x4C:		JMP
;absolute JMP oper
;///////////////////////////////////
OP4C	
	call	ABS
	bra		JMP_COMMON

;///////////////////////////////////
;OPCODE 0x6C:		JMP
;indirect JMP (oper)
;///////////////////////////////////
OP6C
	call	IND
	bra		JMP_COMMON

;**********************************************************************
;JSR INSTRUCTIONS!!!
;**********************************************************************
;JSR 
;Jump to New Location Saving Return Address
;push (PC+2),  N Z C I D V 
;(PC+1) -> PCL - - - - - - 
;(PC+2) -> PCH 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;absolute JSR oper 20 3 6 
;///////////////////////////////////
;OPCODE 0x20:		
;absolute JSR oper
;///////////////////////////////////
OP20
	call	ABS
	PUSHPC2STACK		;MACRO THAT PUSHs PC TO SOFTWARE STACK

	movff	emulOP+1,emulPCL			;(PC+1) -> PCL
	movff	emulOP+2,emulPCH			;(PC+2) -> PCH 
	goto	EXECUTE


;**********************************************************************
;LDA INSTRUCTIONS!!!
;**********************************************************************
;LDA 
;Load Accumulator with Memory 
;M -> A 
;N Z C I D V 
;+ + - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;(indirect,X) LDA (oper,X) A1 2 6 
;zeropage LDA oper A5 2 3 
;immidiate LDA #oper A9 2 2 
;absolute LDA oper AD 3 4 
;(indirect),Y LDA (oper),Y B1 2 5* 
;zeropage,X LDA oper,X B5 2 4 
;absolute,Y LDA oper,Y B9 3 4* 
;absolute,X LDA oper,X BD 3 4* 
;///////////////////////////////////
;///////////////////////////////////
LDA_COMMON
	movwf	emulAC		;store readed value on AC
	movf	emulAC,F	;force Z and N flags to be modified

	MODIFY_FLAGS	(FLAG_N | FLAG_Z) ;modify flags
	goto	EXECUTE
;///////////////////////////////////
;///////////////////////////////////



;///////////////////////////////////
;OPCODE 0xA1:		LDA
;(indirect,X) LDA (oper,X) A1 2 6 
;///////////////////////////////////
OPA1
	call	BPX
	bra	LDA_COMMON


;///////////////////////////////////
;OPCODE 0xA5:		LDA
;zeropage LDA oper A5 2 3 
;///////////////////////////////////
OPA5
	call	ZPG
	bra	LDA_COMMON

;///////////////////////////////////
;OPCODE 0xA9:		LDA
;immidiate LDA #oper A9 2 2 
;///////////////////////////////////
OPA9
	call	IMM
	bra	LDA_COMMON

;///////////////////////////////////
;OPCODE 0xAD:		LDA
;absolute LDA oper AD 3 4 
;///////////////////////////////////
OPAD
	call	ABS
	bra	LDA_COMMON

;///////////////////////////////////
;OPCODE 0xB1:		LDA
;(indirect),Y LDA (oper),Y
;///////////////////////////////////
OPB1
	call	INDY
	bra	LDA_COMMON

;///////////////////////////////////
;OPCODE 0xB5:		LDA
;zeropage,X LDA oper,X
;///////////////////////////////////
OPB5
	call	ZPGX
	bra	LDA_COMMON

;///////////////////////////////////
;OPCODE 0xB9:		LDA
;absolute,Y LDA oper,Y
;///////////////////////////////////
OPB9
	call	ABSY
	bra	LDA_COMMON

;///////////////////////////////////
;OPCODE 0xBD:		LDA
;absolute,X LDA oper,X
;///////////////////////////////////
OPBD
	call	ABSX
	bra	LDA_COMMON



;**********************************************************************
;LDX INSTRUCTIONS!!!
;**********************************************************************
;LDX 
;Load Index X with Memory 
;M -> X 
;N Z C I D V 
;+ + - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;immidiate LDX #oper A2 2 2 
;zeropage LDX oper A6 2 3 
;absolute LDX oper AE 3 4 
;zeropage,Y LDX oper,Y B6 2 4 
;absolute,Y LDX oper,Y BE 3 4* 
;///////////////////////////////////
;///////////////////////////////////
LDX_COMMON
	movwf	emulX
	movf	emulX,F	;force flags Z and N to be modified

	MODIFY_FLAGS	(FLAG_N | FLAG_Z) ;modify flags
	goto	EXECUTE
;///////////////////////////////////
;///////////////////////////////////



;///////////////////////////////////
;OPCODE 0xA2:			LDX
;immidiate LDX #oper
;///////////////////////////////////
OPA2
	call	IMM
	bra	LDX_COMMON

;///////////////////////////////////
;OPCODE 0xA6:			LDX
;zeropage LDX oper
;///////////////////////////////////
OPA6
	call	ZPG
	bra	LDX_COMMON

;///////////////////////////////////
;OPCODE 0xB6:			LDX
;zeropage,Y LDX oper,Y
;///////////////////////////////////
OPB6
	call	ZPGY
	bra	LDX_COMMON

;///////////////////////////////////
;OPCODE 0xAE:			LDX
;absolute LDX oper
;///////////////////////////////////
OPAE
	call	ABS
	bra	LDX_COMMON

;///////////////////////////////////
;OPCODE 0xBE:			LDX
;absolute,Y LDX oper,Y
;///////////////////////////////////
OPBE
	call	ABSY
	bra	LDX_COMMON


;**********************************************************************
;LDY INSTRUCTIONS!!!
;**********************************************************************
;LDY 
;Load Index Y with Memory 
;M -> Y 
;N Z C I D V 
;+ + - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;immidiate LDY #oper A0 2 2 
;zeropage LDY oper A4 2 3 
;absolute LDY oper AC 3 4 
;zeropage,X LDY oper,X B4 2 4 
;absolute,X LDY oper,X BC 3 4* 
;///////////////////////////////////
;///////////////////////////////////
LDY_COMMON
	movwf	emulY
	movf	emulY,F	;force flags Z and N to be modified

	MODIFY_FLAGS	(FLAG_N | FLAG_Z) ;modify flags
	goto	EXECUTE
;///////////////////////////////////
;///////////////////////////////////



;///////////////////////////////////
;OPCODE 0xA0:			LDY
;immidiate LDY #oper A0 2 2 
;///////////////////////////////////
OPA0
	call	IMM
	bra	LDY_COMMON

;///////////////////////////////////
;OPCODE 0xA4:			LDY
;zeropage LDY oper A4 2 3 
;///////////////////////////////////
OPA4
	call	ZPG
	bra	LDY_COMMON

;///////////////////////////////////
;OPCODE 0xAC:			LDY
;absolute LDY oper AC 3 4 
;///////////////////////////////////
OAC
	call	ZPGY
	bra	LDY_COMMON

;///////////////////////////////////
;OPCODE 0xB4:			LDY
;zeropage,X LDY oper,X B4 2 4 
;///////////////////////////////////
OPB4
	call	ABS
	bra	LDY_COMMON

;///////////////////////////////////
;OPCODE 0xBC:			LDY
;absolute,X LDY oper,X BC 3 4* 
;///////////////////////////////////
OPBC
	call	ABSY
	bra	LDY_COMMON


;**********************************************************************
;LSR INSTRUCTIONS!!!
;**********************************************************************
;LSR 
;Shift One Bit Right 
;(Memory or Accumulator) 0 -> [76543210] -> C 
;N Z C I D V 
;+ + + - - - //DOCUMENTATION BUG(http://www.masswerk.at/6502/6502_instruction_set.html says - + + - - -)
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;zeropage LSR oper 46 2 5 
;accumulator LSR A 4A 1 2 
;absolute LSR oper 4E 3 6 
;zeropage,X LSR oper,X 56 2 6 
;absolute,X LSR oper,X 5E 3 7 
;///////////////////////////////////
;///////////////////////////////////
LSR_SEMICOMMON
	movwf	aux1		;store value in auxiliar variable
	bcf		STATUS,C	;clean carry 
	rrcf	aux1,F		;CANT USE RRNCF because it wont affect C flag
	
	;set flags...	
	MODIFY_FLAGS	(FLAG_N | FLAG_Z | FLAG_C)
	
	movf	aux1,W		;ramL and ramH previously set(when reading M)
	call	writeRAM	;finally, write new value in address
	goto	EXECUTE	
;///////////////////////////////////
;///////////////////////////////////



;///////////////////////////////////
;OPCODE 0x46:				LSR
;zeropage LSR oper
;///////////////////////////////////
OP46
	call	ZPG
	bra		LSR_SEMICOMMON

;///////////////////////////////////
;OPCODE 0x4A:				LSR
;accumulator LSR A
;///////////////////////////////////
OP4A
	bcf		STATUS,C		;clean carry 
	rrcf	emulAC,F		;CANT USE RRNCF because it wont affect C flag
	
	;set flags...	
	MODIFY_FLAGS	(FLAG_N | FLAG_Z | FLAG_C)
	goto	EXECUTE

;///////////////////////////////////
;OPCODE 0x4E:				LSR
;absolute LSR oper
;///////////////////////////////////
OP4E
	call	ABS
	bra		LSR_SEMICOMMON

;///////////////////////////////////
;OPCODE 0x56:				LSR
;zeropage,X LSR oper,X
;///////////////////////////////////
OP56
	call	ZPGX
	bra		LSR_SEMICOMMON

;///////////////////////////////////
;OPCODE 0x5E:				LSR
;absolute,X LSR oper,X
;///////////////////////////////////
OP5E
	call	ABSX
	bra		LSR_SEMICOMMON


;**********************************************************************
;NOP INSTRUCTIONS!!!
;**********************************************************************
;NOP 
;No Operation --- 
;N Z C I D V 
;- - - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;implied NOP EA 1 2 
;///////////////////////////////////
;OPCODE 0xEA:			NOP
;implied NOP
;///////////////////////////////////
OPEA
	goto	EXECUTE


;**********************************************************************
;ORA INSTRUCTIONS!!!
;**********************************************************************
;ORA 
;OR Memory with Accumulator 
;A OR M -> A 
;N Z C I D V 
;+ + - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;(indirect,X) ORA (oper,X) 01 2 6 
;zeropage ORA oper 05 2 3 
;immidiate ORA #oper 09 2 2 
;(indirect),Y ORA (oper),Y 11 2 5* 
;absolute ORA oper 0D 3 4 
;zeropage,X ORA oper,X 15 2 
;absolute,Y ORA oper,Y 19 3 4* 
;absolute,X ORA oper,X 1D 3 4* 
;///////////////////////////////////
;///////////////////////////////////
ORA_COMMON
	iorwf	emulAC,F

	;set flags...	
	MODIFY_FLAGS	(FLAG_N | FLAG_Z)
	goto	EXECUTE
;///////////////////////////////////
;///////////////////////////////////


;///////////////////////////////////
;OPCODE 0x01:	ORA
;(indirect,X) ORA (oper,X)
;///////////////////////////////////
OP01
	call	BPX
	bra		ORA_COMMON

;///////////////////////////////////
;OPCODE 0x05:	ORA
;zeropage ORA oper
;///////////////////////////////////
OP05
	call	ZPG
	bra		ORA_COMMON

;///////////////////////////////////
;OPCODE 0x09:	ORA
;immidiate ORA #oper
;///////////////////////////////////
OP09
	call	IMM
	bra		ORA_COMMON

;///////////////////////////////////
;OPCODE 0x11:	ORA
;(indirect),Y ORA (oper),Y
;///////////////////////////////////
OP11
	call	ABS
	bra		ORA_COMMON

;///////////////////////////////////
;OPCODE 0x0D:	ORA
;absolute ORA oper
;///////////////////////////////////
OP0D
	call	INDY
	bra		ORA_COMMON

;///////////////////////////////////
;OPCODE 0x15:	ORA
;zeropage,X ORA oper,X
;///////////////////////////////////
OP15
	call	ZPGX
	bra		ORA_COMMON

;///////////////////////////////////
;OPCODE 0x19:	ORA
;absolute,Y ORA oper,Y
;///////////////////////////////////
OP19
	call	ABSY
	bra		ORA_COMMON

;///////////////////////////////////
;OPCODE 0x1D:	ORA
;absolute,X ORA oper,X
;///////////////////////////////////
OP1D
	call	ABSX
	bra		ORA_COMMON


;**********************************************************************
;PHA INSTRUCTIONS!!!
;**********************************************************************
;PHA 
;Push Accumulator on Stack 
;push A 
;N Z C I D V 
;- - - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;implied PHA 48 1 3 
;///////////////////////////////////
;OPCODE 0x48:			PHA
;implied PHA
;///////////////////////////////////
OP48
	PUSHPC2STACK		;MACRO THAT PUSHs PC TO SOFTWARE STACK
	goto		EXECUTE


;**********************************************************************
;PHP INSTRUCTIONS!!!
;**********************************************************************
;PHP 
;Push Processor Status on Stack 
;push SR 
;N Z C I D V 
;- - - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;implied PHP 08 1 3 
;///////////////////////////////////
;OPCODE 0x08:			PHP
;implied PHP
;///////////////////////////////////
OP08
	PUSHSR2STACK		;MACRO THAT PUSHs SR TO SOFTWARE STACK
	goto		EXECUTE

;**********************************************************************
;PLA INSTRUCTIONS!!!
;**********************************************************************
;PLA 
;Pull Accumulator from Stack 
;pull A 
;N Z C I D V 
;- - - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;implied PLA 68 1 4 
;///////////////////////////////////
;OPCODE 0x68:			PLA
;implied PLA
;///////////////////////////////////
OP68
	movlw	0x01
	movwf	ramH	;point to 0x01XX
	
	movf	emulSP,W	;point to 0x0100 + emulSP
	call	readRAM

	movwf	emulAC		;save value in AC

	decf	emulSP,F	;decrement SP

	goto	EXECUTE

;**********************************************************************
;PLP INSTRUCTIONS!!!
;**********************************************************************
;PLP 
;Pull Processor Status from Stack 
;pull SR 
;N Z C I D V
;from stack 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;implied PHP 28 1 4 
;///////////////////////////////////
;OPCODE 0x28:			PLP
;implied PLP
;///////////////////////////////////
OP28
	movlw	0x01
	movwf	ramH	;point to 0x01XX
	
	movf	emulSP,W	;point to 0x0100 + emulSP
	call	readRAM

	movwf	emulSR		;save value in SR

	decf	emulSP,F	;decrement SP

	goto	EXECUTE

;**********************************************************************
;ROL INSTRUCTIONS!!!
;**********************************************************************
;ROL 
;Rotate One Bit Left 
;(Memory or Accumulator) C <- [76543210] <- C 
;N Z C I D V 
;+ + + - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;zeropage ROL oper 26 2 5 
;accumulator ROL A 2A 1 2 
;absolute ROL oper 2E 3 6 
;zeropage,X ROL oper,X 36 2 6 
;absolute,X ROL oper,X 3E 3 7 
;///////////////////////////////////
;///////////////////////////////////
ROL_SEMICOMMON
	movwf	aux1			;store value in auxiliar variable
	bcf		STATUS,C		;clean carry 
	btfsc	emul_C			;test emul Carry
	bsf		STATUS,C		;set carry 	

	rlcf	aux1,F			;CANT USE RLNCF because it wont affect C flag
	
	;set flags...	
	MODIFY_FLAGS	(FLAG_N | FLAG_Z | FLAG_C)
	
	movf	aux1,W		;ramL and ramH previously set(when reading M)
	call	writeRAM	;finally, write new value in address
	goto	EXECUTE
;///////////////////////////////////
;///////////////////////////////////




;///////////////////////////////////
;OPCODE 0x26:		ROL
;zeropage ROL oper 26 2 5 
;///////////////////////////////////
OP26
	call	ZPG
	bra		ROL_SEMICOMMON

;///////////////////////////////////
;OPCODE 0x2A:		ROL
;accumulator ROL A 2A 1 2 
;///////////////////////////////////
OP2A
	bcf		STATUS,C		;clean carry 
	btfsc	emul_C			;test emul Carry
	bsf		STATUS,C		;set carry 	
	
	rlcf	emulAC,F		;CANT USE RLNCF because it wont affect C flag
	
	;set flags...	
	MODIFY_FLAGS	(FLAG_N | FLAG_Z | FLAG_C)
	goto	EXECUTE

;///////////////////////////////////
;OPCODE 0x2E:		ROL
;absolute ROL oper 2E 3 6 
;///////////////////////////////////
OP2E
	call	ABS
	bra		ROL_SEMICOMMON

;///////////////////////////////////
;OPCODE 0x36:		ROL
;zeropage,X ROL oper,X 36 2 6 
;///////////////////////////////////
OP36
	call	ZPGX
	bra		ROL_SEMICOMMON

;///////////////////////////////////
;OPCODE 0x3E:		ROL
;absolute,X ROL oper,X 3E 3 7 
;///////////////////////////////////
OP3E
	call	ABSX
	bra		ROL_SEMICOMMON

;**********************************************************************
;ROR INSTRUCTIONS!!!
;**********************************************************************
;ROR 
;Rotate One Bit Right 
;(Memory or Accumulator) C -> [76543210] -> C 
;N Z C I D V 
;+ + + - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;zeropage ROR oper 66 2 5 
;accumulator ROR A 6A 1 2 
;absolute ROR oper 6E 3 6 
;zeropage,X ROR oper,X 76 2 6 
;absolute,X ROR oper,X 7E 3 7 
;///////////////////////////////////
;///////////////////////////////////
ROR_SEMICOMMON
	movwf	aux1			;store value in auxiliar variable
	bcf		STATUS,C		;clean carry 
	btfsc	emul_C			;test emul Carry
	bsf		STATUS,C		;set carry 	

	rrcf	aux1,F			;CANT USE RRNCF because it wont affect C flag
	
	;set flags...	
	MODIFY_FLAGS	(FLAG_N | FLAG_Z | FLAG_C)
	
	movf	aux1,W		;ramL and ramH previously set(when reading M)
	call	writeRAM	;finally, write new value in address
	goto	EXECUTE
;///////////////////////////////////
;///////////////////////////////////




;///////////////////////////////////
;OPCODE 0x66:		ROR
;zeropage ROR oper
;///////////////////////////////////
OP66
	call	ZPG
	bra		ROR_SEMICOMMON

;///////////////////////////////////
;OPCODE 0x6A:		ROR
;accumulator ROR A
;///////////////////////////////////
OP6A
	bcf		STATUS,C		;clean carry 
	btfsc	emul_C			;test emul Carry
	bsf		STATUS,C		;set carry 	
	
	rrcf	emulAC,F		;CANT USE RRNCF because it wont affect C flag
	
	;set flags...	
	MODIFY_FLAGS	(FLAG_N | FLAG_Z | FLAG_C)
	goto	EXECUTE

;///////////////////////////////////
;OPCODE 0x6E:		ROR
;absolute ROR oper
;///////////////////////////////////
OP6E
	call	ABS
	bra		ROR_SEMICOMMON

;///////////////////////////////////
;OPCODE 0x76:		ROR
;zeropage,X ROR oper,X
;///////////////////////////////////
OP76
	call	ZPGX
	bra		ROR_SEMICOMMON

;///////////////////////////////////
;OPCODE 0x7E:		ROR
;absolute,X ROR oper,X
;///////////////////////////////////
OP7E
	call	ABSX
	bra		ROR_SEMICOMMON


;**********************************************************************
;RTI INSTRUCTIONS!!!
;**********************************************************************
;RTI 
;Return from Interrupt 
;pull SR, pull PC 
;N Z C I D V 
;from stack 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;implied RTI 40 1 6 
;///////////////////////////////////
;OPCODE 0x40:			RTI			
;implied RTI
;///////////////////////////////////
OP40
	PUSHSR2STACK		;MACRO THAT PUSHs SR TO SOFTWARE STACK	
	PUSHPC2STACK		;MACRO THAT PUSHs PC TO SOFTWARE STACK
	goto	EXECUTE

;**********************************************************************
;RTS INSTRUCTIONS!!!
;**********************************************************************
;RTS 
;Return from Subroutine 
;pull PC, PC+1 -> PC 
;N Z C I D V 
;- - - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;implied RTS 60 1 6 
;///////////////////////////////////
;OPCODE 0x60:			
;implied RTS
;///////////////////////////////////
OP60
	PULLPCFROMSTACK		;MACRO THAT PULLs PC FROM SOFTWARE STACK
	goto	EXECUTE

;**********************************************************************
;SBC INSTRUCTIONS!!!
;**********************************************************************
;SBC 
;Subtract Memory from Accumulator with Borrow 
;A - M - C -> A 
;N Z C I D V 
;+ + + - - + 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;(indirect,X) SBC (oper,X) E1 2 6 
;zeropage SBC oper E5 2 3 
;immidiate SBC #oper E9 2 2 
;absolute SBC oper ED 3 4 
;(indirect),Y SBC (oper),Y F1 2 5* 
;zeropage,X SBC oper,X F5 2 4 
;absolute,Y SBC oper,Y F9 3 4* 
;absolute,X SBC oper,X FD 3 4* 
;///////////////////////////////////
;///////////////////////////////////
SBC_COMMON
	subwf	emulAC,F		;AC = AC - M
	btfsc	emul_C			;habia carry?
	decf	emulAC,F		;Si. AC = AC - C

	MODIFY_FLAGS	(FLAG_N | FLAG_Z | FLAG_C | FLAG_V) ;modify flags
	goto	EXECUTE

;///////////////////////////////////
;///////////////////////////////////



;///////////////////////////////////
;OPCODE 0xE1:				SBC
;(indirect,X) SBC (oper,X)
;///////////////////////////////////
OPE1
	call	BPX
	bra		SBC_COMMON

;///////////////////////////////////
;OPCODE 0xE5:				SBC
;zeropage SBC oper
;///////////////////////////////////
OPE5
	call	ZPG
	bra		SBC_COMMON

;///////////////////////////////////
;OPCODE 0xE9:				SBC
;immidiate SBC #oper
;///////////////////////////////////
OPE9
	call	IMM
	bra		SBC_COMMON

;///////////////////////////////////
;OPCODE 0xED:				SBC
;absolute SBC oper
;///////////////////////////////////
OPED
	call	ABS
	bra		SBC_COMMON

;///////////////////////////////////
;OPCODE 0xF1:				SBC
;(indirect),Y SBC (oper),Y
;///////////////////////////////////
OPF1
	call	INDY
	bra		SBC_COMMON

;///////////////////////////////////
;OPCODE 0xF5:				SBC
;zeropage,X SBC oper,X
;///////////////////////////////////
OPF5
	call	ZPGX
	bra		SBC_COMMON

;///////////////////////////////////
;OPCODE 0xF9:				SBC
;absolute,Y SBC oper,Y
;///////////////////////////////////
OPF9
	call	ABSY
	bra		SBC_COMMON

;///////////////////////////////////
;OPCODE 0xFD:				SBC
;absolute,X SBC oper,X
;///////////////////////////////////
OPFD
	call	ABSX
	bra		SBC_COMMON


;**********************************************************************
;SEC INSTRUCTIONS!!!
;**********************************************************************
;SEC 
;Set Carry Flag 
;1 -> C 
;N Z C I D V 
;- - 1 - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;implied SEC 38 1 2 
;///////////////////////////////////
;OPCODE 0x38:				SEC		
;implied SEC
;///////////////////////////////////
OP38
	bsf		emul_C
	goto	EXECUTE


;**********************************************************************
;SED INSTRUCTIONS!!!
;**********************************************************************
;SED 
;Set Decimal Flag 
;1 -> D 
;N Z C I D V 
;- - - - 1 - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;implied SED F8 1 2 
;///////////////////////////////////
;OPCODE 0xF8:				SED
;implied SED
;///////////////////////////////////
OPF8
	bsf		emul_D	
	goto	EXECUTE


;**********************************************************************
;SEI INSTRUCTIONS!!!
;**********************************************************************
;SEI 
;Set Interrupt Disable Status 
;1 -> I 
;N Z C I D V 
;- - - 1 - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;implied SEI 78 1 2 
;///////////////////////////////////
;OPCODE 0x78:				SEI
;implied SEI
;///////////////////////////////////
OP78
	bsf		emul_I
	goto	EXECUTE


;**********************************************************************
;STA INSTRUCTIONS!!!
;**********************************************************************
;STA 
;Store Accumulator in Memory 
;A -> M 
;N Z C I D V 
;- - - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;(indirect,X) STA (oper,X) 81 2 6 
;zeropage STA oper 85 2 3 
;absolute STA oper 8D 3 4 
;(indirect),Y STA (oper),Y 91 2 6 
;zeropage,X STA oper,X 95 2 4 
;absolute,Y STA oper,Y 99 3 5 
;absolute,X STA oper,X 9D 3 5 
;///////////////////////////////////
;///////////////////////////////////
STA_COMMON
	movf	emulAC,W		;IGNORE READED MEMORY POSITION and copy AC to W
	call	writeRAM		;write AC to M (ramH and ramL addresses were founded when calling addressing mode)

	goto	EXECUTE
;///////////////////////////////////
;///////////////////////////////////



;///////////////////////////////////
;OPCODE 0x81:				STA
;(indirect,X) STA (oper,X) 81 2 6 
;///////////////////////////////////
OP81
	call	BPX
	bra		STA_COMMON

;///////////////////////////////////
;OPCODE 0x85:				STA
;zeropage STA oper 85 2 3 
;///85////////////////////////////////
OP85
	call	ZPG
	bra		STA_COMMON

;///////////////////////////////////
;OPCODE 0x8D:				STA
;absolute STA oper 8D 3 
;///////////////////////////////////
OP8D
	call	ABS
	bra		STA_COMMON

;///////////////////////////////////
;OPCODE 0x91:				STA
;(indirect),Y STA (oper),Y 91 2 6 
;///////////////////////////////////
OP91
	call	INDY
	bra		STA_COMMON

;///////////////////////////////////
;OPCODE 0x95:				STA
;zeropage,X STA oper,X 95 2 4 
;///////////////////////////////////
OP95
	call	ZPGX
	bra		STA_COMMON

;///////////////////////////////////
;OPCODE 0x99:				STA
;absolute,Y STA oper,Y 99 3 5 
;///////////////////////////////////
OP99
	call	ABSY
	bra		STA_COMMON

;///////////////////////////////////
;OPCODE 0x9D:				STA
;absolute,X STA oper,X 9D 3 5 
;///////////////////////////////////
OP9D
	call	ABSX
	bra		STA_COMMON


;**********************************************************************
;STX INSTRUCTIONS!!!
;**********************************************************************
;STX 
;Store Index X in Memory 
;X -> M 
;N Z C I D V 
;- - - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;zeropage STX oper 86 2 3 
;absolute STX oper 8E 3 4 
;zeropage,Y STX oper,Y 96 2 4 
;///////////////////////////////////
;///////////////////////////////////
STX_COMMON
	movf	emulX,W		;IGNORE READED MEMORY POSITION and copy X to W
	call	writeRAM		;write X to M (ramH and ramL addresses were founded when calling addressing mode)
	goto	EXECUTE
;///////////////////////////////////
;///////////////////////////////////



;///////////////////////////////////
;OPCODE 0x86:				STX
;zeropage STX oper
;///////////////////////////////////
OP86
	call	ZPG
	bra		STX_COMMON

;///////////////////////////////////
;OPCODE 0x8E:				STX
;absolute STX oper 
;///////////////////////////////////
OP8E
	call	ABS
	bra		STX_COMMON

;///////////////////////////////////
;OPCODE 0x96:				STX
;zeropage,Y STX oper,Y
;///////////////////////////////////
OP96
	call	ZPGY
	bra		STX_COMMON

;**********************************************************************
;STY INSTRUCTIONS!!!
;**********************************************************************
;STY 
;Sore Index Y in Memory 
;Y -> M 
;N Z C I D V 
;- - - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;zeropage STY oper 84 2 3 
;absolute STY oper 8C 3 4 
;zeropage,X STY oper,X 94 2 4 
;///////////////////////////////////
;///////////////////////////////////
STY_COMMON
	movf	emulY,W			;IGNORE READED MEMORY POSITION and copy Y to W
	call	writeRAM		;write Y to M (ramH and ramL addresses were founded when calling addressing mode)
	goto	EXECUTE
;///////////////////////////////////
;///////////////////////////////////



;///////////////////////////////////
;OPCODE 0x84:				STY
;zeropage STY oper
;///////////////////////////////////
OP84
	call	ZPG
	bra		STY_COMMON

;///////////////////////////////////
;OPCODE 0x8C:				STY
;absolute STY oper
;///////////////////////////////////
OP8C
	call	ABS
	bra		STY_COMMON

;///////////////////////////////////
;OPCODE 0x94:				STY
;zeropage,X STY oper,X
;///////////////////////////////////
OP94
	call	ZPGX
	bra		STY_COMMON


;**********************************************************************
;TAX INSTRUCTIONS!!!
;**********************************************************************
;TAX 
;Transfer Accumulator to Index X 
;A -> X 
;N Z C I D V 
;+ + - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;implied TAX AA 1 2 
;///////////////////////////////////
;OPCODE 0xAA:			TAX	
;implied TAX
;///////////////////////////////////
OPAA
	movf	emulAC,W		;Do not use movff because it wont affect Z and N flags..
	movwf	emulX
	
	MODIFY_FLAGS	(FLAG_N | FLAG_Z) ;modify flags	

	goto	EXECUTE

;**********************************************************************
;TAY INSTRUCTIONS!!!
;**********************************************************************
;TAY 
;Transfer Accumulator to Index Y 
;A -> Y 
;N Z C I D V 
;+ + - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;implied TAY A8 1 2 
;///////////////////////////////////
;OPCODE 0xA8:			TAY
;implied TAY
;///////////////////////////////////
OPA8
	movf	emulAC,W		;Do not use movff because it wont affect Z and N flags..
	movwf	emulY
	
	MODIFY_FLAGS	(FLAG_N | FLAG_Z) ;modify flags	

	goto	EXECUTE


;**********************************************************************
;TSX INSTRUCTIONS!!!
;**********************************************************************
;TSX 
;Transfer Stack Pointer to Index X 
;SP -> X 
;N Z C I D V 
;+ + - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;implied TSX BA 1 2 
;///////////////////////////////////
;OPCODE 0xBA:			TSX
;implied TSX
;///////////////////////////////////
OPBA
	movf	emulSP,W		;Do not use movff because it wont affect Z and N flags..
	movwf	emulX
	
	MODIFY_FLAGS	(FLAG_N | FLAG_Z) ;modify flags	

	goto	EXECUTE


;**********************************************************************
;TXA INSTRUCTIONS!!!
;**********************************************************************
;TXA 
;Transfer Index X to Accumulator 
;X -> A 
;N Z C I D V 
;+ + - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;implied TXA 8A 1 2 
;///////////////////////////////////
;OPCODE 0x8A:			TXA
;implied TXA
;///////////////////////////////////
OP8A
	movf	emulX,W		;Do not use movff because it wont affect Z and N flags..
	movwf	emulAC
	
	MODIFY_FLAGS	(FLAG_N | FLAG_Z) ;modify flags	

	goto	EXECUTE


;**********************************************************************
;TXS INSTRUCTIONS!!!
;**********************************************************************
;TXS 
;Transfer Index X to Stack Register 
;X -> SP 
;N Z C I D V 
;+ + - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;implied TXS 9A 1 2 
;///////////////////////////////////
;OPCODE 0x9A:			TXS
;implied TXS
;///////////////////////////////////
OP9A
	movf	emulX,W		;Do not use movff because it wont affect Z and N flags..
	movwf	emulSP
	
	MODIFY_FLAGS	(FLAG_N | FLAG_Z) ;modify flags	

	goto	EXECUTE

;**********************************************************************
;TYA INSTRUCTIONS!!!
;**********************************************************************
;TYA 
;Transfer Index Y to Accumulator 
;Y -> A 
;N Z C I D V 
;+ + - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;implied TYA 98 1 2 
;///////////////////////////////////
;OPCODE 0x98:			TYA
;implied TYA
;///////////////////////////////////
OP98
	movf	emulY,W		;Do not use movff because it wont affect Z and N flags..
	movwf	emulAC
	
	MODIFY_FLAGS	(FLAG_N | FLAG_Z) ;modify flags	

	goto	EXECUTE
















;///////////////////////A D D R E S S I N G    M O D E S/////////////////////////////**********************************
;///////////////////////A D D R E S S I N G    M O D E S/////////////////////////////**********************************
;///////////////////////A D D R E S S I N G    M O D E S/////////////////////////////**********************************
;///////////////////////A D D R E S S I N G    M O D E S/////////////////////////////**********************************

;///////////////////////
;ACUMM MODE:
;accumulator
;///////////////////////
;ACUMM
	;its an operation over the acumulator. Shouldnt need nothing to do at all...
	;return

;///////////////////////
;ABS MODE:
;absolute
;///////////////////////
ABS
	movlw	.2
	call	ReadOP				;this command consumes 3 bytes(OP+3 more...)	

	movf	emulOP+2,W		;move second byte of instruction to W
	movwf	ramH			;store high RAM address in var
	movf	emulOP+1,W		;move second byte of instruction to W
	goto	readRAM			;READ effective 16 bit RAM address + RETURN

;///////////////////////
;ABSX MODE:
;absolute, X
;///////////////////////
ABSX
	movlw	.2
	call	ReadOP				;this command consumes 3 bytes(OP+2 more...)	

	movf	emulOP+2,W		;move second byte of instruction to W
	movwf	ramH			;store high RAM address in var

	movf	emulOP+1,W		;move second byte of instruction to W
	addwf	emulX,W
	btfsc	STATUS,C
	incf	ramH,F

	goto	readRAM			;READ effective 16 bit RAM address + RETURN

;///////////////////////
;ABSY MODE:
;absolute, Y
;///////////////////////
ABSY
	movlw	.2
	call	ReadOP				;this command consumes 3 bytes(OP+2 more...)	

	movf	emulOP+2,W		;move second byte of instruction to W
	movwf	ramH			;store high RAM address in var

	movf	emulOP+1,W		;move second byte of instruction to W
	addwf	emulY,W
	btfsc	STATUS,C
	incf	ramH,F

	goto	readRAM			;READ effective 16 bit RAM address + RETURN

;///////////////////////
;IMM MODE:
;immidiate
;///////////////////////
IMM
	movlw	.1
	call	ReadOP				;this command consumes 2 bytes(OP+1 more...)	

	movf	emulOP+1,W			;get zero page memory address
	call	readZEROPAGERAM		;read content of it
	return

;///////////////////////
;IMPL MODE:
;implied
;///////////////////////
;IMPL
	;its an implied mode. Shouldnt need nothing to do at all...
;	return

;///////////////////////
;IND MODE:	INDIRECT MODE
;indirect
;///////////////////////
IND
	movlw	.2
	call	ReadOP				;this command consumes 3 bytes(OP+2 more...)		

	movf	emulOP+2,W		;move third byte of instruction to W
	movwf	ramH			;store high RAM address in var
	movf	emulOP+1,W		;move second byte of instruction to W
	goto	readRAM			;READ effective 16 bit RAM address + RETURN

;///////////////////////
;BP,X MODE:
;(indirect,X)
;///////////////////////
BPX		
	movlw	.1
	call	ReadOP				;this command consumes 2 bytes(OP+1 more...)		

	movf	emulOP+1,W		;move second byte of instruction to W
	addwf	emulX,W			;and add it to emulX register. HERES, W holds the low address of memory data to read...
	movwf	aux1			;store in aux var
	incf	aux1,W
	call	readZEROPAGERAM
	movwf	ramH			;store high RAM address in var
	movf	aux1,W
	call	readZEROPAGERAM	;get low RAM address in var
	goto	readRAM			;READ effective 16 bit RAM address + RETURN

;///////////////////////
;INDY MODE:
;(indirect,Y)
;///////////////////////
INDY
	movlw	.1
	call	ReadOP				;this command consumes 2 bytes(OP+1 more...)		

	incf	emulOP+1,W
	movwf	ramH			;get high 8 bit ZERO PAGE effective address

	movf	emulOP+1,W		;move second byte of instruction to W
	call	readZEROPAGERAM
	addwf	emulY,W			;and add it to emulX register. HERES, W holds the low address of memory data to read...
	btfsc	STATUS,C
	incf	ramH,F			;if overflow, increase high byte addresss

	goto	readRAM			;READ effective 16 bit RAM address + RETURN



;///////////////////////
;REL MODE: 
;relative
;///////////////////////
REL
	movlw	.1
	call	ReadOP				;this command consumes 2 bytes(OP+1 more...)		

	movf	emulOP+1,W		;move second byte of instruction to W
	btfsc	emulOP+1,7		;check if negative or positive
	bra		NEG_OFF

	;positive OFFSET
	addwf	emulPCL,F		;add offset to Program Counter
	btfsc	STATUS,C		;check if overflow
	incf	emulPCH,F		;if OV, then increase high byte of Program Counter
	return

NEG_OFF
	andlw	0x7F			;erase sign
	subwf	emulPCL,F
	btfss	STATUS,C		;check if underflow
	decf	emulPCH,F		;if Underflow, then decrease high byte of Program Counter
	return

;///////////////////////
;ZPG: 
;zeropage
;///////////////////////
ZPG
	movlw	.1
	call	ReadOP				;this command consumes 2 bytes(OP+1 more...)		

	movf	emulOP+1,W		;move second byte of instruction to W
	goto	readZEROPAGERAM	;get ZERO PAGE MEMORY Value + RETURN

;///////////////////////
;ZPGX: 
;zeropage, X
;///////////////////////
ZPGX
	movlw	.1
	call	ReadOP				;this command consumes 2 bytes(OP+1 more...)		

	movf	emulOP+1,W		;move second byte of instruction to W
	addwf	emulX,W			;add Index X
	goto	readZEROPAGERAM	;get ZERO PAGE MEMORY Value + RETURN

;///////////////////////
;ZPGY: 
;zeropage, Y
;///////////////////////
ZPGY
	movlw	.1
	call	ReadOP				;this command consumes 2 bytes(OP+1 more...)		

	movf	emulOP+1,W		;move second byte of instruction to W
	addwf	emulY,W			;add Index X
	goto	readZEROPAGERAM	;get ZERO PAGE MEMORY Value + RETURN


;////////////////////////////////////////////////////////////////////////////////////
;I/O FUNCTIONS...
;////////////////////////////////////////////////////////////////////////////////////

;*******************************
;WRITES A BYTE FROM RS232 CHANNEL
;*******************************
PutC
	movwf	aux232
	bcf		emul_TX					;START BIT

	movlw	DELAY232
	call	DelayUs

	;now, lets send each bit
	movlw	.8
	movwf	cont232
NextBit
	btfsc	aux232,0				;set or clear bit
	bsf		emul_TX
	btfss	aux232,0
	bcf		emul_TX

	rrcf	aux232,F				;rotate to the right the byte

	movlw	DELAY232
	call	DelayUs

	decfsz	cont232,F				;decrement counter
	bra		NextBit					;go to get next bit

	bsf		emul_TX					;STOP BIT

	movlw	DELAY232
	call	DelayUs	

	retlw	.1						;return with success

;*******************************
;READS A BYTE FROM RS232 CHANNEL
;*******************************
GetC
	btfss	emul_RX					;if RX pin is low, then a byte was in progress. We're  too late surely...
	retlw	.0						;so return	with  FAIL

	movlw	DELAY232/2

Espera
	btfsc	emul_RX					;wait for begining of START bit
	bra		Espera

	call	DelayUs					;call soubrutine to put us in the middle of each bit life
	
	;btg		emul_TX
	;btg		emul_TX

	;now, lets read each bit
	movlw	.8
	movwf	cont232

ReceiveNext
	movlw	DELAY232
	call	DelayUs

	;btg		emul_TX
	;btg		emul_TX

	bcf		STATUS,C				;assume that is a low bit
	btfsc	emul_RX
	bsf		STATUS,C				;Ok,ok. it was high...so change it

	rrcf	aux232,F				;rotate to the right the byte

	decfsz	cont232,F				;decrement counter
	bra		ReceiveNext				;go to get next bit

	movlw	DELAY232
	call	DelayUs	

	;btg		emul_TX
	;btg		emul_TX

	btfss	emul_RX					;if RX pin is low, then something went wrong
	retlw	.0						;so return	with  FAIL

	retlw	.1						;return with success

;////////////////////////////////
;DELAY_US(12 MIPS)
;////////////////////////////////
DelayUs
		movwf	del1
		decf	del1,F
		btfsc	STATUS,Z
		bra		Sale
	
Loop
		goto	$+2
		goto	$+2
		goto	$+2
		decfsz	del1,F
		bra		Loop

		nop
		nop
Sale
		goto	$+2
		return
;/////////////////////////////////
;WRITE RAM POSITION.W must contain data to write. ramL must have LOW byte address to write to. High byte of address to write to must be pre-stored in ramH byte.
;////////////////////////////////
writeRAM
		movwf	data8

		bcf		emul_PIN_RAMROM	;RAMROM = 0 = RAM
		bsf		emul_PIN_WR		;indicate writing RAM...
		clrf	TRISB			;PORTB ALL as output

		movff	data8,LATB		;put data8 in port now(so we dont have to wait later)

		;***********
		movlw	0x0F		;put address in ports
		andwf	PORTA,F		;clean low nibble
		movf	ramL,W
		andlw	0x0F
		iorwf	PORTA,F		;put address [A0-A3] bits

		movlw	0xF0		;put address in ports
		andwf	PORTC,F		;clean high nibble
		movf	ramL,W
		andlw	0xF0
		iorwf	PORTC,F		;put address [A4-A7] bits

		movff	ramH,PORTD		;put ramH byte in address [A8-A15] bits
		;***********
		
		bcf		emul_PIN_RD			;RD=0=ASK FOR WRITE RAM DATA

		nop							;wait 83nS
		nop							;wait 83nS

		bsf		emul_PIN_RD			;RD=1=RAM WRITE DONE...

		return

;/////////////////////////////////
;READ ZERO PAGE RAM POSITION. W must have address to read [0,255]
;////////////////////////////////
readZEROPAGERAM
		movwf	ramL

		setf	TRISB			;PORTB ALL as input
		bcf		emul_PIN_RAMROM	;RAMROM = 0 = RAM
		bsf		emul_PIN_WR		;indicate reading RAM...

		;***********
		movlw	0x0F		;put addres s in ports
		andwf	PORTA,F		;clean low nibble
		movf	ramL,W
		andlw	0x0F
		iorwf	PORTA,F		;put address [A0-A3] bits

		movlw	0xF0		;put address in ports
		andwf	PORTC,F		;clean high nibble
		movf	ramL,W
		andlw	0xF0
		iorwf	PORTC,F		;put address [A4-A7] bits

		clrf	ramH			;force ZERO PAGE
		clrf	PORTD		;put 0x00 in address [A8-A15] bits(ZERO PAGE)
		;***********

		bcf		emul_PIN_RD			;RD=0=ASK FOR RAM DATA

		nop							;wait 83nS
		movf	PORTB,W				;read RAM byte...

		bsf		emul_PIN_RD			;RD=1=RAM READ DONE...

		return

;/////////////////////////////////
;READ RAM POSITION. W must have LOW byte address to read. High byte must be pre-stored in ramH byte
;////////////////////////////////
readRAM
		movwf	ramL

		setf	TRISB			;PORTB ALL as input
		bcf		emul_PIN_RAMROM	;RAMROM = 0 = RAM
		bsf		emul_PIN_WR		;indicate reading RAM...
		
		;***********
		movlw	0x0F		;put address in ports
		andwf	PORTA,F		;clean low nibble
		movf	ramL,W
		andlw	0x0F
		iorwf	PORTA,F		;put address [A0-A3] bits

		movlw	0xF0		;put address in ports
		andwf	PORTC,F		;clean high nibble
		movf	ramL,W
		andlw	0xF0
		iorwf	PORTC,F		;put address [A4-A7] bits

		movff	ramH,PORTD		;put ramH byte in address [A8-A15] bits
		;***********
		
		bcf		emul_PIN_RD			;RD=0=ASK FOR RAM DATA

		nop							;wait 83nS
		movf	PORTB,W				;read RAM byte...

		bsf		emul_PIN_RD			;RD=1=RAM READ DONE...

		return


;////////////////////////////////************//////////////////////////////
;////////////////////////////////************//////////////////////////////
;SUB-ROUTINES
;////////////////////////////////************//////////////////////////////
;////////////////////////////////************//////////////////////////////

;/////////////////////////////////
;READS ACTUAL FLASH PC POSITION..+ AUTO POST INCREMENT PC
;////////////////////////////////
ReadOP
		movwf	counter			;store number of bytes to read...
		movf	counter,f
		btfsc	STATUS,Z		;counter=0?
		return					;yes, return

		setf	TRISB			;PORTB ALL as input
		bsf		emul_PIN_RAMROM	;RAMROM = 1 = ROM

NEXTBYTE
		;***********
		movlw	0x0F		;put address in ports
		andwf	PORTA,F		;clean low nibble
		movf	emulPCL,W
		andlw	0x0F
		iorwf	PORTA,F		;put address [A0-A3] bits

		movlw	0xF0		;put address in ports
		andwf	PORTC,F		;clean high nibble
		movf	emulPCL,W
		andlw	0xF0
		iorwf	PORTC,F		;put address [A4-A7] bits

		movff	emulPCH,PORTD	;put address [A8-A15] bits
		;***********

		bcf		emul_PIN_RD			;RD=0=ASK FOR ROM DATA

		infsnz	emulPCL,F			;increment emulPC address
		incf	emulPCH,F

		movf	PORTB,W				;read FLASH Data...
		movwf	POSTINC0			;and store it in PIC RAM...(+ increment indirect file addressing position...)

		bsf		emul_PIN_RD			;RD=1=ROM READ DONE...

		decfsz	counter,F			;decrement bytes to read...
		bra		NEXTBYTE			;continue reading next byte
		return
;////////////////////////////////

		END


;*******************************
;READS A BYTE FROM FLASH MEMORY
;*******************************
emulReadFlash	MACRO	DirecL,DirecH
		setf	TRISB			;PORTB ALL as input
		bsf		emul_PIN_RAMROM	;RAMROM = 1 = ROM

		;***********
		movlw	0x0F		;put address in ports
		andwf	PORTA,F		;clean low nibble
		movf	DirecL,W
		andlw	0x0F
		iorwf	PORTA,F		;put address [A0-A3] bits

		movlw	0xF0		;put address in ports
		andwf	PORTC,F		;clean high nibble
		movf	DirecL,W
		andlw	0xF0
		iorwf	PORTC,F		;put address [A4-A7] bits

		movff	DirecH,PORTD	;put address [A8-A15] bits
		;***********

		bcf		emul_PIN_RD			;RD=0=ASK FOR ROM DATA

		nop							;83nS delay..
		nop							;83nS delay..

		movf	PORTB,W				;read Data...

		bsf		emul_PIN_RD			;RD=1=ROM READ DONE...
	endm