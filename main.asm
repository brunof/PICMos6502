;******************************************************************************
	LIST P=18F4550				;Directiva para definir el procesador
	#include <P18F4550.INC>		;Definicion de SFRs para el procesador

;******************************************************************************
;Bits de Configuracion 
;********	Configuracion del Oscilador	**********
    CONFIG	FOSC = INTOSCIO_EC         ;Osc interno, RA6 como pin, USB usa Osc EC
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
emulPC 	res 2		;Stores Program Counter
emulAC	res 1		;Acumulator
emulX	res 1		;X register
emulY	res 1		;Y register
emulSR	res 1		;Status Register
emulSP	res 1		;Stack Register

;emul aux registers
emulOP	res 3		;stores actual OPCODE+CMD readed(3 bytes maximum for 6502)
counter	res 1		;a counter to read X bytes of OPT+CMD :)

aux1	res	1		;aux variables for addressing modes and instruction set
aux2	res	1
aux3	res	1

;aux for writing and reading routines
ramL	res	1
ramH	res	1
data8	res	1

emulTempPC 	res 2	;stores aux PC counter

;auxiliar variables
auxDir		res	2
WREG_TEMP 	res 1			;variable usada para salvar contexto
STATUS_TEMP res 1		;variable usada para salvar contexto
BSR_TEMP 	res 1			;variable usada para salvar contexto

;******************************************************************************
; Definicion de bits constantes
;******************************************************************************

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
MODIFY_FLAGS	MACRO	FLAGS, SOURCEVAR, FINALVAR
		if(FLAGS & FLAG_C)
			MODIFY_C_FLAG
		endif
		if(FLAGS & FLAG_Z)
			MODIFY_Z_FLAG
		endif
		if(FLAGS & FLAG_N)
			MODIFY_N_FLAG	FINALVAR
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
			MODIFY_V_FLAG SOURCEVAR,FINALVAR
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
MODIFY_N_FLAG	MACRO		finalVar
		bcf		emul_N				;modify Negative flag
		btfsc	finalVar,7			;negative bit set?
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
;WARNING! THIS FUNCTION DESTROYS PIC STATUS,Z FLAG!! MUST be the last one in be modified!!!
MODIFY_V_FLAG	MACRO		sourceVar, finalVar
		bcf		emul_V
		movf	sourceVar,W
		xorwf	finalVar,W
		andlw	0x80
		btfss	STATUS,Z
		bsf		emul_V			;hmmm 7th bit change value...it must then, overflowed...:)
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
; Comienzo del programa principal
; El codigo del programa principal es colocado aqui

Main:
;	*** EL codigo principal va aqui ***

;********************	Inicializacion de perifericos	*********************
	movlw	B'01100000'
	movwf	OSCCON					;Ajustamos el oscilador interno a 4 MHz
	movlw	B'00001111'
	movwf	ADCON1,0				;Todos los pines como I/O digitales

	clrf	TRISA,0				;Puerto A como salida	
	clrf	TRISB,0				;Puerto B como salida	
	clrf	TRISC,0				;Puerto C como salida	
	clrf	TRISD,0				;Puerto D como salida
	clrf	TRISE,0				;Puerto E como salida	

;********************	Inicializacion del EMULADOR DE uP	*********************
	;clear emulator Registers...
	clrf	emulPC			;emulator Program Counter...
	clrf	emulPC+1
	clrf	emulAC			;emulator Acumulator
	clrf	emulX			;emulator X pointer
	clrf	emulY			;emulator Y pointer
	movlw	b'00110000'
	movwf	emulSR			;emulator STATUS REGISTER
	clrf	emulSP			;emulator STACK POINTER

EXECUTE
	lfsr	FSR0, emulOP	;load indirect data register FSR0 with start of data array were command will be stored...
	movlw	.1				;indicate that we will read 1 byte...(only OP)
	call	ReadOP

	;split OPCODEs in two tables...
	btfsc	emulOP,7		;check if OPCODE is bigger than 0x7F...
	bra	HIGH_OP

LOW_OP						;OPCODE is smaller than 0x80
	rlncf	emulOP,W		;multiply OCPODE per 2
	addwf	PCL,F
	;goto	OP00			;OP = 0x00. Break interrupt
	goto	OP01			;OP = 0x01. ORA ¡¡¡¡?

	bra		EXECUTE

HIGH_OP						;OPCODE is bigger than 0x7F
	rlncf	emulOP,W		;multiply OCPODE per 2
	addwf	PCL,F

	bra		EXECUTE








;///////////////////////////////////
;OPCODE SET OF INSTRUCTIONS:
;///////////////////////////////////

;///////////////////////////////////
;OPCODE 0x01:	ORA	X, ind
;///////////////////////////////////
OP01
	call	BPX				;execute Indexed Indirect [(BX, X)]


ORA_COMMON
	iorwf	emulAC,F			;store M OR AC in AC

	;finally, set flags
	bcf		emul_Z
	btfsc	STATUS,Z
	bsf		emul_Z

	;I dont know how to interpetate N flag...:( whould this be right?
	bcf		emul_N
	btfsc	emulAC,7
	bsf		emul_N

	goto	EXECUTE

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
	incf	aux1,w				;yes...	
	addwf	emulAC,F			;and store result in Acumulator

	;set flags...	
	MODIFY_FLAGS	(FLAG_N | FLAG_Z | FLAG_C | FLAG_V),aux1,emulAC
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
	MODIFY_FLAGS	(FLAG_N | FLAG_Z),0,emulAC
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
	call	ABSY
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
	rlcf	aux1,F		;CANT USE RLNCF because it wont affect C bit
	
	;set flags...	
	MODIFY_FLAGS	(FLAG_N | FLAG_Z | FLAG_C),0,aux1
	
	movf	aux1,W
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
	rlcf	emulAC,F		;CANT USE RLNCF because it wont affect C bit
	
	;set flags...	
	MODIFY_FLAGS	(FLAG_N | FLAG_Z | FLAG_C),0,emulAC	
	goto	EXECUTE

;///////////////////////////////////
;OPCODE 0x0E:		ASL
;absolute ASL oper
;///////////////////////////////////
OP
	call	ABS
	bra		ASL_SEMICOMMON

;///////////////////////////////////
;OPCODE 0x16:		ASL
;zeropage,X ASL oper,X
;///////////////////////////////////
OP
	call	ZPGX
	bra		ASL_SEMICOMMON

;///////////////////////////////////
;OPCODE 0x1E:		ASL
;absolute,X ASL oper,X
;///////////////////////////////////
OP
	call	ABSX
	bra		ASL_SEMICOMMON



































;///////////////////////ADDRESSING MODES/////////////////////////////**********************************
;///////////////////////ADDRESSING MODES/////////////////////////////**********************************
;///////////////////////ADDRESSING MODES/////////////////////////////**********************************
;///////////////////////ADDRESSING MODES/////////////////////////////**********************************
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
	movlw	.3
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
	addwf	emulPC,F		;add offset to Program Counter
	btfsc	STATUS,C		;check if overflow
	incf	emulPC+1,F		;if OV, then increase high byte of Program Counter
	return

NEG_OFF
	andlw	0x7F			;erase sign
	subwf	emulPC,F
	btfss	STATUS,C		;check if underflow
	decf	emulPC+1,F		;if Underflow, then decrease high byte of Program Counter
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
		btfsc	STATUS,Z		;counter=0?
		return					;si,volver...

		setf	TRISB			;PORTB ALL as input
		bsf		emul_PIN_RAMROM	;RAMROM = 1 = ROM

NEXTBYTE
		;***********
		movlw	0x0F		;put address in ports
		andwf	PORTA,F		;clean low nibble
		movf	emulPC,W
		andlw	0x0F
		iorwf	PORTA,F		;put address [A0-A3] bits

		movlw	0xF0		;put address in ports
		andwf	PORTC,F		;clean high nibble
		movf	emulPC,W
		andlw	0xF0
		iorwf	PORTC,F		;put address [A4-A7] bits

		movff	emulPC+1,PORTD	;put address [A8-A15] bits
		;***********

		bcf		emul_PIN_RD			;RD=0=ASK FOR ROM DATA

		infsnz	emulPC,F			;increment emulPC address
		incf	emulPC+1,F

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