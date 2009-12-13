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
	MODIFY_FLAGS	(FLAG_N | FLAG_Z | FLAG_C)
	
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
;relative BCC oper 90 2 2** 
;///////////////////////////////////
OP90
	btfsc	emul_C			;Carry clear?
	goto	EXECUTE

	movf	emulOP+1,W		;Yes. Branch!
	addwf	emulPC,F
	btfsc	STATUS,C		;page overflow?
	incf	emulPC+1,F		;yes, increase page number

	MODIFY_FLAGS	(0)		;no flags affected
	goto	EXECUTE			;done!

;**********************************************************************
; INSTRUCTIONS!!!
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
;relative BCS oper B0 2 2** 
;///////////////////////////////////
OPB0
	btfss	emul_C			;Carry set?
	goto	EXECUTE

	movf	emulOP+1,W		;Yes. Branch!
	addwf	emulPC,F
	btfsc	STATUS,C		;page overflow?
	incf	emulPC+1,F		;yes, increase page number

	MODIFY_FLAGS	(0)		;no flags affected
	goto	EXECUTE			;done!


;**********************************************************************
; INSTRUCTIONS!!!
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
;relative BEQ oper F0 2 2** 
;///////////////////////////////////
OPF0
	btfss	emul_Z			;Zero set?
	goto	EXECUTE

	movf	emulOP+1,W		;Yes. Branch!
	addwf	emulPC,F
	btfsc	STATUS,C		;page overflow?
	incf	emulPC+1,F		;yes, increase page number	

	MODIFY_FLAGS	(0)		;no flags affected
	goto	EXECUTE

;**********************************************************************
; INSTRUCTIONS!!!
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
;zeropage BIT oper 24 2 3 
;///////////////////////////////////
OP24
	call	ZPG
	bra		BIT_COMMON

;///////////////////////////////////
;OPCODE 0x2C:		BIT
;absolute BIT oper 2C 3 4 
;///////////////////////////////////
OP2C
	call	ABS
	bra		BIT_COMMON

;**********************************************************************
; INSTRUCTIONS!!!
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
;///////////////////////////////////
;///////////////////////////////////
;OPCODE 0x30:		BMI		
;relative BMI oper 30 2 2** 
;///////////////////////////////////
OP30	
	btfss	emul_N			;Negative set?
	goto	EXECUTE

	movf	emulOP+1,W		;Yes. Branch!
	addwf	emulPC,F
	btfsc	STATUS,C		;page overflow?
	incf	emulPC+1,F		;yes, increase page number	

	MODIFY_FLAGS	(0)		;no flags affected
	goto	EXECUTE


;**********************************************************************
; INSTRUCTIONS!!!
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
;///////////////////////////////////
;///////////////////////////////////
;OPCODE 0xD0:		BMI		
;relative BNE oper D0 2 2** 
;///////////////////////////////////
OPD0	
	btfsc	emul_Z			;Zero clear?
	goto	EXECUTE

	movf	emulOP+1,W		;Yes. Branch!
	addwf	emulPC,F
	btfsc	STATUS,C		;page overflow?
	incf	emulPC+1,F		;yes, increase page number	

	MODIFY_FLAGS	(0)		;no flags affected
	goto	EXECUTE


;**********************************************************************
; INSTRUCTIONS!!!
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
;///////////////////////////////////
;///////////////////////////////////
;OPCODE 0x10:		BMI		
;relative BPL oper 10 2 2** 
;///////////////////////////////////
OP10	
	btfsc	emul_N			;Negative clear?
	goto	EXECUTE

	movf	emulOP+1,W		;Yes. Branch!
	addwf	emulPC,F
	btfsc	STATUS,C		;page overflow?
	incf	emulPC+1,F		;yes, increase page number	

	MODIFY_FLAGS	(0)		;no flags affected
	goto	EXECUTE


;**********************************************************************
; INSTRUCTIONS!!!
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
;///////////////////////////////////



;**********************************************************************
; INSTRUCTIONS!!!
;**********************************************************************
;BVC 
;Branch on Overflow Clear branch on V = 0 
;N Z C I D V 
;- - - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;relative BVC oper 50 2 2** 

;**********************************************************************
; INSTRUCTIONS!!!
;**********************************************************************
;BVS 
;Branch on Overflow 
;Set branch on V = 1 
;N Z C I D V 
;- - - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;relative BVC oper 70 2 2** 

;**********************************************************************
; INSTRUCTIONS!!!
;**********************************************************************
;CLC 
;Clear Carry Flag 0 -> C 
;N Z C I D V 
;- - 0 - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;implied CLC 18 1 2 

;**********************************************************************
; INSTRUCTIONS!!!
;**********************************************************************
;CLD 
;Clear Decimal Mode 
;0 -> D 
;N Z C I D V
; - - - - 0 - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;implied CLD D8 1 2 

;**********************************************************************
; INSTRUCTIONS!!!
;**********************************************************************
;CLI 
;Clear Interrupt Disable Bit 0 -> I 
;N Z C I D V 
;- - - 0 - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;implied CLI 58 1 2 

;**********************************************************************
; INSTRUCTIONS!!!
;**********************************************************************
;CLV 
;Clear Overflow Flag 0 -> V 
;N Z C I D V 
;- - - - - 0 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;implied CLV B8 1 2 

;**********************************************************************
; INSTRUCTIONS!!!
;**********************************************************************
;CMP 
;Compare Memory with Accumulator 
;A - M 
;N Z C I D V 
;+ + + - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;immidiate CMP #oper C9 2 2 
;zeropage CMP oper C5 2 3 
;zeropage,X CMP oper,X D5 2 4 
;absolute CMP oper CD 3 4 
;absolute,X CMP oper,X DD 3 4* 
;absolute,Y CMP oper,Y D9 3 4* 
;(indirect,X) CMP (oper,X) C1 2 6 
;(indirect),Y CMP (oper),Y D1 2 5* 

;**********************************************************************
; INSTRUCTIONS!!!
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

;**********************************************************************
; INSTRUCTIONS!!!
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

;**********************************************************************
; INSTRUCTIONS!!!
;**********************************************************************
;DEC 
;Decrement Memory by One 
;M - 1 -> M 
;N Z C I D V 
;+ + - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;zeropage DEC oper C6 2 5 
;zeropage,X DEC oper,X D6 2 6 
;absolute DEC oper CE 3 3 
;absolute,X DEC oper,X DE 3 7 

;**********************************************************************
; INSTRUCTIONS!!!
;**********************************************************************
;DEX 
;Decrement Index X by One 
;X - 1 -> X 
;N Z C I D V 
;+ + - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;implied DEC CA 1 2 

;**********************************************************************
; INSTRUCTIONS!!!
;**********************************************************************
;DEY 
;Decrement Index Y by One 
;Y - 1 -> Y 
;N Z C I D V 
;+ + - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;implied DEC 88 1 2 

;**********************************************************************
; INSTRUCTIONS!!!
;**********************************************************************
;EOR 
;Exclusive-OR Memory with Accumulator 
;A EOR M -> A 
;N Z C I D V 
;+ + - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;immidiate EOR #oper 49 2 2 
;zeropage EOR oper 45 2 3 
;zeropage,X EOR oper,X 55 2 4 
;absolute EOR oper 4D 3 4 
;absolute,X EOR oper,X 5D 3 4* 
;absolute,Y EOR oper,Y 59 3 4* 
;(indirect,X) EOR (oper,X) 41 2 6 
;(indirect),Y EOR (oper),Y 51 2 5* 

;**********************************************************************
; INSTRUCTIONS!!!
;**********************************************************************
;INC 
;Increment Memory by One 
;M + 1 -> M 
;N Z C I D V 
;+ + - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;zeropage INC oper E6 2 5 
;zeropage,X INC oper,X F6 2 6 
;absolute INC oper EE 3 6 
;absolute,X INC oper,X FE 3 7 

;**********************************************************************
; INSTRUCTIONS!!!
;**********************************************************************
;INX 
;Increment Index X by One 
;X + 1 -> X 
;N Z C I D V 
;+ + - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;implied INX E8 1 2 

;**********************************************************************
; INSTRUCTIONS!!!
;**********************************************************************
;INY 
;Increment Index Y by One 
;Y + 1 -> Y 
;N Z C I D V 
;+ + - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;implied INY C8 1 2 

;**********************************************************************
; INSTRUCTIONS!!!
;**********************************************************************
;JMP 
;Jump to New Location 
;(PC+1) -> PCL N Z C I D V     
;(PC+2) -> PCH - - - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;absolute JMP oper 4C 3 3 
;indirect JMP (oper) 6C 3 5 

;**********************************************************************
; INSTRUCTIONS!!!
;**********************************************************************
;JSR 
;Jump to New Location Saving Return Address 
;push (PC+2),  N Z C I D V 
;(PC+1) -> PCL - - - - - - 
;(PC+2) -> PCH 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;absolute JSR oper 20 3 6 

;**********************************************************************
; INSTRUCTIONS!!!
;**********************************************************************
;LDA 
;Load Accumulator with Memory 
;M -> A 
;N Z C I D V 
;+ + - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;immidiate LDA #oper A9 2 2 
;zeropage LDA oper A5 2 3 
;zeropage,X LDA oper,X B5 2 4 
;absolute LDA oper AD 3 4 
;absolute,X LDA oper,X BD 3 4* 
;absolute,Y LDA oper,Y B9 3 4* 
;(indirect,X) LDA (oper,X) A1 2 6 
;(indirect),Y LDA (oper),Y B1 2 5* 

;**********************************************************************
; INSTRUCTIONS!!!
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
;zeropage,Y LDX oper,Y B6 2 4 
;absolute LDX oper AE 3 4 
;absolute,Y LDX oper,Y BE 3 4* 

;**********************************************************************
; INSTRUCTIONS!!!
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
;zeropage,X LDY oper,X B4 2 4 
;absolute LDY oper AC 3 4 
;absolute,X LDY oper,X BC 3 4* 

;**********************************************************************
; INSTRUCTIONS!!!
;**********************************************************************
;LSR 
;Shift One Bit Right 
;(Memory or Accumulator) 0 -> [76543210] -> C 
;N Z C I D V 
;- + + - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;accumulator LSR A 4A 1 2 
;zeropage LSR oper 46 2 5 
;zeropage,X LSR oper,X 56 2 6 
;absolute LSR oper 4E 3 6 
;absolute,X LSR oper,X 5E 3 7 

;**********************************************************************
; INSTRUCTIONS!!!
;**********************************************************************
;NOP 
;No Operation --- 
;N Z C I D V 
;- - - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;implied NOP EA 1 2 

;**********************************************************************
; INSTRUCTIONS!!!
;**********************************************************************
;ORA 
;OR Memory with Accumulator 
;A OR M -> A 
;N Z C I D V 
;+ + - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;immidiate ORA #oper 09 2 2 
;zeropage ORA oper 05 2 3 
;zeropage,X ORA oper,X 15 2 4 
;absolute ORA oper 0D 3 4 
;absolute,X ORA oper,X 1D 3 4* 
;absolute,Y ORA oper,Y 19 3 4* 
;(indirect,X) ORA (oper,X) 01 2 6 
;(indirect),Y ORA (oper),Y 11 2 5* 

;**********************************************************************
; INSTRUCTIONS!!!
;**********************************************************************
;PHA 
;Push Accumulator on Stack 
;push A 
;N Z C I D V 
;- - - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;implied PHA 48 1 3 

;**********************************************************************
; INSTRUCTIONS!!!
;**********************************************************************
;PHP 
;Push Processor Status on Stack 
;push SR 
;N Z C I D V 
;- - - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;implied PHP 08 1 3 

;**********************************************************************
; INSTRUCTIONS!!!
;**********************************************************************
;PLA 
;Pull Accumulator from Stack 
;pull A 
;N Z C I D V 
;- - - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;implied PLA 68 1 4 

;**********************************************************************
; INSTRUCTIONS!!!
;**********************************************************************
;PLP 
;Pull Processor Status from Stack 
;pull SR 
;N Z C I D V
;from stack 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;implied PHP 28 1 4 

;**********************************************************************
; INSTRUCTIONS!!!
;**********************************************************************
;ROL 
;Rotate One Bit Left 
;(Memory or Accumulator) C <- [76543210] <- C 
;N Z C I D V 
;+ + + - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;accumulator ROL A 2A 1 2 
;zeropage ROL oper 26 2 5 
;zeropage,X ROL oper,X 36 2 6 
;absolute ROL oper 2E 3 6 
;absolute,X ROL oper,X 3E 3 7 

;**********************************************************************
; INSTRUCTIONS!!!
;**********************************************************************
;ROR 
;Rotate One Bit Right 
;(Memory or Accumulator) C -> [76543210] -> C 
;N Z C I D V 
;+ + + - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;accumulator ROR A 6A 1 2 
;zeropage ROR oper 66 2 5 
;zeropage,X ROR oper,X 76 2 6 
;absolute ROR oper 6E 3 6 
;absolute,X ROR oper,X 7E 3 7 

;**********************************************************************
; INSTRUCTIONS!!!
;**********************************************************************
;RTI 
;Return from Interrupt 
;pull SR, pull PC 
;N Z C I D V 
;from stack 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;implied RTI 40 1 6 

;**********************************************************************
; INSTRUCTIONS!!!
;**********************************************************************
;RTS 
;Return from Subroutine 
;pull PC, PC+1 -> PC 
;N Z C I D V 
;- - - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;implied RTS 60 1 6 

;**********************************************************************
; INSTRUCTIONS!!!
;**********************************************************************
;SBC 
;Subtract Memory from Accumulator with Borrow 
;A - M - C -> A 
;N Z C I D V 
;+ + + - - + 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;immidiate SBC #oper E9 2 2 
;zeropage SBC oper E5 2 3 
;zeropage,X SBC oper,X F5 2 4 
;absolute SBC oper ED 3 4 
;absolute,X SBC oper,X FD 3 4* 
;absolute,Y SBC oper,Y F9 3 4* 
;(indirect,X) SBC (oper,X) E1 2 6 
;(indirect),Y SBC (oper),Y F1 2 5* 

;**********************************************************************
; INSTRUCTIONS!!!
;**********************************************************************
;SEC 
;Set Carry Flag 
;1 -> C 
;N Z C I D V 
;- - 1 - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;implied SEC 38 1 2 

;**********************************************************************
; INSTRUCTIONS!!!
;**********************************************************************
;SED 
;Set Decimal Flag 
;1 -> D 
;N Z C I D V 
;- - - - 1 - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;implied SED F8 1 2 

;**********************************************************************
; INSTRUCTIONS!!!
;**********************************************************************
;SEI 
;Set Interrupt Disable Status 
;1 -> I 
;N Z C I D V 
;- - - 1 - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;implied SEI 78 1 2 

;**********************************************************************
; INSTRUCTIONS!!!
;**********************************************************************
;STA 
;Store Accumulator in Memory 
;A -> M 
;N Z C I D V 
;- - - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;zeropage STA oper 85 2 3 
;zeropage,X STA oper,X 95 2 4 
;absolute STA oper 8D 3 4 
;absolute,X STA oper,X 9D 3 5 
;absolute,Y STA oper,Y 99 3 5 
;(indirect,X) STA (oper,X) 81 2 6 
;(indirect),Y STA (oper),Y 91 2 6 

;**********************************************************************
; INSTRUCTIONS!!!
;**********************************************************************
;STX 
;Store Index X in Memory 
;X -> M 
;N Z C I D V 
;- - - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;zeropage STX oper 86 2 3 
;zeropage,Y STX oper,Y 96 2 4 
;absolute STX oper 8E 3 4 

;**********************************************************************
; INSTRUCTIONS!!!
;**********************************************************************
;STY 
;Sore Index Y in Memory 
;Y -> M 
;N Z C I D V 
;- - - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;zeropage STY oper 84 2 3 
;zeropage,X STY oper,X 94 2 4 
;absolute STY oper 8C 3 4 

;**********************************************************************
; INSTRUCTIONS!!!
;**********************************************************************
;TAX 
;Transfer Accumulator to Index X 
;A -> X 
;N Z C I D V 
;+ + - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;implied TAX AA 1 2 

;**********************************************************************
; INSTRUCTIONS!!!
;**********************************************************************
;TAY 
;Transfer Accumulator to Index Y 
;A -> Y 
;N Z C I D V 
;+ + - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;implied TAY A8 1 2 

;**********************************************************************
; INSTRUCTIONS!!!
;**********************************************************************
;TSX 
;Transfer Stack Pointer to Index X 
;SP -> X 
;N Z C I D V 
;+ + - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;implied TSX BA 1 2 

;**********************************************************************
; INSTRUCTIONS!!!
;**********************************************************************
;TXA 
;Transfer Index X to Accumulator 
;X -> A 
;N Z C I D V 
;+ + - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;implied TXA 8A 1 2 

;**********************************************************************
; INSTRUCTIONS!!!
;**********************************************************************
;TXS 
;Transfer Index X to Stack Register 
;X -> SP 
;N Z C I D V 
;+ + - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;implied TXS 9A 1 2 

;**********************************************************************
; INSTRUCTIONS!!!
;**********************************************************************
;TYA 
;Transfer Index Y to Accumulator 
;Y -> A 
;N Z C I D V 
;+ + - - - - 
;addressing assembler opc bytes cyles 
;-------------------------------------------- 
;implied TYA 98 1 2 



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