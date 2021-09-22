
; TITLE ATTiny85 eForth

.nolist
	.include "tn85def.inc"
.list

;===============================================================
;   UART
; Adapted from:
;	UART on attiny85 in C
;   Author: Mark Osborne, BecomingMaker.com
;	http://becomingmaker.com/usi-serial-uart-attiny85/
;	http://becomingmaker.com/usi-serial-send-attiny/
;	https://github.com/MarkOsborne/becomingmaker
;	https://raw.githubusercontent.com/MarkOsborne/becomingmaker/master/USISerial/USISerial.ino
;
;===============================================================
;	tn85eForth v1.0
;	- no need for RWW/NRWW division, the whole flash is NRWW
;	- relocated/redefined variables/buffers
;	- added UART, interrupts, functions
;	- rewrote ?KEY EMIT !IO
;	- rewrote UM* due to missing 'mul Rd, Rr'; corrected appnote
;	- READ WRITE ERASE
;	- rewrote call, CALLL to compile into RCALL instruction
;   - reorganized memory map
;   - reduced number of buffers to one
;	- .ESC ASCII UNICODE TCHARM
;   - rewrote FLUSH - flushes buffer and system variables
;	- TODO DUMP IDUMP
;	- TODO disable PCINT during ."
;	- TODO cursor left/right
;
;	328eForth v2.20, Chen-Hanson Ting, July 2011
;		Fix error, quit, 2/ and ?stack
;
;	328eForth v2.10, Chen-Hanson Ting, March 2011
;	Adapted from 
;		86se4th.asm by Richard Haskell 
;		Amforth by Matthias Trute
;	Assembled with AVR Studio 4 from Atmel
;	-Subroutine threaded model
;	-Uniform byte addressing for flash, RAM and registers
;	-Ping-pong block buffers for optimal flash programming
;	-FORTH interpreter & tools are in NRWW flash
;	-FORTH compiler & user extension are in RWW flash
;	-No interrupt, no multitasking 
;	-turnkey capability
;	-Case insensitive
;	-9600 baud, 1 start, 8 data, no parity, 1 stop bit
;	ANS FORTH compatible, but not compliant.
;
;	Subroutine threaded eForth; Version. 1.0, 1991 
;	by Richard E. Haskell
;	Dept. of Computer Science and Engineering
;	Oakland University
;	Rochester, Michigan 48309
;
;	eForth 1.0 by Bill Muench and C. H. Ting, 1990
;	Much of the code is derived from the following sources:
;	8086 figForth by Thomas Newman, 1981 and Joe smith, 1983
;	aFORTH by John Rible
;	bFORTH by Bill Muench
;
;	The goal of this implementation is to provide a simple eForth Model
;	which can be ported easily to many 8, 16, 24 and 32 bit CPU's.
;	The following attributes make it suitable for CPU's of the '90:
;
;	small machine dependent kernel and portable high level code
;	subroutine threaded code
;	single code dictionaries
;	each word record has a link field, a name field and a code field
;	simple terminal and file interface to host computer
;	aligned with the proposed ANS Forth Standard
;	easy upgrade path to optimize for specific CPU
;	easy mixing of Forth and assembly language
;	all assembly language tools can be used directly
;
;	You are invited to implement this Model on your favorite CPU and
;	contribute it to the eForth Library for public use. You may use
;	a portable implementation to advertise more sophisticated and
;	optimized version for commercial purposes. However, you are
;	expected to implement the Model faithfully. The eForth Working
;	Group reserves the right to reject implementation which deviates
;	significantly from this Model.
;
;	Representing the eForth Working Group in the Silicon Valley FIG Chapter.
;	Send contributions to:
;
;	Dr. Chen-Hanson Ting
;	156 14th Avenue
;	San Mateo, CA 94402
;	(650) 571-7639
;	ting@offete.com
;
;===============================================================

;; Version control

.EQU	VER	=	1	;major release version
.EQU	EXT	=	0	;minor extension

;; Constants

.EQU	COMPO	=	$040	;lexicon compile only bit
.EQU	IMEDD	=	$080	;lexicon immediate bit

.EQU	BASEE	=	16	;default radix

.EQU	BKSPP	=	8	;back space
.EQU	LF	=	10	;line feed
.EQU	CRR	=	13	;carriage return

.EQU	RETT	=	$9508
;.EQU	CALLL	=	$940E	;long call, 2 words instruction
.EQU	CALLL	=	$D000	;relative call, 1 word instruction

;PAGESIZE is defined in the .inc file
;.EQU	PAGESIZE =	32	;flash pagesize in words

;; Memory allocation for attiny85, all byte addresses
;
;	Flash memory
;	$0		Reset and interrupt vectors
;	$60	Initial values for variables
;	$xx	interrupt routines
;	---$200	Start of compiler and user words
;	---$1000	Start of interpreter words
;	$1FFF	End of flash memory
;
;	RAM memory
;	$0	CPU and I/O registers
;	$60	Variables
;	$7C	OLDER, 2 bytes
;	$80	Terminal input buffer, 80 bytes
;	$d0	Free RAM memory (DTOP)
;	($110) PAD (HERE+$40 = DTOP + $40), grows to lower addresses
;	$1b0	Top of data stack, grows to lower addresses
;	$21e	Top of return stack, grows to lower addresses
;	$220	Flash buffer 0, 64 bytes
;	$25F	End of RAM memory

.EQU	RPP	=	$21e	;start of return stack (RP0)
.EQU	TIBB=	$80		;terminal input buffer (TIB)
.EQU	UPP	=	$060	;start of user area (UP0)
.EQU	UPPF=	$020	;start of user area in flash (word address)
.EQU	SPP	=	$1b0	;start of data stack (SP0)

;;	Flash programmming

.EQU	BUF0	=	$220
.EQU	OLDER	=	$7C 		;flash pointer
.equ    BUFINIT0=   $1F80
; buffer pointer word format:	dirty,page_addr,cell_addr,buf?
.equ    BUFMASK_DIRTYBIT    =   $8000
.equ    BUFMASK_PAGEADDR    =   $7fc0
.equ    BUFMASK_BYTEADDR    =   $3f
.equ    BUF_NEW_ADDR        =   $3e


;;----------------------------------------
;; UART

; Adapted from:
; Author: Mark Osborne, BecomingMaker.com
; See accompanying post http://becomingmaker.com/usi-serial-uart-attiny85/ 
; 
; An example of USI Serial for ATtiny25/45/85.
;   
;  ATTiny85 Hookup
;
;      RESET -|1 v 8|- Vcc
; LED -  PB3 -|2   7|- PB2/SCK
;        PB4 -|3   6|- PB1/MISO     = Tx
;        GND -|4 _ 5|- PB0/MOSI/SDA = Rx
;
;ATTiny85 PB0/MOSI/SDA -> Serial UART Rx, connect to Tx of serial input device
;ATTiny85 PB1/MISO/DO = Serial UART Tx -> connect to Rx of serial output device
;ATTiny85 PB3 -> connect to LED
;
; Supported combinations:
;   F_CPU 1000000   BAUDRATE 1200, 2400 
;   F_CPU 8000000   BAUDRATE 9600, 19200
;   F_CPU 16000000  BAUDRATE 9600, 19200, 28800, 38400

; Set your baud rate and here
; .EQU BAUDRATE = 9600
.EQU BAUDRATE = 19200
.EQU F_CPU    = 8000000
.EQU STOPBITS = 1

; If bit width in cpu cycles is greater than 255 then  divide by 8 to fit in timer
; Calculate prescaler setting
.EQU CYCLES_PER_BIT  = ( F_CPU / BAUDRATE )
.if (CYCLES_PER_BIT > 255)
	.EQU DIVISOR     = 8
	.EQU CLOCKSELECT = 2
.else
	.EQU DIVISOR     = 1
	.EQU CLOCKSELECT = 1
.endif
.EQU FULL_BIT_TICKS  = ( CYCLES_PER_BIT / DIVISOR )
.EQU HALF_BIT_TICKS  = ( FULL_BIT_TICKS / 2)

; Number of code CPU cycles from from pin change to starting USI timer
.EQU START_DELAY     = ( 99 )

; Number of CPU cycles delay after setting COMPA timer until global interrupt is enabled
.EQU COMPA_DELAY     = 42
.EQU TIMER_MIN       = ( COMPA_DELAY / DIVISOR )
  
.EQU TIMER_START_DELAY = ( START_DELAY  / DIVISOR )
.if (HALF_BIT_TICKS - TIMER_START_DELAY)>0
	.EQU TIMER_TICKS = ( HALF_BIT_TICKS - TIMER_START_DELAY )
	.if (TIMER_TICKS < TIMER_MIN)
		.warning "TIMER_TICKS too low, USI bit sample will be after center of bit"
	.endif
.else
	.error "TIMER_TICKS invalid, choose different values for F_CPU, BAUDRATE and START_DELAY"
	.EQU TIMER_TICKS = 1
.endif

;; UART
;;----------------------------------------


;; Initialize assembly variable

.SET _LINK		=	0		;init a null link

;	Compile a code definition header.

.MACRO	CODE				;;LEX,NAME 
	.DW		_LINK*2			;;link pointer
	.SET _LINK	=	pc		;;link points to a name string
	.DB		@0,@1
	.ENDM

;	Colon header is identical to code header.

.MACRO	COLON				;;LEX,NAME,LABEL
	.DW		_LINK*2			;;link pointer
	.SET _LINK	=	pc		;;link points to a name string
	.DB		@0,@1
	.ENDM

;; Macros defined by amForth

; The CPU registers are assigned various functions required in a FORTH Virtual
; Machine as follows:
;+------------+---------------+---------------------------------------------+
;|Register    | Alternate Name| Function                                    |
;|------------+---------------+---------------------------------------------+
;|pc          |               | Program counter                             |
;|sp          |               | Return stack pointer                        |
;|r0          |               | Reserved for multiply and memory operations |
;|r1          |               | Reserved for multiply and memory operations |
;|r2          | zerol         | Provide constant 0                          |
;|r3          | zeroh         | Provide constant 0                          |
;|r4          |               | UART                                        |
;|r5          |               | UART                                        |
;|r6          |               | UART                                        |
;|r7          |               | UART                                        |
;|r8          |               | UART                                        |
;|r9          | dirty         | flash buffer dirty flag                     |
;|r10         |               | Not used                                    |
;|r11         |               | Not used                                    |
;|r12         |               | Not used                                    |
;|r13         |               | Not used                                    |
;|r14         | temp4         | Scratch pad                                 |
;|r15         | temp5         | Scratch pad                                 |
;|r16         | temp0         | Scratch pad                                 |
;|r17         | temp1         | Scratch pad                                 |
;|r18         | temp2         | Scratch pad                                 |
;|r19         | temp3         | Scratch pad                                 |
;|r20         | temp6         | Scratch pad                                 |
;|r21         | temp7         | Scratch pad                                 |
;|r22         | looplo        | Flash memory operations                     |
;|r23         | loophi        | Flash memory operations                     |
;|r24         | tosl          | Top of parameter stack low                  |
;|r25         | tosh          | Top of parameter stack high                 |
;|r26         | x1            | Scratch pad                                 |
;|r27         | xh            | Scratch pad                                 |
;|r28         | yl            | Parameter stack pointer low                 |
;|r29         | yh            | Parameter stack pointer high                |
;|r30         | zl            | Used for memory address low                 |
;|r31         | zh            | Used for memory address high                |
;+------------+---------------+---------------------------------------------+
.DEF zerol = r2
.DEF zeroh = r3
.DEF temp4 = r14
.DEF temp5 = r15
.DEF temp0 = r16
.DEF temp1 = r17
.DEF temp2 = r18
.DEF temp3 = r19
.DEF temp6 = r20
.DEF temp7 = r21
.DEF tosl = r24
.DEF tosh = r25

.def usi_state		=r4
.def serialRxDataReady	=r5  ; 0=false ff=true
.def serialRxData	=r6
.def serialTxData	=r7
.def serialTxBuffer	=r8
.def dirty			=r9

.equ USI_STATE_AVAILABLE	=0
.equ USI_STATE_FIRST		=1
.equ USI_STATE_SECOND		=2
.equ USI_STATE_RECEIVING	=3


; popD
.macro loadtos
	ld tosl, Y+
	ld tosh, Y+
.endmacro

; pushD
.macro savetos
	st -Y, tosh
	st -Y, tosl
.endmacro

.macro in_
.if (@1 < $40)
  	in @0,@1
.else
  	lds @0,@1
.endif
.endmacro

.macro out_
.if (@0 < $40)
  	out @0,@1
.else
  	sts @0,@1
.endif
.endmacro

.macro readflashcell
	lsl zl
	rol zh
	lpm @0, Z+
	lpm @1, Z+
.endmacro

.macro writeflashcell
	lsl zl
	rol zh
.endmacro

;; Main entry points and COLD start data

	.CSEG
	.ORG	0
	RJMP	ORIG ; Address 0x0000
	RETI			;RJMP	INT0_ISR       ; Address 0x0001
	RJMP	PCINT0_ISR     ; Address 0x0002
	RETI			;RJMP	TIM1_COMPA_ISR ; Address 0x0003
	RETI			;RJMP	TIM1_OVF_ISR   ; Address 0x0004
	RETI			;RJMP	TIM0_OVF_ISR   ; Address 0x0005
	RETI			;RJMP	EE_RDY_ISR     ; Address 0x0006
	RETI			;RJMP	ANA_COMP_ISR   ; Address 0x0007
	RETI			;RJMP	ADC_ISR        ; Address 0x0008
	RETI			;RJMP	TIM1_COMPB_ISR ; Address 0x0009
	RJMP	TIM0_COMPA_ISR ; Address 0x000A
	RETI			;RJMP	TIM0_COMPB_ISR ; Address 0x000B
	RETI			;RJMP	WDT_ISR        ; Address 0x000C
	RETI			;RJMP	USI_START_ISR  ; Address 0x000D
	RJMP	USI_OVF_ISR    ; Address 0x000E

	.ORG	UPPF	;byte address $40, copy to ram on boot, 
					;saved from ram for turnkey system
	
UZERO:	
	.DW		HI*2	    ;'BOOT
	.DW		0	        ;reserved
	.DW		BASEE	    ;BASE
	.DW		0	        ;tmp
	.DW		0	        ;SPAN
	.DW		0	        ;>IN
	.DW		0	        ;#TIB
	.DW		TIBB	    ;TIB
	.DW		INTER*2	    ;'EVAL
	.DW		0	        ;HLD
	.DW		LASTN	    ;CONTEXT pointer
	.DW		CTOP	    ;CP (see the end of the file)
	.DW		DTOP	    ;DP ($0a) (see the end of the file)
	.DW		LASTN	    ;LAST (see the end of the file)
	.DW		BUFINIT0    ;PTR0 to BUF0
	.DW		$7F         ;TCHARM



	.ORG	UPPF+PAGESIZE	; reserve one page for flash operations


; Will fire for all enabled pin change interrupt pins
PCINT0_ISR:
	push temp0
	in temp0, SREG
	push temp0

	sbic PINB, PINB0                ; Trigger only if DI is Low
	RJMP temp0reti

	ldi temp0, USI_STATE_AVAILABLE
	cp temp0, usi_state
	brne temp0reti

	ldi temp0, USI_STATE_RECEIVING
	mov usi_state, temp0

	in  temp0, GIMSK
    andi temp0, ~(1<<PCIE)          ; Disable pin change interrupts
    out GIMSK, temp0

    ldi temp0, 2<<WGM00             ; CTC mode
    out TCCR0A, temp0

    ldi temp0, CLOCKSELECT;         ; Set prescaler to clk or clk /8
    out TCCR0B, temp0

    in  temp0, GTCCR
    ori temp0, 1 << PSR0;           ; Reset prescaler
    out GTCCR, temp0

    ldi temp0, TIMER_TICKS;         ; Delay to the middle of start bit accounting for interrupt startup and code execution delay before timer start
    out OCR0A, temp0

    ldi temp0, 0;                   ; Count up from 0
    out TCNT0, temp0

    ldi temp0, 1 << OCF0A;          ; Clear output compare interrupt flag
    out TIFR, temp0

    in  temp0, TIMSK
    ori temp0, 1<<OCIE0A;           ; Enable output compare interrupt
    out TIMSK, temp0
temp0reti:
	pop temp0
	out SREG, temp0
	pop temp0
	RETI


TIM0_COMPA_ISR:
    ; COMPA interrupt indicates middle of bit 0
	push temp0
	in temp0, SREG
	push temp0

    in temp0, TIMSK
    andi temp0, ~(1<<OCIE0A)
    out TIMSK, temp0                ; Disable COMPA interrupt

    ldi temp0, 0;                   ; Count up from 0
    out TCNT0, temp0

    ldi temp0, FULL_BIT_TICKS       ; Shift every bit width
    out OCR0A, temp0

    ; Enable USI OVF interrupt, and select Timer0 compare match as USI Clock source:
    ldi temp0, 1<<USIOIE | 0<<USIWM0 | 1<<USICS0;
    out USICR, temp0

    ; Clear Start condition interrupt flag, USI OVF flag, and set counter
    ldi temp0, 1<<USIOIF | 8;
    out USISR, temp0

	pop temp0
	out SREG, temp0
	pop temp0
	RETI


; USI overflow interrupt indicates we've received a byte
; or we've sent a byte
USI_OVF_ISR:
	push temp0
	in temp0, SREG
	push temp0

	ldi temp0, USI_STATE_FIRST
	cp temp0, usi_state
	brne OvrSecond

	ldi temp0, USI_STATE_SECOND
	mov usi_state, temp0

	mov temp0, serialTxBuffer ; already reverced
	; initial bit 7 is now bit 0,
	; to send it, place it back to bit 7
    ror temp0               ; save data bit
    ser temp0               ; stop bit (high)
    ror temp0               ; restore data bit
	out USIDR, temp0

    ; Clear USI overflow interrupt flag
    ; Set USI counter to send last data bit and stop bits
    ldi temp0, 1<<USIOIF | (16 - (1 + (STOPBITS)))
	out USISR, temp0

	RJMP temp0retiOvr
OvrSecond:
	ldi temp0, USI_STATE_SECOND
	cp temp0, usi_state
	brne OvrReceiving

	sbi DDRB,  PB1                    ; Configure USI_DO as output.
	sbi PORTB, PB1                    ; Ensure output is high

	clr temp0
    out USICR, temp0                  ; Disable USI.

    ldi temp0, 1<<USIOIF              ; clear interrupt flag
    out USISR, temp0

	ldi temp0, USI_STATE_AVAILABLE
	mov usi_state, temp0

	in  temp0, GIMSK
	ori temp0, 1<<PCIE      ; Enable pin change interrupts
	out GIMSK, temp0

	RJMP temp0retiOvr
OvrReceiving:
	ldi temp0, USI_STATE_RECEIVING
	cp temp0, usi_state
	brne temp0retiOvr

    in serialRxData, USIBR	; got temp data

	clr temp0
	out USICR, temp0        ; Disable USI.

	; ReverseByte
	clr temp0
	rol serialRxData		; 7
	ror temp0
	rol serialRxData		; 6
	ror temp0
	rol serialRxData		; 5
	ror temp0
	rol serialRxData		; 4
	ror temp0
	rol serialRxData		; 3
	ror temp0
	rol serialRxData		; 2
	ror temp0
	rol serialRxData		; 1
	ror temp0
	rol serialRxData		; 0
	ror temp0
	mov serialRxData, temp0
    clr serialRxDataReady
	com serialRxDataReady	; = true;

	ldi temp0, USI_STATE_AVAILABLE
	mov usi_state, temp0

	ldi temp0, 1<<PCIF
    out GIFR, temp0         ; Clear pin change interrupt flag.

	in  temp0, GIMSK
	ori temp0, 1<<PCIE      ; Enable pin change interrupts
	out GIMSK, temp0
	; We are still in the middle of bit 7 and if it is low we will get a pin change event
	; for the stop bit, but we will ignore it because it is high
temp0retiOvr:
	pop temp0
	out SREG, temp0
	pop temp0
	RETI



usiserial_send_byte:
	mov temp0, usi_state
	cpi temp0, USI_STATE_AVAILABLE
	brne usiserial_send_byte        ; Spin until we finish sending previous packet

	in  temp0, GIMSK
	andi temp0, ~(1<<PCIE)      	; Disable pin change interrupts
	out GIMSK, temp0

	ldi temp0, USI_STATE_FIRST
	mov usi_state, temp0

	mov serialTxBuffer, serialTxData

	; ReverseByte
	clr temp0
	rol serialTxBuffer		; 7
	ror temp0
	rol serialTxBuffer		; 6
	ror temp0
	rol serialTxBuffer		; 5
	ror temp0
	rol serialTxBuffer		; 4
	ror temp0
	rol serialTxBuffer		; 3
	ror temp0
	rol serialTxBuffer		; 2
	ror temp0
	rol serialTxBuffer		; 1
	ror temp0
	rol serialTxBuffer		; 0
	ror temp0
	mov serialTxBuffer, temp0

    ; Configure Timer0
    ldi temp0, 2<<WGM00             ; CTC mode
    out TCCR0A, temp0

    ldi temp0, CLOCKSELECT;         ; Set prescaler to clk or clk /8
    out TCCR0B, temp0

    in  temp0, GTCCR
    ori temp0, 1 << PSR0;           ; Reset prescaler
    out GTCCR, temp0

    ldi temp0, FULL_BIT_TICKS;      ; Trigger every full bit width
    out OCR0A, temp0

    ldi temp0, 0;                   ; Count up from 0
    out TCNT0, temp0

    ; Configure USI to send high start bit and 7 bits of data
    ; Start bit (low)
    ; followed by first 7 bits of serial data
	mov temp0, serialTxBuffer
	ror temp0
	andi temp0, $7f
	out USIDR, temp0

	; Enable USI Counter OVF interrupt.
	; Select three wire mode to ensure USI written to PB1
	; Select Timer0 Compare match as USI Clock source.
    ldi temp0, (1<<USIOIE)|(0<<USIWM1)|(1<<USIWM0)|(0<<USICS1)|(1<<USICS0)|(0<<USICLK)
	out USICR, temp0

	sbi DDRB,  PB1                    ; Configure USI_DO as output.

	; Clear USI overflow interrupt flag
	; and set USI counter to count 8 bits
    ldi temp0, 1<<USIOIF | (16 - 8)                
	out USISR, temp0

ret



usiserial_init:
;   ; Tweak clock speed for 5V, comment out if running ATtiny at 3V
;   OSCCAL += 3;
    in temp0, OSCCAL
    subi temp0, 0
    out OSCCAL, temp0

	cbi	DDRB, DDB0			; Set pin 0 to input
	sbi	PORTB, PB0			; Enable internal pull-up on pin PB0

   	sbi	DDRB, DDB1          ; Configure USI_DO as output.
   	sbi	PORTB, PB1          ; Ensure serial output is high when idle

	clr usi_state
	clr serialRxDataReady	; set to false

	clr temp0
	out USICR, temp0        ; Disable USI.

	ldi temp0, 1<<PCIF
    out GIFR, temp0         ; Clear pin change interrupt flag.

	in  temp0, GIMSK
	ori temp0, 1<<PCIE      ; Enable pin change interrupts
	out GIMSK, temp0

    sbi PCMSK, PCINT0       ; Enable pin change on pin PB0

	sei
ret



ULAST:
ORIG:	
	in_ 	r10, MCUSR
	clr 	r11
	clr 	zerol
	clr 	zeroh
	out_ 	MCUSR, zerol
    ; clear ram
    clr     xl
    ldi     xh,2
    ldi     yl,$60
    clr     yh
clrram:
    st      y+,zerol
    sbiw    x,1
    brne    clrram
	; init return stack pointer
	ldi 	xl,low(RPP)
	out_ 	SPL,xl
	ldi 	xh,high(RPP)
	out_ 	SPH,xh
	; init parameter stack pointer
	ldi 	yl,low(SPP)
	ldi 	yh,high(SPP)
	; jump to Forth starting word
	RJMP 	COLD



;; Device dependent I/O

;   ?RX	( -- c T | F )
;	Return input character and true, or a false if no input.
	CODE	4,"?KEY"
QRX:																			
QKEY:
	savetos
	clr 	tosl
	clr 	tosh
	movw	tosl,zerol
	sbrs    serialRxDataReady, 0
	ret
	clr     serialRxDataReady
	mov		tosl, serialRxData
	savetos
	ser		tosl
	ser		tosh
	ret



;   TX!	( c -- )
;	Send character c to the output device.
	CODE	4,"EMIT"
EMIT:
TXSTO:	
	mov		serialTxData, tosl
	RCALL	usiserial_send_byte
	loadtos
	ret


;   !IO	( -- )
;	Initialize the serial I/O devices.
;	CODE	3,"!IO"
STOIO:
; Initialize USI for UART reception.
    RCALL usiserial_init
    ret



;; The kernel

;   doLIT	( -- w )
;	Push an inline literal.

;	CODE	COMPO+5,"doLIT"
DOLIT:
	savetos
	pop	zh
	pop	zl
	readflashcell tosl,tosh
	ror	zh
	ror	zl
	push	zl
	push	zh
	ret

;   next	( -- )
;	Run time code for the single index loop.

;	CODE	COMPO+4,"next"
DONXT:
	POP	zh	;ret addr
	POP	zl	;
	pop	xh	;count
	pop	xl
	sbiw	xl, 1
	brge	NEXT1
	adiw	zl,1
	push	zl
	push	zh
	ret
NEXT1:	
	push	xl	;push count back
	push	xh	
	readflashcell	xl,xh
	push	xl
	push	xh
	ret

;   ?branch	( f -- )
;	Branch if flag is zero.

;	CODE	COMPO+7,"?branch"
QBRAN:
	pop	zh
	pop	zl
	or	tosl, tosh
	loadtos
	breq	BRAN1
	adiw	zl,1
	push	zl
	push	zh
	ret

;   branch	( -- )
;	Branch to an inline address.

;	CODE	COMPO+6,"branch"
BRAN:	
	pop	zh
	pop	zl
BRAN1:	
	readflashcell xl,xh
	push	xl
	push	xh
	ret

;   EXECUTE	( b -- )
;	Execute the word at ca=b/2.

	CODE	7,"EXECUTE"
EXECU:
	asr	tosh	;b/2
	ror	tosl
	push	tosl
	push	tosh
	loadtos
	ret

;   EXIT	( -- )
;	Terminate current colon word.

	CODE	4,"EXIT"
EXIT:
	pop	xh
	pop	xl
	ret

;   !	( w a -- )
;	Pop the data stack to memory.

	CODE	1,"!"
STORE:
	movw 	zl, tosl
	loadtos
	std 	Z+1, tosh
	std 	Z+0, tosl
	loadtos
	RET

;   @	( a -- w )
;	Push memory location to the data stack.

	CODE	1,"@"
AT:
	movw 	zl, tosl
	ld 	tosl, z+
	ld 	tosh, z+
	RET

;   I@	( a -- w )
;	Push flash memory cell to the data stack.

	CODE	2,"I@"
IAT:
	tst		dirty
	brne	IAT1		;dirty
IAT0:
	;clear
	movw 	zl, tosl	;fetch from flash
	lpm	tosl, z+
	lpm	tosh, z+
	RET
IAT1:
	;dirty
	RCALL	DOLIT	;a old
	.DW	OLDER
	RCALL	BUFQ	;a old?
	RCALL	QBRAN	;if a=old, fetch n in old_buf
	.DW	IAT2
	RJMP	IAT0	;dirty, not in buffer
IAT2:
	;dirty, in buffer
	RCALL	BUFAT
	RJMP	AT

;   IC@	( a -- w )
;	Push flash memory byte to the data stack.

	CODE	3,"IC@"
ICAT:
	tst		dirty
	brne	ICAT1		;dirty
ICAT0:
	;clear
	movw 	zl, tosl	;fetch from flash
	clr 	tosh
	lpm 	tosl, Z
	RET
ICAT1:
	;dirty
	RCALL	DOLIT	;a old
	.DW		OLDER
	RCALL	BUFQ	;a old?
	RCALL	QBRAN	;if a=old, fetch n in old_buf
	.DW		ICAT2
	RJMP	ICAT0	;dirty, not in buffer
ICAT2:
	;dirty, in buffer
	RCALL	BUFAT
	RJMP	CAT

;	CODE	6,"BUF?"	; a new/old -- a f
; is in the buffer? f=0 -> in buffer
BUFQ:               ; a1 a2
	RCALL	AT      ; a1 n2
	RCALL	OVER    ; a1 n2 a1
	RCALL	XORR    ; a1 n3
	RCALL	DOLIT   ; a1 n3 m
	.DW	BUFMASK_PAGEADDR
	RCALL	ANDD    ; a1 f
	RET

;	CODE	6,"BUF@"	; a new/old -- buuf_addr
BUFAT:
	RCALL	DOLIT
	.DW	BUF0
	RCALL	SWAPP
	RCALL	DOLIT
	.DW	BUFMASK_BYTEADDR
	RCALL	ANDD
	; RJMP	XORR
	RJMP	PLUS    ; !!!!!

;   I!	( w a -- )
;	Store w to flash memory byte location.

	CODE	2,"I!"
ISTOR:
	tst		dirty
	brne	ISTORD	;dirty
	
	;clear
	RCALL	DOLIT	;n a old
	.DW		OLDER
	RCALL	BUFQ	;n a old_ptr
	RCALL	QBRAN	;clear, in buffer -> store
	.DW		ISTOR5

	;clear, not in buffer -> read, store
	RJMP	ISTOR2

ISTORD:
	;dirty
	RCALL	DOLIT	;n a old
	.DW		OLDER
	RCALL	BUFQ	;n a old_ptr
	RCALL	QBRAN	;dirty, in buffer -> store
	.DW		ISTOR5

	;dirty, not in buffer -> flush, read, store

ISTOR1:	RCALL	FLUSH_OLD
ISTOR2:	RCALL	READ_FLASH
ISTOR3:	RCALL	UPDATE_OLD
ISTOR5:	RJMP 	UPDATE_NEW

;	CODE	8,"FLUSHBUF"	; --
FLUSH_OLD:
	RCALL	DOLIT	;old
	.DW	OLDER
	RCALL	AT	;old_ptr
	RCALL	DOLIT
	.DW	BUFMASK_PAGEADDR
	RCALL	ANDD	;flash_addr 
	RCALL	DUPP	;flash_addr flash_addr
	RCALL	ERASE	;flash_addr
;
	RCALL	DOLIT	;flash_addr buf
	.DW	BUF0
	RCALL	SWAPP	;buf flash_addr
	RJMP	WRITE	

;	CODE	4,"@OLD"	;a -- a
READ_FLASH:	;read new flash data into old_buf
	RCALL	DOLIT	;a buf
	.DW	BUF0
	RCALL	OVER	;a buf a
	RCALL	DOLIT
	.DW	BUFMASK_PAGEADDR
	RCALL	ANDD	;a buf flash_addr
	RCALL	SWAPP	;a flash_addr buf
	RJMP	READ	;a

;	CODE	4,"!OLD"	;a -- a
UPDATE_OLD:			;store new page address
	RCALL	DUPP	;a a
	RCALL	DOLIT	;
	.DW	BUFMASK_PAGEADDR
	RCALL	ANDD	;a page_addr
	RCALL	DOLIT
	.DW	OLDER	;a page_addr old
	RJMP	STORE	;a

;	CODE	4,"!NEW"	;n a --
UPDATE_NEW:			;write data to the buffer, set dirty bit
	RCALL	DOLIT	;n a 3e
	.DW	BUF_NEW_ADDR
	RCALL	ANDD	;n disp
	RCALL	DOLIT	;n disp buf
	.DW	BUF0
UPDAT1:
	; RCALL	ORR		;n buff_addr
	RCALL	PLUS	;n buff_addr !!!!!
	RCALL	STORE	;update word in new_buf

	clr		dirty
	com		dirty	;set dirty
	ret

;	EMPTY-BUFFERS ( -- )
	CODE	5,"FLUSH"
; flush buffer and system variables
EMPTY_BUF:
    RCALL   EMPTY_OLD
	RCALL	DOLIT
	.DW		UPP
	RCALL	DOLIT
	.DW		UPPF
    RCALL   TWOSTAR
    RCALL   DUPP
    RCALL   ERASE
    RJMP    WRITE


;	EMPTY_OLD	;flush old buffer if it is dirty

EMPTY_OLD:
	sbrs	dirty, 0
	ret		; clear

	; dirty
	clr		dirty
	RJMP	FLUSH_OLD


;   C!	( c b -- )
;	Pop the data stack to byte memory.

	CODE	2,"C!"
CSTOR:
	movw 	zl, tosl
	loadtos
	st 	Z, tosl
	loadtos
	RET

;   C@	( b -- c )
;	Push byte memory location to the data stack.

	CODE	2,"C@"
CAT:
	movw 	zl, tosl
	clr 	tosh
	ld 	tosl, Z
	RET

;   R>	( -- w )
;	Pop the return stack to the data stack.

	CODE	COMPO+2,"R>"
RFROM:
	savetos
	pop	xh
	pop	xl
	pop 	tosh
	pop 	tosl
	push 	xl
	push 	xh
	RET

;   R@	( -- w )
;	Copy top of return stack to the data stack.

	CODE	2,"R@"
RAT:
	savetos
	pop	xh
	pop	xl
	pop 	tosh
	pop 	tosl
	push 	tosl
	push 	tosh
	push 	xl
	push 	xh
	RET

;   >R	( w -- )
;	Push the data stack to the return stack.

	CODE	COMPO+2,">R"
TOR:
	pop	xh
	pop	xl
	push 	tosl
	push 	tosh
	push 	xl
	push 	xh
	loadtos
	RET

;   SP@	( -- a )
;	Push the current data stack pointer.

	CODE	3,"SP@"
SPAT:
	savetos
	movw	tosl, yl
	RET

;   SP!	( a -- )
;	Set the data stack pointer.

;	CODE	3,"SP!"
SPSTO:
	movw 	yl, tosl
	loadtos
	RET

;   DROP	( w -- )
;	Discard top stack item.

	CODE	4,"DROP"
DROP:
	loadtos
	RET

;   DUP	( w -- w w )
;	Duplicate the top stack item.

	CODE	3,"DUP"
DUPP:
	savetos
	RET

;   SWAP	( w1 w2 -- w2 w1 )
;	Exchange top two stack items.

	CODE	4,"SWAP"
SWAPP:
	movw 	xl, tosl
	ld	tosl,Y+
	ld	tosh,Y+
	st 	-Y, xh
	st 	-Y, xl
	RET

;   OVER	( w1 w2 -- w1 w2 w1 )
;	Copy second stack item to top.

	CODE	4,"OVER"
OVER:
	savetos
	ldd 	tosl, Y+2
	ldd 	tosh, Y+3
	RET

;   0<	( n -- t )
;	Return true if n is negative.

	CODE	2,"0<"
ZLESS:
	tst 	tosh
	movw 	tosl, zerol
	brge 	ZLESS1
	sbiw 	tosl,1
ZLESS1:
	RET

;   AND	( w w -- w )
;	Bitwise AND.

	CODE	3,"AND"
ANDD:
	ld 	xl, Y+
	ld 	xh, Y+
	and 	tosl, xl
	and 	tosh, xh
	RET

;   OR	( w w -- w )
;	Bitwise inclusive OR.

	CODE	2,"OR"
ORR:
	ld 	xl, Y+
	ld 	xh, Y+
	or 	tosl, xl
	or 	tosh, xh
	RET

;   XOR	( w w -- w )
;	Bitwise exclusive OR.

	CODE	3,"XOR"
XORR:
	ld 	xl, Y+
	ld 	xh, Y+
   	eor 	tosl, xl
	eor 	tosh, xh
	RET

;   UM+	( u u -- udsum )
;	Add two unsigned single numbers and return a double sum.

	CODE	3,"UM+"
UPLUS:
	ld 	xl, Y+
	ld 	xh, Y+
	add 	tosl, xl
	adc 	tosh, xh
	savetos
	clr	tosh
	clr	tosl
	rol	tosl
	RET

;; System and user variables

;   doVAR	( -- a )
;	Run time routine for VARIABLE and CREATE.

;	CODE	COMPO+5,"doVAR"
DOVAR:
	savetos
	pop 	zh
	pop 	zl
	readflashcell tosl,tosh
	RET

;   'BOOT	( -- a )
;	Storage of application address.

	COLON	5,"'BOOT"
TBOOT:
	RCALL	DOVAR
	.DW	UPP

;   BASE	( -- a )
;	Storage of the radix base for numeric I/O.

	COLON	4,"BASE"
BASE:
	RCALL	DOVAR
	.DW	UPP+4

;   tmp	( -- a )
;	A temporary storage location used in parse and find.

	COLON	3,"TMP"
TEMP:
	RCALL	DOVAR
	.DW	UPP+6

;   SPAN	( -- a )
;	Hold character count received by EXPECT.

	COLON	4,"SPAN"
SPAN:
	RCALL	DOVAR
	.DW		UPP+8

;   >IN	( -- a )
;	Hold the character pointer while parsing input stream.

	COLON	3,">IN"
INN:
	RCALL	DOVAR
	.DW		UPP+10

;   #TIB	( -- a )
;	Hold the current count in and address of the terminal input buffer.

	COLON	4,"#TIB"
NTIB:
	RCALL	DOVAR
	.DW		UPP+12

;   'TIB	( -- a )
;	Hold the current count in and address of the terminal input buffer.

	COLON	4,"'TIB"
TTIB:
	RCALL	DOVAR
	.DW		UPP+14

;   'EVAL	( -- a )
;	Execution vector of EVAL.

	COLON	5,"'EVAL"
TEVAL:
	RCALL	DOVAR
	.DW		UPP+16

;   HLD	( -- a )
;	Hold a pointer in building a numeric output string.

	COLON	3,"HLD"
HLD:
	RCALL	DOVAR
	.DW		UPP+18

;   CONTEXT	( -- a )
;	A area to specify vocabulary search order.

	COLON	7,"CONTEXT"
CNTXT:
	RCALL	DOVAR
	.DW		UPP+20

;   CP	( -- a )
;	Point to the top of the code dictionary.

	COLON	2,"CP"
CPP:
	RCALL	DOVAR
	.DW		UPP+22

;   DP	( -- a )
;	Point to the free RAM space.

	COLON	2,"DP"
DPP:
	RCALL	DOVAR
	.DW		UPP+24

;   LAST	( -- a )
;	Point to the last name in the name dictionary.

	COLON	4,"LAST"
LAST:
	RCALL	DOVAR
	.DW		UPP+26

;   TCHARM	( -- a )
;	Printable chars mask
	COLON	6,"TCHARM"
TCHARM:
	RCALL	DOVAR
	.DW		UPP+30

;; Common functions

;   2*	( n -- n )
;	Multiply tos by cell size in bytes.

	COLON	2,"2*"
TWOSTAR:
CELLS:
	lsl		tosl
	rol		tosh
	ret

;   2/	( n -- n )
;	Divide tos by cell size in bytes.

	COLON	2,"2/"
TWOSL:
	asr		tosh
	ror		tosl
	ret

;   ALIGNED	( b -- a )
;	Align address to the cell boundary.

;	COLON	7,"ALIGNED"
ALGND:
	adiw	tosl,1
	andi	tosl,254
	ret

;   BL	( -- 32 )
;	Return 32, the blank character.

	COLON	2,"BL"
BLANK:
	savetos
	ldi		tosl,32
	clr		tosh
	ret

;   ?DUP	( w -- w w | 0 )
;	Dup tos if its is not zero.

	COLON	4,"?DUP"
QDUP:
    mov 	temp0, tosl
    or 		temp0, tosh
    breq 	QDUP1
    savetos
QDUP1:
	RET

;   ROT	( w1 w2 w3 -- w2 w3 w1 )
;	Rot 3rd item to top.

	COLON	3,"ROT"
ROT:
    movw 	temp0, tosl
    ld 		temp2, Y+
    ld 		temp3, Y+ 
    loadtos
    st 		-Y, temp3
    st 		-Y, temp2
    st 		-Y, temp1
    st 		-Y, temp0
	RET

;   2DROP	( w w -- )
;	Discard two items on stack.

	COLON	5,"2DROP"
DDROP:
	loadtos
	loadtos
	ret

;   2DUP	( w1 w2 -- w1 w2 w1 w2 )
;	Duplicate top two items.

	COLON	4,"2DUP"
DDUP:
	RCALL	OVER
	RJMP	OVER

;   +	( w w -- sum )
;	Add top two items.

	COLON	1,"+"
PLUS:
    ld 		temp0, Y+
    ld 		temp1, Y+
    add 	tosl, temp0
    adc 	tosh, temp1
	RET

;   NOT	( w -- w )
;	One's complement of tos.

	COLON	6,"INVERT"
INVER:
    com 	tosl
    com 	tosh
	ret

;   NEGATE	( n -- -n )
;	Two's complement of tos.

	COLON	6,"NEGATE"
NEGAT:
	RCALL	INVER
	adiw	tosl,1
	ret

;   DNEGATE	( d -- -d )
;	Two's complement of top double.

	COLON	7,"DNEGATE"
DNEGA:
	RCALL	INVER
	RCALL	TOR
	RCALL	INVER
	RCALL	DOLIT
	.DW	1
	RCALL	UPLUS
	RCALL	RFROM
	RJMP	PLUS

;   -	( n1 n2 -- n1-n2 )
;	Subtraction.

	COLON	1,"-"
SUBB:
    ld 		temp0, Y+
    ld 		temp1, Y+
    sub 	temp0, tosl
    sbc 	temp1, tosh
    movw 	tosl, temp0
	ret

;   ABS		( n -- n )
;	Return the absolute value of n.

	COLON	3,"ABS"
ABSS:
	RCALL	DUPP
	RCALL	ZLESS
	RCALL	QBRAN
	.DW	ABS1
	RJMP	NEGAT
ABS1:	
	RET

;   =	( w w -- t )
;	Return true if top two are equal.

	COLON	1,"="
EQUAL:
	RCALL	XORR
	RCALL	QBRAN
	.DW		EQU1
	RCALL	DOLIT
	.DW		0
	RET
EQU1:
	RCALL	DOLIT
	.DW		-1
	RET

;   U<	( u u -- t )
;	Unsigned compare of top two items.

	COLON	2,"U<"
ULESS:
	RCALL	DDUP
	RCALL	XORR
	RCALL	ZLESS
	RCALL	QBRAN
	.DW		ULES1
	RCALL	SWAPP
	RCALL	DROP
	RJMP	ZLESS
ULES1:
	RCALL	SUBB
	RJMP	ZLESS

;   <	( n1 n2 -- t )
;	Signed compare of top two items.

	COLON	1,"<"
LESS:
	RCALL	DDUP
	RCALL	XORR
	RCALL	ZLESS
	RCALL	QBRAN
	.DW		LESS1
	RCALL	DROP
	RJMP	ZLESS
LESS1:
	RCALL	SUBB
	RJMP	ZLESS

;   MAX	( n n -- n )
;	Return the greater of two top stack items.

	COLON	3,"MAX"
MAX:
	RCALL	DDUP
	RCALL	LESS
	RCALL	QBRAN
	.DW		MAX1
	RCALL	SWAPP
MAX1:
	RJMP	DROP

;   MIN	( n n -- n )
;	Return the smaller of top two stack items.

	COLON	3,"MIN"
MIN:
	RCALL	DDUP
	RCALL	SWAPP
	RCALL	LESS
	RCALL	QBRAN
	.DW		MIN1
	RCALL	SWAPP
MIN1:
	RJMP	DROP

;   WITHIN	( u ul uh -- t )
;	Return true if u is within the range of ul and uh. ( ul <= u < uh )

	COLON	6,"WITHIN"
WITHI:
	RCALL	OVER
	RCALL	SUBB
	RCALL	TOR
	RCALL	SUBB
	RCALL	RFROM
	RJMP	ULESS

;; Divide

;   UM/MOD	( udl udh un -- ur uq )
;	Unsigned divide of a double by a single. Return mod and quotient.

	COLON	6,"UM/MOD"
UMMOD:
    movw 	temp4, tosl
    ld 		temp2, Y+
    ld 		temp3, Y+
    ld 		temp0, Y+
    ld 		temp1, Y+
;; unsigned 32/16 -> 16r16 divide
  ; set 	loop counter
    ldi 	temp6,$10
UMMOD1:
    ; shift left, saving high bit
    clr 	temp7
    lsl 	temp0
    rol 	temp1
    rol 	temp2
    rol 	temp3
    rol 	temp7
  ; try subtracting divisor
    cp 		temp2, temp4
    cpc 	temp3, temp5
    cpc 	temp7,zerol
    brcs 	UMMOD3
UMMOD2:
    ; dividend is large enough
    ; do the subtraction for real
    ; and set lowest bit
    inc 	temp0
    sub 	temp2, temp4
    sbc 	temp3, temp5
UMMOD3:
    dec  	temp6
    brne 	UMMOD1
UMMOD4:
    ; put remainder on stack
    st 		-Y,temp3
    st 		-Y,temp2
    ; put quotient on stack
    movw 	tosl, temp0
	ret

;   M/MOD	( d n -- r q )
;	Signed floored divide of double by single. Return mod and quotient.

	COLON	5,"M/MOD"
MSMOD:
	RCALL	DUPP
	RCALL	ZLESS
	RCALL	DUPP
	RCALL	TOR
	RCALL	QBRAN
	.DW	MMOD1
	RCALL	NEGAT
	RCALL	TOR
	RCALL	DNEGA
	RCALL	RFROM
MMOD1:	
	RCALL	TOR
	RCALL	DUPP
	RCALL	ZLESS
	RCALL	QBRAN
	.DW	MMOD2
	RCALL	RAT
	RCALL	PLUS
MMOD2:	
	RCALL	RFROM
	RCALL	UMMOD
	RCALL	RFROM
	RCALL	QBRAN
	.DW	MMOD3
	RCALL	SWAPP
	RCALL	NEGAT
	RCALL	SWAPP
MMOD3:	
	RET

;   /MOD	( n n -- r q )
;	Signed divide. Return mod and quotient.

	COLON	4,"/MOD"
SLMOD:
	RCALL	OVER
	RCALL	ZLESS
	RCALL	SWAPP
	RJMP	MSMOD

;   MOD	( n n -- r )
;	Signed divide. Return mod only.

	COLON	3,"MOD"
MODD:
	RCALL	SLMOD
	RJMP	DROP


;   /	( n n -- q )
;	Signed divide. Return quotient only.

	COLON	1,"/"
SLASH:
	RCALL	SLMOD
	RCALL	SWAPP
	RJMP	DROP

;; Multiply

;   UM*	( u u -- ud )
;	Unsigned multiply. Return double product.

	COLON	3,"UM*"
UMSTA:
    movw 	temp0, tosl
    loadtos
    ; [temp1:temp0] x [tosh:tosl] -> [temp3:temp2:zh:zl]

;***************************************************************************
;* https://www.microchip.com/en-us/application-notes/an0936
;*
; *     corrected.
; *     before:
; mpy16u:	clr	temp3		;clear 2 highest bytes of result
; 	clr	temp2
; 	ldi	temp6,16	;init loop counter
; 	lsr	tosh
; 	ror	tosl
; m16u_1:	brcc	noad8		;if bit 0 of multiplier set
; *     after
; mpy16u:	clr	temp3		;clear 2 highest bytes of result
; 	clr	temp2
; 	ldi	temp6,16	;init loop counter
; m16u_1:	lsr	tosh
; 	ror	tosl
; 	brcc	noad8		;if bit 0 of multiplier set
; *
;* "mpy16u" - 16x16 Bit Unsigned Multiplication
;*
;* This subroutine multiplies the two 16-bit register variables 
;* mp16uH:mp16uL and mc16uH:mc16uL.
;* The result is placed in m16u3:m16u2:m16u1:m16u0.
;*  
;* Number of words	:14 + return
;* Number of cycles	:153 + return
;* Low registers used	:None
;* High registers used  :7 (mp16uL,mp16uH,mc16uL/m16u0,mc16uH/m16u1,m16u2,
;*                          m16u3,mcnt16u)	
;*
;***************************************************************************
;***** Subroutine Register Variables
;.def	mc16uL	=temp0		;multiplicand low byte
;.def	mc16uH	=temp1		;multiplicand high byte
;.def	mp16uL	=tosl		;multiplier low byte
;.def	mp16uH	=tosh		;multiplier high byte
;.def	m16u0	=zl		;result byte 0 (LSB)
;.def	m16u1	=zh		;result byte 1
;.def	m16u2	=temp2		;result byte 2
;.def	m16u3	=temp3		;result byte 3 (MSB)
;.def	mcnt16u	=temp6		;loop counter
;***** Code
mpy16u:	clr	temp3		;clear 2 highest bytes of result
	clr	temp2
	ldi	temp6,16	;init loop counter
m16u_1:	lsr	tosh
	ror	tosl
	brcc	noad8		;if bit 0 of multiplier set
	add	temp2,temp0	;add multiplicand Low to byte 2 of res
	adc	temp3,temp1	;add multiplicand high to byte 3 of res
noad8:	ror	temp3		;shift right result byte 3
	ror	temp2		;rotate right result byte 2
	ror	zh		;rotate result byte 1 and multiplier High
	ror	zl		;rotate result byte 0 and multiplier Low
	dec	temp6		;decrement loop counter
	brne	m16u_1		;if not done, loop more

	movw 	tosl, zl
	savetos
	movw 	tosl, temp2
	ret

;   *	( n n -- n )
;	Signed multiply. Return single product.

	COLON	1,"*"
STAR:
	RCALL	MSTAR
	RJMP	DROP

;   M*		( n n -- d )
;	Signed multiply. Return double product.

	COLON	2,"M*"
MSTAR:
	RCALL	DDUP
	RCALL	XORR
	RCALL	ZLESS
	RCALL	TOR
	RCALL	ABSS
	RCALL	SWAPP
	RCALL	ABSS
	RCALL	UMSTA
	RCALL	RFROM
	RCALL	QBRAN
	.DW	MSTA1
	RCALL	DNEGA
MSTA1:	
	RET

;   */MOD	( n1 n2 n3 -- r q )
;	Multiply n1 and n2, then divide by n3. Return mod and quotient.

	COLON	5,"*/MOD"
SSMOD:
	RCALL	TOR
	RCALL	MSTAR
	RCALL	RFROM
	RJMP	MSMOD

;   */	( n1 n2 n3 -- q )
;	Multiply n1 by n2, then divide by n3. Return quotient only.

	COLON	2,"*/"
STASL:
	RCALL	SSMOD
	RCALL	SWAPP
	RJMP	DROP

;; Miscellaneous

;   >CHAR	( c -- c )
;	Filter non-printing characters.

;	COLON	5,">CHAR"
TCHAR:
	RCALL	DUPP
	RCALL	BLANK
	RCALL	TCHARM
	RCALL	AT
	RCALL	WITHI
	RCALL	QBRAN
	.DW		TCHAR1
	RET
TCHAR1:	
	RCALL	DROP
	RCALL	DOLIT
	.DW		'_'
	RET


;   DEPTH	( -- n )
;	Return the depth of the data stack.

	COLON	5,"DEPTH"
DEPTH:
	RCALL	SPAT
	RCALL	DOLIT
	.DW		SPP-2
	RCALL	SWAPP
	RCALL	SUBB
	RJMP	TWOSL

;   PICK	( ... +n -- ... w )
;	Copy the nth stack item to tos.

	COLON	4,"PICK"
PICK:
	ADIW	TOSL,1
	RCALL	CELLS
	RCALL	SPAT
	RCALL	PLUS
	RJMP	AT

;; Memory access

;   +!	( n a -- )
;	Add n to the contents at address a.

	COLON	2,"+!"
PSTOR:
	RCALL	SWAPP
	RCALL	OVER
	RCALL	AT
	RCALL	PLUS
	RCALL	SWAPP
	RJMP	STORE

;   COUNT	( b -- b +n )
;	Return count byte of a string and add 1 to byte address.

	COLON	5,"COUNT"
COUNT:
	movw	zl, tosl
	ld		temp0, z+
	movw	tosl, zl
	savetos
	mov		tosl, temp0
	clr		tosh
	ret

;   ICOUNT	( b -- b +n )
;	Return count byte of a string and add 1 to byte address.

	COLON	6,"ICOUNT"
ICOUNT:
	RCALL	DUPP
	adiw	tosl,1
	RCALL	SWAPP
	RJMP	ICAT

;   HERE	( -- a )
;	Return the top of the code dictionary.

	COLON	4,"HERE"
HEREE:
	RCALL	DPP
	RJMP	AT

;   PAD	( -- a )
;	Return the address of the text buffer above the code dictionary.

	COLON	3,"PAD"
PAD:
	RCALL	HEREE
	RCALL	DOLIT
	.DW		$40
	RJMP	PLUS

;   TIB	( -- a )
;	Return the address of the terminal input buffer.

	COLON	3,"TIB"
TIB:
	RCALL	NTIB
	ADIW	TOSL,2
	RJMP	AT

;   @EXECUTE	( a -- )
;	Execute vector stored in address a.

	COLON	8,"@EXECUTE"
ATEXE:
	RCALL	AT
	RCALL	QDUP	;?address or zero
	RCALL	QBRAN
	.DW		EXE1
	RCALL	EXECU	;execute if non-zero
EXE1:
	RET				;do nothing if zero

;   CMOVE	( b1 b2 u -- )
;	Copy u bytes from b1 to b2.

	COLON	5,"CMOVE"
CMOVE:
	RCALL	TOR
	RJMP	CMOV2
CMOV1:
	RCALL	TOR
	RCALL	COUNT
	RCALL	RAT
	RCALL	CSTOR
	RCALL	RFROM
	ADIW	TOSL,1
CMOV2:
	RCALL	DONXT
	.DW		CMOV1
	RJMP	DDROP

;	UPPER	( c -- c' )
;	Change character to upper case

;	COLON	5,"UPPER"
UPPER:
	RCALL	DUPP
	RCALL	DOLIT
	.DW		$61
	RCALL	DOLIT
	.DW		$7B
	RCALL	WITHI
	RCALL	QBRAN
	.DW		UPPER1
	RCALL	DOLIT
	.DW		$5F
	RCALL	ANDD
UPPER1:
	RET

;   UMOVE	( a b u -- )
;	Copy u bytes from b1 to b2, changing to upper case.

;	COLON	5,"UMOVE"
UMOVE:
	RCALL	TOR
	RJMP	UMOV2
UMOV1:
	RCALL	TOR
	RCALL	COUNT
	RCALL	UPPER
	RCALL	RAT
	RCALL	CSTOR
	RCALL	RFROM
	ADIW	TOSL,1
UMOV2:
	RCALL	DONXT
	.DW		UMOV1
	RJMP	DDROP

;   FILL	( b u c -- )
;	Fill u bytes of character c to area beginning at b.

	COLON	4,"FILL"
FILL:
	RCALL	SWAPP
	RCALL	TOR
	RCALL	SWAPP
	RJMP	FILL2
FILL1:
	RCALL	DDUP
	RCALL	CSTOR
	ADIW	TOSL,1
FILL2:
	RCALL	DONXT
	.DW		FILL1
	RJMP	DDROP

;; Numeric output, single precision

;   DIGIT	( u -- c )
;	Convert digit u to a character.

;	COLON	5,"DIGIT"
DIGIT:
	RCALL	DOLIT
	.DW		9
	RCALL	OVER
	RCALL	LESS
	RCALL	DOLIT
	.DW		7
	RCALL	ANDD
	RCALL	PLUS
	RCALL	DOLIT
	.DW		'0'
	RJMP	PLUS

;   EXTRACT	( n base -- n c )
;	Extract the least significant digit from n.

;	COLON	7,"EXTRACT"
EXTRC:
	RCALL	DOLIT
	.DW		0
	RCALL	SWAPP
	RCALL	UMMOD
	RCALL	SWAPP
	RJMP	DIGIT

;   <#	( -- )
;	Initiate the numeric output process.

	COLON	2,"<#"
BDIGS:
	RCALL	PAD
	RCALL	HLD
	RJMP	STORE

;   HOLD	( c -- )
;	Insert a character into the numeric output string.

	COLON	4,"HOLD"
HOLD:
	RCALL	HLD
	RCALL	AT
	SBIW	TOSL,1
	RCALL	DUPP
	RCALL	HLD
	RCALL	STORE
	RJMP	CSTOR

;   #	( u -- u )
;	Extract one digit from u and append the digit to output string.

	COLON	1,"#"
DIG:
	RCALL	BASE
	RCALL	AT
	RCALL	EXTRC
	RJMP	HOLD

;   #S	( u -- 0 )
;	Convert u until all digits are added to the output string.

	COLON	2,"#S"
DIGS:
DIGS1:
	RCALL	DIG
	RCALL	DUPP
	RCALL	QBRAN
	.DW		DIGS2
	RJMP	DIGS1
DIGS2:
	RET

;   SIGN	( n -- )
;	Add a minus sign to the numeric output string.

	COLON	4,"SIGN"
SIGN:
	RCALL	ZLESS
	RCALL	QBRAN
	.DW		SIGN1
	RCALL	DOLIT
	.DW		'-'
	RCALL	HOLD
SIGN1:	RET

;   #>	( w -- b u )
;	Prepare the output string to be TYPE'd.

	COLON	2,"#>"
EDIGS:
	RCALL	DROP
	RCALL	HLD
	RCALL	AT
	RCALL	PAD
	RCALL	OVER
	RJMP	SUBB

;   str		( w -- b u )
;	Convert a signed integer to a numeric string.

;	COLON	3,"str"
STR:
	RCALL	DUPP
	RCALL	TOR
	RCALL	ABSS
	RCALL	BDIGS
	RCALL	DIGS
	RCALL	RFROM
	RCALL	SIGN
	RJMP	EDIGS

;   HEX		( -- )
;	Use radix 16 as base for numeric conversions.

	COLON	3,"HEX"
HEX:
	RCALL	DOLIT
	.DW	16
	RCALL	BASE
	RJMP	STORE

;   DECIMAL	( -- )
;	Use radix 10 as base for numeric conversions.

	COLON	7,"DECIMAL"
DECIM:
	RCALL	DOLIT
	.DW	10
	RCALL	BASE
	RJMP	STORE

;; Numeric input, single precision

;   DIGIT?	( c base -- u t )
;	Convert a character to its numeric value. A flag indicates success.

;	COLON	6,"DIGIT?"
DIGTQ:
	RCALL	TOR
	RCALL	DOLIT
	.DW		'0'
	RCALL	SUBB
	RCALL	DOLIT
	.DW		9
	RCALL	OVER
	RCALL	LESS
	RCALL	QBRAN
	.DW		DGTQ1
	RCALL	DOLIT
	.DW		7
	RCALL	SUBB
	RCALL	DUPP
	RCALL	DOLIT
	.DW		10
	RCALL	LESS
	RCALL	ORR
DGTQ1:
	RCALL	DUPP
	RCALL	RFROM
	RJMP	ULESS

;   NUMBER?	( a -- n T | a F )
;	Convert a number string to integer. Push a flag on tos.

	COLON	7,"NUMBER?"
NUMBQ:
	RCALL	BASE
	RCALL	AT
	RCALL	TOR
	RCALL	DOLIT
	.DW		0
	RCALL	OVER
	RCALL	COUNT
	RCALL	OVER
	RCALL	CAT
	RCALL	DOLIT
	.DW		'$'
	RCALL	EQUAL
	RCALL	QBRAN
	.DW		NUMQ1
	RCALL	HEX
	RCALL	SWAPP
	adiw	tosl,1
	RCALL	SWAPP
	sbiw	tosl,1
NUMQ1:
	RCALL	OVER
	RCALL	CAT
	RCALL	DOLIT
	.DW		'-'
	RCALL	EQUAL
	RCALL	TOR
	RCALL	SWAPP
	RCALL	RAT
	RCALL	SUBB
	RCALL	SWAPP
	RCALL	RAT
	RCALL	PLUS
	RCALL	QDUP
	RCALL	QBRAN
	.DW		NUMQ6
	sbiw	tosl,1
	RCALL	TOR
NUMQ2:
	RCALL	DUPP
	RCALL	TOR
	RCALL	CAT
	RCALL	BASE
	RCALL	AT
	RCALL	DIGTQ
	RCALL	QBRAN
	.DW		NUMQ4
	RCALL	SWAPP
	RCALL	BASE
	RCALL	AT
	RCALL	STAR
	RCALL	PLUS
	RCALL	RFROM
	adiw	tosl,1
	RCALL	DONXT
	.DW		NUMQ2
	RCALL	DROP
	RCALL	RAT
	RCALL	QBRAN
	.DW		NUMQ3
	RCALL	NEGAT
NUMQ3:
	RCALL	SWAPP
	RJMP	NUMQ5
NUMQ4:
	RCALL	RFROM
	RCALL	RFROM
	RCALL	DDROP
	RCALL	DDROP
	RCALL	DOLIT
	.DW		0
NUMQ5:
	RCALL	DUPP
NUMQ6:
	RCALL	RFROM
	RCALL	DDROP
	RCALL	RFROM
	RCALL	BASE
	RJMP	STORE

;; Basic I/O

;   KEY	( -- c )
;	Wait for and return an input character.

	COLON	3,"KEY"
KEY:
KEY1:
	RCALL	QRX
	RCALL	QBRAN
	.DW		KEY1
	RET

;   SPACE	( -- )
;	Send the blank character to the output device.

	COLON	5,"SPACE"
SPACE:
	RCALL	BLANK
	RJMP	EMIT

;   CHARS	( +n c -- )
;	Send n characters to the output device.

;	COLON	5,"CHARS"
CHARS:
	RCALL	SWAPP
	RCALL	TOR
	RJMP	CHAR2
CHAR1:
	RCALL	DUPP
	RCALL	EMIT
CHAR2:
	RCALL	DONXT
	.DW		CHAR1
	RJMP	DROP

;   SPACES	( +n -- )
;	Send n spaces to the output device.

	COLON	6,"SPACES"
SPACS:
	RCALL	BLANK
	RJMP	CHARS

;   TYPE	( b u -- )
;	Output u characters from b.

	COLON	4,"TYPE"
TYPES:
	RCALL	TOR
	RJMP	TYPE2
TYPE1:
	RCALL	COUNT
	RCALL	TCHAR
	RCALL	EMIT
TYPE2:
	RCALL	DONXT
	.DW		TYPE1
	RJMP	DROP

;   ITYPE	( b u -- )
;	Output u characters from b.

	COLON	5,"ITYPE"
ITYPES:
	RCALL	TOR
	RJMP	ITYPE2
ITYPE1:
	RCALL	ICOUNT
	RCALL	TCHAR
	RCALL	EMIT
ITYPE2:
	RCALL	DONXT
	.DW		ITYPE1
	RJMP	DROP

;   CR	( -- )
;	Output a carriage return and a line feed.

	COLON	2,"CR"
CR:
	RCALL	DOLIT
	.DW		CRR
	RCALL	EMIT
	RCALL	DOLIT
	.DW		LF
	RJMP	EMIT

;   do$	( -- a )
;	Return the address of a compiled string.

;	COLON	COMPO+3,"do$"
DOSTR:
	RCALL	RFROM	;ra
	RCALL	RFROM	;ra a
	RCALL	DUPP	;ra a a
	RCALL	DUPP	;ra a a a
	movw	zl,tosl
	readflashcell	tosl,tosh
	clr		tosh	;ra a a count
	RCALL	TWOSL
	RCALL	PLUS
	ADIW	TOSL,1	;ra a a' 
	RCALL	TOR	;ra a
	RCALL	SWAPP	;a ra
	RCALL	TOR	;a
	RCALL	CELLS	;byte address
	RET

;   $"|	( -- a )
;	Run time routine compiled by $". Return address of a compiled string.

;	COLON	COMPO+3,'$'
;	.DB		'"','|'
STRQP:
	RCALL	DOSTR
	RET				;force a call to do$

;   ."|	( -- )
;	Run time routine of ." . Output a compiled string.

;	COLON	COMPO+3,'.'
;	.DB		'"','|'
DOTQP:
	RCALL	DOSTR
	RCALL	ICOUNT
	RJMP	ITYPES

;   .R		( n +n -- )
;	Display an integer in a field of n columns, right justified.

	COLON	2,".R"
DOTR:
	RCALL	TOR
	RCALL	STR
	RCALL	RFROM
	RCALL	OVER
	RCALL	SUBB
	RCALL	SPACS
	RJMP	TYPES

;   U.R	( u +n -- )
;	Display an unsigned integer in n column, right justified.

	COLON	3,"U.R"
UDOTR:
	RCALL	TOR
	RCALL	BDIGS
	RCALL	DIGS
	RCALL	EDIGS
	RCALL	RFROM
	RCALL	OVER
	RCALL	SUBB
	RCALL	SPACS
	RJMP	TYPES

;   U.	( u -- )
;	Display an unsigned integer in free format.

	COLON	2,"U."
UDOT:
	RCALL	BDIGS
	RCALL	DIGS
	RCALL	EDIGS
	RCALL	SPACE
	RJMP	TYPES

;   .		( w -- )
;	Display an integer in free format, preceeded by a space.

	COLON	1,"."
DOT:
	RCALL	BASE
	RCALL	AT
	RCALL	DOLIT
	.DW	10
	RCALL	XORR	;?decimal
	RCALL	QBRAN
	.DW	DOT1
	RJMP	UDOT
DOT1:	
	RCALL	STR
	RCALL	SPACE
	RJMP	TYPES

;   ?	( a -- )
;	Display the contents in a memory cell.

	COLON	1,"?"
QUEST:
	RCALL	AT
	RJMP	DOT

;; Parsing

;   parse	( b u c -- b u delta ; <string> )
;	Scan string delimited by c. Return found string and its offset.

;	COLON	5,"parse"
PARS:
	RCALL	TEMP
	RCALL	STORE
	RCALL	OVER
	RCALL	TOR
	RCALL	DUPP
	RCALL	QBRAN
	.DW		PARS8
	SBIW	TOSL,1
	RCALL	TEMP
	RCALL	CAT
	RCALL	BLANK
	RCALL	EQUAL
	RCALL	QBRAN
	.DW		PARS3
	RCALL	TOR
PARS1:
	RCALL	BLANK
	RCALL	OVER
	RCALL	CAT	;skip leading blanks ONLY
	RCALL	SUBB
	RCALL	ZLESS
	RCALL	INVER
	RCALL	QBRAN
	.DW		PARS2
	ADIW	TOSL,1
	RCALL	DONXT
	.DW		PARS1
	RCALL	RFROM
	RCALL	DROP
	RCALL	DOLIT
	.DW		0
	RCALL	DUPP
	RET
PARS2:
	RCALL	RFROM
PARS3:
	RCALL	OVER
	RCALL	SWAPP
	RCALL	TOR
PARS4:
	RCALL	TEMP
	RCALL	CAT
	RCALL	OVER
	RCALL	CAT
	RCALL	SUBB	;scan for delimiter
	RCALL	TEMP
	RCALL	CAT
	RCALL	BLANK
	RCALL	EQUAL
	RCALL	QBRAN
	.DW		PARS5
	RCALL	ZLESS
PARS5:
	RCALL	QBRAN
	.DW		PARS6
	ADIW	TOSL,1
	RCALL	DONXT
	.DW		PARS4
	RCALL	DUPP
	RCALL	TOR
	RJMP	PARS7
PARS6:
	RCALL	RFROM
	RCALL	DROP
	RCALL	DUPP
	ADIW	TOSL,1
	RCALL	TOR
PARS7:
	RCALL	OVER
	RCALL	SUBB
	RCALL	RFROM
	RCALL	RFROM
	RJMP	SUBB
PARS8:
	RCALL	OVER
	RCALL	RFROM
	RJMP	SUBB

;   PARSE	( c -- b u ; <string> )
;	Scan input stream and return counted string delimited by c.

;	COLON	5,"PARSE"
PARSE:
	RCALL	TOR
	RCALL	TIB
	RCALL	INN
	RCALL	AT
	RCALL	PLUS	;current input buffer pointer
	RCALL	NTIB
	RCALL	AT
	RCALL	INN
	RCALL	AT
	RCALL	SUBB	;remaining count
	RCALL	RFROM
	RCALL	PARS
	RCALL	INN
	RJMP	PSTOR

;   .(	( -- )
;	Output following string up to next ) .

	COLON	IMEDD+2,".("
DOTPR:
	RCALL	DOLIT
	.DW		')'
	RCALL	PARSE
	RJMP	TYPES

;   (	( -- )
;	Ignore following string up to next ) . A comment.

	COLON	IMEDD+1,"("
PAREN:
	RCALL	DOLIT
	.DW		')'
	RCALL	PARSE
	RJMP	DDROP

;   \	( -- )
;	Ignore following text till the end of line.

	COLON	IMEDD+1,"\\"
BKSLA:
	RCALL	DOLIT
	.DW		$D
	RCALL	PARSE
	RJMP	DDROP


;   CHAR	( -- c )
;	Parse next word and return its first character.

	COLON	4,"CHAR"
CHARR:
	RCALL	BLANK
	RCALL	PARSE
	RCALL	DROP
	RJMP	CAT

;   TOKEN	( -- a ; <string> )
;	Parse a word from input stream and copy it to name dictionary.

;	COLON	5,"TOKEN"
TOKEN:
	RCALL	BLANK
	RCALL	PARSE
	RCALL	DOLIT
	.DW		31
	RCALL	MIN
	RCALL	HEREE
	RCALL 	DDUP
	RCALL	CSTOR
	RCALL 	DDUP
	RCALL	PLUS
	ADIW	TOSL,1
	RCALL	DOLIT
	.DW		0
	RCALL	SWAPP
	RCALL	CSTOR
	ADIW	TOSL,1
	RCALL	SWAPP
	RCALL	UMOVE
	RJMP	HEREE

;   WORD	( c -- a ; <string> )
;	Parse a word from input stream and copy it to code dictionary.

	COLON	4,"WORD"
WORDD:
	RCALL	PARSE
	RCALL	HEREE
	RCALL 	DDUP
	RCALL	CSTOR
	RCALL 	DDUP
	RCALL	PLUS
	ADIW	TOSL,1
	RCALL	DOLIT
	.DW		0
	RCALL	SWAPP
	RCALL	CSTOR
	ADIW	TOSL,1
	RCALL	SWAPP
	RCALL	CMOVE
	RJMP	HEREE

;; Dictionary search

;   NAME>	( na -- ca )
;	Return a code address given a name address.

	COLON	5,"NAME>"
NAMET:
	RCALL	ICOUNT
	RCALL	DOLIT
	.DW		$1F
	RCALL	ANDD
	RCALL	PLUS
	RJMP	ALGND

;   SAME?	( b a u -- b a f \ -0+ )
;	Compare u bytes in two strings. Return 0 if identical.

;	COLON	5,"SAME?"
SAMEQ:
	RCALL	TWOSL
	RCALL	TOR
	RJMP	SAME2
SAME1:
	RCALL	OVER
	RCALL	RAT
	RCALL	CELLS
	RCALL	PLUS
	RCALL	AT
	RCALL	OVER
	RCALL	RAT
	RCALL	CELLS
	RCALL	PLUS
	RCALL	IAT
	RCALL	SUBB
	RCALL	QDUP
	RCALL	QBRAN
	.DW		SAME2
	RCALL	RFROM
	RJMP	DROP
SAME2:
	RCALL	DONXT
	.DW		SAME1
	RCALL	DOLIT
	.DW		0
	RET

;   find	( a va -- ca na | a F )
;	Search a vocabulary for a string. Return ca and na if succeeded.

;	COLON	4,"FIND"
FIND:
	RCALL	SWAPP
	RCALL	DUPP
	RCALL	CAT
	RCALL	TEMP
	RCALL	STORE
	RCALL	DUPP
	RCALL	AT
	RCALL	TOR
	ADIW	TOSL,2	;va a+2 --
	RCALL	SWAPP	;a+2 va --
FIND1:
	RCALL	DUPP
	RCALL	QBRAN
	.DW		FIND6
	RCALL	DUPP
	RCALL	IAT
	RCALL	DOLIT
	.DW		$FF3F
	RCALL	ANDD
	RCALL	RAT
	RCALL	XORR
	RCALL	QBRAN
	.DW		FIND2
	ADIW	TOSL,2	;a+2 va+2 --
	RCALL	DOLIT
	.DW		-1
	RJMP	FIND3
FIND2:
	ADIW	TOSL,2	;a+2 va+2 --
	RCALL	TEMP
	RCALL	AT
	RCALL	SAMEQ
FIND3:
	RJMP	FIND4
FIND6:
	RCALL	RFROM
	RCALL	DROP
	RCALL	SWAPP
	SBIW	TOSL,2
	RJMP	SWAPP
FIND4:
	RCALL	QBRAN
	.DW		FIND5
	SBIW	TOSL,4
	RCALL	IAT
	RJMP	FIND1
FIND5:
	RCALL	RFROM
	RCALL	DROP
	RCALL	SWAPP
	RCALL	DROP
	SBIW	TOSL,2
	RCALL	DUPP
	RCALL	NAMET
	RJMP	SWAPP

;   NAME?	( a -- ca na | a F )
;	Search all context vocabularies for a string.

;	COLON	5,"NAME?"
NAMEQ:
	RCALL	CNTXT
	RCALL	AT
	RJMP	FIND

;; Terminal response

;   ^H	( bot eot cur -- bot eot cur )
;	Backup the cursor by one character.

;	COLON	2,"^H"
BKSP:
	RCALL	TOR
	RCALL	OVER
	RCALL	RFROM
	RCALL	SWAPP
	RCALL	OVER
	RCALL	XORR
	RCALL	QBRAN
	.DW		BACK1
	RCALL	DOLIT
	.DW		BKSPP
	RCALL	EMIT
	SBIW	TOSL,1
	RCALL	BLANK
	RCALL	EMIT
	RCALL	DOLIT
	.DW		BKSPP
	RCALL	EMIT
BACK1:
	RET

;   TAP	( bot eot cur c -- bot eot cur )
;	Accept and echo the key stroke and bump the cursor.

;	COLON	3,"TAP"
TAP:
	RCALL	DUPP
	RCALL	EMIT
	RCALL	OVER
	RCALL	CSTOR
	adiw	tosl,1
	ret

;   kTAP	( bot eot cur c -- bot eot cur )
;	Process a key stroke, CR or backspace.

;	COLON	4,"kTAP"
KTAP:
	RCALL	DUPP
	SBIW	TOSL,CRR
	RCALL	QBRAN
	.DW		KTAP2
	SBIW	TOSL,BKSPP
	RCALL	QBRAN
	.DW		KTAP1
	RCALL	BLANK
	RJMP	TAP
KTAP1:
	RJMP	BKSP
KTAP2:
	RCALL	DROP
	RCALL	SWAPP
	RCALL	DROP
	RJMP	DUPP

;   accept	( b u -- b u )
;	Accept characters to input buffer. Return with actual count.

;	COLON	6,"accept"
ACCEP:
	RCALL	OVER
	RCALL	PLUS
	RCALL	OVER
ACCP1:
	RCALL	DDUP
	RCALL	XORR
	RCALL	QBRAN
	.DW		ACCP4
	RCALL	KEY
	RCALL	DUPP
	RCALL	BLANK
	RCALL	SUBB
	RCALL	DOLIT
	.DW		$5F
	RCALL	ULESS
	RCALL	QBRAN
	.DW		ACCP2
	RCALL	TAP
	RJMP	ACCP3
ACCP2:
	RCALL	KTAP
ACCP3:
	RJMP	ACCP1
ACCP4:
	RCALL	DROP
	RCALL	OVER
	RJMP	SUBB

;   EXPECT	( b u -- )
;	Accept input stream and store count in SPAN.

	COLON	6,"EXPECT"
EXPEC:
	RCALL	ACCEP
	RCALL	SPAN
	RCALL	STORE
	RJMP	DROP

;   QUERY	( -- )
;	Accept input stream to terminal input buffer.

	COLON	5,"QUERY"
QUERY:
	RCALL	TIB
	RCALL	DOLIT
	.DW		80
	RCALL	ACCEP
	RCALL	NTIB
	RCALL	STORE
	RCALL	DROP
	RCALL	DOLIT
	.DW		0
	RCALL	INN
	RJMP	STORE

;; Error handling


;   ERROR	( a -- )
;	Return address of a null string with zero count.

;	COLON	5,"ERROR"
ERROR:
	RCALL	SPACE
	RCALL	COUNT
	RCALL	TYPES
	RCALL	DOLIT
	.DW		$3F
	RCALL	EMIT
ABORT:
	RCALL	CR
	RCALL	EMPTY_BUF
	ldi 	yl,low(SPP)
	ldi 	yh,high(SPP)
	RJMP	QUIT

;   abort"	( f -- )
;	Run time routine of ABORT" . Abort with a message.

;	COLON	COMPO+6,"abort"
;	.DB		'"'
ABORQ:
	RCALL	QBRAN
	.DW		ABOR1	;text flag
	RCALL	DOSTR
	RCALL	ICOUNT	;pass error string
	RCALL	ITYPES
	RCALL	ABORT
	RJMP	QUIT
ABOR1:
	RCALL	DOSTR
	RJMP	DROP

;; The text interpreter

;   $INTERPRET	( a -- )
;	Interpret a word. If failed, try to convert it to an integer.

;	COLON	10,"$INTERPRET"
INTER:
	RCALL	NAMEQ
	RCALL	QDUP	;?defined
	RCALL	QBRAN
	.DW		INTE1
	RCALL	IAT
	RCALL	DOLIT
	.DW		COMPO
	RCALL	ANDD	;?compile only lexicon bits
	RCALL	ABORQ
	.DB		13," compile only"
	RCALL	EXECU
	RET	;execute defined word
INTE1:
	RCALL	NUMBQ
	RCALL	QBRAN
	.DW		INTE2
	RET
INTE2:
	RJMP	ERROR	;error

;   [	( -- )
;	Start the text interpreter.

	COLON	IMEDD+1,"["
LBRAC:
	RCALL	DOLIT
	.DW		INTER*2
	RCALL	TEVAL
	RJMP	STORE

;   .OK	( -- )
;	Display "ok" only while interpreting.

;	COLON	3,".OK"
DOTOK:
	RCALL	DOLIT
	.DW		INTER*2
	RCALL	TEVAL
	RCALL	AT
	RCALL	EQUAL
	RCALL	QBRAN
	.DW		DOTO1
	RCALL	DOTQP
	.DB		3	," ok"
DOTO1:	RJMP	CR

;   ?STACK	( -- )
;	Abort if the data stack underflows.

;	COLON	6,"?STACK"
QSTAC:
	RCALL	DEPTH
	RCALL	ZLESS	;check only for underflow
	RCALL	ABORQ
	.DB		10," underflow"
	RET

;   EVAL	( -- )
;	Interpret the input stream.

	COLON	4,"EVAL"
EVAL:
EVAL1:	RCALL	TOKEN
	RCALL	DUPP
	RCALL	CAT	;?input stream empty
	RCALL	QBRAN
	.DW		EVAL2
	RCALL	TEVAL
	RCALL	ATEXE
;	RCALL	INTER
	RCALL	QSTAC	;evaluate input, check stack
	RJMP	EVAL1
EVAL2:
	RCALL	DROP
	RJMP	DOTOK

;; Shell

;   QUIT	( -- )
;	Reset return stack pointer and start text interpreter.

	COLON	4,"QUIT"
QUIT:
	ldi 	xl,low(RPP)
	out_ 	SPL,xl
	ldi 	xh,high(RPP)
	out_ 	SPH,xh
	RCALL	DOLIT
	.DW		TIBB
	RCALL	TTIB
	RCALL	STORE
QUIT1:
	RCALL	LBRAC	;start interpretation
QUIT2:
	RCALL	QUERY	;get input
	RCALL	EVAL
	RJMP	QUIT2	;continue till error

;; The compiler

;   '	( -- ca )
;	Search context vocabularies for the next word in input stream.

	COLON	1,"'"
TICK:
	RCALL	TOKEN
	RCALL	NAMEQ	;?defined
	RCALL	QBRAN
	.DW		TICK1
	RET				;yes, push code address
TICK1:
	RJMP	ERROR	;no, error

;; Tools

;   DUMP	( a u -- )
;	Dump 128 bytes from ain RAM, in a formatted manner.

	COLON	4,"DUMP"
DUMP:
	RCALL	DOLIT
	.DW		7
	RCALL	TOR		;start count down loop
DUMP1:	RCALL	CR
	RCALL	DUPP
	RCALL	DOLIT
	.DW		5
	RCALL	UDOTR
	RCALL	SPACE
	RCALL	DOLIT
	.DW		15
	RCALL	TOR
DUMP2:
	RCALL	COUNT
	RCALL	DOLIT
	.DW		3
	RCALL	UDOTR
	RCALL	DONXT	;display printable characters
	.DW		DUMP2
	RCALL	SPACE
	RCALL	DUPP
	RCALL	DOLIT
	.DW		16
	RCALL	SUBB
	RCALL	DOLIT
	.DW		16
	RCALL	TYPES
	RCALL	DONXT
	.DW		DUMP1	;loop till done
	RJMP	DROP

;   IDUMP	( a -- )
;	Dump 128 bytes from a in flash, in a formatted manner.

	COLON	5,"IDUMP"
IDUMP:
	RCALL	DOLIT
	.DW		7
	RCALL	TOR	;start count down loop
IDUMP1:
	RCALL	CR
	RCALL	DUPP
	RCALL	DOLIT
	.DW		5
	RCALL	UDOTR
	RCALL	SPACE
	RCALL	DOLIT
	.DW		15
	RCALL	TOR
IDUMP2:
	RCALL	ICOUNT
	RCALL	DOLIT
	.DW		3
	RCALL	UDOTR
	RCALL	DONXT	;display printable characters
	.DW		IDUMP2
	RCALL	SPACE
	RCALL	DUPP
	RCALL	DOLIT
	.DW		16
	RCALL	SUBB
	RCALL	DOLIT
	.DW		16
	RCALL	ITYPES
	RCALL	DONXT
	.DW		IDUMP1	;loop till done
	RJMP	DROP


;   .S	( ... -- ... )
;	Display the contents of the data stack.

	COLON	2,".S"
DOTS:
	RCALL	DEPTH	;stack depth
	RCALL	TOR	;start count down loop
	RJMP	DOTS2	;skip first pass
DOTS1:
	RCALL	RAT
	RCALL	PICK
	RCALL	DOT	;index stack, display contents
DOTS2:
	RCALL	DONXT 
	.DW		DOTS1	;loop till done
	RCALL	DOTQP
	.DB		4," <sp"
	RET

;   >NAME	( ca -- na | F )
;	Convert code address to a name address.

;	COLON	5,">NAME"
TNAME:
	RCALL	TOR
	RCALL	CNTXT
	RCALL	AT	;na
TNAM1:	
	RCALL	DUPP	;na na
	RCALL	QBRAN
	.DW		TNAM2
	RCALL	DUPP	;na na
	RCALL	NAMET	;na ca
	RCALL	RAT	;na ca ca
	RCALL	XORR	;na f
	RCALL	QBRAN
	.DW		TNAM2
	SBIW	TOSL,2	;la
	RCALL	IAT	;na'
	RCALL	BRAN
	.DW		TNAM1
TNAM2:
	RCALL	RFROM	;na or 0
	RJMP	DROP

;   .ID	( na -- )
;	Display the name at address.

;	COLON	3,".ID"
DOTID:
	RCALL	ICOUNT
	RCALL	DOLIT
	.DW		31
	RCALL	ANDD
	RJMP 	ITYPES

;   WORDS	( -- )
;	Display the names in the context vocabulary.

	COLON	5,"WORDS"
WORDS:
	RCALL	CR
	RCALL	CNTXT
	RCALL	AT	;na
WORS1:	
	RCALL	QDUP	;end of list?
	RCALL	QBRAN
	.DW		WORS2
	RCALL	DUPP	;na na
	RCALL	SPACE
	RCALL	DOTID	;display a name
	SBIW	TOSL,2	;la
	RCALL	IAT	;na'
	RCALL	BRAN
	.DW		WORS1
WORS2:
	RET


;; Hardware reset

;   hi	( -- )
;	Display the sign-on message of eForth.

;	COLON	2,"hi"
HI:
	RCALL	CR
	RCALL	UNICODE
	RCALL	DOTESC
	.DB		3,"[2J"	; clear screen
	; .DB		1,"c"		; reset terminal
	RCALL	DOTQP
	; .DB	15,"tn85eForth v1.0"	;model
	.DB	16,"tn85eForth v1.0 "
	RCALL	DOTESC
	.DB		4,"[33m"	; yellow color
	RCALL	DOTQP		; minasan konnichiwa
	.DB	24,$e7,$9a,$86,$e3,$81,$95,$e3,$82,$93
	.DB	   $e3,$81,$93,$e3,$82,$93
	.DB	   $e3,$81,$ab,$e3,$81,$a1
	.DB	   $e3,$81,$af
	RCALL	DOTESC
	.DB		3,"[0m"		; reset color
	RCALL	ASCII
	RCALL	CR
	RCALL	CR
	RCALL	DOTQP
	.DB	12,"memory dump:"
    RCALL   MEMDUMP
	RCALL	CR
	RCALL	DOTQP
	.DB	10,"code dump:"
    RCALL   CODEDUMP
	RJMP	CR

;   COLD	( -- )
;	The hilevel cold start sequence.

	COLON	4,"COLD"
COLD:
COLD1:
	RCALL	STOIO	

	RCALL	DOLIT
	.DW	    UPPF*2  ;flash, byte address
	RCALL	DOLIT
	.DW     UPP     ;ram
	RCALL	READ	;initialize user area

	RCALL	DOLIT	;init older buffer
	.DW	OLDER
	RCALL	AT		;
	RCALL	READ_FLASH
	RCALL	DROP
	clr		dirty

	RCALL	TBOOT   ;hi
	RCALL	ATEXE

	RJMP	QUIT	;start interpretation

;===========================
; flash page ERASE WRITE READ
; go to https://www.microchip.com/en-us/product/ATmega328P#document-table
; get "ATmega48A/PA/88A/PA/168A/PA/328/P Data Sheet"
; read 26.2.5 Simple Assembly Code Example for a Boot Loader
;
;PAGESIZE is defined in the .inc file
.equ 	PAGESIZEB = PAGESIZE*2 ;PAGESIZEB is page size in BYTES, not words
.def	spmcrval = r20
.def	looplo = r22
.def	loophi = r23

; Page Erase
;	ERASE ( a -- )
;	Erase a page of flash memory

	COLON	5,"ERASE"
ERASE:
	movw	zl,tosl
	loadtos
ERASE_1:
	ldi 	spmcrval, (1<<PGERS) | (1<<SELFPRGEN)
	RJMP 	Do_spm

; Page Write
; 	WRITE ( ram flash -- )	
; 	transfer data from RAM to Flash page buffer

	COLON	5,"WRITE"
WRITE:
	movw	zl, tosl
	loadtos
	movw	xl, tosl
	loadtos
WRITE_1:
	ldi 	looplo, low(PAGESIZEB) ;init loop variable
Wrloop:
	ld 		r0, X+
	ld 		r1, X+
	ldi 	spmcrval, (1<<SELFPRGEN)
	RCALL 	Do_spm
	adiw 	ZL, 2
	subi 	looplo, 2 ;use subi for PAGESIZEB<=256
	brne 	Wrloop
; execute Page Write
	subi 	ZL, low(PAGESIZEB) ;restore pointer
	sbci 	ZH, high(PAGESIZEB) ;not required for PAGESIZEB<=256
	ldi 	spmcrval, (1<<PGWRT) | (1<<SELFPRGEN)
	RJMP 	Do_spm

; Page Read
; 	READ ( flash ram -- )	
; 	transfer data from Flash to RAM page buffer

	COLON	4,"READ"
READ:
	movw	xl,tosl
	loadtos
	movw	zl,tosl
	loadtos
READ_1:	
; read back and check, optional
	ldi 	looplo, low(PAGESIZEB) ;init loop variable
Rdloop:
	lpm 	r0, Z+
	st 	X+, r0
	subi 	looplo, 1 ;use subi for PAGESIZEB<=256
	brne 	Rdloop
	ret

Do_spm:
; check for previous SPM complete
Wait_spm:
	in 	temp1, SPMCSR
	sbrc 	temp1, SELFPRGEN
	RJMP 	Wait_spm
; input: spmcrval determines SPM action
; disable interrupts if enabled, store status
	in 	temp2, SREG
	cli
; check that no EEPROM write access is present
Wait_ee:
	sbic 	EECR, EEPE
	RJMP 	Wait_ee
; SPM timed sequence
	out 	SPMCSR, spmcrval
	spm
; restore SREG (to enable interrupts if originally enabled)
	out 	SREG, temp2
	ret


;===============================================================
; Compiler

;.org	$100

;   1+	( a -- a )
;	Add 1 to address.

	COLON	2,"1+"
ONEP:
	adiw	tosl,1
	ret

;   1-	( a -- a )
;	Subtract 1 from address.

	COLON	2,"1-"
ONEM:
	sbiw	tosl,1
	ret


;   2+	( a -- a )
;	Add cell size in byte to address.

	COLON	2,"2+"
CELLP:
	adiw	tosl,2
	ret


;   2-	( a -- a )
;	Subtract cell size in byte from address.

	COLON	2,"2-"
CELLM:
	sbiw	tosl,2
	ret

; 	>	( n1 n2 -- flag ) Compare
; 	compares two values (signed)

	COLON	1,">"
GREATER:
	ld 		temp2, Y+
	ld 		temp3, Y+
	cp 		temp2, tosl
	cpc 	temp3, tosh
	RJMP 	DGRE1

; 	D>	( d1 d2 -- flag ) Compare
; 	compares two d values (signed)

	COLON	2,"D>"
DGRE:	
	ld 		temp0, Y+
	ld 		temp1, Y+
	ld 		temp2, Y+
	ld 		temp3, Y+
	ld 		temp4, Y+
	ld 		temp5, Y+
	cp 		temp4, temp0
	cpc 	temp5, temp1
	cpc 	temp2, tosl
	cpc 	temp3, tosh
DGRE1:
	movw 	tosl,zerol
	brlt 	DGRE2
	brbs 	1, DGRE2
	sbiw 	tosl,1
	ret
DGRE2:
	ret

; 	D+	( d1 d2 -- d3) Arithmetics
; 	add double cell values

	COLON	2,"D+"
DPLUS:
	ld 		temp2, Y+
	ld 		temp3, Y+
	ld 		temp4, Y+
	ld 		temp5, Y+
	ld 		temp6, Y+
	ld 		temp7, Y+
	add 	temp2, temp6
	adc 	temp3, temp7
	adc 	tosl, temp4
	adc 	tosh, temp5
	st 		-Y, temp3
	st 		-Y, temp2
	ret

; 	D-	( d1 d2 -- d3 ) Arithmetics
; 	subtract double cell values

	COLON	2,"D-"
DMINUS:
	ld 		temp2, Y+
	ld 		temp3, Y+
	ld 		temp4, Y+
	ld 		temp5, Y+
	ld 		temp6, Y+
	ld 		temp7, Y+
	sub 	temp6, temp2
	sbc 	temp7, temp3
	sbc 	temp4, tosl
	sbc 	temp5, tosh
	st 		-Y, temp7
	st 		-Y, temp6
	movw 	tosl, temp4
	ret

;	ALLOT	( n -- )
;	Allocate n bytes to the code dictionary.

	COLON	5,"ALLOT"
ALLOT:
	RCALL	DPP
	RJMP		PSTOR

;   IALLOT	( n -- )
;	Allocate n bytes to the code dictionary.

	COLON	6,"IALLOT"
IALLOT:
	RCALL	CPP
	RJMP		PSTOR

;   ,	( w -- )
;	Compile an integer into the code dictionary.

	COLON	1,","
COMMA:
	RCALL	CPP
	RCALL	AT
	RCALL	DUPP
	RCALL	CELLP	;cell boundary
	RCALL	CPP
	RCALL	STORE
	RJMP		ISTOR

;   call,	( ca -- )
;	Assemble a call instruction to ca.

;	COLON	5,"call,"
CALLC:
; build RCALL instruction
	RCALL	CPP
    RCALL   AT
	RCALL	CELLP
    RCALL   TWOSL   ; 2/
	RCALL	SUBB
	RCALL	DOLIT
	.DW	$FFF
	RCALL	ANDD
	RCALL	DOLIT
	.DW	CALLL	; 1101 0000 0000 0000
	RCALL	ORR	; 1101 kkkk kkkk kkkk = RCALL k
	RJMP	COMMA	; store tn85 short/relative call

;   [COMPILE]	( -- ; <string> )
;	Compile the next immediate word into code dictionary.

	COLON	IMEDD+9,"[COMPILE]"
BCOMP:
	RCALL	TICK
	RCALL	TWOSL
	RJMP	CALLC

;   COMPILE	( -- )
;	Compile the next address in colon list to code dictionary.

	COLON	COMPO+7,"COMPILE"
COMPI:
	RCALL	RFROM
	RCALL	CELLS
	RCALL	DUPP
	RCALL	AT
	RCALL	COMMA	;compile call instruction
	RCALL	CELLP
	RCALL	DUPP
	RCALL	AT
	RCALL	COMMA	;compile address
	RCALL	CELLP
	RCALL	TWOSL
	RCALL	TOR
	RET				;adjust return address

;   LITERAL	( w -- )
;	Compile tos to code dictionary as an integer literal.

	COLON	7,"LITERAL"
LITER:
	RCALL	DOLIT
	.DW		DOLIT
	RCALL	CALLC
	RJMP	COMMA

;   $,"	( -- )
;	Compile a literal string up to next " .

;	COLON	3,'$'
;	.DB		',','"'
STRCQ:
	RCALL	DOLIT
	.DW		'"'
	RCALL	WORDD	;move string to code dictionary
	RCALL	DUPP
	RCALL	CAT
	RCALL	TWOSL
	RCALL	TOR
STRCQ1:
	RCALL	DUPP
	RCALL	AT
	RCALL	COMMA
	RCALL	CELLP
	RCALL	DONXT
	.DW		STRCQ1
	RJMP		DROP

;; Structures

;   BEGIN	( -- a )
;	Start an infinite or indefinite loop structure.

	COLON	IMEDD+5,"BEGIN"
BEGIN:
	RCALL	CPP
	RJMP		AT

;   FOR	( -- a )
;	Start a FOR-NEXT loop structure in a colon definition.

	COLON	IMEDD+3,"FOR"
FOR:
	RCALL	DOLIT
	.DW		TOR
	RCALL	CALLC
	RJMP	BEGIN

;   NEXT	( a -- )
;	Terminate a FOR-NEXT loop structure.

	COLON	IMEDD+4,"NEXT"
NEXT:
	RCALL	DOLIT
	.DW		DONXT
	RCALL	CALLC
	RCALL	TWOSL
	RJMP	COMMA

;   UNTIL	( a -- )
;	Terminate a BEGIN-UNTIL indefinite loop structure.

	COLON	IMEDD+5,"UNTIL"
UNTIL:
	RCALL	DOLIT
	.DW		QBRAN
	RCALL	CALLC
	RCALL	TWOSL
	RJMP	COMMA

;   AGAIN	( a -- )
;	Terminate a BEGIN-AGAIN infinite loop structure.

	COLON	IMEDD+5,"AGAIN"
AGAIN:
	RCALL	DOLIT
	.DW		BRAN
	RCALL	CALLC
	RCALL	TWOSL
	RJMP	COMMA

;   IF	( -- A )
;	Begin a conditional branch structure.

	COLON	IMEDD+2,"IF"
IFF:
	RCALL	DOLIT
	.DW		QBRAN
	RCALL	CALLC
	RCALL	BEGIN
	RCALL	DOLIT
	.DW		2
	RJMP	IALLOT

;   AHEAD	( -- A )
;	Compile a forward branch instruction.

;	COLON	IMEDD+5,"AHEAD"
AHEAD:
	RCALL	DOLIT
	.DW		BRAN
	RCALL	CALLC
	RCALL	BEGIN
	RCALL	DOLIT
	.DW		2
	RJMP		IALLOT

;   REPEAT	( A a -- )
;	Terminate a BEGIN-WHILE-REPEAT indefinite loop.

	COLON	IMEDD+6,"REPEAT"
REPEA:
	RCALL	AGAIN
	RCALL	BEGIN
	RCALL	TWOSL
	RCALL	SWAPP
	RJMP		ISTOR

;   THEN	( A -- )
;	Terminate a conditional branch structure.

	COLON	IMEDD+4,"THEN"
THENN:
	RCALL	BEGIN
	RCALL	TWOSL
	RCALL	SWAPP
	RJMP		ISTOR

;   AFT	( a -- a A )
;	Jump to THEN in a FOR-AFT-THEN-NEXT loop the first time through.

	COLON	IMEDD+3,"AFT"
AFT:
	RCALL	DROP
	RCALL	AHEAD
	RCALL	BEGIN
	RJMP		SWAPP

;   ELSE	( A -- A )
;	Start the false clause in an IF-ELSE-THEN structure.

	COLON	IMEDD+4,"ELSE"
ELSEE:
	RCALL	AHEAD
	RCALL	SWAPP
	RJMP		THENN

;   WHILE	( a -- A a )
;	Conditional branch out of a BEGIN-WHILE-REPEAT loop.

	COLON	IMEDD+5,"WHILE"
WHILE:
	RCALL	IFF
	RJMP		SWAPP

;   ABORT"	( -- ; <string> )
;	Conditional abort with an error message.

	COLON	IMEDD+6,"ABORT"
	.DB		'"'
ABRTQ:
	RCALL	DOLIT
	.DW		ABORQ
	RCALL	CALLC
	RCALL	STRCQ
	RET

;   $"	( -- ; <string> )
;	Compile an inline string literal.

	COLON	IMEDD+2,'$'
	.DB		'"'
STRQ:
	RCALL	DOLIT
	.DW		STRQP
	RCALL	CALLC
	RCALL	STRCQ
	RET

;   ."	( -- ; <string> )
;	Compile an inline string literal to be typed out at run time.

	COLON	IMEDD+2,'.'
	.DB		'"'
DOTQ:
	RCALL	DOLIT
	.DW		DOTQP
	RCALL	CALLC
	RCALL	STRCQ
	RET

;; Name compiler

;   ?UNIQUE	( a -- a )
;	Display a warning message if the word already exists.

;	COLON	7,"?UNIQUE"
UNIQU:
	RCALL	DUPP
	RCALL	NAMEQ	;?name exists
	RCALL	QBRAN
	.DW		UNIQ1
	RCALL	DOTQP	;redefinitions are OK
	.DB		7," reDef "	;but the user should be warned
	RCALL	OVER
	RCALL	COUNT
	RCALL	TYPES	;just in case its not planned
UNIQ1:
	RJMP		DROP

;   $,n	( na -- )
;	Build a new dictionary name using the string at na.

;	COLON	3,"$,n"
SNAME:
	RCALL	DUPP
	RCALL	CAT	;?null input
	RCALL	QBRAN
	.DW		SNAM2
	RCALL	UNIQU	;?redefinition
	RCALL	LAST
	RCALL	AT
	RCALL	COMMA	;compile link 
	RCALL	CPP
	RCALL	AT
	RCALL	LAST
	RCALL	STORE	;save new nfa in LAST	
	RCALL	DUPP
	RCALL	CAT
	RCALL	TWOSL	;na count/2
	RCALL	TOR
SNAME1:
	RCALL	DUPP
	RCALL	AT
	RCALL	COMMA	;compile name
	RCALL	CELLP
	RCALL 	DONXT
	.DW		SNAME1
	RJMP		DROP
SNAM2:
	RCALL	STRQP
	.DB		5," name"	;null input
	RJMP		ERROR

;; FORTH compiler

;   $COMPILE	( a -- )
;	Compile next word to code dictionary as a token or literal.

;	COLON	8,"$COMPILE"
SCOMP:
	RCALL	NAMEQ
	RCALL	QDUP	;?defined
	RCALL	QBRAN
	.DW		SCOM2
	RCALL	IAT
	RCALL	DOLIT
	.DW		IMEDD
	RCALL	ANDD	;?immediate
	RCALL	QBRAN
	.DW		SCOM1
	RJMP		EXECU
SCOM1:
	RCALL	TWOSL
	RJMP		CALLC
SCOM2:
	RCALL	NUMBQ
	RCALL	QBRAN
	.DW		SCOM3
	RJMP		LITER
SCOM3:
	RJMP		ERROR	;error

;   OVERT	( -- )
;	Link a new word into the current vocabulary.

	COLON	5,"OVERT"
OVERT:
	RCALL	LAST
	RCALL	AT
	RCALL	CNTXT
	RJMP		STORE

;   ;	( -- )
;	Terminate a colon definition.

	COLON	IMEDD+COMPO+1,";"
SEMIS:
	RCALL	DOLIT
	.DW		RETT
	RCALL	COMMA
	RCALL	LBRAC
	RJMP		OVERT


;   ]	( -- )
;	Start compiling the words in the input stream.

	COLON	1,"]"
RBRAC:
	RCALL	DOLIT
	.DW		SCOMP*2
	RCALL	TEVAL
	RJMP		STORE

;   :	( -- ; <string> )
;	Start a new colon definition using next word as its name.

	COLON	1,":"
COLONN:
	RCALL	TOKEN
	RCALL	SNAME
	RJMP		RBRAC

;   IMMEDIATE	( -- )
;	Make the last compiled word an immediate word.

	COLON	9,"IMMEDIATE"
IMMED:
	RCALL	DOLIT
	.DW		IMEDD
	RCALL	LAST
	RCALL	AT
	RCALL	IAT
	RCALL	ORR
	RCALL	LAST
	RCALL	AT
	RJMP		ISTOR

;; Defining words

;   CREATE	( -- ; <string> )
;	Compile a new array entry without allocating code space.

	COLON	6,"CREATE"
CREAT:
	RCALL	TOKEN
	RCALL	SNAME
	RCALL	OVERT
 	RCALL	DOLIT
	.DW		DOVAR
	RCALL	CALLC
	RCALL	DPP
	RCALL	AT
	RJMP		COMMA

;   CONSTANT	( n -- ; <string> )
;	Compile a constant.

	COLON	8,"CONSTANT"
CONST:
	RCALL	TOKEN
	RCALL	SNAME
	RCALL	OVERT
 	RCALL	DOLIT
	.DW		DOVAR
	RCALL	CALLC
	RJMP		COMMA

;   VARIABLE	( -- ; <string> )
;	Compile a new variable uninitialized.

	COLON	8,"VARIABLE"
VARIA:
	RCALL	CREAT
	RCALL	DOLIT
	.DW		2
	RJMP		ALLOT



;-----------------------------------------



;	UNICODE	( -- )
; switch the output to 8-bit UNICODE
	COLON	7,"UNICODE"
UNICODE:
	RCALL	DOLIT
	.DW		$ff
	RCALL	TCHARM
	RJMP	STORE


;	ASCII	( -- )
; switch the output to 7-bit ASCII
	COLON	5,"ASCII"
ASCII:
	RCALL	DOLIT
	.DW		$7f
	RCALL	TCHARM
	RJMP	STORE


;	DOTESC	( -- )
; emits the ESC char, $1b, followed by escape sequence
	COLON	4,".ESC"
DOTESC:
	RCALL	DOLIT
	.DW		$1b
	RCALL	EMIT
	RJMP	DOTQP


;-----------------------------------------
; debugging tools

;   MEMDUMP	( -- )
;	Dumps the whole RAM.
	COLON	7,"MEMDUMP"
MEMDUMP:
    RCALL   toggleLED
	; RCALL	CR
	RCALL	dolit
	.DW		$0
	RCALL	DUMP
	RCALL	dolit
	.DW		$80;60
	RCALL	DUMP
	RCALL	dolit
	.DW		$100;E0
	RCALL	DUMP
	RCALL	dolit
	.DW		$180;160
	RCALL	DUMP
	RCALL	dolit
	.DW		$200;1E0
	RCALL	DUMP
	RCALL	CR
    RCALL   toggleLED
RET


;   CODEDUMP	( -- )
;	Dumps the last 128 bytes of code in flash.
	COLON	8,"CODEDUMP"
CODEDUMP:
    RCALL   CPP
    RCALL   AT
	RCALL	dolit
	.DW		$80
    RCALL   SUBB
	RCALL	IDUMP
    ; RCALL   CR
RET



; debugging with LED
;   TOGGLELED ( -- )
;   toggle led on/off
	COLON	9,"TOGGLELED"
toggleLED:
   	sbi	DDRB, DDB3  ; output - LED
	sbic PORTB, PB3
	RJMP tled
   	sbi	PORTB, PB3  ; switch on
ret
tled:
	cbi	PORTB, PB3  ; switch off
ret


;============================================================================

.EQU	LASTN	=	_LINK*2	;last name address in name dictionary

.EQU	DTOP	=	$d0	;next available memory in name dictionary
.EQU	CTOP	=	pc*2	;next available memory in code dictionary



;===============================================================




