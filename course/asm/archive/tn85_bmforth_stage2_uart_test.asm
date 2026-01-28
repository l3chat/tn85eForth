; TITLE ATtiny85 Bare-Metal Forth — Stage 2 (UART bring-up)
; ----------------------------------------------------------
; This stage adds the same USI-UART pins and routines as tn85eForth,
; but without the full Forth kernel yet.
;
; What you get:
;   - reset + SRAM clear + stack init
;   - LED blink on PB3 (proof-of-life)
;   - USI-UART init (same as tn85eForth)
;   - TX test: repeatedly sends 'U' over serial
;   - RX path is present (PCINT + TIM0 COMPA + USI OVF ISR) for next stages
;
; Assembler: AVRASM2

.nolist
    .include "tn85def.inc"
.list

; --- Register conventions (subset, aligned with tn85eForth) ---
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

; --- UART timing constants (copied from tn85eForth) ---
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

; --- USI state constants ---
.equ USI_STATE_AVAILABLE	=0
.equ USI_STATE_FIRST		=1
.equ USI_STATE_SECOND		=2
.equ USI_STATE_RECEIVING	=3

; --- Memory map constants ---
.equ SRAM_START = 0x0060
.equ SRAM_SIZE  = 0x0200
.equ RPP = 0x021E
.equ SPP = 0x01B0

; --- Interrupt vectors ---
.ORG 0x0000
    RJMP ORIG
    RETI                ; INT0
    RJMP PCINT0_ISR      ; PCINT0 (UART RX start bit detect)
    RETI                ; TIM1_COMPA
    RETI                ; TIM1_OVF
    RETI                ; TIM0_OVF
    RETI                ; EE_RDY
    RETI                ; ANA_COMP
    RETI                ; ADC
    RETI                ; TIM1_COMPB
    RJMP TIM0_COMPA_ISR  ; TIM0_COMPA (RX bit-center sync)
    RETI                ; TIM0_COMPB
    RETI                ; WDT
    RETI                ; USI_START
    RJMP USI_OVF_ISR     ; USI_OVF (RX/TX shift done)

; --- Reset handler ---
ORIG:
    ; capture + clear reset cause
    in   temp0, MCUSR
    clr  zerol
    out  MCUSR, zerol

    ; clear SRAM 0x0060..0x025F
    ldi  yl, low(SRAM_START)
    ldi  yh, high(SRAM_START)
    clr  xl
    ldi  xh, high(SRAM_SIZE)   ; 0x02 => X=0x0200
CLR_RAM:
    st   Y+, zerol
    sbiw X, 1
    brne CLR_RAM

    ; init HW return stack (SP)
    ldi  xl, low(RPP)
    out  SPL, xl
    ldi  xh, high(RPP)
    out  SPH, xh

    ; init data stack pointer (Y) for later Forth stages
    ldi  yl, low(SPP)
    ldi  yh, high(SPP)

    ; LED on PB3
    sbi  DDRB, PB3

    ; init UART (same pins/routines as tn85eForth)
    rcall usiserial_init

MAIN:
    ; blink once
    sbi  PORTB, PB3
    rcall DELAY_50MS
    cbi  PORTB, PB3
    rcall DELAY_50MS

    ; send 'U' (0x55) repeatedly as a serial test pattern
    ldi  serialTxData, 0x55
    rcall usiserial_send_byte
    rcall DELAY_50MS
    rjmp MAIN

; --- Busy delay (bring-up only) ---
DELAY_50MS:
    ldi  temp0, 200
D50_1:
    ldi  temp1, 255
D50_2:
    dec  temp1
    brne D50_2
    dec  temp0
    brne D50_1
    ret

; --- UART-related ISRs (copied from tn85eForth) ---
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




; --- UART routines (copied from tn85eForth) ---
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




; End of Stage 2