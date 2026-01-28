;
; Bare-Metal Forth on ATtiny85 — Step-by-step rebuild (Stage 1)
; ------------------------------------------------------------
; Goal of this stage:
;   - minimal vector table
;   - reset handler
;   - SRAM clear (0x0060..end)
;   - stack init (HW SP = return stack)
;   - a visible proof-of-life: blink PB3 (Arduino/Trinket LED often on PB1, adjust as needed)
;
; This file is intentionally small and heavily commented for teaching.
;
; Build assumptions:
;   - AVRASM2 / avra / avr-gcc (assemble+link workflow varies)
;   - You can adapt to your preferred toolchain later.
;
; NOTE: Replace device include / register names if your assembler differs.
;

.nolist
    .include "tn85def.inc"     ; your existing definitions (register aliases, bit names, etc.)
.list

; -----------------------
; Constants / “memory map”
; -----------------------
.equ SRAM_START = 0x0060       ; start of true SRAM (after regs + IO)
.equ SRAM_SIZE  = 0x0200       ; 512 bytes on ATtiny85
.equ SRAM_END   = SRAM_START + SRAM_SIZE - 1

; Return stack (hardware SP) top. Keep a little safety margin below end.
.equ RPP = 0x021E              ; matches your existing project

; Data stack top (we’ll later use Y as data stack pointer, like your tn85eForth).
.equ SPP = 0x01B0              ; matches your existing project

; -----------------------
; Interrupt vector table
; -----------------------
.ORG 0x0000
    rjmp ORIG                  ; Reset vector

    ; ATtiny85 vectors (order matters). Keep unused as reti for now.
    reti                       ; INT0
    rjmp PCINT0_ISR            ; PCINT0 (we may later use for UART RX or wake)
    reti                       ; TIM1_COMPA
    reti                       ; TIM1_OVF
    reti                       ; TIM0_OVF
    reti                       ; EE_RDY
    reti                       ; ANA_COMP
    reti                       ; ADC
    reti                       ; TIM1_COMPB
    reti                       ; TIM0_COMPA (we’ll enable later)
    reti                       ; TIM0_COMPB
    reti                       ; WDT
    reti                       ; USI_START
    reti                       ; USI_OVF

; -----------------------
; Reset / init
; -----------------------
ORIG:
    ; 1) Capture & clear reset cause (MCUSR)
    in_  r16, MCUSR            ; r16 holds reset cause for debugging (later we’ll store it)
    clr  r0
    out_ MCUSR, r0             ; clear flags

    ; 2) Clear SRAM from 0x0060 to end
    ;    Y = write pointer, X = countdown
    ldi  yl, low(SRAM_START)
    ldi  yh, high(SRAM_START)

    clr  xl
    ldi  xh, high(SRAM_SIZE)   ; 0x02
    ; X now equals 0x0200 when (xh=0x02, xl=0x00)

CLR_RAM:
    st   y+, r0
    sbiw x, 1
    brne CLR_RAM

    ; 3) Init return stack (HW SP)
    ldi  xl, low(RPP)
    out_ SPL, xl
    ldi  xh, high(RPP)
    out_ SPH, xh

    ; 4) Init data stack pointer (Y) for later Forth
    ldi  yl, low(SPP)
    ldi  yh, high(SPP)

    ; 5) Proof-of-life: configure PB3 as output and blink
    ;    LED is on PB3 for your board.
    sbi  DDRB, 3               ; PB1 output

BLINK:
    sbi  PORTB, 3              ; PB1 high
    rcall DELAY_100MS
    cbi  PORTB, 3              ; PB1 low
    rcall DELAY_100MS
    rjmp BLINK

; -----------------------
; Simple delay (busy-loop)
; -----------------------
; ~100ms-ish at 8 MHz. This is only for early bring-up.
; We’ll replace with Timer0 tick in the next stage.
DELAY_100MS:
    ldi  r18, 255
D1:
    ldi  r19, 255
D2:
    dec  r19
    brne D2
    dec  r18
    brne D1
    ret

; -----------------------
; Interrupt stubs
; -----------------------
PCINT0_ISR:
    reti
