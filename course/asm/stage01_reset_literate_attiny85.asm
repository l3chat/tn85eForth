;
; ======================================================================
;  Bare-Metal Forth — Stage 01 (Literate Assembly)
;  Lesson 1: Reset, Memory, and the Empty Machine
;  Target MCU : ATtiny85
;  Assembler  : AVRASM2
;  LED        : PB3 (user confirmed)
;
;  What this stage does (and nothing more):
;    1) defines a minimal interrupt vector table
;    2) implements a reset handler:
;         - captures and clears reset-cause flags (MCUSR)
;         - clears SRAM (0x0060..end)
;         - initializes TWO stacks:
;             * hardware return stack via SP
;             * software data stack via Y register pair
;    3) provides a proof-of-life LED blink loop
;
;  Philosophy:
;    - We prefer boring correctness over cleverness.
;    - We explain each new assembly “spell” at first use.
; ======================================================================


; ----------------------------------------------------------------------
; 0) Includes and listing control
; ----------------------------------------------------------------------
; .include pulls in another source file before assembly.
; Here we reuse your existing ATtiny85 definitions and helper macros.
;
; NOTE: This file assumes tn85def.inc is in the same directory.
;
; .nolist / .list controls whether the included file is printed in the
; assembler listing. This keeps the listing readable for beginners.

.nolist
    .include "tn85def.inc"
.list


; ----------------------------------------------------------------------
; 1) Constants — our first “memory map decisions”
; ----------------------------------------------------------------------
; .equ defines a constant symbol (like a #define).
; We will use these constants to avoid “magic numbers”.

; SRAM on ATtiny85:
; - addresses 0x0000..0x005F are registers + I/O space (not SRAM)
; - usable SRAM begins at 0x0060
.equ SRAM_START = 0x0060
.equ SRAM_SIZE  = 0x0200          ; 512 bytes SRAM total on ATtiny85
.equ SRAM_END   = SRAM_START + SRAM_SIZE - 1

; LED pin (for “proof of life”)
.equ LED_BIT    = 3               ; PB3

; Stack layout decisions (chosen to match your tn85eForth project)
; Return stack uses the AVR hardware stack pointer SP (grows downward).
.equ RPP        = 0x021E          ; top of return stack (RP0)
; Data stack uses a software pointer in register pair Y (grows downward).
.equ SPP        = 0x01B0          ; top of data stack (SP0)


; ----------------------------------------------------------------------
; 2) Interrupt Vector Table — where the MCU starts and how interrupts enter
; ----------------------------------------------------------------------
; .ORG sets the assembly origin (address). For the vector table, we must
; place code at address 0x0000, because reset jumps there.
;
; Each vector location contains one instruction (usually RJMP/RETI).
; We keep unused vectors as RETI (return-from-interrupt).
; RETI is like RET, but it also re-enables global interrupts properly.
;
; IMPORTANT: In this stage we do NOT enable interrupts (SEI is never used).
; Even so, a correct vector table is good hygiene.

.ORG 0x0000
    rjmp ORIG                     ; Reset vector: start of *our* program

    reti                          ; INT0
    reti                          ; PCINT0
    reti                          ; TIM1_COMPA
    reti                          ; TIM1_OVF
    reti                          ; TIM0_OVF
    reti                          ; EE_RDY
    reti                          ; ANA_COMP
    reti                          ; ADC
    reti                          ; TIM1_COMPB
    reti                          ; TIM0_COMPA
    reti                          ; TIM0_COMPB
    reti                          ; WDT
    reti                          ; USI_START
    reti                          ; USI_OVF


; ----------------------------------------------------------------------
; 3) Reset handler — ORIG
; ----------------------------------------------------------------------
; Reset is a semantic boundary: everything after it depends on how we
; establish initial conditions.
;
; We’ll use these registers for this stage:
;   r0  : always zero (we keep it cleared)
;   r16 : temporary register to capture MCUSR
;   X   : register pair r27:r26 (XH:XL) used as a counter
;   Y   : register pair r29:r28 (YH:YL) used as a pointer
;
; AVR register pair names:
;   XL = r26, XH = r27
;   YL = r28, YH = r29
;
; NOTE: tn85def.inc defines helper macros in_ and out_ that expand to
; IN/OUT with the correct I/O addresses for the chosen device.

ORIG:

; --- 3.1 Capture and clear reset cause (MCUSR) ------------------------
; IN reads from an I/O register into a CPU register.
; OUT writes from a CPU register into an I/O register.

    in_  r16, MCUSR               ; r16 <- MCUSR (why did we reset?)
    clr  r0                       ; CLR sets a register to 0
    out_ MCUSR, r0                ; clear flags so next reset is meaningful

; At this point:
;   r16 contains the reset reason (we do not use it yet, but we saved it)
;   r0 is guaranteed to be 0 (we will reuse it as “zero constant”)

; --- 3.2 Clear SRAM (0x0060 .. end) ----------------------------------
; SRAM after reset is undefined. We choose to erase randomness.
;
; We implement a simple loop:
;   Y points to the current SRAM byte to clear
;   X counts how many bytes remain
;
; LDI loads an 8-bit immediate constant into a register.
; ST stores a register value to SRAM address in Y, then increments Y (Y+).
; SBIW subtracts an immediate from a register pair (X = X - 1).
; BRNE branches if the Zero flag is NOT set (i.e., result != 0).

    ldi  yl, low(SRAM_START)      ; Y <- SRAM_START (low byte)
    ldi  yh, high(SRAM_START)     ; Y <- SRAM_START (high byte)

    clr  xl                       ; X low byte = 0
    ldi  xh, high(SRAM_SIZE)      ; X high byte = 0x02 => X = 0x0200

CLR_RAM:
    st   y+, r0                   ; *Y = 0; Y++
    sbiw x, 1                     ; X--
    brne CLR_RAM                  ; loop until X becomes 0

; --- 3.3 Initialize the return stack (hardware SP) -------------------
; AVR has a hardware stack pointer SP (SPL/SPH).
; CALL/RCALL push return addresses there; RET pops them.
; If SP is wrong, even simple subroutines will fail.

    ldi  xl, low(RPP)
    out_ SPL, xl                  ; SPL <- low(RPP)
    ldi  xh, high(RPP)
    out_ SPH, xh                  ; SPH <- high(RPP)

; --- 3.4 Initialize the data stack pointer (software, in Y) -----------
; In later stages, Y will become the “data stack pointer” (DSP).
; For now, we just set it to a chosen top-of-stack address.

    ldi  yl, low(SPP)
    ldi  yh, high(SPP)

; --- 3.5 Proof of life: LED blink loop --------------------------------
; SBI/CBI set/clear one bit in an I/O register.
; DDRB controls pin direction (1=output).
; PORTB controls output value (1=high).
;
; We configure PB3 as output and then toggle it forever.
; This tells the human: “reset worked; the CPU is executing.”

    sbi  DDRB, LED_BIT            ; PB3 becomes output

BLINK:
    sbi  PORTB, LED_BIT           ; LED on (PB3 high)
    rcall DELAY_100MS
    cbi  PORTB, LED_BIT           ; LED off (PB3 low)
    rcall DELAY_100MS
    rjmp BLINK                    ; RJMP is a relative unconditional jump


; ----------------------------------------------------------------------
; 4) Delay routine (temporary scaffolding)
; ----------------------------------------------------------------------
; This delay is deliberately “dumb”: a busy-wait loop.
; It wastes CPU cycles. We accept that *only* in Stage 01.
; In later stages we replace this with a timer-based tick.
;
; RCALL calls a subroutine (relative call).
; RET returns to the caller (uses the hardware return stack).

DELAY_100MS:
    ldi  r18, 255
D1:
    ldi  r19, 255
D2:
    dec  r19                      ; DEC decrements a register by 1
    brne D2
    dec  r18
    brne D1
    ret


; ----------------------------------------------------------------------
; Appendix A — Instruction glossary introduced in this stage
; ----------------------------------------------------------------------
;  rjmp  label   : jump (relative)
;  rcall label   : call subroutine (relative), pushes return address to SP
;  ret           : return from subroutine
;  reti          : return from interrupt (like ret, plus interrupt semantics)
;  ldi  Rd, K    : load immediate constant K into register Rd
;  clr  Rd       : clear register (Rd = 0)
;  in   Rd, A    : read I/O register A into Rd   (here via in_ macro)
;  out  A, Rr    : write Rr into I/O register A  (here via out_ macro)
;  st   Y+, Rr   : store Rr to SRAM[Y], then increment Y
;  sbiw X, 1     : subtract immediate from register pair X (X = X - 1)
;  brne label    : branch if result was not zero
;  sbi  io, bit  : set bit in I/O register
;  cbi  io, bit  : clear bit in I/O register
;  dec  Rd       : decrement register by 1
;
; Appendix B — Exercises (from the book)
; ----------------------------------------------------------------------
; 1) Comment out CLR_RAM loop and observe determinism across resets.
; 2) Move SPP and consider overlap with other RAM regions.
; 3) Break SP init and observe what fails first.
; 4) Draw the SRAM map before/after reset.
;
; ======================================================================
