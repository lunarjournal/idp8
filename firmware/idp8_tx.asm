;
; @file idp8_tx.asm
; @brief 3-bit IR Data Protocol - idp8_tx.asm
;
; +---------------------------------------+
; |   .-.         .-.         .-.         |
; |  /   \       /   \       /   \        |
; | /     \     /     \     /     \     / |
; |        \   /       \   /       \   /  |
; |         "_"         "_"         "_"   |
; |                                       |
; |  _   _   _ _  _   _   ___   ___ _  _  |
; | | | | | | | \| | /_\ | _ \ / __| || | |
; | | |_| |_| | .` |/ _ \|   /_\__ \ __ | |
; | |____\___/|_|\_/_/ \_\_|_(_)___/_||_| |
; |                                       |
; |                                       |
; | Lunar RF Labs                         |
; | https://lunar.sh                      |
; |                                       |
; | RF Research Laboratories              |
; | Copyright (C) 2022-2024               |
; |                                       |
; +---------------------------------------+
;
; Copyright (c) 2015 Lunar RF Labs
; All rights reserved.
;
; Redistribution and use in source and binary forms, with or without modification,
; are permitted provided that the following conditions are met:
;
;     * Redistributions of source code must retain the above copyright notice,
;       this list of conditions and the following disclaimer.
;     * Redistributions in binary form must reproduce the above copyright notice,
;       this list of conditions and the following disclaimer in the documentation
;       and/or other materials provided with the distribution.
;
; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
; ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
; WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
; DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
; ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
; (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
; ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
; (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
; SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
;

; *************************************
; * IDP8 (TX) assembler source        *
; * Written by: Dylan Müller          *
; * Target architecture: AVR ATtiny85 *
; *************************************

; IDP8 = Infrared Data Protocol 8

 .NOLIST
 .INCLUDE "tn85def.inc"
 .LIST 
 
 ; OCR1C timer value
 ; T_TOP ~ 50% (duty cycle) 38095 Hz square output on 0C1A
 ; Fo = Frequency output on 0C1A
 ; Fo = (8000000)/(2*(104+1))

.equ      T_TOP = 0x68
.def      tlapse = R21

; Protocol definitions (Specified in 100 microseconds)
; This is our chosen timer resolution
.equ      DPACKET_HEADER = 30  ; 3ms (3000μs)
.equ      DPDATA_0 =       20  ; 2ms (2000μs)
.equ      DPDATA_1 =       10  ; 1ms (1000μs)
.equ      TX_SLEEP =       200 ; 20ms
; Time our 38kHz burst pulse is high (IR receiver specific)
.equ      TXPULSE_HIGH =   6   ; 0.6ms (600μs)

; Define packet data to be transmitted for respective inputs
.equ      I0_PACKET_DATA = 0x1 ; 001  (last bit)(middle bit)(first bit) LSB
                               ; Least significant bit first
.equ      I1_PACKET_DATA = 0x4 ; 100
.equ      I2_PACKET_DATA = 0x5 ; 101
.equ      I3_PACKET_DATA = 0x7 ; 111

; Define input bit masks for PORTB
.equ      I0_MASK = (1 << PB0)
.equ      I1_MASK = (1 << PB2)
.equ      I2_MASK = (1 << PB3)
.equ      I3_MASK = (1 << PB4)


.CSEG
.ORG      0x00

; Define ISR (Interrupt Service Routine) vectors
; Reset vector
rjmp      reset
reti
reti  ; PCINT0 dummy vector for wake-up
reti
reti
reti
reti
reti
reti
reti
;TIMER0_COMPA - Timer 0 (Output Compare Match A) ISR
rjmp      tim0_compa ; 



;.org  0xF
; Reset vector ISR
reset:
; Stack initialize
ldi      R16,	low(RAMEND) ; 
out      SPL,	R16

; Load timer 1 TOP value
ldi      R16,	T_TOP 
out      OCR1C,	R16


; Initialize CTC mode + (CLK/8) prescaler select (TIMER 0)
ldi      R16,   (1 << WGM01)
out      TCCR0A,R16
ldi      R16,   (1 << CS01)
out      TCCR0B,R16

; Load OCR0A value for 0.1ms (100us) resolution on timer 0
; Each compare match calls the OCIE0A vector's ISR
; which increments the time lapsed (tlapse).
ldi      R16,   99 ; 0.1ms
out      OCR0A, R16    

; Initialize CTC mode + (CLK/1) prescaler select (TIMER 1)
ldi      R16,	(1 << CS10) | (1 << CTC1) 
out      TCCR1,	R16

; Enable output driver (0C1A)
sbi      DDRB,	1

;Enter power reduction mode | Disable: USI
ldi      R16,   (1 << PRUSI)
out      PRR,   R16

; Enable pin change interrupts for MCU wake up
; Set PCINT masks via PCMSK
ldi      R16,   (1 << PCIE)
out      GIMSK, R16
ldi      R16,   (1 << PCINT0) | (1 << PCINT2) | (1 << PCINT3) | (1 << PCINT4)
out      PCMSK, R16

sei ; Enable interrupts

; Main loop
main:
; Enters idle-sleep and waits for input
; Sleep mode is terminated upon an external pin change event/interrupt.

ldi      R16,   (1 << SE) ; Set SE bit + Idle mode
out      MCUCR, R16
sleep
clr      R16
out      MCUCR, R16

; Pin status checks
; Jumps to respective stub if relative status port (PINB) bit(s) are set

sbic	 PINB,  0
rjmp     istub0
sbic	 PINB,  2
rjmp     istub1
sbic	 PINB,  3
rjmp     istub2
sbic	 PINB,  4
rjmp     istub3

rjmp     main


; 0C1A output connect | Mode: Toggle on compare
oc1a_enable:
in       R16, TCCR1
ori      R16, (1 <<  COM1A0)
out      TCCR1, R16
clr      R16
out      TCNT1,	R16
ret

; 0C1A output disconnect
oc1a_disable: 
in       R16, TCCR1
andi     R16, ~(1 <<  COM1A0)
out      TCCR1, R16
ret

; Timer sub-routines for generating pulse delays
; specified in our protocol definition
; These functions are used when sending data packet(s)

; Reset time
tim0_reset:
clr      R16
out      TCNT0,	R16  
clr      tlapse
ret

; TIMER0_COMPA enabled 
tim0_start:
ldi      R16,   (1 << OCIE0A)
out      TIMSK, R16
rcall    tim0_reset
ret
; TIMER0_COMPA disabled 
tim0_stop:
clr      R16
clr      tlapse
out      TIMSK, R16
ret

; R23 = Time to wait in number of .1ms (100μs)
tim0_wait:
push     R16
rcall    tim0_start
tim0_loop:
cp       tlapse,R23
brne     tim0_loop
rcall    tim0_stop
pop      R16
ret

; * Pulse sub-routine *
; This sub-routine is responsible for the repeated 
; transmission of data packets.
; R20 = pin mask
; R22 = packet data

pulse_ms:  
push     R16
in       R16,	SREG ; Save status register
push     R20
push     R22

; Continuously transmit data packets until input release
pulse_loop:
rcall    transmit_packet
ldi      R23,TX_SLEEP
rcall    tim0_wait

in       R19,   PINB
and      R19,	R20
brne     pulse_loop

pop      R22
pop      R20
out      SREG,  R16
pop      R16
ret

; TIMER0_COMPA - Timer 0 (Output Compare Match A) ISR
; This ISR is used as the 100μs clock counter
tim0_compa:
inc  tlapse 
reti

; R22 = Packet data
transmit_packet:
; Transmit packet header
rcall    tx_ph

; Send all 3 data bits

sbrc     R22, 0
rcall    txd1
sbrs     R22, 0
rcall    txd0

sbrc     R22, 1
rcall    txd1
sbrs     R22, 1
rcall    txd0

sbrc     R22, 2
rcall    txd1
sbrs     R22, 2
rcall    txd0

; Send all 3 data bits inverted

sbrc     R22, 0
rcall    txd0
sbrs     R22, 0
rcall    txd1

sbrc     R22, 1
rcall    txd0
sbrs     R22, 1
rcall    txd1

sbrc     R22, 2
rcall    txd0
sbrs     R22, 2
rcall    txd1

rjmp tx_ret

; Generate 38kHz IR burst
tx_38irp:
rcall    oc1a_enable
ldi      R23,    TXPULSE_HIGH
rcall    tim0_wait
rcall    oc1a_disable
ret
txd1:
; Transmit logic 1
rcall    tx_38irp
ldi      R23,    DPDATA_1
rcall    tim0_wait
ret

txd0:
; Transmit logic 0
rcall    tx_38irp
ldi      R23,    DPDATA_0
rcall    tim0_wait
ret

tx_ph:
; Transmit packet header
rcall    tx_38irp
ldi      R23,    DPACKET_HEADER
rcall    tim0_wait
ret

tx_ret:
; Transmit stop pulse
rcall    tx_38irp
ret

; Input stub handlers & masks
istub0:

ldi      R20,	I0_MASK
ldi      R22,	I0_PACKET_DATA
rcall    pulse_ms
rjmp     main

istub1:

ldi      R20,	 I1_MASK
ldi      R22,	 I1_PACKET_DATA
rcall    pulse_ms
rjmp     main

istub2:

ldi      R20,    I2_MASK
ldi      R22,	 I2_PACKET_DATA
rcall    pulse_ms
rjmp     main

istub3:

ldi      R20,    I3_MASK
ldi      R22,	 I3_PACKET_DATA
rcall    pulse_ms
rjmp     main
