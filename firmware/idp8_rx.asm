;
; @file idp8_rx.asm
; @brief 3-bit IR Data Protocol - idp8_rx.asm
;
; Author: Dylan Müller
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
; * IDP8 (RX) assembler source        *
; * Target architecture: AVR ATtiny85 *
; *************************************

; IDP8 = Infrared Data Protocol 8

 .NOLIST
 .INCLUDE "tn85def.inc"
 .LIST 
 
 .equ     CLK_DEL = 99
 .equ     TIM_THRESH = 5 

  ;f(t_timeout) = 25.5ms (25500μs)  * TIM_THRESH
 ; The desired packet timeout time must be multiplied by 25.5ms as a result of register (R25) overflow ,
 ; used in conjunction with R26 to achieve longer time delays (see below) 
 ; and because our clock/counter has a 100us resolution.
 ; The relative time-out delay is achieved by overflowing the 8-bit t_timeout(R25) register and thereafter
 ; incrementing t_timeout_2(R26), continuously comparing R26 with TIM_THRESH, 
 ; until we exceed our delay threshold.

.def     packet_info = R24
.def     dinv_info =   R19
.def     t_timeout =   R25
.def     t_timeout_2 = R26

; Protocol definitions (Specified as a multiple of 100 microseconds)
; This is our chosen timer resolution
.equ      DPACKET_HEADER = 30  ; 3ms (3000μs)
.equ      DPDATA_0 =       20  ; 2ms (2000μs)
.equ      DPDATA_1 =       10  ; 1ms (1000μs)
; Defines pulse delay tolerance 
.equ      SIG_TOLERANCE  = 1   ; +/- 100μs

 ; Reserved registers
 .def     pflags  = R21
 .def     tlapse  = R20
 
 ;TX packet data to be compared with
.equ      I0_PACKET_DATA = 0x1 ; 001  (last bit)(middle bit)(first bit) LSB
; Least significant bit first
.equ      I1_PACKET_DATA = 0x4 ; 100
.equ      I2_PACKET_DATA = 0x5 ; 101
.equ      I3_PACKET_DATA = 0x7 ; 111

; packet_info (R24) register explanation
; 7  6  5  4  3  2  1  0  (bit positions) LSB
; -  -  -  -  -  -  -  -  (packet_info) register
;          -  -  -  -     (packet_info mask bits)
; The packet_info mask bits determine which bit should be validated upon a 
; jump to pk_dtest
; 0 = Packet header validated
; 1 = Data bit 1 validated
; 2 = Data bit 2 validated
; 3 = Data bit 3 validated
; 4 = Data bit 1 value
; 5 = Data bit 2 value
; 6 = Data bit 3 value
; 7 = Reserved

; dinv_info (R19) register explanation
; 7  6  5  4  3  2  1  0  (bit positions) LSB
; -  -  -  -  -  -  -  -  (packet_info) register
; The dinv_info (data inverted) information register is used to
; capture and validate the 3 inverted data bits which are transmitted after
; the 3 normal data bits, this mechanism is used to verify data integrity.

; 0 = INV Data bit 1 validated
; 1 = INV Data bit 2 validated
; 2 = INV Data bit 3 validated
; 3 = Reserved
; 4 = INV Data bit 1 value
; 5 = INV Data bit 2 value
; 6 = INV Data bit 3 value
; 7 = Reserved

; pflags (R21) register explanation
; 7  6  5  4  3  2  1  0  (bit positions) LSB
; -  -  -  -  -  -  -  -  (pflags) register

; 0 = Last pin status of pin 0 (PINB0)
; 1 = tlapse register available for processing
; Status bit 1 is set upon a low-high logic transition on input pin 0 (PINB0)

.CSEG
.ORG      0x00
; Define ISR (Interrupt Service Routine) vectors
; Reset vector
rjmp      reset
reti
; PCINT0 (Pin change interrupt) vector
rjmp      pc_int
reti
reti
reti
reti
reti
reti
reti
;TIMER0_COMPA - Timer 0 (Output Compare Match A) ISR
rjmp      tim0_compa 
;TIMER0_COMPA - Timer 0 (Output Compare Match B) ISR
rjmp      tim0_compb ; Timer compare interrupt

;.org  0xF
; Reset vector sub-routine
reset:
; Stack initialize
ldi      R16,	low(RAMEND) ; 
out      SPL,	R16
; Enable output drivers
ldi      R16,   0x1E
out      DDRB,  R16
; Initialize CTC mode + (CLK/8) prescaler select (TIMER 0)
ldi      R16,   (1 << WGM01)
out      TCCR0A,R16
ldi      R16,   (1 << CS01)
out      TCCR0B,R16
ldi      R16,	CLK_DEL
out      OCR0A, R16
ldi      R16,	CLK_DEL
out      OCR0B,	R16
;Enter power reduction mode | Disable: USI
ldi      R16,   (1 << PRUSI)
out      PRR,   R16
; Enable pin change interrupts for wake up
; Set PCINT masks via PCMSK
ldi      R16,   (1 << PCIE)
out      GIMSK, R16
ldi      R16,   (1 << PCINT0)
out      PCMSK, R16
sei ; Enable interrupts

; Main loop
main:
; Jump if we have a reading
sbrc     pflags,0x1
rjmp     smp_match
rjmp     main

set_packet:
ldi      packet_info, 0x1
rjmp smp_end
packet_reset:
clr      packet_info
rjmp smp_end

smp_match:
cli
in       R22,	SREG
; Has the packet header been validated?
; i.e the packet header validation bit in the packet_header register
; been set?
sbrc     packet_info, 0
rjmp     pd_aquire
; If not test for packet header
pkheader_test:
cpi      tlapse, (DPACKET_HEADER - SIG_TOLERANCE)
brge     pkht_upper
rjmp     smp_end
pkht_upper:
cpi      tlapse, (DPACKET_HEADER + (SIG_TOLERANCE+1))
brlo     set_packet

; Our packet header has been validated at this stage.
pd_aquire:
; Test if any of the 3 data bits in our packet have been processed. 
; If not jump to the respective handler routines
sbrs     packet_info, 1
rjmp     data_1
sbrs     packet_info, 2
rjmp     data_2
sbrs     packet_info, 3
rjmp     data_3
; Testing remaining 3 inverted data bits
sbrs     dinv_info, 0
rjmp     dinv_1
sbrs     dinv_info, 1
rjmp     dinv_2
sbrs     dinv_info, 2
rjmp     dinv_3


; At this stage all the data bits have been
; successfully captured and validated.
; We are ready to process our captured packet
rjmp     data_proc
 

; Handler routines pass the relevant bit mask of the packet_info
; register to the pk_dtest sub-routine. 
; This sub-routine's purpose is to validate the pulse delay
; between high-low and low-high logic transitions (stored in tlapse) 
; on pin 0 (PINB0)
; Furthermore the sub-routine uses this information to differentiate between logic 1
; and logic 0 bit information provided the pulse delay is within configured bounds 
; for those respective logic levels.
; The relevant packet bits are then set accordingly and a jump to smp_end 
; is executed
data_1:
ldi      R23, 0x1
rjmp     pk_dtest
data_2:
ldi      R23, 0x2
rjmp     pk_dtest
data_3:
ldi      R23, 0x4
rjmp     pk_dtest

dinv_1:
ldi      R23, 0x1
rjmp     dinv_test
dinv_2:
ldi      R23, 0x2
rjmp     dinv_test
dinv_3:
ldi      R23, 0x4
rjmp     dinv_test

; Process data packet
data_proc:
; Optional unused status bit. Not needed in this case
;sbr      packet_info, 0x80
;extract data bit values
andi     packet_info, 0x70
ldi      R16, 4
shift_4:
lsr      packet_info
dec      R16
brne     shift_4
; Compare packet data
mov      R16, packet_info
cpi      R16, I0_PACKET_DATA
breq     led
mov      R16, packet_info
cpi      R16, I1_PACKET_DATA
breq     led2
mov      R16, packet_info
cpi      R16, I2_PACKET_DATA
breq     led3
mov      R16, packet_info
cpi      R16, I3_PACKET_DATA
breq     led4
rjmp     smp_clear

;data handler 1
led:
ldi      R16,    0x2
out      PORTB,  R16
rjmp     smp_clear
;data handler 2
led2:
ldi      R16,    0x4
out      PORTB,  R16
rjmp     smp_clear
;data handler 3
led3:
ldi      R16,    0x10
out      PORTB,  R16
rjmp     smp_clear
;data handler 4
led4:
ldi      R16,    0x8
out      PORTB,  R16

smp_clear:
clr      t_timeout
clr      t_timeout_2
; Activate TIMER0_COMPB (tim0_compb) ISR
in       R16,    TIMSK
ori      R16,    (1<< OCIE0B)
out      TIMSK,	 R16
rjmp     packet_reset  

smp_end:  
clr      tlapse
clr      R23
; Clear second bit in pflags
andi     pflags, 0x1
out      SREG,   R22     
sei
rjmp main

; Packet data test sub-routine
; R23 = packet_info bit mask. This mask describes
; which data bit is to be processed if it has not already.
; As noted above, the individual bit meanings in the packet_info register
; are as follows, starting with the least significant bit (LSB)
; 7  6  5  4  3  2  1  0  (bit positions) LSB
; -  -  -  -  -  -  -  -  (packet_info) register
;          -  -  -  -     (packet_info bit mask bits)
; The packet_info mask bits determine which bit should be validated upon a 
; jump to pk_dtest
; 0 = Packet header validated
; 1 = Data bit 1 validated
; 2 = Data bit 2 validated
; 3 = Data bit 3 validated
; 4 = Data bit 1 value
; 5 = Data bit 2 value
; 6 = Data bit 3 value
; 7 = Reserved

; After the sub-routine has finished execution
; the relevant bits in the packet_info register
; would have been set
pk_dtest:

; Shift R23 one bit to the left 
; All bit shifting found in this sub-routine
; is used for setting the correct bits in the
; packet_info register
lsl      R23
; Perform validation via boundary checks,
; taking into account pulse delay tolerances
test_high:
cpi      tlapse, (DPDATA_1 - SIG_TOLERANCE)
brge     dhtest_upper
rjmp     test_low
dhtest_upper:
cpi      tlapse, (DPDATA_1 + (SIG_TOLERANCE+1))
brlo     dset_high
test_low:
cpi      tlapse, (DPDATA_0 - SIG_TOLERANCE)
brge     dltest_upper
rjmp     packet_reset
dltest_upper:
cpi      tlapse, (DPDATA_0 + (SIG_TOLERANCE+1))
brlo     dset_low
rjmp     packet_reset

; The tlapse register (pulse delay) has been validated to logic 1
; the requested validation bit, set by the relevant mask,
; is now set as well as the respective data value bit.
dset_high:
or       packet_info, R23
; shift 3 left
ldi      R16, 3
shift_3:
lsl      R23
dec      R16
brne     shift_3
or       packet_info, R23
rjmp     smp_end

; The tlapse register (pulse delay) has been validated to logic 0
; the requested validation bit, obtained by the relevant mask,
; is now set and the data value bit cleared.
dset_low:
or       packet_info, R23
rjmp     smp_end

dinv_reset:
clr      packet_info
clr      dinv_info
rjmp     smp_end


dinv_test:
dinvtest_high:
cpi      tlapse, (DPDATA_1 - SIG_TOLERANCE)
brge     dhtest_invupper
rjmp     invtest_low
dhtest_invupper:
cpi      tlapse, (DPDATA_1 + (SIG_TOLERANCE+1))
brlo     dinv_hcheck
invtest_low:
cpi      tlapse, (DPDATA_0 - SIG_TOLERANCE)
brge     dltest_invupper
rjmp     dinv_reset
dltest_invupper:
cpi      tlapse, (DPDATA_0 + (SIG_TOLERANCE+1))
brlo     dinv_lcheck
rjmp     dinv_reset

dinv_lcheck:
or       dinv_info, R23
rjmp     dinv_checkl
dinv_hcheck:
or       dinv_info, R23
lsl      R23
ldi      R16, 3
shiftinv_3:
lsl      R23
dec      R16
brne     shiftinv_3
or       dinv_info, R23      
dinv_checkl:
mov      R16, packet_info
andi     R16, 0x70
and      R16, R23
cp       R16, R23
brne     dinvchk_end
rjmp     dinv_reset
dinvchk_end:
rjmp     smp_end


; PCINT0 ISR
; Pin change interrupt service routine
; This ISR is used for timing the delay between
; high-low and low-high state transitions.
; This time delay is then stored in the tlapse register
pc_int:
in       R22,    SREG ; Save SREG
sbrc     pflags, 0
rjmp     pcint_htol

; Status bit 0 in pflags was low before interrupt
pcint_ltoh: 
sbic     PINB,   0
sbr      pflags, 1

sbrs     pflags, 0
rjmp     pcint_ret
in       R16,    TIMSK
andi     R16,    (1 << OCIE0B)
out      TIMSK,	 R16
; Signal reading status
ori      pflags, 0x2
rjmp     pcint_ret

; Status bit 0 in pflags was high before interrupt
pcint_htol: 
sbis     PINB,   0
cbr      pflags, 1

sbrc     pflags, 0
rjmp     pcint_ret
in       R16,    TIMSK
ori      R16,    (1 << OCIE0A)
out      TIMSK,	 R16
clr      R16
clr      tlapse
out      TCNT0,	 R16  

pcint_ret:
out      SREG,   R22
reti 

; TIMER0_COMPA - Timer 0 (Output Compare Match A) ISR
; This ISR is used as the 100μs clock counter
tim0_compa:
inc      tlapse
reti

; TIMER0_COMPB - Timer 0 (Output Compare Match B) ISR
; This ISR is activated upon setting any of the output
; bits in the PORTB register. 
; The ISR serves as a timeout function ,setting all output
; bits in the PORTB register to a logic 0 if a packet is not
; received within the specified time threshold.
; The TIM_THRESH preprocessor definition defines this time threshold.
tim0_compb:
in       R22,    SREG ; Save status register
; Increment until t_timeout overflows
inc      t_timeout
brne     tim0_ret
inc      t_timeout_2
; If t_timeout_2 matches our threshold value carry
; on with execution
cpi      t_timeout_2, TIM_THRESH
brlo     tim0_ret   

; Clear respective registers
clr      R16
out      PORTB,  R16
out      TCNT0,	 R16  
out      TIMSK,	 R16
clr      tlapse
clr      t_timeout
clr      t_timeout_2
clr      pflags
clr      packet_info
clr      dinv_info

tim0_ret:
out      SREG,   R22 ; Restore status register
reti
