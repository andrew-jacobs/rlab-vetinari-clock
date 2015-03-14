;===============================================================================
; __     __   _   _                  _    ____ _            _    
; \ \   / /__| |_(_)_ __   __ _ _ __(_)  / ___| | ___   ___| | __
;  \ \ / / _ \ __| | '_ \ / _` | '__| | | |   | |/ _ \ / __| |/ /
;   \ V /  __/ |_| | | | | (_| | |  | | | |___| | (_) | (__|   < 
;    \_/ \___|\__|_|_| |_|\__,_|_|  |_|  \____|_|\___/ \___|_|\_\
;
; A Simple Vetinari Clock
;-------------------------------------------------------------------------------
; Copyright (C)2013 Reading Hackspace
; All rights reserved.
;
; Redistribution and use in source and binary forms, with or without
; modification, are permitted provided that the following conditions
; are met:
;
; 1. Redistributions of source code must retain the above copyright
;    notice, this list of conditions and the following disclaimer.
; 2. Redistributions in binary form must reproduce the above copyright
;    notice, this list of conditions and the following disclaimer in the
;    documentation and/or other materials provided with the distribution.
;
; THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ''AS IS'' AND
; ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
; ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
; FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
; DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
; OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
; HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
; LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
; OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
; SUCH DAMAGE.
;===============================================================================
; Notes:
;
;
;===============================================================================
; Revision History:
;
; 2013-06-20 Initial version
;-------------------------------------------------------------------------------
; $Id: Firmware.asm 3 2013-09-21 23:35:08Z andrew $
;-------------------------------------------------------------------------------

                include P12F683.inc

                errorlevel -312
                errorlevel -302

#define M(X)    (1<<(X))

#define HI(X)   (((X) >> .8) & h'ff')
#define LO(X)   (((X) >> .0) & h'ff')

;===============================================================================
; Hardware Configuration
;-------------------------------------------------------------------------------

; CPU

OSC             equ     .4000000        ; Use built in oscilator

; I/O Pin Configuration

MAGL_PIN        equ     .1
MAGR_PIN        equ     .2

TRISIO_INIT     equ     ~(M(MAGR_PIN)|M(MAGL_PIN))

; Charge time for electro-magnet coil

CHARGE_MS       equ     .50

; Timer 0 Configuration (1kHz)

TMR0_HZ         equ     .1000
TMR0_PRESCALE   equ     .32

TMR0_PERIOD     equ     OSC / (.4 * TMR0_HZ * TMR0_PRESCALE)

; Timer 1 Configuration (32.768kHz)

TMR1_HZ         equ     .32768
TMR1_PRESCALE   equ     .4

TICKS_PER_SEC   equ     TMR1_HZ / TMR1_PRESCALE

; Time period for normal, short and long ticks

TICKS_NORM      equ     TICKS_PER_SEC
TICKS_SHORT     equ     .1 * TICKS_NORM / .2
TICKS_LONG      equ     .3 * TICKS_NORM / .2

;===============================================================================
; Device Configuration
;-------------------------------------------------------------------------------

CONFIG_STD      equ     _INTOSCIO & _MCLRE_ON & _BOREN_OFF & _IESO_OFF & _FCMEN_OFF
CONFIG_DEV      equ     _WDT_OFF & _PWRTE_ON & _CP_OFF & _CPD_OFF

                __config CONFIG_STD & CONFIG_DEV

;===============================================================================
; Data Areas
;-------------------------------------------------------------------------------

                udata_shr

LFSR            res     .2                      ; Current LFSR value

COUNT           res     .1                      ; MS Delay counter

;===============================================================================
; Interrupt Handler
;-------------------------------------------------------------------------------

.Interrupt      code    h'004'

                retfie                          ; Never actually called

;===============================================================================
; Power On Reset
;-------------------------------------------------------------------------------

.ResetVector    code    h'000'

                clrf    STATUS
                pagesel PowerOnReset    ; Jump to main initialisation
                goto    PowerOnReset

;-------------------------------------------------------------------------------

                code
PowerOnReset:
                banksel OPTION_REG      ; Configure option register
                movlw   b'10000100'
;                         1-------              GPIO pull-ups disabled
;                         -x------              Interrupt edge select 
;                         --0-----              TMR0 uses instruction clock
;                         ---x----              Source edge select
;                         ----0---              Prescaler assigned to TMR0
;                         -----100              Prescate 1:32
                movwf   OPTION_REG

;-------------------------------------------------------------------------------

                banksel GPIO            ; Initialise the GPIO port
                clrf    GPIO
                movlw   h'07'           ; Turn the comparator off
                movwf   CMCON0
                banksel ANSEL           ; Turn analog inputs off
                clrf    ANSEL

                movlw   TRISIO_INIT     ; Set the pin I/O states
                movwf   TRISIO

;-------------------------------------------------------------------------------

                banksel T1CON           ; Configure Timer1 to use watch crystal
                movlw   b'00101110'
;                         0-------              Timer gate is not inverted
;                         -0------              Timer gate is not enabled
;                         --10----              Prescaler 1:4
;                         ----1---              Oscillator is enabled
;                         -----1--              Asynchronous external input
;                         ------1-              External input enabled
;                         -------0              Timer disabled
                movwf   T1CON

                clrf    TMR1L           ; Clear the timer
                clrf    TMR1H

                banksel T1CON           ; And start the timer
                bsf     T1CON,TMR1ON

                banksel PIE1            ; Enable interrupts to allow wake-up
                bsf     PIE1,TMR1IE
                bsf     INTCON,PEIE

;-------------------------------------------------------------------------------

                movlw   h'ff'           ; Seed the LFSR
                movwf   LFSR+.0
                movwf   LFSR+.1

;===============================================================================
; Main Task
;-------------------------------------------------------------------------------

MainTask:
                movlw   high JumpTable  ; Jump into pattern table based on
                movwf   PCLATH          ; .. bits from the LFSR
                movf    LFSR+.0,W
                andlw   h'0f'
                addlw   low JumpTable
                btfsc   STATUS,C
                incf    PCLATH,F
                movwf   PCL

JumpTable
                goto    Pattern0
                goto    Pattern0
                goto    Pattern1
                goto    Pattern0
                goto    Pattern2
                goto    Pattern0
                goto    Pattern0
                goto    Pattern1
                goto    Pattern0
                goto    Pattern3
                goto    Pattern0
                goto    Pattern1
                goto    Pattern0
                goto    Pattern2
                goto    Pattern0
                goto    Pattern0

;-------------------------------------------------------------------------------

Pattern0:
                call    NormalTickL     ; Normal pattern of eight ticks
                call    NormalTickR
                call    NormalTickL
                call    NormalTickR
                call    NormalTickL
                call    NormalTickR
                call    NormalTickL
                call    NormalTickR
                goto    MainTask

Pattern1:
                call    ShortTickL     ; One irregular tick
                call    NormalTickR
                call    NormalTickL
                call    NormalTickR
                call    LongTickL
                call    NormalTickR
                call    NormalTickL
                call    NormalTickR
                goto    MainTask

Pattern2:
                call    ShortTickL     ; Two irregular ticks
                call    ShortTickR
                call    NormalTickL
                call    NormalTickR
                call    LongTickL
                call    NormalTickR
                call    LongTickL
                call    NormalTickR
                goto    MainTask

Pattern3:
                call    ShortTickL     ; Three irregular ticks
                call    ShortTickR
                call    NormalTickL
                call    ShortTickR
                call    LongTickL
                call    LongTickR
                call    NormalTickL
                call    LongTickR
                goto    MainTask

;-------------------------------------------------------------------------------

NormalTickL:
                banksel T1CON           ; Stop the timer
                bcf     T1CON,TMR1ON
                movlw   HI(-TICKS_NORM) ; Load count for next period
                movwf   TMR1H
                movlw   LO(-TICKS_NORM)
                movwf   TMR1L
                goto    LeftPulse

ShortTickL:
                banksel T1CON           ; Stop the timer
                bcf     T1CON,TMR1ON
                movlw   HI(-TICKS_SHORT); Load count for next period
                movwf   TMR1H
                movlw   LO(-TICKS_SHORT)
                movwf   TMR1L
                goto    LeftPulse

LongTickL:
                banksel T1CON           ; Stop the timer
                bcf     T1CON,TMR1ON
                movlw   HI(-TICKS_LONG) ; Load count for next period
                movwf   TMR1H
                movlw   LO(-TICKS_LONG)
                movwf   TMR1L

LeftPulse:
                bsf     T1CON,TMR1ON    ; Restart the timer

                banksel GPIO
                bsf     GPIO,MAGL_PIN
                call    PulseDelay
                bcf     GPIO,MAGL_PIN

                call    Shift
                banksel PIR1            ; Clear the interrupt flag
                bcf     PIR1,TMR1IF
                sleep

                nop
                return

;-------------------------------------------------------------------------------

NormalTickR:
                banksel T1CON           ; Stop the timer
                bcf     T1CON,TMR1ON
                movlw   HI(-TICKS_NORM) ; Load count for next period
                movwf   TMR1H
                movlw   LO(-TICKS_NORM)
                movwf   TMR1L
                goto    RightPulse

ShortTickR:
                banksel T1CON           ; Stop the timer
                bcf     T1CON,TMR1ON
                movlw   HI(-TICKS_SHORT); Load count for next period
                movwf   TMR1H
                movlw   LO(-TICKS_SHORT)
                movwf   TMR1L
                goto    RightPulse

LongTickR:
                banksel T1CON           ; Stop the timer
                bcf     T1CON,TMR1ON
                movlw   HI(-TICKS_LONG) ; Load count for next period
                movwf   TMR1H
                movlw   LO(-TICKS_LONG)
                movwf   TMR1L

RightPulse:
                bsf     T1CON,TMR1ON    ; Restart the timer

                banksel GPIO
                bsf     GPIO,MAGR_PIN
                call    PulseDelay
                bcf     GPIO,MAGR_PIN

                call    Shift
                banksel PIR1            ; Clear the interrupt flag
                bcf     PIR1,TMR1IF
                sleep

                nop
                return

;===============================================================================
; MS Delay
;-------------------------------------------------------------------------------

; Generate a delay in milliseconds (based on the value in WREG) by iterating
; aroung a 1mS delay generated using Timer 0.

PulseDelay:
                movlw   CHARGE_MS       ; Save the mS count       
                movwf   COUNT           

DelayLoop:
                banksel TMR0            ; Set Timer 0 for a 1mS period
                movlw   LO(-TMR0_PERIOD)
                movwf   TMR0

                bcf     INTCON,T0IF     ; Clear the interrupt flag
                btfss   INTCON,T0IF     ; And wait for it to become set again
                goto    $-1

                decfsz  COUNT,F         ; Reduce count and repeat
                goto    DelayLoop
                return                  ; .. until complete

;===============================================================================
; Random Number Generator
;-------------------------------------------------------------------------------

Shift:
                bcf     STATUS,C        ; Shift the LFSR one bit
                rrf     LFSR+.1,F
                rrf     LFSR+.0,F
                btfss   STATUS,C
                return
                movlw   h'b4'           ; And XOR back it dropped bits
                xorwf   LFSR+.1,F
                return

                end