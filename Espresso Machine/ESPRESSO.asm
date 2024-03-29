;
; ESPRESSO.asm
;
; Created: 3/27/2024 3:11:29 PM
; Author : ONAMI OMARIBA COLLINS
; REG NO: ENM221-0062/2020
;

; Initialize registers, ports, and peripherals
.include "m328Pdef.inc"

.def temp = r16    ; Temperature register
.def status = r17  ; Machine status register
.def btn = r18     ; Button input register
.def pressure = r19  ; Pressure/Water level register

; LCD Display Constants
.equ LCD_DATA = PORTC ; Using PORTC pins PC0-PC5 for LCD data
.equ LCD_CTRL = PORTD
.equ LCD_RS = 0
.equ LCD_EN = 1

; LED Status Constants
.equ STATUS_LED_HEATING = 2
.equ STATUS_LED_BREWING = 3

; Buzzer Constant
.equ BUZZER_PIN = PORTB4

; Button Constants
.equ BTN_START = PIND3
.equ BTN_ESPRESSO = PIND4
.equ BTN_AMERICANO = PIND5
.equ BTN_LATTE = PIND6

; Status Constants
.equ STATUS_IDLE = 0
.equ STATUS_HEATING = 1
.equ STATUS_BREWING = 2
.equ STATUS_READY = 3

reset:
    ; Initialize ports and peripherals
    ldi r16, 0xFF  ; Set all pins as inputs
    out DDRC, r16  ; Initialize PORTC as output (for LCD data)
    out DDRD, r16  ; Initialize PORTD as output (for LCD control and button inputs)
    ldi r16, 0b00001111  ; Set PB0-PB3 as outputs (for relay, LEDs, and buzzer)
    out DDRB, r16

    ; Set up LCD display
    rcall lcd_init

    ; Set up button inputs
    ldi r16, (1<<BTN_START) | (1<<BTN_ESPRESSO) | (1<<BTN_AMERICANO) | (1<<BTN_LATTE)
    out PORTD, r16  ; Enable internal pull-up resistors for button inputs

    ; Set up ADC for temperature sensor and potentiometer
    ldi r16, 0b11000011  ; Enable ADC0 and ADC1
    sts ADMUX, r16
    ldi r16, 0b11000000  ; Enable ADC, set prescaler to 128
    sts ADCSRA, r16

    ; Initialize status to idle
    ldi status, STATUS_IDLE

loop:
    ; Read button inputs
    in btn, PIND

    ; Check if Start/Stop button is pressed
    sbrc btn, BTN_START
    rjmp start_stop

    ; Check if Espresso button is pressed
    sbrc btn, BTN_ESPRESSO
    rjmp brew_espresso

    ; Check if Americano button is pressed
    sbrc btn, BTN_AMERICANO
    rjmp brew_americano

    ; Check if Latte button is pressed
    sbrc btn, BTN_LATTE
    rjmp brew_latte

    ; Monitor temperature and pressure/water level
    rcall read_temperature
    rcall read_pressure

    ; Update status based on temperature and pressure
    rcall update_status

    ; Update LCD display, LEDs, and buzzer
    rcall update_display

    rjmp loop

; Subroutine for brewing espresso
brew_espresso:
    ldi status, STATUS_BREWING
    rcall start_brewing
    ldi r24, 'E'
    ldi r25, 'S'
    ldi r26, 'P'
    ldi r27, 'R'
    ldi r28, 'E'
    ldi r29, 'S'
    ldi r30, 'S'
    ldi r31, 'O'
    rcall lcd_print
    rjmp loop

; Subroutine for brewing Americano
brew_americano:
    ldi status, STATUS_BREWING
    rcall start_brewing
    ldi r24, 'A'
    ldi r25, 'M'
    ldi r26, 'E'
    ldi r27, 'R'
    ldi r28, 'I'
    ldi r29, 'C'
    ldi r30, 'A'
    ldi r31, 'N'
    ldi r16, 'O'
    rcall lcd_print_char
    rjmp loop

; Subroutine for brewing Latte
brew_latte:
    ldi status, STATUS_BREWING
    rcall start_brewing
    ldi r24, 'L'
    ldi r25, 'A'
    ldi r26, 'T'
    ldi r27, 'T'
    ldi r28, 'E'
    rcall lcd_print
    rjmp loop

; Subroutine for reading temperature
read_temperature:
    ; Read temperature from ADC0
    ldi r16, 0b01000000  ; Start ADC conversion on ADC0
    sts ADCSRA, r16
read_temp_loop:
    lds r16, ADCSRA
    sbrc r16, ADIF  ; Check if conversion is complete
    rjmp read_temp_done
    rjmp read_temp_loop
read_temp_done:
    lds temp, ADCL  ; Read temperature value from ADC data register
    ret

; Subroutine for reading pressure/water level
read_pressure:
    ; Read pressure/water level from ADC1
    ldi r16, 0b01000010  ; Start ADC conversion on ADC1
    sts ADCSRA, r16
read_pressure_loop:
    lds r16, ADCSRA
    sbrc r16, ADIF  ; Check if conversion is complete
    rjmp read_pressure_done
    rjmp read_pressure_loop
read_pressure_done:
    lds pressure, ADCL  ; Read pressure/water level value from ADC data register
    ret

; Subroutine for updating status based on temperature and pressure
update_status:
    ; Check if temperature is below the threshold for heating
    ldi r16, 150  ; Adjust the temperature threshold as needed
    cp temp, r16
    brlo start_heating

    ; Check if pressure/water level is below the threshold for brewing
    ldi r16, 100  ; Adjust the pressure/water level threshold as needed
    cp pressure, r16
    brlo refill_water

    ; If both temperature and pressure/water level are good, set status to ready
    ldi status, STATUS_READY
    ret

start_heating:
    ldi status, STATUS_HEATING
    sbi PORTB, 0  ; Turn on heating element
    ret

refill_water:
    ldi status, STATUS_IDLE
    cbi PORTB, 0  ; Turn off heating element
    cbi PORTB, 1  ; Turn off pump
    ; Display "Refill Water" message on LCD
    rcall lcd_clear
    ldi r24, 'R'
    ldi r25, 'E'
    ldi r26, 'F'
    ldi r27, 'I'
    ldi r28, 'L'
    ldi r29, 'L'
    ldi r30, ' '
    ldi r31, 'W'
    rcall lcd_print
    ldi r24, 'A'
    ldi r25, 'T'
    ldi r26, 'E'
    ldi r27, 'R'
    rcall lcd_print
    ret

; Subroutine for starting the brewing process
start_brewing:
    sbi PORTB, 0  ; Turn on heating element
    sbi PORTB, 1  ; Turn on pump
    ; Display "Brewing..." message on LCD
    rcall lcd_clear
    ldi r24, 'B'
    ldi r25, 'R'
    ldi r26, 'E'
    ldi r27, 'W'
    ldi r28, 'I'
    ldi r29, 'N'
    ldi r30, 'G'
    ldi r31, '.'
    rcall lcd_print
    ldi r24, '.'
    ldi r25, '.'
    rcall lcd_print
    rcall delay   ; Simulate brewing time
    cbi PORTB, 0  ; Turn off heating element
    cbi PORTB, 1  ; Turn off pump
    ret

; Subroutine for updating LCD display, LEDs, and buzzer
update_display:
    rcall lcd_clear
    ldi r16, 0x80  ; Set cursor to the beginning of the first line
    rcall lcd_cmd

    ; Display status message
    ldi r16, STATUS_IDLE
    cp status, r16
    breq display_idle

    ldi r16, STATUS_HEATING
    cp status, r16
    breq display_heating

    ldi r16, STATUS_BREWING
    cp status, r16
    breq display_brewing

    rjmp display_ready

display_idle:
    ldi r24, 'I'
    ldi r25, 'D'
    ldi r26, 'L'
    ldi r27, 'E'
    rcall lcd_print
    cbi PORTB, 2  ; Turn off status LED
    rjmp loop

display_heating:
    ldi r24, 'H'
    ldi r25, 'E'
    ldi r26, 'A'
    ldi r27, 'T'
    ldi r28, 'I'
    ldi r29, 'N'
    ldi r30, 'G'
    rcall lcd_print
    sbi PORTB, 2  ; Turn on status LED (heating)
    rjmp loop

display_brewing:
    ldi r24, 'B'
    ldi r25, 'R'
    ldi r26, 'E'
    ldi r27, 'W'
    ldi r28, 'I'
    ldi r29, 'N'
    ldi r30, 'G'
    rcall lcd_print
    sbi PORTB, 3  ; Turn on status LED (brewing)
    sbi PORTB, 4  ; Beep buzzer
    rcall delay
    cbi PORTB, 4  ; Stop buzzer
    rjmp loop

display_ready:
    ldi r24, 'R'
    ldi r25, 'E'
    ldi r26, 'A'
    ldi r27, 'D'
    ldi r28, 'Y'
    rcall lcd_print
    sbi PORTB, 4  ; Beep buzzer
    rcall delay
    cbi PORTB, 4  ; Stop buzzer
    rjmp loop

start_stop:
    rjmp loop  ; Ignore Start/Stop button for now

; Delay subroutine
delay:
    ldi r24, 100  ; Adjust the value for the desired delay
delay_loop:
    dec r24
    brne delay_loop
    ret


; LCD initialization and printing subroutines
lcd_init:
    ldi r16, 0x28  ; Initialize the LCD in 4-bit mode
    rcall lcd_cmd
    ldi r16, 0x0C  ; Display ON, Cursor OFF
    rcall lcd_cmd
    ldi r16, 0x01  ; Clear display
    rcall lcd_cmd
    ldi r16, 0x06  ; Entry mode set (increment cursor)
    rcall lcd_cmd
    ret

; Send Command to LCD Subroutine
lcd_cmd:
    cbi LCD_CTRL, LCD_RS  ; Clear RS (Command register)
    rcall lcd_write
    ret

; Print Characters to LCD Subroutine
lcd_print:
    ldi r31, 0  ; Initialize r31 (counter) to 0

lcd_print_loop:
    mov r30, r24  ; Move the character to be printed to r30
    rcall lcd_print_char
    lpm r24, Z+  ; Load next character from program memory
    inc r31  ; Increment counter
    cpi r31, 8  ; Check if 8 characters have been printed
    brne lcd_print_loop  ; Branch if not all characters have been printed
    ret


; Print Single Character to LCD Subroutine
lcd_print_char:
    sbi LCD_CTRL, LCD_RS  ; Set RS (Data register)
    rcall lcd_write
    ret


; Clear LCD Display Subroutine
lcd_clear:
    ldi r16, 0x01  ; Clear display command
    rcall lcd_cmd
    ret

; LCD Write Subroutine (Helper function)
lcd_write:
    sbi LCD_CTRL, LCD_EN  ; Set Enable pin high
    mov r30, r16  ; Move data/command to r30 (temporary register)
    andi r30, 0xF0  ; Mask the upper nibble
    out LCD_DATA, r30  ; Write the upper nibble to LCD
    nop  ; Small delay
    cbi LCD_CTRL, LCD_EN  ; Clear Enable pin
    sbi LCD_CTRL, LCD_EN  ; Set Enable pin high
    swap r30  ; Swap nibbles
    andi r30, 0xF0  ; Mask the upper nibble (now containing the lower nibble)
    out LCD_DATA, r30  ; Write the lower nibble to LCD
    nop  ; Small delay
    cbi LCD_CTRL, LCD_EN  ; Clear Enable pin
    ldi r24, 0x02  ; Delay value (adjust as needed)
    rcall delay_ms  ; Call delay subroutine
    ret


; Delay Milliseconds Subroutine (Helper function)
delay_ms:
    push r24  ; Save r24 value
    push r25  ; Save r25 value

delay_ms_loop:
    ldi r25, 0x06  ; Adjust this value for desired delay

delay_ms_inner:
    dec r25
    brne delay_ms_inner
    dec r24
    brne delay_ms_loop
    pop r25  ; Restore r25 value
    pop r24  ; Restore r24 value
    ret











