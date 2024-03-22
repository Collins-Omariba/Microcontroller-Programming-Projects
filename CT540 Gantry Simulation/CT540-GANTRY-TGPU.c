#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define F_CPU 16000000UL // Define the CPU clock frequency
#define BAUD 9600 // Define the desired baud rate
#define MYUBRR F_CPU/16/BAUD-1 // Calculate UBRR value

#define AXIAL_DRIVE_ENABLE_PIN PD3
#define CLOCKWISE_PIN PD4
#define ANTI_CLOCKWISE_PIN PD5
#define NORMAL_PIN PD6
#define SERVICE_PIN PD7
#define INTERRUPT_SWITCH_PIN PD2
#define TEMPERATURE_SENSOR_PIN PB1

#define GREEN_LED_PIN PC0
#define YELLOW_LED_PIN PC1
#define RED_LED_PIN PC3

volatile uint8_t axial_drive_enabled = 0;

void USART_Init(unsigned int ubrr) {
    // Set baud rate
    UBRR0H = (unsigned char)(ubrr>>8);
    UBRR0L = (unsigned char)ubrr;
    // Enable transmitter
    UCSR0B = (1<<TXEN0);
    // Set frame format: 8data, 1stop bit, no parity
    UCSR0C = (3<<UCSZ00);
}

void USART_Transmit(unsigned char data) {
    // Wait for empty transmit buffer
    while (!(UCSR0A & (1<<UDRE0)));
    // Put data into buffer, sends the data
    UDR0 = data;
}

void GPIO_Init() {
    DDRD &= ~(1 << INTERRUPT_SWITCH_PIN); // Set PD2/INT0 as input for interrupt switch
    PORTD |= (1 << INTERRUPT_SWITCH_PIN); // Enable pull-up resistor for interrupt switch
    DDRD |= (1 << AXIAL_DRIVE_ENABLE_PIN) | (1 << CLOCKWISE_PIN) | (1 << ANTI_CLOCKWISE_PIN); // Set PD3, PD4, PD5 as output for buttons
    DDRD &= ~((1 << NORMAL_PIN) | (1 << SERVICE_PIN)); // Set PD6, PD7 as input for mode pins
    PORTD |= (1 << NORMAL_PIN) | (1 << SERVICE_PIN); // Enable pull-up resistors for mode pins
    DDRC |= (1 << GREEN_LED_PIN) | (1 << YELLOW_LED_PIN) | (1 << RED_LED_PIN); // Set PC0, PC1, PC3 as output for LEDs
    // Set PB1 as input for temperature sensor (analog input)
    DDRB &= ~(1 << TEMPERATURE_SENSOR_PIN);
}

void interrupt_Init() {
    EICRA |= (1 << ISC01); // Trigger INT0 on falling edge
    EIMSK |= (1 << INT0); // Enable external interrupt INT0
}

ISR(INT0_vect) {
    // Disable axial drive and flash red LED
    axial_drive_enabled = 0;
    while(1) {
        PORTC ^= (1 << RED_LED_PIN); // Toggle red LED
        _delay_ms(500); // Delay 500ms
    }
}

void check_mode_pins() {
    // Check if NORMAL or SERVICE mode is selected
    if (!(PIND & (1 << NORMAL_PIN))) {
        PORTC |= (1 << GREEN_LED_PIN); // Turn on green LED
    } else if (!(PIND & (1 << SERVICE_PIN))) {
        PORTC |= (1 << YELLOW_LED_PIN); // Turn on yellow LED
    }
}

uint16_t read_temperature_sensor() {
    // Read 10-bit ADC value from temperature sensor
    ADMUX = (1 << REFS0) | (1 << ADLAR) | (1 << MUX0);
    ADCSRA |= (1 << ADEN) | (1 << ADSC) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

    while (ADCSRA & (1 << ADSC)); // Wait for conversion to complete

    return ADC;
}

float map_temperature(uint16_t adc_value) {
    // Map ADC value to temperature (-55°C to +150°C)
    return (adc_value * (205.0 / 1024.0)) - 55.0;
}

int main(void) {
    USART_Init(MYUBRR); // Initialize UART
    GPIO_Init(); // Initialize GPIO
    interrupt_Init(); // Initialize interrupt

    sei(); // Enable global interrupts

    while (1) {
        check_mode_pins(); // Check mode pins

        if (axial_drive_enabled) {
            if (!(PIND & (1 << AXIAL_DRIVE_ENABLE_PIN))) {
                // Read temperature sensor
                uint16_t temperature_adc = read_temperature_sensor();
                float temperature_celsius = map_temperature(temperature_adc);
                
                if (temperature_celsius > 100.0) {
                    // Temperature exceeds 100°C, turn off motor
                    USART_Transmit('2');
                } else {
                    // Check if either CLOCKWISE or ANTI_CLOCKWISE is pressed
                    if (!(PIND & (1 << CLOCKWISE_PIN))) {
                        USART_Transmit('1'); // Send '1' to ORP controller for clockwise motion
                    } else if (!(PIND & (1 << ANTI_CLOCKWISE_PIN))) {
                        USART_Transmit('0'); // Send '0' to ORP controller for anti-clockwise motion
                    }
                }
            }
        }
    }

    return 0;
}
