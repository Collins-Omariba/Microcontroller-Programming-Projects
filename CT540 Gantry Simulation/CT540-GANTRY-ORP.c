#include <avr/io.h>
#include <util/delay.h>

#define F_CPU 16000000UL // Define the CPU clock frequency
#define BAUD 9600 // Define the desired baud rate
#define MYUBRR F_CPU/16/BAUD-1 // Calculate UBRR value

#define Q1 PB5
#define Q2 PB4
#define Q3 PB3
#define Q4 PB2

void USART_Init(unsigned int ubrr) {
    // Set baud rate
    UBRR0H = (unsigned char)(ubrr>>8);
    UBRR0L = (unsigned char)ubrr;
    // Enable receiver
    UCSR0B = (1<<RXEN0);
    // Set frame format: 8data, 1stop bit, no parity
    UCSR0C = (3<<UCSZ00);
}

unsigned char USART_Receive(void) {
    // Wait for data to be received
    while (!(UCSR0A & (1<<RXC0)));
    // Get and return received data from buffer
    return UDR0;
}

void GPIO_Init() {
    DDRB |= (1 << Q1) | (1 << Q2) | (1 << Q3) | (1 << Q4); // Set PB2, PB3, PB4, PB5 as output for H-bridge switches
}

void control_motor(char direction) {
    switch (direction) {
        case '1':
            PORTB |= (1 << Q1) | (1 << Q4); // Turn on Q1 and Q4 for clockwise motion
            PORTB &= ~((1 << Q2) | (1 << Q3)); // Turn off Q2 and Q3
            break;
        case '0':
            PORTB |= (1 << Q2) | (1 << Q3); // Turn on Q2 and Q3 for anti-clockwise motion
            PORTB &= ~((1 << Q1) | (1 << Q4)); // Turn off Q1 and Q4
            break;
        case '2':
            PORTB &= ~((1 << Q1) | (1 << Q2) | (1 << Q3) | (1 << Q4)); // Turn off all switches
            break;
        default:
            break;
    }
}

int main(void) {
    USART_Init(MYUBRR); // Initialize UART
    GPIO_Init(); // Initialize GPIO

    while (1) {
        char received_data = USART_Receive(); // Receive data from TGPU controller
        control_motor(received_data); // Control motor based on received data
    }

    return 0;
}
