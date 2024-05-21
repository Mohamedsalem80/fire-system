#define F_CPU 16000000UL  // Define CPU clock speed
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>

#define IR_RECEIVE_PIN 6
#define RX_PIN 4
#define TX_PIN 5
#define RST_PIN 1
#define SS_PIN 2
#define FLAME_PIN 2
#define GAS_PIN 3
#define BUZZER_PIN 0
#define RED_LED_PIN 1
#define GREEN_LED_PIN 2

volatile char buff[32];
volatile char pass[10];
volatile int passcount = 0;
volatile bool admin_authorization = false;
volatile bool pass_enter = false;
volatile bool flame = false;
volatile bool gas = false;

void USART_Init(unsigned int baud) {
    UBRRH = (unsigned char)(baud>>8);
    UBRRL = (unsigned char)baud;
    UCSRB = (1<<RXEN)|(1<<TXEN);
    UCSRC = (1<<URSEL)|(3<<UCSZ0);
}

void USART_Transmit(unsigned char data) {
    while (!(UCSRA & (1<<UDRE)));
    UDR = data;
}

unsigned char USART_Receive(void) {
    while (!(UCSRA & (1<<RXC)));
    return UDR;
}

void USART_SendString(char* str) {
    while (*str) {
        USART_Transmit(*str++);
    }
}

void init() {
    DDRB |= (1 << SS_PIN) | (1 << RST_PIN) | (1 << BUZZER_PIN) | (1 << RED_LED_PIN) | (1 << GREEN_LED_PIN);
    DDRD &= ~((1 << FLAME_PIN) | (1 << GAS_PIN));
    PORTD |= (1 << FLAME_PIN) | (1 << GAS_PIN);

    USART_Init(9600);
}

void FlameDetected() {
    // Implement flame detection logic here
    flame = true;
}

void gasDetected() {
    // Implement gas detection logic here
    gas = true;
}

void loop() {
    // IR receiving logic or any other input receiving mechanism goes here
    // For example, if using interrupt-driven UART receiving:
    if (UCSRA & (1 << RXC)) {
        char receivedData = UDR;
        // Process the received data
    }

    // Process IR received data or any other sensor data
    if (pass_enter && passcount == 9) {
        // Do something when 9 digits are entered
    }

    // Perform actions based on received commands
    char receivedCommand = 'A'; // Replace with your logic to receive commands
    switch (receivedCommand) {
        case '0':
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
        case '*':
        case '#':
            if (pass_enter) {
                pass[passcount++] = receivedCommand;
            } else {
                // Access denied
            }
            break;
        case 'A':
            if (admin_authorization) {
                // Terminate or perform other actions
            }
            break;
        case 'B':
            if (admin_authorization) {
                // Trigger alarm or make a call
            }
            break;
        case 'C':
            admin_authorization = false;
            pass_enter = false;
            memset(buff, 0, sizeof(buff)); // Clear buffer
            memset(pass, 0, sizeof(pass)); // Clear password
            passcount = 0; // Reset count
            break;
        case 'D':
            // Process RFID data
            break;
        default:
            break;
    }

    // Authenticate based on entered password
    if (strcmp(pass, "123456789") == 0 && !admin_authorization) {
        admin_authorization = true;
        USART_SendString("Authenticated\n");
        // Clear LCD, print authentication message, etc.
        pass_enter = false;
    } else if (pass_enter && passcount == 9) {
        // Do something if wrong password entered
    }

    // Update LCD display, LED indicators, etc.
    if (flame || gas) {
        // Update LCD and LEDs for danger indication
    } else {
        // Update LCD for normal operation
    }

    _delay_ms(200);
}

int main(void) {
    init();
    while (1) {
        loop();
    }
    return 0;
}
#define F_CPU 16000000UL  // Define CPU clock speed
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>

#define IR_RECEIVE_PIN 6
#define RX_PIN 4
#define TX_PIN 5
#define RST_PIN 1
#define SS_PIN 2
#define FLAME_PIN 2
#define GAS_PIN 3
#define BUZZER_PIN 0
#define RED_LED_PIN 1
#define GREEN_LED_PIN 2

volatile char buff[32];
volatile char pass[10];
volatile int passcount = 0;
volatile bool admin_authorization = false;
volatile bool pass_enter = false;
volatile bool flame = false;
volatile bool gas = false;

void USART_Init(unsigned int baud) {
    UBRRH = (unsigned char)(baud>>8);
    UBRRL = (unsigned char)baud;
    UCSRB = (1<<RXEN)|(1<<TXEN);
    UCSRC = (1<<URSEL)|(3<<UCSZ0);
}

void USART_Transmit(unsigned char data) {
    while (!(UCSRA & (1<<UDRE)));
    UDR = data;
}

unsigned char USART_Receive(void) {
    while (!(UCSRA & (1<<RXC)));
    return UDR;
}

void USART_SendString(char* str) {
    while (*str) {
        USART_Transmit(*str++);
    }
}

void init() {
    DDRB |= (1 << SS_PIN) | (1 << RST_PIN) | (1 << BUZZER_PIN) | (1 << RED_LED_PIN) | (1 << GREEN_LED_PIN);
    DDRD &= ~((1 << FLAME_PIN) | (1 << GAS_PIN));
    PORTD |= (1 << FLAME_PIN) | (1 << GAS_PIN);

    USART_Init(9600);
}

void FlameDetected() {
    // Implement flame detection logic here
    flame = true;
}

void gasDetected() {
    // Implement gas detection logic here
    gas = true;
}

void loop() {
    // IR receiving logic or any other input receiving mechanism goes here
    // For example, if using interrupt-driven UART receiving:
    if (UCSRA & (1 << RXC)) {
        char receivedData = UDR;
        // Process the received data
    }

    // Process IR received data or any other sensor data
    if (pass_enter && passcount == 9) {
        // Do something when 9 digits are entered
    }

    // Perform actions based on received commands
    char receivedCommand = 'A'; // Replace with your logic to receive commands
    switch (receivedCommand) {
        case '0':
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
        case '*':
        case '#':
            if (pass_enter) {
                pass[passcount++] = receivedCommand;
            } else {
                // Access denied
            }
            break;
        case 'A':
            if (admin_authorization) {
                // Terminate or perform other actions
            }
            break;
        case 'B':
            if (admin_authorization) {
                // Trigger alarm or make a call
            }
            break;
        case 'C':
            admin_authorization = false;
            pass_enter = false;
            memset(buff, 0, sizeof(buff)); // Clear buffer
            memset(pass, 0, sizeof(pass)); // Clear password
            passcount = 0; // Reset count
            break;
        case 'D':
            // Process RFID data
            break;
        default:
            break;
    }

    // Authenticate based on entered password
    if (strcmp(pass, "123456789") == 0 && !admin_authorization) {
        admin_authorization = true;
        USART_SendString("Authenticated\n");
        // Clear LCD, print authentication message, etc.
        pass_enter = false;
    } else if (pass_enter && passcount == 9) {
        // Do something if wrong password entered
    }

    // Update LCD display, LED indicators, etc.
    if (flame || gas) {
        // Update LCD and LEDs for danger indication
    } else {
        // Update LCD for normal operation
    }

    _delay_ms(200);
}

int main(void) {
    init();
    while (1) {
        loop();
    }
    return 0;
}
