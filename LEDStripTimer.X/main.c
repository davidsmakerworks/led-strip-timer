/*
 * File:   main.c
 * Author: David Rice
 *
 * Created on October 11, 2016, 5:31 PM
 * 
 * Controls a string of WS281x LEDs with a Bluetooth (UART) interface to act as a timer.
 * 
 * Supports 60 LEDs.
 * 
 * Supports WS2811 in fast mode only.
 * 
 * Version 2.2 - supports CLC with blocking SPI - corrected GIE status issue and Timer0 ISR handler
 * Added four-minute timer mode for practice bouts
 * 
 */

// PIC16F18325 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FEXTOSC = OFF    // FEXTOSC External Oscillator mode Selection bits (Oscillator not enabled)
#pragma config RSTOSC = HFINT32 // Power-up default value for COSC bits (HFINTOSC with 2x PLL (32MHz))
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; I/O or oscillator function on OSC2)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config MCLRE = ON       // Master Clear Enable bit (MCLR/VPP pin function is MCLR; Weak pull-up enabled)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
#pragma config WDTE = OFF       // Watchdog Timer Enable bits (WDT disabled; SWDTEN is ignored)
#pragma config LPBOREN = OFF    // Low-power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bits (Brown-out Reset enabled, SBOREN bit ignored)
#pragma config BORV = LOW       // Brown-out Reset Voltage selection bit (Brown-out voltage (Vbor) set to 2.45V)
#pragma config PPS1WAY = ON     // PPSLOCK bit One-Way Set Enable bit (The PPSLOCK bit can be cleared and set only once; PPS registers remain locked after one clear/set cycle)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a Reset)
#pragma config DEBUG = OFF      // Debugger enable bit (Background debugger disabled)

// CONFIG3
#pragma config WRT = OFF        // User NVM self-write protection bits (Write protection off)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low Voltage programming enabled. MCLR/VPP pin function is MCLR. MCLRE configuration bit is ignored.)

// CONFIG4
#pragma config CP = OFF         // User NVM Program Memory Code Protection bit (User NVM code protection disabled)
#pragma config CPD = OFF        // Data NVM Memory Code Protection bit (Data NVM code protection disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>
#include <stdlib.h>

//#define WS2811
#define WS2812B

/* Frequency must be specified for delay loops */
#define _XTAL_FREQ 32000000

/* 
 * Baud Rate Generator value for 9600 baud at Fosc = 32MHz
 * 9600 baud: ((32000000 / 9600) / 64) - 1
 */
#define BAUD_RATE 51 

/* Define output pin for WS281x data stream */
#define LED_DATA_OUT LATAbits.LATA5

/* Define TRIS register for Bluetooth (UART) data */
#define UART_RX_TRIS TRISCbits.TRISC5

/* Maximum LED index supported by this driver (i.e., number of LEDs - 1) */
#define MAX_SUPPORTED_LED_INDEX 59

/* LEDs support 8 bits per color channel */
#define NUM_COLOR_BITS 8

/* Bit number (i.e., MSB first) that will be shifted out to WS2812B */
#define ACTIVE_BIT 7

/* Define timer modes */
#define MODE_STOPPED 0
#define MODE_RUNNING 1
#define MODE_STOPPING 2
#define MODE_TURNING_OFF 3

/* Define display stages */
#define STAGE_PRESTART 0
#define STAGE_RUNNING 1
#define STAGE_PRESTOP 2

/* Define Boolean values */
#define FALSE 0
#define TRUE !FALSE

/* 
 * Macros for putting color values at the appropriate index
 * Note that the WS2812B uses GRB format instead of RGB
 */
#ifdef WS2811
#define RED(x)   (x * 3)
#define GREEN(x) (x * 3) + 1
#define BLUE(x)  (x * 3) + 2
#else
#ifdef WS2812B
#define RED(x)   (x * 3) + 1
#define GREEN(x) (x * 3)
#define BLUE(x)  (x * 3) + 2
#else
#error LED controller type must be specified
#endif
#endif

uint8_t color_data[(MAX_SUPPORTED_LED_INDEX + 1) * 3];
uint16_t max_led_index = MAX_SUPPORTED_LED_INDEX;

uint8_t mode = MODE_RUNNING;

/* Data received from Bluetooth module via USART */
volatile uint8_t serial_data;

/* Flag to indicate if Bluetooth data was received in ISR */
volatile uint8_t serial_pending = FALSE;

/* System tick counter for timekeeping */
volatile uint8_t ticks = 0;

/* ISR processes received data from Bluetooth (EUSART1) and increments system tick */
void __interrupt() isr(void) {
    if (INTCONbits.PEIE) {
        if (PIE1bits.RCIE && PIR1bits.RCIF) {
            serial_data = RC1REG;
            serial_pending = TRUE; /* Set flag so data can be processed in main loop */
            
            PIR1bits.RCIF = 0;
        }    
    }
    
    if (PIE0bits.TMR0IE && PIR0bits.TMR0IF) {
        ticks++;
            
        PIR0bits.TMR0IF = 0;
    }
}

/* 
 * Standard port initialization
 * Later functions will change some of these settings 
 */
void init_ports(void) {
    /* Disable all analog features */
    ANSELA = 0x00;
    ANSELC = 0x00;
    
    /* Set all ports to output */
    TRISA = 0x00;
    TRISC = 0x00;
    
    /* Pull all outputs low */
    LATA = 0x00;
    LATC = 0x00;
    
    /* Set TTL on PC5 due to 3.3V output from Bluetooth module */
    INLVLCbits.INLVLC5 = 0;
}

/* Initialize EUSART1 hardware registers */
void init_uart(void) {
    UART_RX_TRIS = 1; /* Set RX pin as input */
    
    RC1STAbits.CREN = 1; /* Continuous receive */
    TX1STAbits.SYNC = 0; /* Asynchronous */
    SP1BRG = BAUD_RATE; /* Baud rate as defined above */
    RC1STAbits.SPEN = 1; /* Enable serial port */
}

void init_timers(void) {
    TMR0H = 61; /* Timer0 period of 61 in 8-bit mode provides about 8 interrupts per second */
    
    T0CON1bits.T0CS = 0b010; /* Timer0 clock source is Fosc/4 (i.e., 8 MHz at Fosc = 32 MHz) */
    T0CON1bits.T0CKPS = 0b1110; /* Timer0 prescaler value 16384 */
    
    T0CON0bits.T0EN = 1; /* Enable Timer0 */
    
    PR2 = 4; /* 0.625 uSec at Fosc = 32 MHz */
    T2CONbits.TMR2ON = 1; /* Enable Timer2 */
}

void init_spi(void) {
    /* SPI used only to drive CLC so no outputs defined */
    
    SSP1CON1bits.SSPM = 0b0011; /* Set SPI mode with CLK = T2_match/2 */
    SSP1CON1bits.SSPEN = 1; /* Enable MSSP */
}

void init_pwm(void) {
    PWM5DCH = 1;
    PWM5DCL = 0;
    
    PWM5CONbits.PWM5EN = 1; /* Enable PWM generator */
}

void init_clc(void) {
    RA5PPS = 0b00100; /* CLC1OUT on RA5 */
    
    CLC1SEL0bits.LC1D1S = 0b10011; /* CLC1 input 1 is SDO1 */
    CLC1SEL1bits.LC1D2S = 0b10010; /* CLC1 input 2 is SCK1 */
    CLC1SEL2bits.LC1D3S = 0b10000; /* CLC1 input 3 is PWM5OUT */
    
    CLC1GLS0 = 0x00;
    CLC1GLS1 = 0x00;
    CLC1GLS2 = 0x00;
    CLC1GLS3 = 0x00; /* Gate behavior is undefined at power-on so must be set to zero */
    
    CLC1GLS0bits.LC1G1D1T = 1; /* SDO input to AND gate 1 */
    
    CLC1GLS1bits.LC1G2D2T = 1; /* SCK input to AND gate 1 */
    
    /* nSDO && SCK = n(SDO || nSCK) */
    CLC1GLS2bits.LC1G3D1T = 1;
    CLC1GLS2bits.LC1G3D2N = 1; /* SDO || nSCK input to AND gate 2 */
    
    CLC1GLS3bits.LC1G4D3T = 1; /* PWM5OUT input to AND gate 2 */
    
    CLC1POL = 0x00; /* Clear all inversion bits */
    CLC1POLbits.LC1G3POL = 1; /* Gate 3 n(SDO || nSCK) is inverted to obtain (nSDO && SCK) */
    
    CLC1CONbits.LC1EN = 1; /* Enable CLC1 */
}

void transfer_led_byte(uint8_t data) {
    SSP1BUF = data;

    while (!SSP1STATbits.BF); /* Spin until transfer is complete */
}

/* Sets all LEDs to a given RGB value */
void set_all(uint8_t red, uint8_t green, uint8_t blue) {
    uint16_t cur_led;
    
    for (cur_led = 0; cur_led <= max_led_index; cur_led++) {
        color_data[RED(cur_led)] = red;
        color_data[GREEN(cur_led)] = green;
        color_data[BLUE(cur_led)] = blue;
    }
}

void set_one(uint16_t index, uint8_t red, uint8_t green, uint8_t blue) {
    color_data[RED(index)] = red;
    color_data[GREEN(index)] = green;
    color_data[BLUE(index)] = blue;
}

void set_timer_display(uint16_t num_off, uint8_t display_stage) {
    uint16_t max_index;
    
    uint8_t red;
    uint8_t green;
    uint8_t blue;
    
    max_index = max_led_index - num_off;
    
    set_all(0x00, 0x00, 0x00);
    
    switch (display_stage) {
        case STAGE_RUNNING: 
            set_one(num_off, 0xFF, 0xFF, 0x00);
            set_one(max_index, 0xFF, 0xFF, 0x00);

            for (uint16_t i = num_off + 1; i < max_index; i++) {
                set_one(i, 0x00, 0xFF, 0x00); 
            }
            break;
        case STAGE_PRESTART:            
            set_one(num_off, 0x00, 0x00, 0xFF);
            set_one(max_index, 0x00, 0x00, 0xFF);
            break;
        case STAGE_PRESTOP:    
            for (uint16_t i = num_off; i <= max_index; i++) {
                set_one(i, 0xFF, 0x00, 0x00); 
            }
            break;      
    }
}

/* 
 * Sends all data to WS281x LEDs
 * This is a time-critical function so interrupts are disabled during data transmission
 * Interrupts could be disabled for up to 10 ms when using all 300 LEDs
 */
void send_data(void) {
    uint16_t cur_byte_num;
    uint16_t num_bytes;
    uint8_t state;
    
    /* Disable interrupts during time-critical section */
    state = INTCONbits.GIE;
    INTCONbits.GIE = 0;
    
    num_bytes = ((uint16_t)max_led_index + 1) * 3;
    
    for (cur_byte_num = 0; cur_byte_num < num_bytes; cur_byte_num++) {
        transfer_led_byte(color_data[cur_byte_num]); 
    }
    
    /* Restore previous interrupt state */
    INTCONbits.GIE = state;
}

void pre_start() {
    uint16_t pre_off;
    
    for (pre_off = 30; pre_off > 0; pre_off -= 1) {
        set_timer_display(pre_off, STAGE_PRESTART);
        send_data();
        __delay_ms(30);
    }
}

void pre_stop() {
    uint16_t pre_off;
    
    for (pre_off = 30; pre_off > 0; pre_off -= 1) {
        set_timer_display(pre_off, STAGE_PRESTOP);
        send_data();
        __delay_ms(35);
    }
}

uint8_t get_ticks(void) {
    uint8_t current_ticks;
    
    PIE0bits.TMR0IE = 0;
    current_ticks = ticks;
    PIE0bits.TMR0IE = 1;
    
    return current_ticks;
}

void main(void) {
    uint8_t start_ticks;
    uint16_t off;
    uint8_t length;
    
    max_led_index = MAX_SUPPORTED_LED_INDEX;
    
    length = 16; /* 1 minute */
    
    init_ports(); /* Initialize I/O ports */
    init_spi();
    init_timers();
    init_pwm();
    init_clc();

    mode = MODE_STOPPED;
    off = 0;
    set_all(0x00, 0x00, 0x00); 
    send_data(); /* Update display */
    
    /* Begin LED test sequence */
    set_all(0xFF, 0x00, 0x00);
    send_data();
    __delay_ms(1000);
    
    set_all(0x00, 0xFF, 0x00);
    send_data();
    __delay_ms(1000);
    
    set_all(0x00, 0x00, 0xFF);
    send_data();
    __delay_ms(1000);
    
    set_all(0xFF, 0xFF, 0xFF);
    send_data();
    __delay_ms(1000);
    
    set_all(0x00, 0x00, 0x00);
    send_data();
    /* End LED test sequence */
    
    init_uart(); /* Initialize EUSART1 hardware */
    
    PIE1bits.RCIE = 1; /* Enable EUSART1 receive interrupt */
    PIE0bits.TMR0IE = 1; /* Enable Timer0 overflow interrupt */
    INTCONbits.PEIE = 1; /* Enable peripheral interrupts */
    INTCONbits.GIE = 1; /* Enable global interrupts */
    
    while(1) {
        if (serial_pending) {
            switch (serial_data) {
                case 'G':
                    pre_start();
                    off = 0;
                    set_timer_display(off, STAGE_RUNNING);
                    send_data();
                    start_ticks = get_ticks();
                    mode = MODE_RUNNING;
                    break;
                case 'S':
                    mode = MODE_STOPPING;
                    break;
                case '0':
                    mode = MODE_TURNING_OFF;
                    break;
                case '1':
                    length = 4; /* 15 seconds */
                    break;
                case '2':
                    length = 8; /* 30 seconds */
                    break;
                case '3':
                    length = 12; /* 45 seconds */
                    break;
                case '4':
                    length = 16; /* 1 minute */
                    break;
                case '5':
                    length = 24; /* 90 seconds */
                    break;
                case '6':
                    length = 32; /* 2 minutes */
                    break;
                case '7':
                    length = 48; /* 3 minutes */
                    break;
                case '8':
                    length = 64; /* 4 minutes */
                    break;
            }
            serial_pending = FALSE;
        }

       switch (mode) {
            case MODE_RUNNING:
                if ((uint8_t)(get_ticks() - start_ticks) >= length) {
                    off++;
                    
                    if (off > (max_led_index / 2)) {
                        mode = MODE_STOPPING;
                    } else {
                    
                    set_timer_display(off, STAGE_RUNNING);
                    send_data();
                    
                    start_ticks = get_ticks();
                    }
                }
                break;
            case MODE_STOPPING:
                set_all(0x00, 0x00, 0x00);
                pre_stop();
                set_all(0xFF, 0x00, 0x00);
                send_data();
                mode = MODE_STOPPED;
                break;
            case MODE_TURNING_OFF:
                set_all(0x00, 0x00, 0x00);
                send_data();
                mode = MODE_STOPPED;
            case MODE_STOPPED:
                break;    
        }
    }
}
