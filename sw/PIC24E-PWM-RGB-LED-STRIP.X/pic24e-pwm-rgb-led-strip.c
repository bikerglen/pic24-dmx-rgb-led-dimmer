#include </Applications/microchip/xc16/v1.25/support/generic/h/xc.h>
#include </Applications/microchip/xc16/v1.25/support/PIC24E/h/p24EP128MC202.h>
#include "DEE Emulation 16-bit.h"
#include "sine.h"

#define FOSC 80000000
#define FCY  40000000

#define U1BAUDRATE 250000
#define U1BRGVAL (((FCY/U1BAUDRATE)/16) - 1)

// FICD
#pragma config ICS = PGD3               // ICD Communication Channel Select bits (Communicate on PGEC3 and PGED3)
#pragma config JTAGEN = OFF             // JTAG Enable bit (JTAG is disabled)

// FOSCSEL
#pragma config FNOSC    = FRC
#pragma config PWMLOCK  = OFF
#pragma config IESO     = OFF

// FOSC
#pragma config POSCMD   = EC
#pragma config OSCIOFNC = OFF
#pragma config IOL1WAY  = OFF
#pragma config FCKSM    = CSECMD

// FWDT
#pragma config PLLKEN   = ON
#pragma config WINDIS   = OFF
#pragma config FWDTEN   = OFF

void ConfigureFixture (void);
void RunDMX512 (void);
void RunSineEffect (void);
void RunStrobeEffect (void);
void RunLampTest (void);
unsigned char CheckButton (void);


unsigned char startup_timer;
unsigned char orange_led_timer;

unsigned short base_address;
unsigned short color_mode;

unsigned char rx_state;
unsigned short rx_addr;
unsigned char rx_level;
unsigned char rx_byte;
unsigned char rx_nbytes;
unsigned char rx_data[6];

unsigned char red, green, blue;

unsigned char button_state;
unsigned char mode;

static const unsigned short PMapLut[256] = {
       0,    63,  127,  191,  191,  255,  255,  319,  319,  383,  447,  447,  511,  511,  575,  639,
      703,  703,  767,  831,  895,  895,  959, 1023, 1087, 1151, 1151, 1215, 1279, 1343, 1407, 1471, 
     1535, 1599, 1663, 1727, 1791, 1855, 1919, 1983, 2047, 2111, 2239, 2303, 2367, 2431, 2495, 2623, 
     2687, 2751, 2815, 2943, 3007, 3135, 3199, 3263, 3391, 3455, 3583, 3647, 3775, 3839, 3967, 4031, 
     4159, 4287, 4351, 4479, 4607, 4735, 4799, 4927, 5055, 5183, 5311, 5439, 5567, 5631, 5759, 5887,
     6015, 6207, 6335, 6463, 6591, 6719, 6847, 6975, 7167, 7295, 7423, 7615, 7743, 7871, 8063, 8191,
     8383, 8511, 8703, 8831, 9023, 9151, 9343, 9535, 9727, 9855,10047,10239,10431,10623,10751,10943,
    11135,11327,11519,11775,11967,12159,12351,12543,12735,12991,13183,13375,13631,13823,14079,14271,
    14527,14719,14975,15167,15423,15679,15871,16127,16383,16639,16895,17151,17407,17663,17919,18175,
    18431,18687,18943,19199,19519,19775,20031,20351,20607,20863,21183,21503,21759,22079,22335,22655,
    22975,23295,23551,23871,24191,24511,24831,25151,25471,25791,26175,26495,26815,27135,27519,27839,
    28223,28543,28927,29247,29631,29951,30335,30719,31103,31423,31807,32191,32575,32959,33343,33727,
    34175,34559,34943,35327,35775,36159,36607,36991,37439,37823,38271,38719,39103,39551,39999,40447,
    40895,41343,41791,42239,42687,43135,43647,44095,44543,45055,45503,46015,46463,46975,47423,47935,
    48447,48959,49471,49983,50495,51007,51519,52031,52543,53055,53631,54143,54655,55231,55743,56319,
    56895,57407,57983,58559,59135,59711,60287,60863,61439,62015,62591,63167,63807,64383,65023,65535
};


int main() 
{
    // Configure PLL prescaler, PLL postscaler, PLL divisor
    // with 10MHz external clock
    // Fin   = 10MHz
    // Fplli = Fin/N1  = 10/2  = 5MHz     0.8 < Fplli < 8.0
    // Fsys  = Fplli*M = 5*32  = 160MHz   120 < Fsys  < 340
    // Fosc  = Fsys/N2 = 160/2 = 80MHz    15  < Fosc  < 120
    // Fcy   = Fosc/2  = 80/2  = 40MHz
    PLLFBD             = 30; // PLLFBD  = M-2  = 32-2 = 30
    CLKDIVbits.PLLPOST =  0; // N2 = 2 => PLLPOST = 0 
    CLKDIVbits.PLLPRE  =  0; // N1 = 2 => PLLPRE  = 0
    
    // Initiate Clock Switch to Primary Oscillator with PLL (NOSC=0b011)
    __builtin_write_OSCCONH(0x03);
    __builtin_write_OSCCONL(OSCCON | 0x01);

    // Wait for Clock switch to occur
    while (OSCCONbits.COSC!= 0b011);
    
    //  Wait for PLL to lock
    while (OSCCONbits.LOCK!= 1);    
 
 	// unlock peripheral pin select and leave it unlocked
	__builtin_write_OSCCONL (OSCCON & ~(1<<6));

    // disable analog inputs
    ANSELA = 0x0000;
    ANSELB = 0x0000;
    
    // set unused pins to outputs and low
    LATA = 0x0000;
    TRISAbits.TRISA0 = 1;   // OPTOISOLATOR 
    TRISAbits.TRISA1 = 0;   // N/C
    TRISAbits.TRISA2 = 1;   // CLKI
    TRISAbits.TRISA3 = 0;   // N/C
    TRISAbits.TRISA4 = 1;   // BUTTON
    LATB = 0x0000;
    TRISBbits.TRISB0 = 0;   // N/C
    TRISBbits.TRISB1 = 0;   // N/C
    TRISBbits.TRISB2 = 0;   // N/C
    TRISBbits.TRISB3 = 1;   // RXD
    TRISBbits.TRISB4 = 0;   // DIR (0=RX, 1=TX)
    TRISBbits.TRISB5 = 0;   // TXD
    TRISBbits.TRISB6 = 0;   // LED (0=ON, 1=OFF)
    TRISBbits.TRISB7 = 0;   // N/C
    TRISBbits.TRISB8 = 0;   // N/C
    TRISBbits.TRISB9 = 0;   // N/C
    TRISBbits.TRISB10 = 0;  // PWM3 - BLUE
    TRISBbits.TRISB11 = 0;  // N/C
    TRISBbits.TRISB12 = 0;  // PWM2 - GREEN
    TRISBbits.TRISB13 = 0;  // N/C
    TRISBbits.TRISB14 = 0;  // PWM1 - RED
    TRISBbits.TRISB15 = 0;  // N/C
            
    // initialize LED
    TRISBbits.TRISB6 = 0;
    LATBbits.LATB6 = 1;
    
    // initialize UART1
    TRISBbits.TRISB3 = 1;           // RP35/RB3 UART1 RxD is an input
    TRISBbits.TRISB4 = 0;           // RP36/RB4 UART1 DIR is an output
    TRISBbits.TRISB5 = 0;           // RP37/RB5 UART1 TxD is an output
    RPINR18bits.U1RXR = 35;         // connect RP35 to U1RXD
    LATBbits.LATB4 = 0;             // set RB4 lo for DIR = RX
    RPOR1bits.RP37R = 1;            // connect U1TXD to RP37
    U1MODE = 0b1000000000000000;    // UARTEN
	U1STA  = 0b0000010000000000;    // UTXEN
	U1BRG = U1BRGVAL;
    U1MODEbits.LPBACK = 0;
    
    // initialize PWM
    PTCONbits.EIPU = 1;
    PTCON2bits.PCLKDIV = 0b000;
    PTPER = 65535;

    PHASE1 = PHASE2 = PHASE3 = 0;
    PDC1 = PDC2 = PDC3 = 0x100;
    DTR1 = DTR2 = DTR3 = 0;
    ALTDTR1 = ALTDTR2 = ALTDTR3 = 0;
    IOCON1 = IOCON2 = IOCON3 = 0xC000;
    PWMCON1 = PWMCON2 = PWMCON3 = 0x0000;
    FCLCON1 = FCLCON2 = FCLCON3 = 0x0003;
    
    PTCONbits.PTEN = 1;
    
    // initialize timer 1 for 50Hz / 20ms period
    // Fcy / prescale / period = 40MHz / 256 / 3125 = 50Hz / 20ms
    _T1IF = 0;
    _T1IE = 0;
    TMR1 = 0x0000;
    PR1 = 3125;
    T1CONbits.TCKPS = 3;
    T1CONbits.TON = 1;

    // initialize orange led timer
    orange_led_timer = 0;
    
    // initialize button state
    button_state = 0;
    
    // initialize data eeprom emulation
    DataEEInit ();
    
	// check if button held down during power up
	if (!PORTAbits.RA4) {
		// must continue to be low for 50 times in the next 1 second
		startup_timer = 50;
		do {
			if (PORTAbits.RA4) {
                startup_timer = 255;
				break;
			}
			while (!_T1IF) {
			}
            _T1IF = 0;
		} while (--startup_timer);

		if (startup_timer == 0) {
			// get new light number from dmx address 1 and store in eeprom
			ConfigureFixture ();
		}
	}
    
    // read fixture configuration from flash
    base_address = DataEERead (0);
    color_mode = DataEERead (2);

    // check for any special modes
	if (base_address == 513) {
		mode = 1;
	} else if (base_address == 514) {
        mode = 2;
	} else if (base_address == 515) {
        mode = 3;
    } else {
        mode = 0;
    }

    // default to address 1 if not configured or invalid address
    if ((base_address < 1) || (base_address > 512)) {
        base_address = 1;
    }
    
    // default to 8-bit perceptual LUT mode if not configured or invalid mode
    if ((color_mode != 0) && (color_mode != 1)) {
        color_mode = 0;
    }
    
    while (1) {
        orange_led_timer = 0;
        
        if (mode == 0) {
            // runs regardless of RA0 state
            RunDMX512 ();
        } else if (mode == 1) {
            // runs regardless of RA0 state
            RunSineEffect ();
        } else if (mode == 2) {
            // requires RA0 to be pulled low to run effect
    		RunStrobeEffect ();
        } else if (mode == 3) {
            // runs regardless of RA0 state
            RunLampTest ();
        }
        mode++;
        if (mode > 3) {
            mode = 0;
        }
    }
            
    return 0;
}


void ConfigureFixture (void)
{
    rx_level = 0;
    rx_addr = 0;
    rx_state = 0;
    
    while (1) {
 
        // clear any over runs
        if (U1STAbits.OERR) {
            U1STAbits.OERR = 0;
        }
        
        // get fixture configuration from DMX data stream
        // if a framing error is detected, assume that is a DMX break
        if (U1STAbits.URXDA) {
            if (U1STAbits.FERR) {
                rx_level = U1RXREG;
                rx_addr = 0;
                rx_state = 1;
            } else {
                rx_level = U1RXREG;
                if (rx_state == 1) {
                    if (rx_addr <= 512) {
                        if ((rx_addr >= 1) && (rx_addr <= 6)) {
                            rx_data[rx_addr-1] = rx_level;
                        } 
                        if (rx_addr == 6) {
                            if ((rx_data[0] + rx_data[3]) == 0xFF) {
                                if ((rx_data[1] + rx_data[4]) == 0xFF) {
                                    if ((rx_data[2] + rx_data[5]) == 0xFF) {
                                        base_address = (rx_data[0] << 8) | rx_data[1];
                                        color_mode = rx_data[2];
                                        if ((base_address >= 1) && (base_address <= 514)) {
                                            if ((color_mode >=0) && (color_mode <= 1)) {
                                                // complement of first three bytes matches complement of last three bytes
                                                // address is between 1 and 512 and mode is 0 or 1
                                                // or special modes 513 or 514 were selected
                                                // program ee with new fixture base address and color mode
                                                // e.g. ./senddmx 192.168.180.83 0x1936 0 7 1 255 248 254, address = 7, color mode = 1
                                                break;
                                            }
                                        }
                                    }
                                }
                            }
                            rx_state = 0;
                        }
                    }
                    rx_addr++;
                }
            }
        }

        // do periodic tasks when timer expires
        if (_T1IF == 1) {
            _T1IF = 0;
            
            // quickly flash orange LED with delay between flashes
            orange_led_timer++;
            if (orange_led_timer == 3) {
                LATBbits.LATB6 = 1;
            } else if (orange_led_timer == 30) {
                LATBbits.LATB6 = 0;
                orange_led_timer = 0;
            }
        }
    }
    
    // store fixture configuration in flash
    DataEEWrite (base_address, 0);
    DataEEWrite (color_mode, 2);
    
    // blink red channel LEDs five times to indicate programming complete
    TMR1 = 0x0000;
    _T1IF = 0;
    for (startup_timer = 0; startup_timer < 5; startup_timer++) {
        PDC1 = 65535;
        PDC2 = 0;
        PDC3 = 0;
        orange_led_timer = 10;
        while (orange_led_timer) {
            while (!_T1IF) {
            }
            _T1IF = 0;
            orange_led_timer--;
        }
        PDC1 = 0;
        PDC2 = 0;
        PDC3 = 0;
        orange_led_timer = 10;
        while (orange_led_timer) {
            while (!_T1IF) {
            }
            _T1IF = 0;
            orange_led_timer--;
        }
    }
    
    // prevent jumps if button still held down
    if (!PORTAbits.RA4) {
        button_state = 2;
    }
}


void RunDMX512 (void)
{
    rx_state = 0;
    rx_addr = 0;
    rx_level = 0;
    rx_byte = 0;
    rx_nbytes = color_mode ? 6 : 3;
    
    while (1) {
        
        // clear any over runs
        if (U1STAbits.OERR) {
            U1STAbits.OERR = 0;
        }
        
        // if a framing error is detected, assume that is a DMX break
        if (U1STAbits.URXDA) {
            if (U1STAbits.FERR) {
                rx_level = U1RXREG;
                rx_addr = 0;
                rx_byte = 0;
                rx_state = 1;
            } else {
                rx_level = U1RXREG;
                if (rx_state == 1) {
                    if (rx_addr >= base_address) {
                        rx_data[rx_byte++] = rx_level;
                        if (rx_byte == rx_nbytes) {
                            if (color_mode == 0) {
                                PDC1 = PMapLut[rx_data[0]];
                                PDC2 = PMapLut[rx_data[1]];
                                PDC3 = PMapLut[rx_data[2]];
                            } else if (color_mode == 1) {
                                PDC1 = (rx_data[0] << 8) | rx_data[1];
                                PDC2 = (rx_data[2] << 8) | rx_data[3];
                                PDC3 = (rx_data[4] << 8) | rx_data[5];
                            }
                            rx_state = 0;
                        }
                    }
                }
                if (rx_addr <= 512) {
                    rx_addr++;
                }
            }
        }

        // do periodic tasks when timer expires
        if (_T1IF == 1) {
            _T1IF = 0;
            
            // blink orange LED at 2.5Hz
            orange_led_timer++;
            if (orange_led_timer == 10) {
                orange_led_timer = 0;
                LATBbits.LATB6 ^= 1;
            }
            if (CheckButton ()) {
                return;
            }
        }
    }
   
}


void RunSineEffect (void)
{
	sine_Init ();

	while (1) {
        // do periodic tasks when timer expires
        if (_T1IF == 1) {
            _T1IF = 0;
            
			// update levels
			PDC1 = PMapLut[red];
			PDC2 = PMapLut[green];
			PDC3 = PMapLut[blue];

			// compute next step in sequence
			sine_Tick ();

            // blink orange LED at 2.5Hz
            orange_led_timer++;
            if (orange_led_timer == 10) {
                orange_led_timer = 0;
                LATBbits.LATB6 ^= 1;
            }
            if (CheckButton ()) {
                return;
            }
        }
	}
}


void RunStrobeEffect (void)
{
    char strobeTimer = -1;
    PDC1 = PDC2 = PDC3 = PMapLut[0];

	while (1) {
        // do periodic tasks when timer expires
        if (_T1IF == 1) {
            _T1IF = 0;
            
            // if idle and opto triggered, immediately turn on LEDs
            if ((strobeTimer == -1) && !PORTAbits.RA0) {
                strobeTimer = 0;
            }
            
            // strobe LEDs, on for 1/50s @ 10Hz
            switch (strobeTimer) {
                case 0:
                    PDC1 = PDC2 = PDC3 = PMapLut[255];
                    break;
                case 1:
                    PDC1 = PDC2 = PDC3 = PMapLut[128];
                    break;
                case 2:
                    PDC1 = PDC2 = PDC3 = PMapLut[64];
                    break;
                case 3:
                    PDC1 = PDC2 = PDC3 = PMapLut[0];
                    break;
            }
            
            // if opto no longer lit, turn off LEDs on next entrance
            if (PORTAbits.RA0) {
                strobeTimer = -1;
                PDC1 = PDC2 = PDC3 = PMapLut[0];
            } else {
                strobeTimer++;
                if (strobeTimer == 7) {
                    strobeTimer = 0;
                }
            }
               
            // blink orange LED at 2.5Hz
            orange_led_timer++;
            if (orange_led_timer == 10) {
                orange_led_timer = 0;
                LATBbits.LATB6 ^= 1;
            }
            if (CheckButton ()) {
                return;
            }
        }
	}
}


void RunLampTest (void)
{
    while (1) {
        orange_led_timer = 50;
        PDC1 = 65535;
        PDC2 = 0;
        PDC3 = 0;
        while (orange_led_timer) {
            while (!_T1IF) {
            }
            _T1IF = 0;
            orange_led_timer--;
            if (CheckButton ()) {
                return;
            }
        }

        orange_led_timer = 50;
        PDC1 = 0;
        PDC2 = 65535;
        PDC3 = 0;
        while (orange_led_timer) {
            while (!_T1IF) {
            }
            _T1IF = 0;
            orange_led_timer--;
            if (CheckButton ()) {
                return;
            }
        }

        orange_led_timer = 50;
        PDC1 = 0;
        PDC2 = 0;
        PDC3 = 65535;
        while (orange_led_timer) {
            while (!_T1IF) {
            }
            _T1IF = 0;
            orange_led_timer--;
            if (CheckButton ()) {
                return;
            }
        }
    }
}


unsigned char CheckButton (void)
{
    switch (button_state) {
        case 0: // if button down, advance to state 1
            if (!PORTAbits.RA4) {
                button_state = 1;
            }
            break;
        case 1: // if button still down, count that as a press and advance to state 2
            if (!PORTAbits.RA4) {
                button_state = 2;
                return 1;
            } else {
                button_state = 0;
            }
            break;
        case 2: // if button up, advance to state 3
            if (PORTAbits.RA4) {
                button_state = 3;
            }
            break;
        case 3: // if button still up, reset state machine and look for next press
            if (PORTAbits.RA4) {
                button_state = 0;
            } else {
                button_state = 2;
            }
            break;
        default:
            button_state = 0;
            break;
    }
    
    return 0;
}