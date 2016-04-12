/*
 * Lab5.c
 *
 * Created: 4/8/2016 4:46:47 PM
 * Author : Robin and Weston
 
 LCD routines adapted from http://web.alfredstate.edu/weimandn/programming/lcd/ATmega328/LCD_code_gcc_4d.html
 ADC routines adapted from http://extremeelectronics.co.in/avr-tutorials/using-adc-of-avr-microcontroller/
 http://homepage.hispeed.ch/peterfleury/avr-software.html
 http://homepage.hispeed.ch/peterfleury/doxygen/avr-gcc-libraries/group__pfleury__ic2master.html
 Frequency routines from Atmel documentation AVR205
 i2c functions source: http://www.procyonengineering.com/embedded/avr/avrlib/
 */ 
/************************************************************************

                -----------                   ----------
               | ATmega328 |                 |   LCD    |
               |           |                 |          |
          -----|   ADC5/PC5|				 |			|
		 |	   |		PC4|				 |			|
		 |	   |        PC3|---------------->|D7        |
Input<---|     |        PC2|---------------->|D6        |
Signal   |     |        PC1|---------------->|D5        |
         |     |        PC0|---------------->|D4        |
         |     |           |                 |D3        |
          -----|     T0/PD4|		         |D2        |
 Pushbutton<---|   INT0/PD2|                 |D1        |
               |           |                 |D0        |
               |           |                 |          |
               |        PB3|---------------->|E         |
               |           |         GND <---|RW        |
               |        PB5|---------------->|RS        |
                -----------                   ----------

************************************************************************/

#define F_CPU 8000000UL							// 8 MHz clock speed
#define BAUD 9600

#include <avr/io.h>
#include <util/delay.h>
#include <compat/ina90.h>						// defines _NOP, _CLI, _SEI
#include <avr/interrupt.h>

// LCD Interface
#define lcd_D7_port     PORTC                   // lcd D7 connection
#define lcd_D7_bit      PORTC3
#define lcd_D7_ddr      DDRC

#define lcd_D6_port     PORTC                   // lcd D6 connection
#define lcd_D6_bit      PORTC2
#define lcd_D6_ddr      DDRC

#define lcd_D5_port     PORTC                   // lcd D5 connection
#define lcd_D5_bit      PORTC1
#define lcd_D5_ddr      DDRC

#define lcd_D4_port     PORTC                   // lcd D4 connection
#define lcd_D4_bit      PORTC0
#define lcd_D4_ddr      DDRC

#define lcd_E_port      PORTB                   // lcd Enable pin
#define lcd_E_bit       PORTB3
#define lcd_E_ddr       DDRB

#define lcd_RS_port     PORTB                   // lcd Register Select pin
#define lcd_RS_bit      PORTB5
#define lcd_RS_ddr      DDRB

// LCD information
#define lcd_lineTwo		0x40

// LCD Instructions
#define lcd_clear			0b00000001	
#define lcd_home			0b00000000
#define	lcd_entryMode		0b00000110
#define	lcd_displayOn		0b00001100
#define	lcd_FunctionSet8bit	0b00000011
#define	lcd_functionSet4bit	0b10000010
#define	lcd_setCursor		0b10000000

// Pushbutton Interface
#define pushbutton_port	PORTD
#define pushbutton_pin	PIND
#define pushbutton_bit	PIND2
#define pushbutton_ddr	DDRD

// TWI Buffer Values
#define I2C_SEND_DATA_BUFFER_SIZE		0x20
#define I2C_RECEIVE_DATA_BUFFER_SIZE	0x20

// TWI Status Register Values
// Master
#define TW_START			0x08
#define TW_REP_START		0x10

// Master transmitter
#define TW_MT_SLA_ACK		0x18
#define TW_MT_SLA_NACK		0x20
#define TW_MT_DATA_ACK		0x28
#define TW_MT_DATA_NACK		0x30
#define TW_MT_ARB_LOST		0x38

// Master Receiver
#define TW_MR_ARB_LOST		0x38
#define TW_MR_SLA_ACK		0x40
#define TW_MR_SLA_NACK		0x48
#define TW_MR_DATA_ACK		0x50
#define	TW_MR_DATA_NACK		0x58

// Misc
#define TW_NO_INFO			0xF8
#define TWCR_STATUS_MASK	0xF8
#define TW_BUS_ERROR		0x00
#define TWCR_CMD_MASK		0x0F

// Standard I2C bit rates are
// 100 KHz - 400 KHz

// Program ID
uint8_t disp_volt[]	=	"Voltage = ";
uint8_t disp_v[]	=	" V";
uint8_t disp_freq[]	=	"Frequency = ";
uint8_t disp_hz[]	=	" Hz";
uint8_t disp_temp[]	=	"Temp = ";
uint8_t disp_deg[]	=	"°";
uint8_t disp_time[]	=	"Current time";

uint8_t time_change[] = "Time Change Requested:";
uint8_t enter_hour[] = "Enter Hour (Two digits):";
uint8_t enter_minutes[] = "Enter Minutes (Two digits):";
uint8_t enter_seconds[] = "Enter Seconds (Two digits):";
uint8_t time_changed[] = "Time changed to ";
uint8_t date_change[] = "Date Change Requested";
uint8_t enter_weekday[] = "Enter Day(1-7):";
uint8_t enter_month[] = "Enter Month (Two digits):";
uint8_t enter_day[] = "Enter Day (Two digits):";
uint8_t enter_year[] = "Enter Year (Two digits):";

const char sdata[] = "Hello World!\n";			// String in SRAM
const char fdata[] PROGMEM = "Flash Gordon\n";	// String in Flash

// Types
typedef enum
{
	I2C_IDLE = 0,
	I2C_BUSY = 1,
	I2C_MASTER_TX = 2,
	I2C_MASTER_RX = 3,
} eI2cStateType;

// Function prototypes
// LCD
void lcd_init_4d(void);
void lcd_write_instruction_4d(uint8_t theInstruction);
void lcd_write_character_4d(uint8_t theData);
void lcd_write_4(uint8_t theByte);
void lcd_write_string_4d(uint8_t theString[]);
// ADC and frequency
uint16_t read_ADC(void);
void init_freq_cntr(void);
unsigned int freq_cntr_get_frequency(void);
// USART
void getDate(int *yy, int *mm, int *dd);
void uart_init();
void uart_putc(char c);
char uart_getc();
void uart_printstr(char *s);
// i2c
void i2c_setBitrate(uint16_t bitRateKHz);
void i2c_sendStart(void);
void i2c_sendStop(void);
void i2c_waitForComplete(void);
void i2c_sendByte(uint8_t data);
void i2c_receiveByte(uint8_t ackFlag);
uint8_t i2c_getReceivedByte(void);
uint8_t i2c_getStatus(void);
void i2c_masterSend(uint8_t deviceAddr, uint8_t length, uint8_t* data);
void i2c_masterReceive(uint8_t deviceAddr, uint8_t length, uint8_t* data);
eI2cStateType i2c_getState(void);

// variables
uint16_t adc_result;
int mode = 0;
int mode_new = 0;
// I2C State and address variables
static volatile eI2cStateType I2cState;
static uint8_t i2cDeviceAddRW;
// Send/Transmit buffer (outgoing data)
static uint8_t i2cSendData[I2C_SEND_DATA_BUFFER_SIZE];
static uint8_t i2cSendDataIndex;
static uint8_t i2cSendDataLength;
// Receive buffer (incoming data)
static uint8_t i2cReceiveData[I2C_RECEIVE_DATA_BUFFER_SIZE];
static uint8_t i2cReceiveDataIndex;
static uint8_t i2cReceiveDataLength;
	

/******************************* Main Program Code *************************/
int main(void)
{
	// configure the microprocessor pins for the data lines
	lcd_D7_ddr |= (1<<lcd_D7_bit);                  // 4 data lines - output
	lcd_D6_ddr |= (1<<lcd_D6_bit);
	lcd_D5_ddr |= (1<<lcd_D5_bit);
	lcd_D4_ddr |= (1<<lcd_D4_bit);

	// configure the microprocessor pins for the control lines
	lcd_E_ddr |= (1<<lcd_E_bit);                    // E line - output
	lcd_RS_ddr |= (1<<lcd_RS_bit);                  // RS line - output
	
	// configure the microprocessor pins for the pushbutton
	pushbutton_ddr &= (1<<pushbutton_bit);
	pushbutton_port |= (1<<pushbutton_bit);
	
	// initialize adc
	ADMUX = ((1<<REFS0)|(1<<MUX2)|(1<<MUX0));				// Aref = Vcc, select ADC5
	ADCSRA = ((1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<ADPS0));	// Prescaler div factor = 128
	
	// initialize the LCD controller as determined by the defines (LCD instructions)
	lcd_init_4d();                                  // initialize the LCD display for a 4-bit interface

	// display the first line of information
	lcd_write_string_4d(disp_time);
	
	// set cursor to start of second line
	lcd_write_instruction_4d(lcd_setCursor | lcd_lineTwo);
	_delay_us(40);                                  // 40 uS delay (min)
	
	if (mode == 0)
	{
		lcd_write_string_4d(disp_volt);
	}
	else if (mode == 1)
	{
		lcd_write_string_4d(disp_freq);
	}
	else
	{
		lcd_write_string_4d(disp_temp);
	}	

	// Code for interfacing with the serial connection
	char str[25];
	int yy,mm,dd;
	sei();					// Enable global interrupts
	uart_init();			// Initialize the USART using baud rate 9600
	uart_printstr(sdata);		// Print a string from SRAM
	uart_printstr(fdata);		// Print a string from FLASH

	
	getDate(&yy,&mm,&dd);	// Get date from user
	sprintf(str,"Date: %d/%d/%d\n",yy,mm,dd);
	uart_printstr(str);

	// I2C code	
	//i2c_init();				// initialize I2C library
	


	// endless loop
	while(1)
	{
		uart_printstr(sdata);		// Print a string from SRAM
		uart_printstr(fdata);
		if(bit_is_clear(pushbutton_pin,pushbutton_bit))
		{
			_delay_ms(10);
			if(bit_is_clear(pushbutton_pin,pushbutton_bit))
			mode_new = (mode + 1) % 3;
		}
		
		if(mode_new != mode)
		{
			// set cursor to start of second line
			lcd_write_instruction_4d(lcd_setCursor | lcd_lineTwo);
			_delay_us(40);                                  // 40 uS delay (min)
	
			if (mode_new == 0)
			{
				lcd_write_string_4d(disp_volt);
			}
			else if (mode_new == 1)
			{
				lcd_write_string_4d(disp_freq);
			}
			else
			{
				lcd_write_string_4d(disp_temp);
			}			
		}
		
		mode = mode_new;
	}
	return 0;
}

/*============================== 4-bit LCD Functions ======================*/
/*
  Name:     lcd_init_4d
  Purpose:  initialize the LCD module for a 4-bit data interface
  Entry:    equates (LCD instructions) set up for the desired operation
  Exit:     no parameters
  Notes:    uses time delays rather than checking the busy flag
*/
void lcd_init_4d(void)
{
// Power-up delay
    _delay_ms(100);                                 // initial 100 mSec delay

// Set up the RS and E lines for the 'lcd_write_4' subroutine.
    lcd_RS_port &= ~(1<<lcd_RS_bit);                // select the Instruction Register (RS low)
    lcd_E_port &= ~(1<<lcd_E_bit);                  // make sure E is initially low

// Reset the LCD controller
    lcd_write_4(lcd_FunctionSet8bit);                 // first part of reset sequence
    _delay_ms(10);                                  // 4.1 mS delay (min)

    lcd_write_4(lcd_FunctionSet8bit);                 // second part of reset sequence
    _delay_us(200);                                 // 100uS delay (min)

    lcd_write_4(lcd_FunctionSet8bit);                 // third part of reset sequence
    _delay_us(200);                                 // this delay is omitted in the data sheet
 
    lcd_write_4(lcd_functionSet4bit);               // set 4-bit mode
    _delay_us(5);                                  // 40uS delay (min)

// Function Set instruction
    lcd_write_instruction_4d(lcd_functionSet4bit);   // set mode, lines, and font
    _delay_us(40);                                  // 40uS delay (min)

// Clear Display instruction
    lcd_write_instruction_4d(lcd_clear);             // clear display RAM
    _delay_ms(5);                                   // 1.64 mS delay (min)

// Display On/Off Control instruction
	lcd_write_instruction_4d(lcd_displayOn);         // turn the display ON
	_delay_us(40);                                  // 40uS delay (min)

// ; Entry Mode Set instruction
    lcd_write_instruction_4d(lcd_entryMode);         // set desired shift characteristics
    _delay_us(40);                                  // 40uS delay (min)
}

/*...........................................................................
  Name:     lcd_write_string_4d
; Purpose:  display a string of characters on the LCD
  Entry:    (theString) is the string to be displayed
  Exit:     no parameters
  Notes:    uses time delays rather than checking the busy flag
*/
void lcd_write_string_4d(uint8_t theString[])
{
    int i = 0;                             // character counter*/
    while (theString[i] != 0)
    {
        lcd_write_character_4d(theString[i]);
        i++;
        _delay_us(40);                              // 40 uS delay (min)
    }
}

/*...........................................................................
  Name:     lcd_write_character_4d
  Purpose:  send a byte of information to the LCD data register
  Entry:    (theData) is the information to be sent to the data register
  Exit:     no parameters
  Notes:    does not deal with RW (busy flag is not implemented)
*/

void lcd_write_character_4d(uint8_t theData)
{
    lcd_RS_port |= (1<<lcd_RS_bit);                 // select the Data Register (RS high)
    lcd_E_port &= ~(1<<lcd_E_bit);                  // make sure E is initially low
    lcd_write_4(theData);                           // write the upper 4-bits of the data
    lcd_write_4(theData << 4);                      // write the lower 4-bits of the data
}

/*...........................................................................
  Name:     lcd_write_instruction_4d
  Purpose:  send a byte of information to the LCD instruction register
  Entry:    (theInstruction) is the information to be sent to the instruction register
  Exit:     no parameters
  Notes:    does not deal with RW (busy flag is not implemented)
*/
void lcd_write_instruction_4d(uint8_t theInstruction)
{
    lcd_RS_port &= ~(1<<lcd_RS_bit);                // select the Instruction Register (RS low)
    lcd_E_port &= ~(1<<lcd_E_bit);                  // make sure E is initially low
    lcd_write_4(theInstruction);                    // write the upper 4-bits of the data
    lcd_write_4(theInstruction << 4);               // write the lower 4-bits of the data
}

/*...........................................................................
  Name:     lcd_write_4
  Purpose:  send a byte of information to the LCD module
  Entry:    (theByte) is the information to be sent to the desired LCD register
            RS is configured for the desired LCD register
            E is low
            RW is low
  Exit:     no parameters
  Notes:    use either time delays or the busy flag
*/
void lcd_write_4(uint8_t theByte)
{
    lcd_D7_port &= ~(1<<lcd_D7_bit);                        // assume that data is '0'
    if (theByte & 1<<7) lcd_D7_port |= (1<<lcd_D7_bit);     // make data = '1' if necessary

    lcd_D6_port &= ~(1<<lcd_D6_bit);                        // repeat for each data bit
    if (theByte & 1<<6) lcd_D6_port |= (1<<lcd_D6_bit);

    lcd_D5_port &= ~(1<<lcd_D5_bit);
    if (theByte & 1<<5) lcd_D5_port |= (1<<lcd_D5_bit);

    lcd_D4_port &= ~(1<<lcd_D4_bit);
    if (theByte & 1<<4) lcd_D4_port |= (1<<lcd_D4_bit);

// write the data
                                                    // 'Address set-up time' (40 nS)
    lcd_E_port |= (1<<lcd_E_bit);                   // Enable pin high
    _delay_us(1);                                   // implement 'Data set-up time' (80 nS) and 'Enable pulse width' (230 nS)
    lcd_E_port &= ~(1<<lcd_E_bit);                  // Enable pin low
    _delay_us(1);                                   // implement 'Data hold time' (10 nS) and 'Enable cycle time' (500 nS)
}

/*================================= ADC Functions =========================*/
/*...........................................................................
  Name:     read_ADC
  Purpose:  get value of input on ADC5
  Exit:     10 bit value from ADC
*/
uint16_t read_ADC()
{
	ADC = 0;
	ADCSRA |= (1<<ADSC);									// Start single conversion
	while(!(ADCSRA & (1<<ADIF)));							// wait for conversion to finish
	ADCSRA |= (1<<ADIF);									// Clear flag
	return(ADC);
}

/*============================== Frequency Functions ======================*/
/*Name:     init_freq_cntr
  Purpose:  configure TC0 to measure frequency
  Exit:     no parameters
 */

void init_freq_cntr()
{
	TCCR0A = 0;					// 16 bit mode is default, overflow at 0xffff
}

unsigned int freq_cntr_get_frequency(void)
{
	unsigned int freq_cntr_freq_divd_by_10;
	_CLI();
	TCNT1H = 0x00; // clear counter lower 8 bits
	TCNT1L = 0x00; // clear counter upper 8 bits
	//		connect FREQ_CNTR to external pin T0 and start counting
	TCCR1B = (1<<CS12)|(1<<CS11);	// select FREQ_CNTR input clock
	
	// Wait until Timer 0 gets its first clock...
	while (TCNT1L == 0) 
		_NOP();
	// Now start a software timer that determines how long FREQ_CNTR runs
	_delay_ms(100); 
	TCCR0B = 0;//(0<<CLOCK_SEL_00);  // stop counter

	// Test TIFR for 16-bit overflow.  If overrange return  0xFFFF
	
	if ((TIFR0 & (1<<TOV1)) !=0) 
	{
		// Test TIFR for 16-bit overflow =  0xFFFF

		TIFR1 = (1<<TOV1);// Clear the OVF flag
		freq_cntr_freq_divd_by_10 = 0xFFFF; // This is to return a OVF condition
	}
	else 
	{
		freq_cntr_freq_divd_by_10 = TCNT1L+(TCNT1H<<8);
	}

	return(freq_cntr_freq_divd_by_10);
}

// UART
void uart_init()
{
	UBRR0 = 0;
	/* Setting the XCKn port pin as output, enables master
	mode. */
	XCK_DDR |= (1<<XCK_BIT);
	/* Set MSPI mode of operation and SPI data mode 0. */
	UCSR0C =
	(1<<UMSEL01)|(1<<UMSEL00)|(0<<UPM00)|(0<<UCPOL0);
	/* Enable receiver and transmitter. */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	/* Set baud rate. */
	/* IMPORTANT: The Baud Rate must be set after the
	transmitter is enabled */
	UBRR0 = BAUD;
	
	// Old code
	//UBRRn = 0;
	//(1<<UMSELn1)|(1<<UMSELn0)|(0<<UCPHAn)|(0<<UCPOLn);
	//UCSRnB = (1<<RXENn)|(1<<TXENn);
	//UBRRN = BAUD
}

// Print a string
void uart_printstr(char *s)
{
	while(*s){
		uart_putc(*s);
		s++;
	}
}

// Gets character
char uart_getc()
{
	return UDR0;
}

// sends character
void uart_putc(char c)
{
	while(!((UCSR0A)& (_BV(UDRE0))));
	UDR0 = c;
}

void getDate(int *yy, int *mm, int *dd)
{
	unsigned char c;
	char str[25];
	int i;
	// Get date from user
	// modify values yy, mm, dd
	uart_printstr("Please Enter Year (yyyy):");
	for (i=0;i<=4-1;i++){
		c = uart_getc(); // Get character
		uart_putc(c); // Echo it back
		str[i] = c;
	}
	str[i] = '\0';
}
/*

I2C controller library
http://homepage.hispeed.ch/peterfleury/doxygen/avr-gcc-libraries/group__pfleury__ic2master.html
Place this library in
C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain
download link: http://homepage.hispeed.ch/peterfleury/i2cmaster.zip

#include <C:/Program Files (x86)/Atmel/Studio/7.0/toolchain/avr8/avr8-gnu-toolchain/i2cmaster/i2cmaster.h>

UART Library
http://homepage.hispeed.ch/peterfleury/doxygen/avr-gcc-libraries/group__pfleury__uart.html
download link: http://homepage.hispeed.ch/peterfleury/uartlibrary.zip

#include <C:/Program Files (x86)/Atmel/Studio/7.0/toolchain/avr8/avr8-gnu-toolchain/uartlibrary/uart.h>

	NOTES
	
	This is how you convert from a string of characters
	that represents a number to an int
	sscanf(c,"%d",&num);


http://www.nxp.com/documents/data_sheet/PCF8583.pdf
http://datasheets.maximintegrated.com/en/ds/DS1621.pdf
The temperature reading is provided in a 9-bit, two’s complement reading by issuing the READ
TEMPERATURE command.

The data is transmitted through the 2-wire serial interface, MSB first

Since data is transmitted over the 2-wire bus MSB first, temperature data may be written to/read from the
DS1621 as either a single byte (with temperature resolution of 1°C)

8 bits are the temperature
last bit is the .5

The DS1621 always powers up in a low power idle state, and the Start Convert T command must be used
to initiate conversions.

The configuration register is defined as follows:
MSb  Bit 6 Bit5 Bit 4 Bit 3 Bit 2 Bit 1 LSb
DONE THF   TLF  NVB   X     X     POL   1SHOT

 A device that sends data
 onto the bus is defined as a transmitter, and a device receiving data as a receiver. The device that controls
 the message is called a “master." The devices that are controlled by the master are “slaves." The bus must
 be controlled by a master device which generates the serial clock (SCL), controls the bus access, and
 generates the START and STOP conditions. The DS1621 operates as a slave on the 2-wire bus.
 Connections to the bus are made via the open-drain I/O lines SDA and SCL.


for RTC connect OSCI and OSCO together
SCL and SDA to uc I2C bus
SDA and SCL to uc (this is for both of the chips 


22. 2-wire Serial Interface

SCL, 2-wire Serial Interface Clock: When the TWEN bit in TWCR is set (one) to enable the 2-wire Serial
Interface, pin PC5 is disconnected from the port and becomes the Serial Clock I/O pin for the 2-wire Serial
Interface. In this mode, there is a spike filter on the pin to suppress spikes shorter than 50 ns on the input signal,
and the pin is driven by an open drain driver with slew-rate limitation.
PC4 / PC5
SCL = clock
SDA = data

The SCL period is controlled by settings
in the TWI Bit Rate Register (TWBR) and the Prescaler bits in the TWI Status Register (TWSR)

Note that the TWI Interrupt Enable (TWIE) bit in TWCR
together with the Global Interrupt Enable bit in SREG allow the application to decide whether or not assertion of
the TWINT Flag should generate an interrupt request.

When the TWINT Flag is asserted, the TWI has finished an operation and awaits application response. In this
case, the TWI Status Register (TWSR) contains a value indicating the current state of the TWI bus. The
application software can then decide how the TWI should behave in the next TWI bus cycle by manipulating the
TWCR and TWDR Registers

how to use on page 214
*/

/*================================= I2C Functions =========================*/

/*
	Set i2c transaction bitrate (in KHz)
	Input: bitRate in KHZ
	Ouput: none
*/

void i2c_setBitrate(uint16_t bitRateKHz)
{
	// set pull up resistors on I2C bus pins
	/*
	sbi(PORTC,0)	// i2c SCL
	sbi(PORTC,1)	// i2c SDA
	*/

	uint8_t bitrate_div;

	// set i2c bitrate
	// SCL freq = F_CPU/(16+2*TWBR)
	// calculate bit rate division
	bitrate_div = (F_CPU/10000)/bitRateKHz;
	if (bitrate_div >= 16)
	{
		bitrate_div = (bitrate_div - 16) / 2;
	}
	TWBR = bitrate_div;
}

/*
	Send start condition
	Input: none
	Output: none
*/
void i2c_sendStart(void)
{
	// send start condition
	TWCR = ((TWCR & TWCR_CMD_MASK)|1<<TWINT|1<<TWSTA);
}

/* 	Send stop condition
	Input: none
	Output: TWEA left on for slave
*/
void i2c_sendStop(void)
{
	TWCR = ((TWCR & TWCR_CMD_MASK)|1<<TWINT|1<<TWEA|1<<TWSTO);
}

/*
	Wait for i2c to complete operation
	Input: none
	Output: none
*/
void i2c_waitForComplete(void)
{
	while(!(TWCR & 1<<TWINT));
}

/*
	Send an address|R/W combination or a data byte over i2c
	Input: uint8_t data - data to be sent
	Output: none
*/
void i2c_sendByte(uint8_t data)
{
	// save data to the TWDR
	TWDR = data;

	// begin send
	TWCR = ((TWCR & TWCR_CMD_MASK)|1<<TWINT);
}

/*
	Receive  data byte over i2c
	Input: uint8_t ackFlag
		  	ackFlag = True, if received data should be acknowledged
		  	ackFlag = False, if received data should not be acknowledged
	Output: none
*/
void i2c_receiveByte(uint8_t ackFlag)
{
	// begin receive over i2c
	if(ackFlag)
	{
		// ackFlag = true: ACK the received data
		TWCR = ((TWCR & TWCR_CMD_MASK)|1<<TWINT|1<<TWEA);
	}
	else
	{
		// ackFlag = false; NACK the recevied data
		TWCR = ((TWCR & TWCR_CMD_MASK)|1<<TWINT);
	}
}	

/*
	Pick up data that was received with i2c_receivedByte
	Input: None
	Output: Returns TWDR
*/
uint8_t i2c_getReceivedByte(void)
{
	// Return received data byte from i2c TWDR
	return TWDR;
}

/*
	Get current I2c bus status from TWSR
	Input: none
	Output: returns value of TWSR
*/
uint8_t i2c_getStatus(void)
{
	// retrieve current status from i2c TWSR
	return TWSR;
}

/*
	Send i2c data to a device on the bus
	Input: 	uint8_t device address - address of device to receive data
			uint8_t length - length of data to be sent
			uint8_t* data - address of data to be sent
	Output: none
*/
void i2c_masterSend(uint8_t deviceAddr, uint8_t length, uint8_t* data)
{
	uint8_t i;

	// wait for interface to be ready
	while(i2c_getState());

	// set state
	I2cState = I2C_MASTER_TX;

	// save data
	i2cDeviceAddRW = (deviceAddr|0xFE);		// RW cleared; write operation
	for(i = 0; i < length; i++)
	{
		i2cSendData[i] = *data++;
	}
	i2cSendDataIndex = 0;
	i2cSendDataLength = length;

	// send start condition
	i2c_sendStart();
}

/*
	Receive i2c data from a device on the bus
	Input: 	uint8_t deviceAddr - address of device sending data
			uint8_t length - length of data to be received
			u09 *data - location where data is to be stored
	Output: none
*/
void i2c_masterReceive(uint8_t deviceAddr, uint8_t length, uint8_t* data)
{
	uint8_t i;

	// wait for interface to be ready
	while(i2c_getState());

	// set state
	I2cState = I2C_MASTER_RX;

	// save data
	i2cDeviceAddRW = (deviceAddr|0x01);		//RW set; read operation
	i2cReceiveDataIndex = 0;
	i2cReceiveDataLength = length;

	// send start condition
	i2c_sendStart();

	// wait for data
	while(i2c_getState());

	// return data
	for(i = 0; i < length; i++)
	{
		*data = i2cReceiveData[i];
	}
}

/* 
	Get current high-level state of the i2c interface
	Input: none
	Return: state type - current state
*/
eI2cStateType i2c_getState(void)
{
	return I2cState;
}

// I2C Interrupt Service Routine
SIGNAL(SIG_2WIRE_SERIAL)
{
	// read status bits
	uint8_t status = TWSR & TWCR_STATUS_MASK;

	switch(status)
	{
		// Master general
		case TW_START:												// 0x08; sent start condition
		case TW_REP_START:											// 0x10; sent repeated start condition
			// send device address
			i2c_sendByte(i2cDeviceAddRW);
			break;

		// Master transmitter and receiver status codes
		case TW_MT_SLA_ACK:											// 0x18; slave address acknowledged
		case TW_MT_DATA_ACK:										// 0x28; data acknowledged
			if(i2cSendDataIndex < i2cSendDataLength)
			{
				// send data
				i2c_sendByte(i2cSendData[i2cSendDataIndex++]);
			}
			else
			{
				// transmit stop condition, enable SLA ACK
				i2c_sendStop();
				// set state
				I2cState = I2C_IDLE;

			}
			break;
		case TW_MR_DATA_NACK:										// 0x58; data received, NACK issued
			// store final received data byte
			i2cReceiveData[i2cReceiveDataIndex++] = TWDR;
			// continue to transmit STOP condition
		case TW_MR_SLA_NACK:										// 0x48; slave address not acknowledged
		case TW_MT_SLA_NACK:										// 0x20; slave address not acknowledged
		case TW_MT_DATA_NACK:										// 0x30; data not acknowledged
			// transmit stop condition, enable sla ack
			i2c_sendStop();
			// set state
			I2cState = I2C_IDLE;
			break;
		case TW_MT_ARB_LOST:										// 0x38; bus arbitration lost
		//case TW_MR_ARB_LOST:										// 0x38; bus arbitration lost
			// release bus
			TWCR = ((TWCR & TWCR_CMD_MASK)|1<<TWINT);

			// set state
			I2cState = I2C_IDLE;

			// release bus and transmit start when bus is free
			// TWCR = ((TWCR & TWCR_CMD_MASK)|1<<TWINT|1<<TWSTA);
			break;
		case TW_MR_DATA_ACK:										// 0x50; data acknowledged
			// store received data byte
			i2cReceiveData[i2cReceiveDataIndex++] = TWDR;
			
			// fall through to see if more bytes will be received
		case TW_MR_SLA_ACK:										// 0x40; slave address acknowledged
			if(i2cReceiveDataIndex < (i2cReceiveDataLength - 1))
			{
				// data byte will be received, reply with ack (more bytes in transfer)
				i2c_receiveByte(1);
			}
			else
			{
				// data byte will be received, reply with nack (final byte in transfer)
				i2c_receiveByte(0);
			}
			break;

		// Misc
		case TW_NO_INFO:											// 0xF8; no relevant state information	
			// do nothing
			break;
		case TW_BUS_ERROR:											// 0x00; bus error due to illegal start or stop condition
			// reset internal hardware and release bus
			TWCR = ((TWCR & TWCR_CMD_MASK)|1<<TWINT|1<<TWSTO|1<<TWEA);

			// set state
			I2cState = I2C_IDLE;
			break;
	}
}