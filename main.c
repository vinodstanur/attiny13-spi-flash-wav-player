#include <avr/io.h>
#define F_CPU 9600000
#include <util/delay.h>
#include <avr/power.h>
#include <avr/interrupt.h>


#define SPI_PORT	PORTB
#define SPI_DDR		DDRB
#define SPI_PIN		PINB
#define CS_PIN		(1<<PB3)
#define MOSI_PIN	(1<<PB2)
#define MISO_PIN 	(1<<PB4)
#define SCK_PIN 	(1<<PB1)

#define SOFT_TX_PIN	(1<<PB1)
#define SOFT_TX_PORT PORTB
#define SOFT_TX_DDR DDRB

#define SOFT_RX_PIN	(1<<PB0)
#define SOFT_RX_DDR DDRB
#define SOFT_RX_PORT PINB

#define read_miso()		(SPI_PIN & (MISO_PIN))
#define select_chip()	(SPI_PORT &= ~(CS_PIN)) 
#define deselect_chip()	(SPI_PORT |= (CS_PIN)) 
#define SPI_SCK_HIGH()	(SPI_PORT |= SCK_PIN)
#define SPI_SCK_LOW()	(SPI_PORT &= ~SCK_PIN)
#define SPI_MOSI_HIGH()	(SPI_PORT |= MOSI_PIN)
#define SPI_MOSI_LOW()	(SPI_PORT &= ~MOSI_PIN)

//global variables
volatile unsigned char uart_rx;
volatile unsigned char uart_rx_flag;
volatile unsigned char c_r_p, c_w_p;	//circular buffer read-write index
volatile uint8_t ser_available;
uint8_t c_buff[16];

//initialize bitbanged SPI 
void spi_init(void)
{
	deselect_chip();
	SPI_DDR = (MOSI_PIN)|(SCK_PIN)|(CS_PIN);
	SPI_SCK_HIGH();
}

//initialize fast PWM with prescaler 1 at PB0
void pwm_init(void)
{
	TCCR0B |= (1 << CS00);
	TCCR0A |= (1 << WGM01) | (1 << WGM00);
	TCCR0A |= (1 << COM0A1);
	DDRB |= 1<<PB0;
}


//bitbanged UART transmit byte
void uart_send_byte(unsigned char data)
{
	unsigned char i;
	TCCR0B = 0;
	TCNT0 = 0;
	TIFR0 |= 1<<OCF0A;
	TCCR0B |= (1 << CS00);
	TIFR0 |= 1<<OCF0A;
	SOFT_TX_PORT &= ~SOFT_TX_PIN;
	while(!(TIFR0 & (1<<OCF0A)));
	TIFR0 |= 1<<OCF0A;
	for(i=0;i<8;i++) {
			if(data & 1)
				SOFT_TX_PORT |= SOFT_TX_PIN;
			else 
				SOFT_TX_PORT &= ~SOFT_TX_PIN;
			data >>= 1;
			while(!(TIFR0 & (1<<OCF0A)));
			TIFR0 |= 1<<OCF0A;
	}
	SOFT_TX_PORT |= SOFT_TX_PIN;
	while(!(TIFR0 & (1<<OCF0A)));
	TIFR0 |= 1<<OCF0A;
}

void uart_print(char *p)
{
	while(*p)
		uart_send_byte(*p++);
}
// uart receive data
ISR(PCINT0_vect)
{
	int cn;
	uart_rx = 0;
	_delay_us(2);
	TCCR0B =0;
	TCNT0 = 0;
	TIFR0 |= 1<<OCF0A;
	TCCR0B |= (1 << CS00);
	for(cn = 0; cn < 8; cn++) {
		while(!(TIFR0 & (1 << OCF0A)));
	  TIFR0 |= 1<<OCF0A;
		uart_rx >>= 1;
		if(PINB & (1<<PB0))
	 	uart_rx |= (1<<7);
	}

	ser_available = 1;
	c_buff[c_w_p] = uart_rx;
	c_w_p++;
	c_w_p &= 15;

	while(!(TIFR0 & (1 << OCF0A)));
	TIFR0 |= 1<<OCF0A;
	GIFR |= 1<<PCIF;
}

unsigned char serial_read()
{
	unsigned char rett = 0;
	ser_available = 0;
	if(c_r_p != c_w_p) {
		rett = c_buff[c_r_p];
		c_r_p++;
		c_r_p &= 15;
	}

	if(c_r_p != c_w_p)
		ser_available = 1;

	return rett;
}
		

void uart_rx_int_init()
{
	SOFT_RX_DDR &= ~(SOFT_RX_PIN);
	SOFT_RX_PORT |= (SOFT_RX_PIN);
	GIMSK |= 1<<PCIE;
	PCMSK |= 1<<PCINT0;
}

void uart_tx_init()
{
	TCCR0A = 1 << WGM01;   // compare mode
	TCCR0B = 1 << CS00; // 1/8 prescaler
	ADMUX |= 1<<REFS0;
	SOFT_TX_PORT |= SOFT_TX_PIN;
	SOFT_TX_DDR |= SOFT_TX_PIN;
	OCR0A = 75; //9600 baud at prescaler 1/8
}

uint8_t spi_transmit_receive(uint8_t c)
{
	uint8_t i;
	for(i = 0; i < 8; i++) {
		if(c & (1<<7)) {
			SPI_MOSI_HIGH();
		} else {
			SPI_MOSI_LOW();
		}
		SPI_SCK_LOW();
		c <<= 1;
		if(read_miso()) {
			c |= 1;
		}
		SPI_SCK_HIGH();
	}	
	return c;
}

void erase_chip()
{
	deselect_chip();select_chip();
	spi_transmit_receive(0x06); //enable write
	deselect_chip();select_chip();

	spi_transmit_receive(0xc7); //chip erase
	deselect_chip();select_chip();

	spi_transmit_receive(0x05); //initiate read status register

 while(spi_transmit_receive(0xff) == 3);

	deselect_chip();	
}

int main()
{
	/// local variables
	volatile uint8_t temp1, temp2;
	uint32_t address = 0UL;
	int i;
	///////////////

	clock_prescale_set(clock_div_1);	//set clock to 9.6MHz, no prescaler

	spi_init();	//init spi. MODE 3

	//if PB0 (PWM pin) is pulled LOW on boot time then enter to 
	//flash loading mode. This is the only way to change song
	//in the spi flash. Else control goes to PLAYBACK mode...
	 DDRB &= ~(1<<PB0);
   PORTB |= 1<<PB0;
   _delay_ms(50);
	 if((PINB & (1<<PB0)) == 0) {
			while((PINB & (1 << PB0)) == 0)
		 		_delay_ms(10);

		/////////// RECORD NEW SONG VIA UART ///////////////
	  uart_tx_init();
		uart_rx_int_init();
	  sei();
		_delay_ms(10);
		

	 while(1) {
		 	//wait for "ok" from PC to start erasing and flashing
			if(ser_available) {
				temp1 = temp2;
				temp2 = serial_read();
				if(temp1 == 'o' && temp2 == 'k')
					break;
			}
		}
		erase_chip();				//ease the spi flash to load new sound
		uart_print("start\n");	//return acknowledgement
		
	while(1) {
			i = 0;
			_delay_us(100);
			
			deselect_chip();	//toggle chip select
			select_chip();
			
			spi_transmit_receive(0x06); //unlock write
			
			deselect_chip();	//toggle chip select
			select_chip();
			
			spi_transmit_receive(0x02); //initiate write operation
			spi_transmit_receive((uint8_t)(address >> 16)); //write address MSB
			spi_transmit_receive((uint8_t)(address >> 8)); 
			spi_transmit_receive((uint8_t)(address >> 0)); //write address LSB
			
		 // fill the page with data received from uart
			for(i = 0; i < 256; i++) {
				while(!ser_available);
				spi_transmit_receive(serial_read());
			}
			
			deselect_chip(); //flash write operation starts at rising edge of CS 
			select_chip();
			
			spi_transmit_receive(0x05);		//read status register of spi flash
			while(spi_transmit_receive(0x05) == 3); //wait until write operation gets completed
			
			deselect_chip();	//toggle chip select
		 	select_chip();
			
			spi_transmit_receive(0x04);	//lock the chip from unintentional write
		  
			deselect_chip();	//deselect chip 
			
			address += 256UL;	//increment address 
			_delay_us(100);
			uart_rx_flag = 0; // clear uart rx flag
			uart_print("x");	//send ack to pc for receiving and flashing 256 bytes
	
			while(!ser_available);	//check return ack, consider it as a write token
			if(serial_read() != 'z')	//if handshaking fails, goto to infinite loop to halt everything because some where some thing went wrong :(
			while(1);
		}
	}
	
////////////////// PLAYBACK APPLICATION//////////////////////
	 //code enters here if button is not pressed while applying power
		spi_init();	//init spi. MODE 3
		pwm_init();	//fast pwm, prescaler 1
			
		deselect_chip();
		select_chip();

		spi_transmit_receive(0x03);	//initiate read from 0x000000
		spi_transmit_receive(0x0);	//address msB
		spi_transmit_receive(0x0);
		spi_transmit_receive(0x0);	//address lsB
		
		//loop play for ever!!!
		while(1) {
			OCR0A = spi_transmit_receive(0xff); //read spi data and send to 
			_delay_us(100);	//delay between samples (adjust it to play song fast or slow)
		}
		
}
