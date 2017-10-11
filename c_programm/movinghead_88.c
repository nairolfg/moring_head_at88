#include<avr/io.h>
#include<avr/interrupt.h>
#include<stdint.h>


//Globale Variablen

uint8_t address;								// enthält die Startadresse

uint8_t kanal_servo1;
uint8_t kanal_servo2;
uint8_t kanal_red;
uint8_t kanal_green;
uint8_t kanal_blue;

volatile uint8_t gDmxState;						// IDLE
int dmxkanal = 0;								// anfang der Kanäle




//ende Globale Variablen//


void datadirec_init(void){

    DDRB |=(1<<PB0)|(1<<PB1)|(1<<PB2);		// Ausgänge festlegen
	//DDRC |=(1<<PC0);		// PA0 als Ausgang
	DDRD |=(1<<PD3)|(1<<PD5)|(1<<PD6);		// Ausgänge festlegen	
}

void timer_init(void){								// RGB PWM
	
	// PWM-Initialisierung für Red u. Green
	
	TCCR0A |= (1<<COM0A1)|(1<<COM0B1)|(1<<WGM00)|(1<<WGM01);		//	Clear on compare match, set on top, Fast PWM TOP = 0xFF
	TCCR0B |= (1<<CS00);											// Fast PWM, no prescaler (ggf. noch ändern)

	// PWM-Initialisierung für Blue

	TCCR2A |= (1<<COM2B1)|(1<<WGM20)|(1<<WGM21);					//	Clear on compare match, set on TOP, Fast PWM TOP = 0xFF
	TCCR2B |= (1<<CS20);											// Fast PWM, no prescaler (ggf. noch änderrn)
}

void servo_init(void){
	TCCR1A |= (1<<COM1A1)|(1<<COM1B1)|(1<<WGM11);					// Clear ORC1A/B on compare match, Set at TOP
	TCCR1B |= (1<<WGM12)|(1<<WGM13)|(1<<CS11); 						// Fast PWM TOP = ICR1, Prescaler 8
	ICR1 = 65536;													// 1ms = 8000 Takte
}


void UART_init (void){

	UBRRH = 0;
	UBRRL = 1;														// Baudrate = 250kbit/s
	UCSRA=0;
	UCSRB=(1<<RXCIE)|(1<<RXEN);										// receive interrupt enable, receive enable
	UCSRC=(1<<URSEL)|(1<<USBS)|(1<<UCSZ1)|(1<<UCSZ0);
}


void anfangszustand(void){

	OCR0A = 0;			// Red
	OCR0B = 0;			// Green
	OCR2B = 0;			// Blue
	
	OCR1A = 12000;		// Servo 1
	OCR1B = 12000;		// Servo 2

}


int get_address(void){

	address = (PINC & 0x3F)|((PIND & 0x10)<<2)|;


}




ISR (USART_RX_vect){
	
	cli();								// deaktivieren der Interrupts
	static uint16_t DmxCount;			// Variable für momentanen Kanal
	uint8_t USARTstate = UCSRA;			// Variable für Status des UCSRA
	uint16_t DmxByte = UDR;				// einlesen des wertes aus dem UDR
	uint8_t DmxState = gDmxState;		// DMX-Status (IDLE->0,BREAK->1,
										// STARTBYTE->2,STARTADRESSE->3)

	if(USARTstate&(1<<4)){				// schaut nach FrameError
		UCSRA &=~(1<<FE);				// setzt FrameError zu 0
		DmxCount = 1;					// 
		gDmxState = 1;					// DMX-Status -> BREAK
		return;
	}

	
	else if(DmxState==1){				// wenn DMX-Status = BREAK
		if (DmxByte==0) gDmxState = 2;	// DMX-Status -> STARTBYTE		
		else gDmxState = 0;				// IDLE
	}


	else if(DmxState==2){											// wenn DMX-Status = STARTBYTE	
		if(--DmxCount == 0){
			DmxCount = 1;
			if(DmxCount==kanal_servo1)OCR1A = DmxByte*31+8000;		// zuweisung des Wertes zum ersten Aktor, wenn Startadresse = 1
			gDmxState = 3;											// DMX-Status -> STARTADRESSE
		}
	}

	else if(DmxState==3){
		DmxCount++;
		if(DmxCount==kanal_servo1)OCR1A = DmxByte*31+8000;			// zuweisung zum 1. Aktor
		if(DmxCount==kanal_servo2)OCR1B = DmxByte*31+8000;			// zuweisung zum 2. Aktor
		if(DmxCount==kanal_red)OCR0A = DmxByte;						// zuweisung zum 3. Aktor
		if(DmxCount==kanal_green)OCR0B = DmxByte;					// zuweisung zum 4. Aktor
		if(DmxCount==kanal_blue)OCR2B = DmxByte;					// zuweisung zum 5. Aktor
		if(DmxCount> kanal_blue) gDmxState = 0;						// DMX-Status -> IDLE
	}

	sei();								// einschalten der Interrupts
	return;
}


int main(void){

	datadirec_init();					// Initialisierung der I/O ports
	timer_init();						// Timer-Initialisierung
	servo_init();						// Timer-Initialisierung für Servos
	UART_init();						// UART-Initialisierung
	anfangszustand();					// festlegen des Anfangszustandes
	gDmxState = 0;						// DMX-Status -> IDLE
	sei();								// einschalten der Interrupts

	

	while(1){

	}



}
