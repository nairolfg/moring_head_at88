#include<avr/io.h>
#include<avr/interrupt.h>
#include<stdint.h>


//Globale Variablen//


uint16_t red_pwm;							// variable für PWM der roten LED
uint16_t green_pwm;							// variable für PWM der roten LED
uint16_t blue_pwm;							// variable für PWM der roten LED
uint16_t servo1_pwm;							// variable für PWM der roten LED
uint16_t servo2_pwm;							// variable für PWM der roten LED

uint16_t kanal_red = 1;
uint16_t kanal_green = 2;
uint16_t kanal_blue = 3;
uint16_t kanal_servo1 = 4;
uint16_t kanal_servo2 = 5;

volatile uint8_t gDmxState;						// IDLE
int dmxkanal = 0;						// anfang der Kanäle




//ende Globale Variablen//


void datadirec_init(void){

    DDRB |=(1<<PB0)|(1<<PB1)|(1<<PB2);		// Ausgänge festlegen
	//DDRC |=(1<<PC0);		// PA0 als Ausgang
	DDRD |=(1<<PD3)|(1<<PD5)|(1<<PD6);		// Ausgänge festlegen	
}

void timer_init(void){
	
	
	
}

void servo_init(void){
	TCCR1A |= (1<<COM1A1)|(1<<COM1B1)|(1<<WGM11);	// Clear ORC1A/B on compare match, Set at TOP
	TCCR1B |= (1<<WGM12)|(1<<WGM13)|(1<<CS11); 				// Fast PWM TOP = ICR1, Prescaler 8
	ICR1 = 65536;										// 1ms = 8000 Takte
	OCR1A = 12000;
	OCR1B = 12000;
}


void UART_init (void){

	UBRRH = 0;
	UBRRL = 1;											// Baudrate = 250kbit/s
	UCSRA=0;
	UCSRB=(1<<RXCIE)|(1<<RXEN);							// receive interrupt enable, receive enable
	UCSRC=(1<<URSEL)|(1<<USBS)|(1<<UCSZ1)|(1<<UCSZ0);
}


void anfangszustand(void){


	red_pwm=0;								// festlegen der PWM Schaltschwelle 0...30 -> High, 31...255 -> Low
	green_pwm=0;							// festlegen der PWM Schaltschwelle 0...30 -> High, 31...255 -> Low									
	blue_pwm=0;								// festlegen der PWM Schaltschwelle 0...30 -> High, 31...255 -> Low
	servo1_pwm=256/2;						// Servo unten
	servo2_pwm=256/2;


}




ISR (USART_RX_vect){
	
	cli();								// deaktivieren der Interrupts
	static uint16_t DmxCount;			// Variable für momentanen Kanal
	uint8_t USARTstate = UCSRA;			// Variable für Status des UCSRA
	uint8_t DmxByte = UDR;				// einlesen des wertes aus dem UDR
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


	else if(DmxState==2){				// wenn DMX-Status = STARTBYTE	
		if(--DmxCount == 0){
			DmxCount = 1;
			//servo1_pwm = DmxByte;		// zuweisung des Wertes zum ersten Aktor
			gDmxState = 3;				// DMX-Status -> STARTADRESSE
			}
		}

	else if(DmxState==3){
		PORTE |=(1<<PE0);
		DmxCount++;
		if(DmxCount==kanal_servo2)servo2_pwm = 225;	// zuweisung zum 2. Aktor
		if(DmxCount==kanal_servo1)servo1_pwm = 225;	// zuweisung zum 2. Aktor
		if(DmxCount==kanal_red)red_pwm = 225;		// zuweisung zum 3. Aktor
		if(DmxCount==kanal_green)green_pwm = 225;	// zuweisung zum 4. Aktor
		if(DmxCount==kanal_blue)blue_pwm = 225;		// zuweisung zum 5. Aktor
		if(DmxCount>=7) gDmxState = 0;	// DMX-Status -> IDLE
		}

	sei();								// einschalten der Interrupts
	PORTE &=~(1<<PE0);
	return;
}


int main(void){

	datadirec_init();					// Initialisierung der I/O ports
	timer_init();						// Timer-Initialisierung
	UART_init();						// UART-Initialisierung
	anfangszustand();					// festlegen des Anfangszustandes
	gDmxState = 0;						// DMX-Status -> IDLE
	sei();								// einschalten der Interrupts

	

	while(1){

	}



}
