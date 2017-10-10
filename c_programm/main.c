/*
 * HeatController.c
 *
 * Created: 11.11.2015 13:02:45
 * Author : agpj
 */ 

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/twi.h>

// Deklaration Globale Variablen
#define EXTERN
#include "stdio.h"
#include "globals.h"
#include "uart.h"
#include "pid.h"
#include "i2c.h"
#include "uart_protocol.h"
#include "read_temperature.h"
#include "heat_profile.h"
#include "model.h"

// IIR Filter für Tiefpass Kammertemperatur - Flüssigkeitst.-Berechnung mgl.
// Noch auslagern in separate Funktion mit Pointerzugriff
//volatile float x[2] = {0, 0};
//volatile float y[2] = {0, 0};
//volatile float b[2] = {0, 0.00014};
//volatile float a[1] = {-0.9995001};
//volatile uint8_t n = 1;		// 2 Koeffizienten


int main(void)
{
	//******************************************
	//			Controller settings
	//******************************************
	
	// Outputs PortB, Pin4 OC0(Lüfter 1), Pin7 OC2(Heizung 1), Pin4 PB0(Lüfter 2)
	DDRB |= (1 << PB4) | (1 << PB7) | (1<<PB0);
	
	// Outputs PortE, Pin3 OC3A (Heizung 2), Pin5 OC3C (Servo)
	DDRE |= (1 << PE3) | (1<<PE5);
	
	// FET Lüfter 1 & 2 sperren 
	PORTB &= ~(1 << PB4);
	PORTB &= ~(1 << PB0);
		
	// LEDs Pin3 Rot Pin5 Grün
	DDRA |= (1 << PA3) | (1 << PA5);
	PORTA &= ~(1 << PA3) | ~(1 << PA5);
	
	
	// Timer2 Fast PWM Heizung 1, Prescaler 1024 -> 61 Hz (244 Hz macht Geräusche)
	TCCR2 |= (1 << WGM20) | (1 << WGM21) | (1 << COM21) | (1 << CS22) | (1 << CS20);
	OCR2 = 0;
		
	// Timer0 Fast PWM Heizung 2, Prescaler 1024 -> 61 Hz
	TCCR0 |= (1<<WGM00) | (1<<WGM01) |(1<<CS02) | (1<<CS01) | (1<<CS00);
	OCR0 = 0;
	TIMSK|=(1<<TOIE0)|(1<<OCIE0);
		
	// Timer1 CTC Sample Time Regler 10 ms
	TCCR1B |= (1 << WGM12) | (1 << CS11) | (1 << CS10);	// CTC Prescaler 64
	OCR1A = 2499;										// 100 Hz
	TIMSK |= (1 << OCIE1A);								// Output Compare A Match Interrupt Enable
	
	// Timer3 Servo, Fast PWM, ICR3=39999, Prescaler 8, 
	TCCR3A|= (1<<WGM31)|(1<<COM3C1)|(1<<COM3C0);// inverted mode
	TCCR3B|= (1<<WGM33)|(1<<WGM32)|(1<<CS31); // Prescaler 8 nach Datenblatt
	ICR3 = 39999;
	OCR3C = ICR3-3150;
	ETIMSK|=(1<<OCIE3C);
		
		
	UART_Init();
	i2c_init();
	
	sei();
	
	//******************************************
	//			Variablen
	//******************************************
	
	// Lokal
	
	uint8_t t_200ms = 0;					// nur alle 200 ms UART senden
	
	uint16_t t_act_1 = 0;					// aktuelle Zeit in sekunden
	uint32_t t_act_counter = 0;				// counter für 100 ms;
	//volatile uint16_t i_soll = 0;			// aktueller Index Sollwert
	
	//uint16_t a_max = 1000;				// maximaler Temperaturanstieg, 100 = 1 K/s beim Sollwertverlauf, anpassbar
	uint8_t durchlauf = 0;					// eintrittsvariable Temperaturprotokoll
	uint8_t warten = 0;
	//uint16_t model_temp = 0;
	uint8_t process = 0;
	uint32_t stell = 0;
	uint8_t anz_bytes = 0;
	uint8_t i = 0;
	uint8_t laengenbyte = 0;
	uint16_t CRC_check;
	
	// Fuer UART, 10 ms warten auf vollstaendigen Befehl
	uint8_t wait_command = 0;
	
	// Definition Globale Variablen
	// Interruptvariablen
	reset=0;								// Reset Befehl 
	rxn=0;									// UART buffer counter
	rx[UART_BUFFER];						// UART buffer
	rx_flag = 0;							// new entry in UART buffer
	pid_flag = 0;							// PID Interrupt timer flag
	t_10ms = 0;								// count up every 10 ms for cyclic uart
	
	t_set = 2500;							// Regler Sollwert 2500 = 25.00 °C
	t_set_2 = 2500;
	pwm_value_1 = 0;						// Leistung Heizwendel 48 V 0-255 (255 = max)
	pwm_value_2 = 0;						// Leistung Heizwendel 24 V 0-255 (255 = max)
	
	servo_open = 1650;						// Wert für Servo Stellung Deckel geöffnet
	servo_close = 3570;						// Wert für Servo Stellung Deckel geschlossen
	
	pid_active = 0;							// Regler ein/aus schalten
	t_protection = 0;						// Automatische abschaltung bei zu hoher Temperatur, muss über UART zurückgesetzt werden
	send_temp = 0;							// Temperatur senden über UART (=1)
	send_resp = 1;							// Empfangene Befehle über UART bestätigen (=1)
	protocol_active = 0;					// 1 = Leistungsverlauf, 2 = Lufttemperatur, 3 = Kammertemperatur
	fan_1_test = 0;							// Sperrung der Lüfterabschaltung in main()
	model_temp = 0;
	p_active = 0;							// Wartezeit aktiv
	terminal = 0;
	temp_thermo_1 = 0;
	temp_thermo_2 = 0;
				
	// PID 1 Regler Parameter
	pid_heat_1.Kp = eeprom_read_word(&PID_Kp_1);
	pid_heat_1.Ki = eeprom_read_word(&PID_Ki_1);			// 123 = 12.3
	pid_heat_1.t_awu = 30;
	pid_heat_1.Kd = eeprom_read_word(&PID_Kd_1);
	pid_heat_1.Kd_t1 = eeprom_read_word(&PID_Kd_t1_1);		// Zeitkonstante D-T1 < 1, 99 = 0.99
	
	// PID 2 Regler Parameter
	pid_heat_2.Kp = eeprom_read_word(&PID_Kp_2);
	pid_heat_2.Ki = eeprom_read_word(&PID_Ki_2);			// 123 = 12.3
	pid_heat_2.t_awu = 30;
	pid_heat_2.Kd = eeprom_read_word(&PID_Kd_2);
	pid_heat_2.Kd_t1 = eeprom_read_word(&PID_Kd_t1_2);		// Zeitkonstante D-T1 < 1, 99 = 0.99
	
	pid_heat_1.sample_time_ms = 10;							// 10 ms_Abtastzeit
	pid_heat_2.sample_time_ms = 10;	
	
	pid_heat_1.max_output = 255;							// obere Stellgrößenbeschränkung
	pid_heat_1.min_output = 0;								// untere Stellgrößenbeschränkung
	
	pid_heat_2.max_output = 255;							// obere Stellgrößenbeschränkung
	pid_heat_2.min_output = 0;								// untere Stellgrößenbeschränkung
	
	// Interne Werte für PID Regler
	pid_heat_1.error_sum = 0; // für den I-Anteil
	pid_heat_1.last_output_d = 0; // für den D-Anteil
	pid_heat_1.last_error = 0; // D-Anteil
	pid_heat_1.last_reference = 0;
	
	pid_heat_2.error_sum = 0;
	pid_heat_2.last_output_d = 0;
	pid_heat_2.last_error = 0;
	pid_heat_2.last_reference = 0;
	
	int altes_T_soll = 4000; // 1. Fall für das Temperaturprofil (wenn noch kein altes_T_soll vorhanden)
	
	//uint8_t cap_old_pos = 3150;
			
    while (1) 
    {
		// UART Protokoll zur Auswertung der empfangenen Befehle
		// Befehle sind immer 4 Zeichen lang
		// 1. Zeichen Befehl, Zeichen 2-4 Wert
		// Bei mehr als 4 empfangenen Zeichen UART Buffer löschen und Befehl verwerfen
		// Bei weniger als 4 Zeichen 10 ms warten auf weitere Zeichen ansonsten auch verwerfen
		
		
		if(rx_flag && (terminal == 1))												//vorige Empfangsroutine mit ASCII
		{
			rx_flag = 0;				// reset receive flag
						
			if (rxn == 4)
			{
				uart_protocol_receive();
			}
			else if (rxn > 4)
			{
				if (send_resp)
				{
					UART_Transmit_string((uint8_t *)"Error");
				}
				rxn = 0;									
			}	
		}
		
		//////////////neue Empfangsroutine Binär
		
		
		if(rx_flag && (terminal == 0))
		{
			rx_flag = 0;				// reset receive flag
			//UART_Transmit_string((uint8_t *)"rx flag gesetzt\n");
			
			anz_bytes = rxn;
			laengenbyte = rx[1]+4;
			
			if (anz_bytes == (laengenbyte))		// wenn die Empfangenen Bytes mit der Anzahl der Datenbytes(Längenbyteangabe)+Funktionbyte+längenbyte übereinstimmt
			{
				
				//UART_Transmit_string((uint8_t *)"stimmt überein\n");
				
				//CRC - Check				
				CRC_check = get_CRC(rx, (laengenbyte-2));

				if(CRC_check == (rx[laengenbyte-2]<<8 | rx[laengenbyte-1]))
				{
					//UART_Transmit_string((uint8_t *)"CRC ok\n");
					uart_protocol_receive();
				}
				else
				{
					//UART_Transmit_string((uint8_t *)"CRC falsch\n");
					UART_Transmit_bytes( rx[0] , 1 , 1 , 0 , 0 , 0 , 0 , 0 , 0 , 0, 0 );
				}
						
				//leeren des UART_buffers
				i=0;
				while(i<=9)
				{
					rx[i] = 0;
					i++;
				}
				
				rxn = 0;
			}
			else if (anz_bytes > (laengenbyte))
			{
				//UART_Transmit_string((uint8_t *)"Überlauf\n");
				UART_Transmit_bytes( rx[0] , 1 , 3 , 0 , 0 , 0 , 0 , 0 , 0 , 0, 0 );				
				rxn = 0;
				i=0;
				while(i<10)
				{
					rx[i] = 0;
					i++;
				}
				
			}
			//rxn = 0;
			
		}
		
		
		// 10 ms sample time for pid controller
		if(pid_flag)
		{
			pid_flag = 0;

			temp_thermo_1 = read_thermo_1();
			temp_thermo_2 = read_thermo_2();
			
			
		
			if((pid_active >= 1) && (t_protection == 0))
			{
				pwm_value_1 = pid(t_set,temp_thermo_1,&pid_heat_1);
				pwm_value_2 = pid(t_set_2, temp_thermo_2,&pid_heat_2);
			}
			
			// Übertemperaturabschaltung, muss über UART Befehl zurückgesetzt werden.
			if(temp_thermo_1 > 15000 || temp_thermo_2 > 15000)
			{
				t_protection = 1;
				pid_active = 0;
				pwm_value_1 = 0;
				pwm_value_2 = 0;
			}
			
			OCR2 = pwm_value_1;
			OCR0 = pwm_value_2;
			
			// Wenn nach 10 ms kein vollständiger UART Befehl erhalten wurden Buffer zurücksetzen
			if (wait_command == 1)
			{
				rxn = 0;
				if (send_resp)
				{
					//UART_Transmit_string((uint8_t *)"Befehl wird verworfen\n");
					//UART_Transmit_bytes( rx[0] , 1 , 2 , 0 , 0 , 0 );
					rx_flag = 0;
					wait_command = 0;
				}	

			}
			if ((rxn > 0) && (rxn < 3))
			{
				wait_command = 1;	
				//UART_Transmit_string((uint8_t *)"noch nicht vollständig \n");					
			}
			else
			{
				wait_command = 0;
			}
		}
					
		// Alle 100 ms aktuelle Temperatur über UART senden
		// und Ausführung der Temperatur/Leistungsprotokolle
		if(t_10ms >= 10)		
		{
			t_10ms = 0;
			
			model_temp = model_calculation(temp_thermo_1, temp_thermo_2, process);	// Abrufen der Modelltemperatur muss aller 100ms geschehen
			stell = model_PID(temp_value,model_temp,process);						// Abrufen der Stellgröße muss aller 100ms geschehen
						
			// Zyklisches senden der aktuellen Temperatur über UART
			// Nur alle 200 ms die Temperatur senden
			t_200ms++;
			
			if(send_temp && (t_200ms >1))
			{
				t_200ms = 0;
				send_temperature(t_set);					// Sollwertvorgabe
				UART_Transmit_char(' ');					// Leerzeichen
				send_temperature(temp_thermo_1);			// unterer Temperatursensor
				UART_Transmit_char(' ');					// Leerzeichen
				send_temperature(pwm_value_1*100);			// PWM-Wert nur für Messzwecke
				UART_Transmit_char(' ');					// Leerzeichen
				send_temperature(temp_thermo_2);			// unterer Temperatursensor
				UART_Transmit_char(' ');					// Leerzeichen
				send_temperature(pwm_value_2*100);			// PWM-Wert nur für Messzwecke
				UART_Transmit_char(' ');					// Leerzeichen
				send_temperature(process*100);				// PWM-Wert nur für Messzwecke
				UART_Transmit_char(' ');					// Leerzeichen
				send_temperature(model_temp);				// Temperatur des Modells
				UART_Transmit_char(' ');					// Leerzeichen
				send_temperature(temp_value);				// Zieltemperatur
				UART_Transmit_char(' ');					// Leerzeichen
				send_temperature(p_active*100);				// Zieltemperatur erreicht
				UART_Transmit_char(' ');					// Leerzeichen
				send_temperature(time_reached*100);				// Zieltemperatur erreicht
				UART_Transmit_char(0x0A);					// LF = Zeilenumbruch
			}
			
			// Leds schalten um Heizvorgang zu signalisieren
			// LEDs Pin3 Rot Pin5 Grün
			if (pwm_value_1>0 || pwm_value_2>0)
			{
				PORTA &= ~(1 << PA5);
				PORTA |= (1 << PA3);
				PORTB |= (1 << PB4);	
			}
			else
			{
				PORTA &= ~(1 << PA3);
				PORTA |= (1 << PA5);	
				if(fan_1_test==0)
				{
				PORTB &= ~(1 << PB4);	
				}
				
			}

			// Sollwertverlauf Lufttemperatur mit Haltezeit
			
			if (protocol_active == 2){										// Aktivierung der Abarbeitung des Temperaturprofils
			
				if (durchlauf == 0){										// einmaliger Abruf des ersten Profilschrittes bei Eintritt
					step_value_heat_profile();								//
				}															//
				durchlauf = 1;
				
				
				
				//wenn Sollwert der Raumtemperatur entspricht
				if(temp_value<=2500)
				{
					process = 0;
					t_set = stell;
					t_set_2 = t_set;
					OCR3C = ICR3 - servo_open;								// Deckel auf (ICR-1650)
					//p_active = 1;											// Wartezeit aktiv
				}
				
				// Kühlung nur in diesem einen Fall aktivieren
				else if(temp_value <= altes_T_soll && ((model_temp > temp_value) && p_active == 0)) // wenn neuer Sollwert kleiner als alter & Modelltemperatur > Sollwert & Sollwert noch nicht erreicht ist
				{
					process = 2;											// Angabe der äußeren Bedingung für Modellbildung
					t_set = stell;											// Sollwert obere Heizung
					//if(t_set<0)t_set=0;									// kann gelöscht werden wenn programm funktioniert
					t_set_2 = t_set;										// Sollwert untere Heizung
					OCR3C = ICR3 - servo_open;								// Deckel auf (ICR-1650)
					if(model_temp < ((((temp_value*35)/100)-1875) + temp_value))PORTB &= ~(1 << PB0);		// Lüfter 2 aus
					else PORTB |= (1 << PB0);								// Lüfter 2 an	
				}
				
				// in allen anderen Fällen Heizung aktiv und Deckel zu
				else if(temp_value > altes_T_soll)							// wenn neuer Sollwert größer als alter Sollwert
				{			
					pid_active = 1;											// Aktivierung der Heizungsregelung
					process = 1;											// Angabe der äußeren Bedingung für Modellbildung														
					t_set = stell;											// Sollwert obere Heizung
					//if(t_set>14500)t_set=14500;							// kann gelöscht werden wenn programm funktioniert
					t_set_2 = t_set;										// Sollwert untere Heizung
					OCR3C = ICR3 - servo_close;								// Position für Deckel zu (ICR3-3150)
					PORTB &= ~(1 << PB0);									// Lüfter 2 aus
				}
				
				//Deckel auf, um erneuten Anstieg nach Kühlphase zu verringern
				else 
				{
					process = 3;											// Angabe der äußeren Bedingung für Modellbildung
					t_set = stell;											// Sollwert obere Heizung
					t_set_2 = t_set;										// Sollwert untere Heizung
					OCR3C = ICR3-servo_open;								// Deckel auf (ICR-1650)
					if(model_temp > temp_value + 50)PORTB |= (1 << PB0);	// Lüfter 2 an wenn Solltemperatur um 0,5°C überschritten wird
					else PORTB &= ~(1 << PB0);								// Lüfter 2 aus	
				}		
				
				
				
				if((model_temp <= temp_value + 100 && model_temp >= temp_value - 100) || (process==0))		// erster wert fürs Abkühlen, zweiter fürs aufheizen
				{
					p_active = 1; // Wartezeit aktiv
					
					if(warten == 0){
						//UART_Transmit_string((uint8_t *)"Schritt:");
						//UART_Transmit_char(step_value+'0');
						//UART_Transmit_string((uint8_t *)" Temperatur erreicht\n");
					}
					
				}				
				
				if(p_active==1)
				{
					/*char strOut[25];
					sprintf(&strOut[0], "soll: %d", T_soll[i_soll]);
					UART_Transmit_string((uint8_t*)strOut);
					UART_Transmit_char(0x0A);				// LF = Zeilenumbruch
					*/
					warten = 1;
					t_act_counter++;			// t in 100 ms
					if (t_act_counter >= 10)	
					{
						t_act_1++;				// t in s
						t_act_counter = 0;
						if (t_act_1 >= time_value/10)	// Wartezeit abgeschlossen
						{
							altes_T_soll=temp_value;
							//UART_Transmit_string((uint8_t *)"Schritt:");
							//UART_Transmit_char(step_value+'0');
							//UART_Transmit_string((uint8_t *)" Haltezeit erreicht\n");
							step_value_heat_profile();
							//i_soll++;
							warten = 0;
							p_active = 0;
							t_act_1 = 0;
							
						}						
						
						if (step_value >= 254)	// Protokoll abgeschlossen -> P = 0, Regler deaktivieren
						{
							process = 0;					// Angabe der äußeren Bedingung für Modellbildung
							warten = 0;						// Wartevariable zurücksetzen
							protocol_active = 0;			// Rücksetzen der Durchführung des Temperaturprofiles
							durchlauf = 0;					// Rücksetzen der Eintrittsbedingung in Temperaturprofil
							pid_active = 0;					// Deaktivierung der Heizungsregelung
							pwm_value_1 = 0;				// PWM-Wert für Heizung = 0
							pwm_value_2 = 0;				// PWM-Wert für Heizung = 0
							PORTB &= ~(1 << PB0);			// Lüfter 2 aus	
							OCR3C = ICR3-1650;				// Deckel auf (ICR-1650)
						}	
					}
				}
				if (step_value == 254)
				{
					process = 0;					// Angabe der äußeren Bedingung für Modellbildung
					warten = 0;						// Wartevariable zurücksetzen
					protocol_active = 0;			// Rücksetzen der Durchführung des Temperaturprofiles
					durchlauf = 0;					// Rücksetzen der Eintrittsbedingung in Temperaturprofil
					pid_active = 0;					// Deaktivierung der Heizungsregelung
					pwm_value_1 = 0;				// PWM-Wert für Heizung = 0
					pwm_value_2 = 0;				// PWM-Wert für Heizung = 0
					PORTB &= ~(1 << PB0);			// Lüfter 2 aus
					OCR3C = ICR3-1650;				// Deckel auf (ICR-1650)
				}	
			}
			
			////////////// Einzelne Sollwertvorgaben mit Haltezeit - Sollwert wird nach erreichen der Haltezeit gehalten, Signalisierung, dass Haltezeit erreicht wurde
			
			if(protocol_active == 3)
			{
				if(new_value == 1)
				{
					new_value = 0;
					p_active = 0;
					time_reached = 0;
					t_act_counter = 0;
					warten = 0;
				}
				
				//wenn Sollwert der Raumtemperatur entspricht
				if(temp_value <= 3500)
				{
					//UART_Transmit_string((uint8_t *)" temp_value<=2500\n");
					pid_active = 1;											// Aktivierung der Heizungsregelung
					process = 0;
					t_set = stell;
					t_set_2 = t_set;
					OCR3C = ICR3 - servo_open;								// Deckel auf (ICR-1650)
					//p_active = 1;											// Wartezeit aktiv
				}
				
				// Kühlung nur in diesem einen Fall aktivieren
				else if(((temp_value < altes_T_soll) && ((model_temp > temp_value) && (p_active == 0)))||((temp_value == altes_T_soll)&&(process ==2))) // wenn: neuer Sollwert < alter & Modelltemperatur > Sollwert & Sollwert noch nicht erreicht ist
				{
					process = 2;											// Angabe der äußeren Bedingung für Modellbildung
					t_set = stell;											// Sollwert obere Heizung
					t_set_2 = t_set;										// Sollwert untere Heizung
					OCR3C = ICR3 - servo_open;								// Deckel auf (ICR-1650)
					if(model_temp < ((((temp_value*37)/100)-1875) + temp_value))PORTB &= ~(1 << PB0);		// Lüfter 2 aus (*35)
					
					else PORTB |= (1 << PB0);								// Lüfter 2 an	
				}
				
				// in allen anderen Fällen Heizung aktiv und Deckel zu
				else if((temp_value > altes_T_soll)	|| ((temp_value == altes_T_soll) && (process == 1)))						// wenn neuer Sollwert größer als alter Sollwert
				{			
					pid_active = 1;											// Aktivierung der Heizungsregelung
					process = 1;											// Angabe der äußeren Bedingung für Modellbildung														
					t_set = stell;											// Sollwert obere Heizung
					//if(t_set>14500)t_set=14500;							// kann gelöscht werden wenn programm funktioniert
					t_set_2 = t_set;										// Sollwert untere Heizung
					OCR3C = ICR3 - servo_close;								// Position für Deckel zu (ICR3-3150)
					PORTB &= ~(1 << PB0);									// Lüfter 2 aus
				}
				
				//Deckel auf, um erneuten Anstieg nach Kühlphase zu verringern
				else 
				{
					process = 3;											// Angabe der äußeren Bedingung für Modellbildung
					t_set = stell;											// Sollwert obere Heizung
					t_set_2 = t_set;										// Sollwert untere Heizung
					OCR3C = ICR3-servo_open;								// Deckel auf (ICR-1650)
					if(model_temp > temp_value + 100)PORTB |= (1 << PB0);	// Lüfter 2 an wenn Solltemperatur um 0,5°C überschritten wird
					else PORTB &= ~(1 << PB0);								// Lüfter 2 aus	
				}		
				
				
				
				if(((model_temp <= (temp_value + 100)) && (model_temp >= (temp_value - 100))) || (process==0))		// erster wert fürs Abkühlen, zweiter fürs aufheizen
				{
					p_active = 1; // Wartezeit aktiv
					
					if(warten == 0){
						//UART_Transmit_string((uint8_t *)"Schritt:");
						//UART_Transmit_char(step_value+'0');
						//UART_Transmit_string((uint8_t *)" Temperatur erreicht\n");
					}
					
				}				
				
				if(p_active==1)
				{
					warten = 1;
					
					if((t_act_counter >= time_value) && (time_reached == 1))
					{
						
					}
					else if (t_act_counter >= time_value && new_value == 0)
					{
						time_reached = 1;
						altes_T_soll = temp_value;
					}
					else
					{
						t_act_counter++;			// t in 100 ms
					}
				}
			}
			
		}	
	}
}

// Sample Time PID 10 ms
ISR (TIMER1_COMPA_vect) 
{
	pid_flag = 1;
	t_10ms++;
	
}

// UART receive Interrupt
ISR (USART1_RX_vect)
{
	while ( !(UCSR1A & (1<<RXC1)) );	// wait for data
	
	if (rxn == UART_BUFFER)				// reset buffer
	{
		rxn = 0;	
	}
	rx[rxn++] = UDR1;

	rx_flag = 1;						//	receive flag
}


// Heizung 2_Software_PWM 
//timer overflow

ISR( TIMER0_OVF_vect )
{
	PORTE |= (1<<PE3);		// set output
}

// compare match
ISR( TIMER0_COMP_vect )
{
	PORTE &= ~(1<<PE3);		// reset output
}

// Servo Hardware_PWM - Compare Interrupt ausgelöst, sobald Timer von 0->OCR gelaufen ist 
ISR (TIMER3_COMPC_vect) 
{
}





