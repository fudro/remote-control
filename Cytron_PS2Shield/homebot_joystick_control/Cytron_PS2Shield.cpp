/*
Original written by:
            Cytron Technologies

Modified:
  29/06/15  Idris, Cytron Technologies    - Point to IDE SoftwareSerial
                                          - Restructure the code style to follow standard Arduino library
*/

#include "HardwareSerial.h"
#include "Cytron_PS2Shield.h"

Cytron_PS2Shield::Cytron_PS2Shield(uint8_t rxpin, uint8_t txpin)
{
  _txpin = txpin;
  _rxpin = rxpin;
}

Cytron_PS2Shield::Cytron_PS2Shield()
{
  _txpin = 16;      //Set digital pin equivalents for MegaPi Serial2 Rx and Tx 
  _rxpin = 17;
}

void Cytron_PS2Shield::begin(uint32_t baudrate)
{
	if(_rxpin == 17 && _txpin == 16)    //Set digital pin equivalents for MegaPi Serial2 Rx and Tx
	{
		hardwareSerial = true;
		Serial2.begin(baudrate);    //Start the Serial2 channel
		while(!Serial2);
	}
	
	else
	{
		hardwareSerial = false;
		pinMode(_rxpin, INPUT);
		pinMode(_txpin, OUTPUT);
		PS2Serial = new SoftwareSerial(_rxpin, _txpin);
		PS2Serial->begin(baudrate);
	}
	pinMode(A1, OUTPUT);
	digitalWrite(A1, HIGH);
}

void Cytron_PS2Shield::write(uint8_t data)
{
	if(hardwareSerial)
	{
		while(Serial2.available() > 0) {
			Serial2.read();
		}
		Serial2.write(data);
		Serial2.flush();		// Wait for all data transmitted
	}
	else
	{
		while(PS2Serial->available() > 0) {	
			PS2Serial->read();
		}
		PS2Serial->write(data);
	}
}

uint8_t Cytron_PS2Shield::read(void)
{
	uint8_t rec_data; 
	long waitcount = 0; 
	
	if(hardwareSerial)
	{
		while(true)
		{
			if(Serial2.available() > 0)
			{
				rec_data = Serial2.read();
				SERIAL_ERR = false;
				return(rec_data);
			}
			waitcount++;
			if(waitcount > 50000)
			{
				SERIAL_ERR = true; 
				return(0xFF); 
			}
		}
	}
	else
	{
		while(true)
		{
			if(PS2Serial->available() > 0)
			{
				rec_data = PS2Serial->read();
				SERIAL_ERR = false; 
				return(rec_data);
			}
			waitcount++; 
			if(waitcount > 50000)
			{
				SERIAL_ERR = true; 
				return (0xFF); 
			}
		}
	}
}

uint8_t Cytron_PS2Shield::readButton(uint8_t key)
{
	if(!hardwareSerial) PS2Serial->listen();
	write(key);
	return read();
}

boolean Cytron_PS2Shield::readAllButton()
{
	uint8_t nbyte; 
	uint32_t waitcount;
	
	write(PS2_BUTTON_JOYSTICK);
	
	if(hardwareSerial)
	{		
		nbyte = Serial2.readBytes(ps_data, 6);
		
		if(nbyte == 6) return(true); 
		else return (false);
	}
		
	else
	{	
		waitcount = 0; 
		while(PS2Serial->available() < 6)
		{		
			waitcount++; 
			if(waitcount > 50000) {				
				return (false); 
			}
		}		
		for(int i = 0; i < 6; i++) {
			ps_data[i] = PS2Serial->read();
		}
		return(true);	
	}
}

void Cytron_PS2Shield::vibrate(uint8_t motor, uint8_t value)
{
	uint8_t _motor;

	if(motor == 1) _motor = PS2_MOTOR_1; 
	else _motor = PS2_MOTOR_2;
	
	write(_motor);
	write(value); 
}

void Cytron_PS2Shield::reset(uint8_t reset)
{
	if(reset == 1) digitalWrite(A1, LOW);
	else digitalWrite(A1, HIGH);
}
