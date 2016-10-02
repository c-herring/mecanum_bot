/*
 * MotorControl.cpp
 *
 * Created: 5/4/2016 7:28:41 PM
 * Author : HeZ
 * AVEDUDE String: avrdude -v -pm328p -carduino -b57600 -PCOM7 -D -U flash:w:MotorControl.hex 
 * Initial PID: p140i2d0o20000
 */ 

#include <avr/io.h>
#include "MotorControl.h"
#include "MotorPID.h"
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <avr/eeprom.h>

MotorPID::PID velocityPID;
// EEPROM Values
MotorPID::PID EEMEM PID_v; /*= {	.Kp = 140,
								.Ki = 2,
								.Kd = 0,
								.K0 = 2000}; */// Velocity PID structure

// Defining the new operator
// http://www.avrfreaks.net/forum/avr-c-micro-how?name=PNphpBB2&file=viewtopic&t=59453
void * operator new(size_t size);
void operator delete(void * ptr);
void * operator new(size_t size)
{
	return malloc(size);
}

void operator delete(void * ptr)
{
	free(ptr);
}


// Create motor objects
static MotorPID *m_motorA;
static MotorPID *m_motorB;

int main(void)
{
	// \/  \/  \/  \/ Setup \/  \/  \/  \/
	// -------- Initialize TWI --------
	TWIInit();
	
	// -------- Pin Registers --------
	// Set all of PORTC to Input
	DDRC = 0x00;
	// Set PORTB and D to outputs
	DDRB = 0xFF;
	DDRD = 0xFF;
	
	// -------- Interrupts --------
	sei();
	// Pin change interrupts
	PCICR = (1 << PCIE1); // Enable interrupts for PCIE2 [14:8]
	PCMSK1 = (1 << PCINT8) | (1 << PCINT9) | (1 << PCINT10) | (1 << PCINT11); // Enable interrupts for PC0 PC1 PC2 and PC3 (PCINT8, 9, 10, 11)
	
	// Pin change interrupts
	//PCICR = (1 << PCIE2); // Enable interrupts for PCIE2 [23:16]
	//PCMSK2 = (1 << PCINT18) | (1 << PCINT19); // Enable interrupts for PD2 and PD3

	
	
	// -------- PID Loop Timer --------
	// Set Timer 1 clock prescaler to 1/64
	TCCR1B = (0 << CS12) | (1 << CS11) | (1 << CS10);
	
	// -------- PWM timer Setup --------
	TCCR0A =	(0 << COM0A0) | (1 << COM0A1) | (0 << COM0B0) | (1 << COM0B1) | // Non inverting mode
	(1 << WGM00) | (1 << WGM01); // Fast PWM mode
	OCR0A = 128;
	OCR0B = 0;
	TCCR0B =	(0 << CS02) | (0 << CS01) | (1 << CS00); // Start timer with 1/64 pre scaler
	
	
	// -------- Read PID Parameters into RAM --------
	// PID parameters
	eeprom_read_block(&velocityPID, &PID_v, sizeof(MotorPID::PID));
	/*velocityPID.Kp = 140;
	velocityPID.Ki = 2;
	velocityPID.Kd = 0;
	velocityPID.K0 = 20000;*/
	
    // Create motor objects
    m_motorA = new MotorPID(&OCR0A, &PORTB, 0x01, 0x02, velocityPID);
    m_motorB = new MotorPID(&OCR0B, &PORTB, 0x04, 0x08, velocityPID);
	
	
    while (1) 
    {
		// -------- 50ms PID Loop --------
		if (TCNT1 >= 12500) // > 50ms PID loop
		{
			if (++watchdog > WATCHDOG)
			{
				m_motorA->motor.Velocity = 0;
				m_motorB->motor.Velocity = 0;
			}
				
			// Take a snapshot of each motor status
			m_motorA->snapshot(TCNT1);
			m_motorB->snapshot(TCNT1);
			// Reset the PID timer once snapshots are taken
			TCNT1 = 0;
			
			// Spin PID for each motor
			m_motorA->spin();
			m_motorB->spin();
		}


		// -------- TWI check --------
		if (TWCR & (1 << TWINT)) // If TWINT is set then we need to handle some TWI comms
		{			
			constructOutStr();
			handleTWI();
		}
    }
}

// Encoder ISR
ISR(PCINT1_vect)
{
	encA_val = encA_val << 2;
	encA_val = encA_val | ((PINC & 0b0011) >> 0);
	encB_val = encB_val << 2;
	encB_val = encB_val | ((PINC & 0b1100) >> 2);
	
	m_motorA->motor.counts += lookup_table[encA_val & 0b1111];
	m_motorB->motor.counts += lookup_table[encB_val & 0b1111];
}

void TWIInit()
{
	// Initialize with hard coded slave address of 4
	TWIInit(0x08);
}

void TWIInit(uint8_t TWIaddr)
{
	// Set pre-scalers (no pre-scaling)
	TWSR = 0;
	// Set bit rate
	TWBR = ((F_CPU / TWI_FREQ) - 16) / 2;
	// Set slave address
	TWAR = (TWIaddr << 1) & 0xFE;
	
	// Enable TWI
	TWCR =	(1 << TWEN) | // Enable TWI
	(1 << TWEA) | // Allow slave to ACK it's address
	(1 << TWINT) | // Clear the interrupt flag
	(TWI_INTERRUPTS << TWIE) | // If we are using interrupts, enable them.
	(0 << TWSTA) | (0 << TWSTO) | (0 << TWWC);
}

void handleTWI()
{
	switch(TWI_STATUS)
	{
		// ------------SLAVE TRANSMITTER------------
		case TWI_ST_SLAR_ACK: // We have been addressed, OK
		TWIBuf.TXindex = 0;
		case TWI_ST_DATA_ACK: // We have transmitted a data byte and returned an ACK
		TWDR = (TWIBuf.TXindex >= TWIBuf.TXlen) ? '\0' : TWIBuf.TXbuff[TWIBuf.TXindex++]; // If we have already transmitted the message but master wants more, send null characters
		TWISendACK();
		break;
		
		case TWI_ST_DATA_NACK: // We have sent a data byte and received NACK response, we are done.
		case TWI_ST_END_ACK: // We never indicate that last data byte has been transmitted. We just send junk. But this is here anyway
		TWIBuf.TXindex = 0;
		TWISendACK(); // Acknowledge
		break;

		
		// ------------SLAVE RECEIVER------------
		case TWI_SR_SLAW_ACK: // We have been addressed, OK
		// Set our RX buffer index to 0
		TWIBuf.RXindex = 0;
		TWISendACK();
		break;
		
		case TWI_SR_DATA_ACK: // We have received a byte and responded with an ACK
		// Save the byte into the TX buffer
		TWIBuf.RXbuff[TWIBuf.RXindex++] = TWDR;
		// If we have already received the maximum number of bytes, responds with a NACK
		TWIBuf.RXindex >= RXbufLen ? TWISendNACK() : TWISendACK();
		break;
		
		case TWI_SR_STOP: // We receive a STOP or REPEATED START signal - transmission master has finished
		TWIBuf.RXbuff[TWIBuf.RXindex++] = TWDR;
		case TWI_SR_DATA_NACK: // We have received maximum data bytes, told master to piss off.
		parseCommand(); // Parse the command we received
		TWISendACK();
		break;
		
		
		
		default:
		TWISendACK(); // Acknowledge
		break;
		
	}
}

void parseCommand()
{
	if (TWIBuf.RXbuff[0] == '?') // Output mode change request
	{		
		switch (TWIBuf.RXbuff[1])
		{
			case 'V': // Requesting all velocities
			outputMode = OUT_V;
			constructOutStr();
			break;
			case 'P': // Requesting all encoder positions
			outputMode = OUT_P;
			constructOutStr();
			break;
			case '?':
			outputMode = OUT_ALL;
			constructOutStr();
			break;
			case '^':
			outputMode = OUT_PID;
			constructOutStr();
			break;
			case 't': // Test
			outputMode = OUT_test;
			constructOutStr();
			break;
		}
	}
	else // Otherwise we have an input
	{
		switch (TWIBuf.RXbuff[0])
		{
			case 'V': // Setting all velocities
				sscanf(TWIBuf.RXbuff+1, "A%liB%li", &(m_motorA->motor.Velocity), &(m_motorB->motor.Velocity));
				//m_motorA->motor.Velocity = (TWIBuf.RXbuff[1] << 8) | (TWIBuf.RXbuff[2]);
				watchdog = 0;
				break;
			case '^': // Setting PID values
				sscanf(TWIBuf.RXbuff+1, "p%ui%ud%uo%u", &(velocityPID.Kp), &(velocityPID.Ki), &(velocityPID.Kd), &(velocityPID.K0));
				m_motorA->setVelPID(velocityPID);
				m_motorB->setVelPID(velocityPID);
				break;
			case '*': // Save PID to EEPROM
				//const void* velocityPID2 = (const void*)&velocityPID;
				eeprom_write_block(&velocityPID, &PID_v, sizeof(MotorPID::PID));
				break;
		}
		
	}
}

void constructOutStr()
{
	switch (outputMode)
	{
		case OUT_V:
			//memcpy(TWIBuf.TXbuff, &(m_motorA->motor.vel), 4);
			//TWIBuf.TXlen = 4;
			sprintf(TWIBuf.TXbuff, "VA%ldB%ld", m_motorA->motor.vel, m_motorB->motor.vel);
			TWIBuf.TXlen = strlen(TWIBuf.TXbuff);
			break;
		case OUT_P:
			sprintf(TWIBuf.TXbuff, "PA%ldB%ld", m_motorA->motor.Pos, m_motorB->motor.Pos);
			TWIBuf.TXlen = strlen(TWIBuf.TXbuff);
			//memcpy(TWIBuf.TXbuff, &(m_motorA->motor.Pos), 4);
			//TWIBuf.TXlen = 4;
			break;
		case OUT_ALL:
			sprintf(TWIBuf.TXbuff, "PA%ldB%ldVA%ldB%ld", m_motorA->motor.Pos, m_motorB->motor.Pos, m_motorA->motor.vel, m_motorB->motor.vel);
			TWIBuf.TXlen = strlen(TWIBuf.TXbuff);
			break;
		case OUT_PID:
			sprintf(TWIBuf.TXbuff, "p%u\ti%u\td%u\to%u", velocityPID.Kp, velocityPID.Ki, velocityPID.Kd, velocityPID.K0);
			//sprintf(TWIBuf.TXbuff, "p%ui%u", velocityPID.Kp, velocityPID.Ki);
			TWIBuf.TXlen = strlen(TWIBuf.TXbuff);
			break;
		case OUT_test:
			sprintf(TWIBuf.TXbuff, "x%ldx%ld", m_motorA->motor.Velocity, m_motorB->motor.Velocity);
			TWIBuf.TXlen = strlen(TWIBuf.TXbuff);
		/*
			m_motorA->motor.Velocity = 22;
			memcpy(TWIBuf.TXbuff, &(m_motorA->motor.Velocity), 2);
			TWIBuf.TXlen = 2;*/
			break;
		default:
			break;
	}
}