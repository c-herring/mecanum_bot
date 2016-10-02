/*
 * MotorControl.h
 *
 * Created: 5/4/2016 7:43:43 PM
 *  Author: HeZ
 */ 


#ifndef MOTORCONTROL_H_
#define MOTORCONTROL_H_

#define DEBUGMode

// -------- Controller Parameters --------
#define WATCHDOG 4
int watchdog = 0;

// -------- Encoder interrupt parameters --------
int8_t lookup_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
uint8_t encA_val = 0;
uint8_t encB_val = 0;

// -------- TWI Defines --------
// TWI defines
#define TWI_FREQ 100000

// Slave Receiver Mode
#define TWI_SR_SLAW_ACK			0x60 // Own SLA+W has been received, ACK has been returned
#define TWI_SR_GEN_ACK			0x70 // General call address has been received, ACK has been returned
#define TWI_SR_DATA_ACK			0x80 // Previously addressed with own SLA+W, data received and ACK returned.
#define TWI_SR_DATA_NACK		0x88 // Previously addressed with own SLA+W, data received and NACK returned.
#define TWI_SR_GDATA_ACK		0x90 // Previously addressed with general call, data received and ACK returned.
#define TWI_SR_GDATA_NACK		0x98 // Previously addressed with general call, data received and NACK returned.
#define TWI_SR_STOP				0xA0 // A STOP or REPEATED START condition has been received whilst still addressed as slave

// Slave Transmitter Mode
#define TWI_ST_SLAR_ACK			0xA8 // Own SLA+R has been received, ACK has been returned
#define TWI_ST_DATA_ACK			0xB8 // Data has been transmitted, ACK has been received
#define TWI_ST_DATA_NACK		0xC0 // Data has been transmitted, NACK has been received
#define TWI_ST_END_ACK			0xC8 // Last data byte has been transmitted (TWEA = 0), ACK has been received


#define TWISendACK()		(TWCR |= (1<<TWINT)|(1<<TWEA))
#define TWISendNACK()		(TWCR |= (1<<TWINT)|(0<<TWEA))
//#define TWISendACK()  TWCR = (1<<TWEN)| (0<<TWIE)|(1<<TWINT)| (1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)| (0<<TWWC)
//#define TWISendNACK()  TWCR = (1<<TWEN)| (0<<TWIE)|(1<<TWINT)| (0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)| (0<<TWWC)


#define TWI_INTERRUPTS 0

// Get TWI status
#define TWI_STATUS	(TWSR & 0xF8)

// Transmit and receive buffers
	// Max TX length: 32bit int is 10 digits. PA%ldB%ldVA%ldb%ld\0\r\n
	// = 2 + 10 + 1 + 10 + 2 + 10 + 1 + 10 + 3(?)= 49
#define TXBufLen 50
#define RXbufLen 50
typedef struct {
	char TXbuff[TXBufLen]; 
	char RXbuff[RXbufLen];
	uint8_t TXindex;
	uint8_t RXindex;
	uint8_t TXlen;
	} TWIBuffers;
TWIBuffers TWIBuf;

typedef enum
{
	OUT_V,
	OUT_P,
	OUT_ALL,
	OUT_PID,
	OUT_test
} OutputModes;
OutputModes outputMode;

// -------- Function Declarations --------
void TWIInit(void);		// Initialize TWI Comms with default address of 4
void handleTWI(void);	// Handle TWI transmissions
void parseCommand(void);// Parse a command string
void TWIInit(uint8_t TWIaddr);	// Initialize TWI Comms
void constructOutStr(void);		// Construct the output string in TXBuff




#endif /* MOTORCONTROL_H_ */