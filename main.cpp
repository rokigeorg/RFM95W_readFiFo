#include <string>
#include <stdint.h>
#include <stdio.h>
#include <sys/types.h>
//#include <sys/socket.h>
//#include <arpa/inet.h>
#include <string.h>
#include <iostream>
#include <cstdlib>
//#include <sys/time.h>
#include <cstring>

#include <sys/ioctl.h>
//#include <net/if.h>

#include <wiringPi.h>
#include <wiringPiSPI.h>

#include "RFM95registers.h"

using namespace std;

// SX1276 - Raspberry connections
int ssPin = 6;
int dio0  = 7;
int RST   = 0;

static const int CHANNEL =0;

// ########### global Variables ########

typedef enum
{
RHModeInitialising = 0, ///< Transport is initialising. Initial default value until init() is called..
RHModeSleep,            ///< Transport hardware is in low power sleep mode (if supported)
RHModeIdle,             ///< Transport is idle.
RHModeTx,               ///< Transport is in the process of transmitting a message.
RHModeRx                ///< Transport is in the process of receiving a message.
} RHMode;

/// The current transport operating mode
volatile RHMode     _mode;
uint8_t _rxBad = 0;
int8_t _lastRssi = 0;
/// Index of next interrupt number to use in _deviceForInterrupt
static uint8_t      _interruptCount;
/// The configured interrupt pin connected to this instance
uint8_t             _interruptPin;
/// The index into _deviceForInterrupt[] for this device (if an interrupt is already allocated)
/// else 0xff
uint8_t             _myInterruptIndex;
/// Number of octets in the buffer
volatile uint8_t    _bufLen;
/// The receiver/transmitter buffer
uint8_t             _buf[RH_RF95_MAX_PAYLOAD_LEN];
/// True when there is a valid message in the buffer
volatile bool       _rxBufValid;
/// The value of the last received RSSI value, in some transport specific units
volatile int8_t     _lastRssi;
/// Count of the number of bad messages (eg bad checksum etc) received
volatile uint16_t   _rxBad;
/// Count of the number of successfully transmitted messaged
volatile uint16_t   _rxGood;
/// Count of the number of bad messages (correct checksum etc) received
volatile uint16_t   _txGood;


//################# copyed from single channel gateway Copyright (c) 2015 Thomas Telkamp ##########################     

void die(const char *s)
{
    
    exit(1);
}

void selectreceiver()
{
    digitalWrite(ssPin, LOW);
}

void unselectreceiver()
{
    digitalWrite(ssPin, HIGH);
}


uint8_t readRegister(uint8_t addr)
{
    unsigned char spibuf[2];

    selectreceiver();
    spibuf[0] = addr & 0x7F;
    spibuf[1] = 0x00;
    wiringPiSPIDataRW(CHANNEL, spibuf, 2);
    unselectreceiver();

    return spibuf[1];
}


void writeRegister(uint8_t addr, uint8_t value)
{
    unsigned char spibuf[2];

    spibuf[0] = addr | 0x80;
    spibuf[1] = value;
    selectreceiver();
    wiringPiSPIDataRW(CHANNEL, spibuf, 2);

    unselectreceiver();
}
//################# end of copyed from single channel gateway ##########################  
//########################################################################################################   
void setModeIdle()
{
    if (_mode != RHModeIdle)
    {
        writeRegister(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_STDBY);
        _mode = RHModeIdle;
    }
}

void spiBurstRead(uint8_t *payload)
{
    
    uint8_t receivedCount = readRegister(RH_RF95_REG_13_RX_NB_BYTES);     //read register which tells the Number of received bytes
    receivedbytes = receivedCount;
        
    for(int i = 0; i < receivedCount; i++)
    {
        payload[i] = readRegister(REG_FIFO);
    }
    return 0;
}

void handleInterrupt()
{
    // Read the interrupt register
    uint8_t irq_flags = readRegister(RH_RF95_REG_12_IRQ_FLAGS);
    if (_mode == RHModeRx && irq_flags & (RH_RF95_RX_TIMEOUT | RH_RF95_PAYLOAD_CRC_ERROR))
    {
    _rxBad++;
    }
    else if (_mode == RHModeRx && irq_flags & RH_RF95_RX_DONE)
    {
        // Have received a packet
        uint8_t len = readRegister(RH_RF95_REG_13_RX_NB_BYTES);
        
        // Reset the fifo read ptr to the beginning of the packet
        writeRegister(RH_RF95_REG_0D_FIFO_ADDR_PTR, readRegister(RH_RF95_REG_10_FIFO_RX_CURRENT_ADDR));
        spiBurstRead(&_buf);
        _bufLen = len;
        writeRegister(RH_RF95_REG_12_IRQ_FLAGS, 0xff); // Clear all IRQ flags
        
        // Remember the RSSI of this packet
        // this is according to the doc, but is it really correct?
        // weakest receiveable signals are reported RSSI at about -66
        _lastRssi = readRegister(RH_RF95_REG_1A_PKT_RSSI_VALUE) - 137;
    /*    
        // We have received a message.
            validateRxBuf(); 
    if (_rxBufValid)
        setModeIdle(); // Got one 
    }
    else if (_mode == RHModeTx && irq_flags & RH_RF95_TX_DONE)
    {
    _txGood++;
    setModeIdle();
    }
    */

    setModeIdle();
    writeRegister(RH_RF95_REG_12_IRQ_FLAGS, 0xff); // Clear all IRQ flags
}


void SetupLoRa()
{
    	digitalWrite(RST, HIGH);
    	delay(100);
    	digitalWrite(RST, LOW);
    	delay(100);

	printf("SX1276 detected, starting.\n");
        
	// sx1276?
        digitalWrite(RST, LOW);
        delay(100);
        digitalWrite(RST, HIGH);
        delay(100);
        uint8_t version = readRegister(REG_VERSION);
        if (version == 0x12) {
            // sx1276
            printf("SX1276 detected, starting.\n");
        } else {
            printf("Unrecognized transceiver.\n");
            //printf("Version: 0x%x\n",version);
            exit(1);
        	}
    	

	// Set Continous Sleep Mode
   	writeRegister(REG_OPMODE, RH_RF95_LONG_RANGE_MODE);
    printf("Set in LONG_RANGE_MODE.\n");
}



int main (void){
	printf("Start main function.");
	wiringPiSetup();
	pinMode(ssPin, OUTPUT);
    	pinMode(dio0, INPUT);
    	pinMode(RST, OUTPUT);

	//int fd = 
    	wiringPiSPISetup(CHANNEL, 500000);
    	//cout << "Init result: " << fd << endl;

	SetupLoRa();	
	printf("Start WiringPi function.\n");
	

    //check if interrupt flag has been set
    //RFM95 Modul sets DIO0 pin (check pinlayout on the breakout board [Adafruit RFM9x -> D]to high when message arrives 
      if(digitalRead(dio0) == TRUE)
    {
        handleInterrupt();
    }


	int i;
	for(i=0; i < sizeof(registers);i++){

        cout << registerNames[i] << "Addr -> 0x";
		printf("%X", registers[i]);
		printf("%s",(char *)(": ") );
		printf("%X \n", readRegister(registers[i]) );
		
	}
	
	//while(1);
 

	return 0;
}
