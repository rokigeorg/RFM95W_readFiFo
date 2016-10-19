#include <string>
#include <stdint.h>
#include <stdio.h>
#include <errno.h>
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

/*******************************************************************************
 *
 * Configure these values!
 *
 *******************************************************************************/

// SX1276 - Raspberry connections
int ssPin = 6;
int dio0  = 7;
int RST   = 0;


#define RF95_FREQ 868100000 //868.1 MHz
#define RF95_SF 7       //SF 6 64 chips/symbol; SF 7 128 chips/symbol (default); SF 8 256 chips/symbol; SF 9 512 chips/symbol; SF 10 1024 chips/symbol; SF 11 2048 chips/symbol; SF 12 4096 chips/symbol
#define RF95_SYMB_TIMEOUT   0x64 //0x08
#define RF95_MAX_PAYLOAD_LENGTH 0x80
#define PAYLOAD_LENGTH 0x40
#define FREQ_HOP_PERIOD 0x00 //0x00 means freq hopping is turned off
//define LNA_MAX_GAIN //LowNoiseAmplifier Gain if the gateway is not set in automatic gain controll

#define SX1276_MODE_Continuos 0x85
/*******************************************************************************
 *
 *
 *******************************************************************************/



static const int CHANNEL =0;

// ########### global Variables ########
uint32_t  _freq = RF95_FREQ; // in Mhz! (868.1)
// Set spreading factor (SF7 - SF12)
uint8_t sf = RF95_SF;

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
volatile int8_t     _lastRssi =0;
/// Count of the number of bad messages (eg bad checksum etc) received
volatile uint16_t   _rxBad =0;
/// Count of the number of successfully transmitted messaged
volatile uint16_t   _rxGood;
/// Count of the number of bad messages (correct checksum etc) received
volatile uint16_t   _txGood;

// System error number indicator for error handling. If system fails at any point this val is set by the system to a an Error Number.
extern int errno;


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

void spiBurstRead(uint8_t * payload , uint8_t size)
{
    
    //uint8_t receivedCount = readRegister(RH_RF95_REG_13_RX_NB_BYTES);     //read register which tells the Number of received bytes
    uint8_t receivedbytes = size;
        
    for(int i = 0; i < receivedbytes; i++)
    {
        payload[i] = readRegister(RH_RF95_REG_00_FIFO);
        //_buf[i] = readRegister(REG_FIFO);
    }

}

void resetLoRaModul(){
    digitalWrite(RST, HIGH);
    delay(100);
    digitalWrite(RST, LOW);
    delay(100);
}

void handleInterrupt()
{
    // Read the interrupt register
    uint8_t irq_flags = readRegister(RH_RF95_REG_12_IRQ_FLAGS);
    if (_mode == RHModeRx && irq_flags & (RH_RF95_RX_TIMEOUT | RH_RF95_PAYLOAD_CRC_ERROR))
    {
    _rxBad++;
    }
    else if (irq_flags & RH_RF95_RX_DONE) //_mode == RHModeRx && irq_flags & RH_RF95_RX_DONE
    {
        // Have received a packet
        uint8_t len = readRegister(RH_RF95_REG_13_RX_NB_BYTES);
        
        // Reset the fifo read ptr to the beginning of the packet
        //writeRegister(RH_RF95_REG_0D_FIFO_ADDR_PTR, readRegister(RH_RF95_REG_10_FIFO_RX_CURRENT_ADDR));
        //writeRegister(RH_RF95_REG_0D_FIFO_ADDR_PTR, (readRegister(RH_RF95_REG_25_FIFO_RX_BYTE_ADDR) - readRegister(RH_RF95_REG_13_RX_NB_BYTES)));
        
        uint8_t fiFo_Addr = readRegister(RH_RF95_REG_10_FIFO_RX_CURRENT_ADDR);

        writeRegister(RH_RF95_REG_0D_FIFO_ADDR_PTR, fiFo_Addr);
        printf("FiFo Addr Ptr: %x\n", readRegister(RH_RF95_REG_0D_FIFO_ADDR_PTR));

        spiBurstRead(_buf, len);
        _bufLen = len;
        writeRegister(RH_RF95_REG_12_IRQ_FLAGS, 0xff); // Clear all IRQ flags
        
        // Remember the RSSI of this packet
        // this is according to the doc, but is it really correct?
        // weakest receiveable signals are reported RSSI at about -66
        _lastRssi = readRegister(RH_RF95_REG_1A_PKT_RSSI_VALUE) - 137;

    }
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

    //setModeIdle();
    writeRegister(RH_RF95_REG_12_IRQ_FLAGS, 0xff); // Clear all IRQ flags
}


bool setFrequency(uint32_t freq)
{

    uint64_t frf = ((uint64_t)freq << 19) / 32000000;
    writeRegister(RH_RF95_REG_06_FRF_MSB, (uint8_t)(frf>>16) );
    writeRegister(RH_RF95_REG_07_FRF_MID, (uint8_t)(frf>> 8) );
    writeRegister(RH_RF95_REG_08_FRF_LSB, (uint8_t)(frf>> 0) );
    return true;
}

void setModemRegisters(){
    writeRegister(RH_RF95_REG_1D_MODEM_CONFIG1, 0x72);
    writeRegister(RH_RF95_REG_1E_MODEM_CONFIG2, (sf<<4) | 0x04);
    writeRegister(RH_RF95_REG_26_MODEM_CONFIG3, 0x04);  //[7-4 bit: unused][3 bit: 0->static node / 1->mobile node] [2 bit: 0->LNA gain set by register LnaGain / 1->LNA gain set by the internal AGC loop][1-0 bit: reserved]
}
/* //Todo: settings combinationen zu einem bauen und an setRegisters übergeben 
selectModemSettings(){
    uint8_t modemConfigVals[3] = {0x00,0x00,0x00};

    //select BW and CR and Header (explicite / impliced)
    enum sf_t { SF7=7, SF8, SF9, SF10, SF11, SF12 };
    uint8_t combinations[16]={ 0x00, 0x01,0x02,0x03,0x04,0x05,0x06,0x07}
    //select BW and CR and Header (explicite / impliced)

}
*/

void setSymbTimeout(uint8_t timeOutPeriod){
    writeRegister(RH_RF95_REG_1F_SYMB_TIMEOUT_LSB,   timeOutPeriod);
}

void setMaxPayloadLength(uint8_t mPayloadLength){
    //Maximum payload length; if header payload length exceeds value a header CRC error is generated. Allows filtering of packet with a bad size.
    writeRegister(RH_RF95_REG_23_MAX_PAYLOAD_LENGTH,   mPayloadLength);
}

void setPayloadLength(uint8_t payll){
    writeRegister(RH_RF95_REG_22_PAYLOAD_LENGTH,  payll);
}

void setFrequencyHoppingPeriod(uint8_t fhhp){
    writeRegister(RH_RF95_REG_24_HOP_PERIOD,fhhp);
}

void setLnaGain(uint8_t lnaMaxGain){
    writeRegister(RH_RF95_REG_0C_LNA, LNA_MAX_GAIN);  // max lna gain
}

void printAllRegisters(){
        //registers I want to read
    uint8_t registers[] = {REG_FIFO, REG_OPMODE, RH_RF95_REG_02_RESERVED, RH_RF95_REG_03_RESERVED, RH_RF95_REG_04_RESERVED, RH_RF95_REG_05_RESERVED, RH_RF95_REG_06_FRF_MSB, RH_RF95_REG_07_FRF_MID, RH_RF95_REG_08_FRF_LSB , RH_RF95_REG_09_PA_CONFIG, RH_RF95_REG_0A_PA_RAMP, RH_RF95_REG_0B_OCP , RH_RF95_REG_0C_LNA , 
    RH_RF95_REG_0D_FIFO_ADDR_PTR, RH_RF95_REG_0E_FIFO_TX_BASE_ADDR,RH_RF95_REG_0F_FIFO_RX_BASE_ADDR , RH_RF95_REG_10_FIFO_RX_CURRENT_ADDR, RH_RF95_REG_11_IRQ_FLAGS_MASK,RH_RF95_REG_12_IRQ_FLAGS, RH_RF95_REG_13_RX_NB_BYTES, RH_RF95_REG_14_RX_HEADER_CNT_VALUE_MSB,
    RH_RF95_REG_15_RX_HEADER_CNT_VALUE_LSB, RH_RF95_REG_16_RX_PACKET_CNT_VALUE_MSB, RH_RF95_REG_17_RX_PACKET_CNT_VALUE_LSB,RH_RF95_REG_18_MODEM_STAT, 
    RH_RF95_REG_19_PKT_SNR_VALUE,RH_RF95_REG_1A_PKT_RSSI_VALUE,RH_RF95_REG_1B_RSSI_VALUE,RH_RF95_REG_1C_HOP_CHANNEL,RH_RF95_REG_1D_MODEM_CONFIG1, RH_RF95_REG_1E_MODEM_CONFIG2, RH_RF95_REG_1F_SYMB_TIMEOUT_LSB, RH_RF95_REG_20_PREAMBLE_MSB ,RH_RF95_REG_21_PREAMBLE_LSB, RH_RF95_REG_22_PAYLOAD_LENGTH, RH_RF95_REG_23_MAX_PAYLOAD_LENGTH ,RH_RF95_REG_24_HOP_PERIOD, RH_RF95_REG_25_FIFO_RX_BYTE_ADDR,RH_RF95_REG_26_MODEM_CONFIG3};

    string registerNames[] = {"REG_FIFO","REG_OPMODE", "RH_RF95_REG_02_RESERVED", "RH_RF95_REG_03_RESERVED", "RH_RF95_REG_04_RESERVED", "RH_RF95_REG_05_RESERVED", "RH_RF95_REG_06_FRF_MSB", "RH_RF95_REG_07_FRF_MID", "RH_RF95_REG_08_FRF_LSB" , "RH_RF95_REG_09_PA_CONFIG", "RH_RF95_REG_0A_PA_RAMP", "RH_RF95_REG_0B_OCP", "RH_RF95_REG_0C_LNA" , 
    "RH_RF95_REG_0D_FIFO_ADDR_PTR", "RH_RF95_REG_0E_FIFO_TX_BASE_ADDR","RH_RF95_REG_0F_FIFO_RX_BASE_ADDR" , "RH_RF95_REG_10_FIFO_RX_CURRENT_ADDR", "RH_RF95_REG_11_IRQ_FLAGS_MASK","RH_RF95_REG_12_IRQ_FLAGS", "RH_RF95_REG_13_RX_NB_BYTES", "RH_RF95_REG_14_RX_HEADER_CNT_VALUE_MSB",
    "RH_RF95_REG_15_RX_HEADER_CNT_VALUE_LSB", "RH_RF95_REG_16_RX_PACKET_CNT_VALUE_MSB", "RH_RF95_REG_17_RX_PACKET_CNT_VALUE_LSB","RH_RF95_REG_18_MODEM_STAT", 
    "RH_RF95_REG_19_PKT_SNR_VALUE","RH_RF95_REG_1A_PKT_RSSI_VALUE","RH_RF95_REG_1B_RSSI_VALUE","RH_RF95_REG_1C_HOP_CHANNEL","RH_RF95_REG_1D_MODEM_CONFIG1", "RH_RF95_REG_1E_MODEM_CONFIG2","RH_RF95_REG_1F_SYMB_TIMEOUT_LSB", "RH_RF95_REG_20_PREAMBLE_MSB","RH_RF95_REG_21_PREAMBLE_LSB", "RH_RF95_REG_22_PAYLOAD_LENGTH", "RH_RF95_REG_23_MAX_PAYLOAD_LENGTH" ,"RH_RF95_REG_24_HOP_PERIOD", "RH_RF95_REG_25_FIFO_RX_BYTE_ADDR","RH_RF95_REG_26_MODEM_CONFIG3"};


    int i;
    for(i=0; i < sizeof(registers);i++){

        cout << registerNames[i] << "Addr -> 0x";
        printf("%X", registers[i]);
        printf("%s",(char *)(": ") );
        printf("%X \n", readRegister(registers[i]) );
        
    }
}

void clearCharBuffer(char * arr){
    memset(&arr[0], 0, sizeof(arr));
}

void SetupLoRa()
{
    //Reset of the RFM95W
    //resetLoRaModul();
    digitalWrite(RST, HIGH);
    delay(100);
    digitalWrite(RST, LOW);
    delay(100);
    printf("SX1276 detected, starting.\n");

    digitalWrite(RST, LOW);
    delay(100);
    digitalWrite(RST, HIGH);
    delay(100);

    uint8_t version = readRegister(REG_VERSION);
    if (version == 0x12) {
        // sx1276
        printf("SX1276 detected, starting.\n");
        printf("Version: 0x%x\n",version);
    } else {
        printf("Unrecognized transceiver.\n");
        printf("Version: 0x%x\n",version);
       // exit(1);
    }
    
	// Set Continous Sleep Mode
   	writeRegister(RH_RF95_REG_01_OP_MODE, RH_RF95_LONG_RANGE_MODE);
    printf("Set in LONG_RANGE_MODE. REG_OPMODE value: %x \n", readRegister(REG_OPMODE));

    //set Frequency to 868.1 MHz by default
    printf("Set frequency to: %d Hz\n", _freq);
    setFrequency(_freq);
    if(errno != 0){
        fprintf(stderr, "Value of errno: %d\n", errno);
        perror("Error printed by perror after setFrequency");}

    setModemRegisters();
    if(errno != 0){
        fprintf(stderr, "Value of errno: %d\n", errno);
        perror("Error printed by perror after setModemRegisters");}

    //setTimeout RX operation time-out value expressed as number of symbols:
    setSymbTimeout(RF95_SYMB_TIMEOUT);
    if(errno != 0){
        fprintf(stderr, "Value of errno: %d\n", errno);
        perror("Error printed by perror after setSymbTimeout");}
    //set Max Payload length to filter for the right packetes
    setMaxPayloadLength(RF95_MAX_PAYLOAD_LENGTH);
    if(errno != 0){
        fprintf(stderr, "Value of errno: %d\n", errno);
        perror("Error printed by perror after setMaxPayloadLength");}

    setPayloadLength(PAYLOAD_LENGTH);
    if(errno != 0){
        fprintf(stderr, "Value of errno: %d\n", errno);
        perror("Error printed by perror after setPayloadLength");}

    setFrequencyHoppingPeriod(FREQ_HOP_PERIOD);
    if(errno != 0){
        fprintf(stderr, "Value of errno: %d\n", errno);
        perror("Error printed by perror after setFrequencyHoppingPeriod");}

    writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_BASE_AD));

    // Set Continous Receive Mode
    writeRegister(RH_RF95_REG_01_OP_MODE, SX1276_MODE_Continuos);
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


    // printAllRegisters
    printAllRegisters();

    char charBuffer[RH_RF95_MAX_PAYLOAD_LEN];

    while(1){

        //busy waiting, when Pin 0 goes HIGH it contiuous  
        while(digitalRead(dio0));
        //check if interrupt flag has been set
        //RFM95 Modul sets DIO0 pin (check pinlayout on the breakout board [Adafruit RFM9x -> D]to high when message arrives          
          if(digitalRead(dio0) == TRUE)
        {
            if(readRegister(RH_RF95_REG_12_IRQ_FLAGS) == RH_RF95_PACKET_RECEPTION_COMPLETE){
            
                printf("Mode: %x\n", readRegister(RH_RF95_REG_01_OP_MODE) );
                printf("Interrupt Register: %x\n", readRegister(RH_RF95_REG_12_IRQ_FLAGS));
                printf("\n");
                printf("Byte Addr of the last writen Rx Byte: %x\n", readRegister(RH_RF95_REG_25_FIFO_RX_BYTE_ADDR));            
                printf("Received Number of Bytes: %x\n", readRegister(RH_RF95_REG_13_RX_NB_BYTES));
                printf("FiFo Current Rx Addr: %x\n", readRegister(RH_RF95_REG_10_FIFO_RX_CURRENT_ADDR));
                printf("FiFo Addr Ptr: %x\n", readRegister(RH_RF95_REG_0D_FIFO_ADDR_PTR));
                printf("************************************\n");
                printf("Interrupt REG_12_IRQ_FLAGS %x\n", readRegister(RH_RF95_REG_12_IRQ_FLAGS));
            }
            handleInterrupt();
            //print buffer
            printf("Buffer: \n "); 
            static int i;
            
            // for(i=0; i < (sizeof(_buf) - 1);i++){

            //     printf("%x ", _buf[i]);     
            // }

            for(i=0; i < _bufLen;i++){

                charBuffer[i] = (char) _buf[i];
                printf("%c ", charBuffer[i]);     
            }
            //fill the charBuffer to all 0
            clearCharBuffer(charBuffer);
        }
        
    }

	
	
    return 0;
}
