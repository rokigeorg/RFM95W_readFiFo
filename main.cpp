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

using namespace std;

// SX1276 - Raspberry connections
int ssPin = 6;
int dio0  = 7;
int RST   = 0;

static const int CHANNEL =0;


//################# copyed from single channel gateway Copyright (c) 2015 Thomas Telkamp ########################## 
	

#define REG_FIFO                    0x00
#define REG_FIFO_ADDR_PTR           0x0D
#define REG_FIFO_TX_BASE_AD         0x0E
#define REG_FIFO_RX_BASE_AD         0x0F
#define REG_RX_NB_BYTES             0x13
#define REG_OPMODE                  0x01
#define REG_FIFO_RX_CURRENT_ADDR    0x10
#define REG_IRQ_FLAGS               0x12
#define REG_DIO_MAPPING_1           0x40
#define REG_DIO_MAPPING_2           0x41
#define REG_MODEM_CONFIG            0x1D
#define REG_MODEM_CONFIG2           0x1E
#define REG_MODEM_CONFIG3           0x26
#define REG_SYMB_TIMEOUT_LSB  		0x1F
#define REG_PKT_SNR_VALUE			0x19
#define REG_PAYLOAD_LENGTH          0x22
#define REG_IRQ_FLAGS_MASK          0x11
#define REG_MAX_PAYLOAD_LENGTH 		0x23
#define REG_HOP_PERIOD              0x24
#define REG_SYNC_WORD				0x39
#define REG_VERSION	  				0x42

#define PAYLOAD_LENGTH              0x40

// LOW NOISE AMPLIFIER
#define REG_LNA                     0x0C
#define LNA_MAX_GAIN                0x23
#define LNA_OFF_GAIN                0x00
#define LNA_LOW_GAIN		    	0x20

// CONF REG
#define REG1                        0x0A
#define REG2                        0x84


#define SX72_MC1_LOW_DATA_RATE_OPTIMIZE  0x01 // mandated for SF11 and SF12

// FRF
#define        REG_FRF_MSB              0x06
#define        REG_FRF_MID              0x07
#define        REG_FRF_LSB              0x08

#define        FRF_MSB                  0xD9 // 868.1 Mhz
#define        FRF_MID                  0x06
#define        FRF_LSB                  0x66

#define BUFLEN 2048  //Max length of buffer

#define PROTOCOL_VERSION  1
#define PKT_PUSH_DATA 0
#define PKT_PUSH_ACK  1
#define PKT_PULL_DATA 2
#define PKT_PULL_RESP 3
#define PKT_PULL_ACK  4

#define TX_BUFF_SIZE  2048
#define STATUS_SIZE	  1024

//#######################################################################################

//############## Copied from Radio Head Library (OpenSource) / RFM95.h ########
// Register names (LoRa Mode, from table 85)


#define RH_RF95_REG_00_FIFO                                0x00
#define RH_RF95_REG_01_OP_MODE                             0x01
#define RH_RF95_REG_02_RESERVED                            0x02
#define RH_RF95_REG_03_RESERVED                            0x03
#define RH_RF95_REG_04_RESERVED                            0x04
#define RH_RF95_REG_05_RESERVED                            0x05
#define RH_RF95_REG_06_FRF_MSB                             0x06
#define RH_RF95_REG_07_FRF_MID                             0x07
#define RH_RF95_REG_08_FRF_LSB                             0x08
#define RH_RF95_REG_09_PA_CONFIG                           0x09
#define RH_RF95_REG_0A_PA_RAMP                             0x0a
#define RH_RF95_REG_0B_OCP                                 0x0b
#define RH_RF95_REG_0C_LNA                                 0x0c
#define RH_RF95_REG_0D_FIFO_ADDR_PTR                       0x0d
#define RH_RF95_REG_0E_FIFO_TX_BASE_ADDR                   0x0e
#define RH_RF95_REG_0F_FIFO_RX_BASE_ADDR                   0x0f
#define RH_RF95_REG_10_FIFO_RX_CURRENT_ADDR                0x10
#define RH_RF95_REG_11_IRQ_FLAGS_MASK                      0x11
#define RH_RF95_REG_12_IRQ_FLAGS                           0x12
#define RH_RF95_REG_13_RX_NB_BYTES                         0x13
#define RH_RF95_REG_14_RX_HEADER_CNT_VALUE_MSB             0x14
#define RH_RF95_REG_15_RX_HEADER_CNT_VALUE_LSB             0x15
#define RH_RF95_REG_16_RX_PACKET_CNT_VALUE_MSB             0x16
#define RH_RF95_REG_17_RX_PACKET_CNT_VALUE_LSB             0x17
#define RH_RF95_REG_18_MODEM_STAT                          0x18
#define RH_RF95_REG_19_PKT_SNR_VALUE                       0x19
#define RH_RF95_REG_1A_PKT_RSSI_VALUE                      0x1a
#define RH_RF95_REG_1B_RSSI_VALUE                          0x1b
#define RH_RF95_REG_1C_HOP_CHANNEL                         0x1c
#define RH_RF95_REG_1D_MODEM_CONFIG1                       0x1d
#define RH_RF95_REG_1E_MODEM_CONFIG2                       0x1e
#define RH_RF95_REG_1F_SYMB_TIMEOUT_LSB                    0x1f
#define RH_RF95_REG_20_PREAMBLE_MSB                        0x20
#define RH_RF95_REG_21_PREAMBLE_LSB                        0x21
#define RH_RF95_REG_22_PAYLOAD_LENGTH                      0x22
#define RH_RF95_REG_23_MAX_PAYLOAD_LENGTH                  0x23
#define RH_RF95_REG_24_HOP_PERIOD                          0x24
#define RH_RF95_REG_25_FIFO_RX_BYTE_ADDR                   0x25
#define RH_RF95_REG_26_MODEM_CONFIG3                       0x26

#define RH_RF95_REG_40_DIO_MAPPING1                        0x40
#define RH_RF95_REG_41_DIO_MAPPING2                        0x41
#define RH_RF95_REG_42_VERSION                             0x42

#define RH_RF95_REG_4B_TCXO                                0x4b
#define RH_RF95_REG_4D_PA_DAC                              0x4d
#define RH_RF95_REG_5B_FORMER_TEMP                         0x5b
#define RH_RF95_REG_61_AGC_REF                             0x61
#define RH_RF95_REG_62_AGC_THRESH1                         0x62
#define RH_RF95_REG_63_AGC_THRESH2                         0x63
#define RH_RF95_REG_64_AGC_THRESH3                         0x64

// RH_RF95_REG_01_OP_MODE                             0x01
#define RH_RF95_LONG_RANGE_MODE                       0x80
#define RH_RF95_ACCESS_SHARED_REG                     0x40
#define RH_RF95_MODE                                  0x07
#define RH_RF95_MODE_SLEEP                            0x00
#define RH_RF95_MODE_STDBY                            0x01
#define RH_RF95_MODE_FSTX                             0x02
#define RH_RF95_MODE_TX                               0x03
#define RH_RF95_MODE_FSRX                             0x04
#define RH_RF95_MODE_RXCONTINUOUS                     0x05
#define RH_RF95_MODE_RXSINGLE                         0x06
#define RH_RF95_MODE_CAD                              0x07

// RH_RF95_REG_09_PA_CONFIG                           0x09
#define RH_RF95_PA_SELECT                             0x80
#define RH_RF95_MAX_POWER                             0x70
#define RH_RF95_OUTPUT_POWER                          0x0f

// RH_RF95_REG_0A_PA_RAMP                             0x0a
#define RH_RF95_LOW_PN_TX_PLL_OFF                     0x10
#define RH_RF95_PA_RAMP                               0x0f
#define RH_RF95_PA_RAMP_3_4MS                         0x00
#define RH_RF95_PA_RAMP_2MS                           0x01
#define RH_RF95_PA_RAMP_1MS                           0x02
#define RH_RF95_PA_RAMP_500US                         0x03
#define RH_RF95_PA_RAMP_250US                         0x0
#define RH_RF95_PA_RAMP_125US                         0x05
#define RH_RF95_PA_RAMP_100US                         0x06
#define RH_RF95_PA_RAMP_62US                          0x07
#define RH_RF95_PA_RAMP_50US                          0x08
#define RH_RF95_PA_RAMP_40US                          0x09
#define RH_RF95_PA_RAMP_31US                          0x0a
#define RH_RF95_PA_RAMP_25US                          0x0b
#define RH_RF95_PA_RAMP_20US                          0x0c
#define RH_RF95_PA_RAMP_15US                          0x0d
#define RH_RF95_PA_RAMP_12US                          0x0e
#define RH_RF95_PA_RAMP_10US                          0x0f

// RH_RF95_REG_0B_OCP                                 0x0b
#define RH_RF95_OCP_ON                                0x20
#define RH_RF95_OCP_TRIM                              0x1f

// RH_RF95_REG_0C_LNA                                 0x0c
#define RH_RF95_LNA_GAIN                              0xe0
#define RH_RF95_LNA_BOOST                             0x03
#define RH_RF95_LNA_BOOST_DEFAULT                     0x00
#define RH_RF95_LNA_BOOST_150PC                       0x11

// RH_RF95_REG_11_IRQ_FLAGS_MASK                      0x11
#define RH_RF95_RX_TIMEOUT_MASK                       0x80
#define RH_RF95_RX_DONE_MASK                          0x40
#define RH_RF95_PAYLOAD_CRC_ERROR_MASK                0x20
#define RH_RF95_VALID_HEADER_MASK                     0x10
#define RH_RF95_TX_DONE_MASK                          0x08
#define RH_RF95_CAD_DONE_MASK                         0x04
#define RH_RF95_FHSS_CHANGE_CHANNEL_MASK              0x02
#define RH_RF95_CAD_DETECTED_MASK                     0x01

// RH_RF95_REG_12_IRQ_FLAGS                           0x12
#define RH_RF95_RX_TIMEOUT                            0x80
#define RH_RF95_RX_DONE                               0x40
#define RH_RF95_PAYLOAD_CRC_ERROR                     0x20
#define RH_RF95_VALID_HEADER                          0x10
#define RH_RF95_TX_DONE                               0x08
#define RH_RF95_CAD_DONE                              0x04
#define RH_RF95_FHSS_CHANGE_CHANNEL                   0x02
#define RH_RF95_CAD_DETECTED                          0x01

// RH_RF95_REG_18_MODEM_STAT                          0x18
#define RH_RF95_RX_CODING_RATE                        0xe0
#define RH_RF95_MODEM_STATUS_CLEAR                    0x10
#define RH_RF95_MODEM_STATUS_HEADER_INFO_VALID        0x08
#define RH_RF95_MODEM_STATUS_RX_ONGOING               0x04
#define RH_RF95_MODEM_STATUS_SIGNAL_SYNCHRONIZED      0x02
#define RH_RF95_MODEM_STATUS_SIGNAL_DETECTED          0x01

// RH_RF95_REG_1C_HOP_CHANNEL                         0x1c
#define RH_RF95_PLL_TIMEOUT                           0x80
#define RH_RF95_RX_PAYLOAD_CRC_IS_ON                  0x40
#define RH_RF95_FHSS_PRESENT_CHANNEL                  0x3f

// RH_RF95_REG_1D_MODEM_CONFIG1                       0x1d
#define RH_RF95_BW                                    0xc0
#define RH_RF95_BW_125KHZ                             0x00
#define RH_RF95_BW_250KHZ                             0x40
#define RH_RF95_BW_500KHZ                             0x80
#define RH_RF95_BW_RESERVED                           0xc0
#define RH_RF95_CODING_RATE                           0x38
#define RH_RF95_CODING_RATE_4_5                       0x00
#define RH_RF95_CODING_RATE_4_6                       0x08
#define RH_RF95_CODING_RATE_4_7                       0x10
#define RH_RF95_CODING_RATE_4_8                       0x18
#define RH_RF95_IMPLICIT_HEADER_MODE_ON               0x04
#define RH_RF95_RX_PAYLOAD_CRC_ON                     0x02
#define RH_RF95_LOW_DATA_RATE_OPTIMIZE                0x01

// RH_RF95_REG_1E_MODEM_CONFIG2                       0x1e
#define RH_RF95_SPREADING_FACTOR                      0xf0
#define RH_RF95_SPREADING_FACTOR_64CPS                0x60
#define RH_RF95_SPREADING_FACTOR_128CPS               0x70
#define RH_RF95_SPREADING_FACTOR_256CPS               0x80
#define RH_RF95_SPREADING_FACTOR_512CPS               0x90
#define RH_RF95_SPREADING_FACTOR_1024CPS              0xa0
#define RH_RF95_SPREADING_FACTOR_2048CPS              0xb0
#define RH_RF95_SPREADING_FACTOR_4096CPS              0xc0
#define RH_RF95_TX_CONTINUOUS_MOE                     0x08
#define RH_RF95_AGC_AUTO_ON                           0x04
#define RH_RF95_SYM_TIMEOUT_MSB                       0x03

// RH_RF95_REG_4D_PA_DAC                              0x4d
#define RH_RF95_PA_DAC_DISABLE                        0x04
#define RH_RF95_PA_DAC_ENABLE                         0x07
//###########################################

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
	
	//while(1);
 

	return 0;
}
