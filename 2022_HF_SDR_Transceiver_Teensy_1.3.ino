/******************************************************************************************************************************

  2022 HF SDR Transceiver by Rod Gatehouse, AD5GH
  (http://www.ad5gh.com)
  
  SDR Transceiver QSD, QSE, LO, Teensy 4.1 DSP, & Audio Board Software
 
  by Rod Gatehouse, AD5GH

  The software in this file is written by me specifically for the SDR Transceiver QSD, QSE, LO, Teensy 4.1 DSP, & Audio Board.

  This software uses the following files: DSP_Routines.ccp, DSP_Routines.h, and DSP_Coeffs.h, and attribution is as follows:

  (c) Frank Dziock, DD4WH, 2020_05_8
  "TEENSY CONVOLUTION SDR" substantially modified by Jack Purdum, W8TEE, and Al Peter, AC8GY

  This software is made available under the GNU GPL v3 license agreement. If commercial use of this
  software is planned, we would appreciate it if the interested parties get written approval
  from Jack Purdum, W8TEE, and Al Peter, AC8GY.

  Any and all other commercial uses, written or implied, are forbidden without written permission from
  from Jack Purdum, W8TEE, and Al Peter, AC8GY.
 
  I have significantly refactored Al and Jack's software to extract the core DSP software necessary for 
  processing receiver 48-KHz In-phase ("I") and Quadrature ("Q") IF to audio, and transmit audio to IQ audio; 
  including, in the receiver, IF filtering, frequency conversion from 48-KHz IF to audio, unwanted sideband filtering,
  and SSB demodulation; and in the transmitter, low pass filtering of the audio, and audio phase shift to provide
  I and Q transmit audio streams. Refer to AL and Jack's book for a description of the full software package for the T41-EP.

  VERSION 1.3
  August 5, 2023

******************************************************************************************************************************/

#include <DSP_Routines.h>
#include <Wire.h>

DSP_Routines dspRoutines;

uint16_t currentVersion = 10101;                            // current software version X.YY.ZZ, i.e., 1.00.00

//#define DEBUG                                             // remove comment (//) to enable Arduino serial port for debug messages


/********************Frequency, Band, & Mode Variables************************************************************************/


/********************I2C_0 and I2C_1 Allocation*******************************************************************************/

// Teensy 4.1, Xmt & Rcv Audio Board  I2C_0
// I2C Relay Control Board            I2C_1
// QSD / QSE Board                    I2C_0

/*********************I/O Port Definitions ***********************************************************************************/

#define RX_TX         24
#define USB_LSB       25
#define TX_AF_GAIN    26
#define RX_IF_GAIN    27
#define RX_IQ_AMP     28
#define RX_IQ_PHS     29
#define TX_IQ_AMP     30
#define TX_IQ_PHS     31

#define PCM5102_RX_MUTE 5

#define TEENSY_LED    13
#define TX_RF_DRV_PD  14
#define STATUS_LED    22                                     // LED on Teensy 4.1, Xmt & Rcv Audio Boardto indicate program running

#define TEENSY_I2C_ADDRESS 0x80

typedef union 
{
  int16_t dataReceived;                                      // this union allows floats to be sent as binary representations over the I2C
  uint8_t binary[sizeof(dataReceived)];
} binaryData;

binaryData rcvData;

enum 
{
  RCV,
  XMT
};


/*********************Loop State Machine**************************************************************************************/

uint32_t HEART_BEAT = 750000;                               // heart beat period
uint32_t heartBeat = 0;                                     // heart beat counter

boolean pttStatus = RCV;                                    // initialize PTT status to receive, lastPPTStatus to transmit
boolean lastPTTStatus = XMT;

/*********************setup() ************************************************************************************************/
PROGMEM

void setup() 
{
#ifdef DEBUG
  Serial.begin(38400);
  Serial.println("Main Program Debug Enabled");
#endif

  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, HIGH);

  pinMode(TEENSY_LED, OUTPUT);
  digitalWrite(TEENSY_LED, LOW);

  pinMode(TX_RF_DRV_PD, OUTPUT);
  digitalWrite(TX_RF_DRV_PD, HIGH);

  pinMode(PCM5102_RX_MUTE, OUTPUT);
  digitalWrite(PCM5102_RX_MUTE, HIGH);

  pinMode(RX_TX, INPUT);
  pinMode(USB_LSB, INPUT);
  pinMode(TX_AF_GAIN, INPUT);
  pinMode(RX_IF_GAIN, INPUT);
  pinMode(RX_IQ_AMP, INPUT);
  pinMode(RX_IQ_PHS, INPUT);
  pinMode(TX_IQ_AMP, INPUT);
  pinMode(TX_IQ_PHS, INPUT);

  dspRoutines.Begin();

  Wire1.begin(TEENSY_I2C_ADDRESS);                          // using I2C_1 on Teensy
  Wire1.onReceive(receiveEvent);
}

/*********************loop()**********************************************************************************************/
FASTRUN

void loop() 
{
  if (digitalRead(RX_TX) == HIGH) pttStatus = RCV;          // check PTT status
  if (digitalRead(RX_TX) == LOW) pttStatus = XMT;

  if (pttStatus == RCV) 
  {
    if (lastPTTStatus != RCV) 
    {
      digitalWrite(TX_RF_DRV_PD, HIGH);
      dspRoutines.RCV_Mode();
      lastPTTStatus = RCV;
      heartBeat = 0;
    }

    dspRoutines.ReceiveIQData();

    if (++heartBeat == HEART_BEAT) 
    {
      heartBeat = 0;
      digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
    }
  }

  if (pttStatus == XMT) 
  {
    if (lastPTTStatus != XMT) 
    {
      digitalWrite(TX_RF_DRV_PD, LOW);
      dspRoutines.XMT_Mode();
      lastPTTStatus = XMT;
      heartBeat = 0;
    }

    dspRoutines.TransmitIQData();

    if (++heartBeat == HEART_BEAT / 3) 
    {
      heartBeat = 0;
      digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
    }
  }

if(digitalRead(RX_IF_GAIN) == LOW || digitalRead(TX_AF_GAIN) == LOW || digitalRead(RX_IQ_AMP) == LOW || digitalRead(RX_IQ_PHS) == LOW
  || digitalRead(TX_IQ_AMP) == LOW || digitalRead(TX_IQ_PHS) == LOW) AudioNoInterrupts();

else AudioInterrupts();
}


void receiveEvent(int numBytes) 
{
  for (uint8_t n = 0; n < numBytes; n++) rcvData.binary[n] = Wire1.read();
  
  if(digitalRead(RX_IF_GAIN) == LOW) dspRoutines.SetRX_IF_Gain(rcvData.dataReceived);

  else if(digitalRead(TX_AF_GAIN) == LOW) dspRoutines.SetTX_AF_Gain(rcvData.dataReceived);
  
  else if(digitalRead(RX_IQ_AMP) == LOW) dspRoutines.SetRX_AmpCorrection(rcvData.dataReceived);
    
  else if(digitalRead(RX_IQ_PHS) == LOW) dspRoutines.SetRX_PhsCorrection(rcvData.dataReceived);
  
  else if(digitalRead(TX_IQ_AMP) == LOW) dspRoutines.SetTX_AmpCorrection(rcvData.dataReceived);
  
  else if(digitalRead(TX_IQ_PHS) == LOW) dspRoutines.SetTX_PhsCorrection(rcvData.dataReceived);
}

/*********************END OF PROGRAM*****************************************************************************************/