/******************************************************************************

 2022 HF SDR Transceiver by Rod Gatehouse, AD5GH
 (http://www.ad5gh.com)

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

******************************************************************************/

#ifndef DSP_Routines_h
#define DSP_Routines_h

#include <arm_math.h>
#include <arm_const_structs.h>
#include <Audio.h>
#include <utility/imxrt_hw.h>

#define FFT_LENGTH                        512


class DSP_Routines
{
  public:
    DSP_Routines(void);
    void Begin(void);
    void SetRX_IF_Gain(int16_t gain);
    void SetTX_AF_Gain(int16_t gain);
    void XMT_Mode(void);
    void RCV_Mode(void);
    void SetRX_AmpCorrection(int16_t gain);
    void SetRX_PhsCorrection(int16_t phase);
    void SetTX_AmpCorrection(int16_t gain);
    void SetTX_PhsCorrection(int16_t phase);
    void ReceiveIQData(void);
    void TransmitIQData(void);
  
  private:
    void CalcCplxFIRCoeffs(float * coeffs_I, float * coeffs_Q, int numCoeffs, float32_t FLoCut, float32_t FHiCut, float SampleRate);
    void CalcFIRCoeffs(float *coeffs_I, int numCoeffs, float32_t fc, float32_t Astop, int type, float dfc, float Fsamprate);
    void IQPhaseCorrection(float32_t *I_buffer, float32_t *Q_buffer, float32_t factor, uint32_t blocksize);
    void SetIIRCoeffs(float32_t f0, float32_t Q, float32_t sample_rate, uint8_t filter_type);
    void MyDelay(unsigned long millisWait);
    void SetDecIntFilters(void);
    void FilterBandwidth(void);
    int SetI2SFreq(int freq);
    void FreqShift1(void);
    void InitFilterMask();
    float MSinc(int m, float fc);
    float32_t Izero(float32_t x);
};

#endif