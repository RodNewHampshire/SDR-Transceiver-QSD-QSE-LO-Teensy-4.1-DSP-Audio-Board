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

#include <arduino.h>
#include <DSP_Coeffs.h>
#include <DSP_Routines.h>

//#define DEBUG
#define BUFFER_SIZE   128

int16_t *sp_L;
int16_t *sp_R;


int sampleRate        = 192000;                                                 // other sample rates are defined in original (referenced) code
                                                                                // LRCLK - Frame Synch / Sample Rate clock = 192-KHz
                                                                                // MCLK - 256 x Sample Rate = 49.152-MHz
                                                                                // BCLK - Sample Rate x #Bits x #channels = 192,000 x 32 x 2 = 12.288-MHz


AudioInputI2SQuad     i2s_quadIn;                                               // I2S Quad Inputs,  0, 1, 2, & 3: Teensy 4.1 Pin 8 TX PCM1808 I2S L(0) & R(1) Out; Pin 6 RX PCM1808 I2S L(2) & R(3) Out
AudioOutputI2SQuad    i2s_quadOut;                                              // I2S Quad Outputs, 0, 1, 2, & 3: Teensy 4.1 Pin 7 TX PCM5102 I2S L(0) & R(1) In;  Pin 32 RX PCM5102 I2S L(3) & R(2) In

AudioMixer4           modeSelectOutL;                                           // set up Teensy Audio Mixers
AudioMixer4           modeSelectOutR;
AudioMixer4           modeSelectOutExL;
AudioMixer4           modeSelectOutExR;

AudioMixer4           modeSelectInL;
AudioMixer4           modeSelectInR;
AudioMixer4           modeSelectInExL;
AudioMixer4           modeSelectInExR;

AudioPlayQueue        Q_out_L;                                                  // set up Teensy Audio Play and Record Ques
AudioPlayQueue        Q_out_R;
AudioPlayQueue        Q_out_L_Ex;
AudioPlayQueue        Q_out_R_Ex;

AudioRecordQueue      Q_in_L;
AudioRecordQueue      Q_in_R;
AudioRecordQueue      Q_in_L_Ex;
AudioRecordQueue      Q_in_R_Ex;

                                                                                // *****GET XMT I2S AUDIO INPUT FROM PCM1808 TX ADC (only using L chan)*****
AudioConnection       patchCord1(i2s_quadIn, 0, modeSelectInExL, 0);            // patch TX ADC I2S L Chan input to Teensy Input Audio Mixer modeSelectInL, port 0
AudioConnection       patchCord2(i2s_quadIn, 1, modeSelectInExR, 0);            // patch TX ADC I2S R Chan input to Teensy Input Audio Mixer modeSelectInR, port 0

AudioConnection       patchCord3(modeSelectInExL, 0, Q_in_L_Ex, 0);             // patch Input Audio Mixer XMT L Chan to Xmt L Audio Record Que
AudioConnection       patchCord4(modeSelectInExR, 0, Q_in_R_Ex, 0);             // patch Input Audio Mixer XMT R Chan to Xmt R Audio Record Ques

                                                                                // *****SEND PROCESSED XMT AUDIO TO PCM5102 TX DAC*****
AudioConnection       patchCord5(Q_out_L_Ex, 0, modeSelectOutExL, 0);           // patch Teensy XMT L Audio Play Que to Teensy XMT L chan output Audio Mixer, port 0
AudioConnection       patchCord6(Q_out_R_Ex, 0, modeSelectOutExR, 0);           // patch Teensy XMT R Audio Play Que to Teensy XMT R chan output Audio Mixer, port 0

AudioConnection       patchCord7(modeSelectOutExL, 0, i2s_quadOut, 0);          // patch Teensy XMT L(Q) chan output Audio Mixer to TX DAC I2S Input
AudioConnection       patchCord8(modeSelectOutExR, 0, i2s_quadOut, 1);          // patch Teensy XMT R(I) chan output Audio Mixer to TX DAC I2S Input



                                                                                // *****GET RCV I2S 48-KHz IF INPUT FROM PCM1808 ADC*****
AudioConnection       patchCord9(i2s_quadIn, 2, modeSelectInL, 0);              // patch PCM1808 I2S L Chan ouptut to Teensy Input Audio Mixer modeSelectInL, port 0
AudioConnection       patchCord10(i2s_quadIn, 3, modeSelectInR, 0);             // patch PCM1808 I2S R Chan ouptut to Teensy Input Audio Mixer modeSelectInR, port 0

AudioConnection       patchCord11(modeSelectInL, 0, Q_in_L, 0);                 // patch Input Audio Mixer RCV L Chan Output I2S to RCV Audio Record Ques
AudioConnection       patchCord12(modeSelectInR, 0, Q_in_R, 0);                 // patch Input Audio Mixer RCV R Chan Output I2S to RCV Audio Record Ques


                                                                                // *****SEND PROCESSED RCV AUDIO TO PCM5102 DAC*****
AudioConnection       patchCord13(Q_out_L, 0, modeSelectOutL, 0);               // patch RCV L Chan I2S Audio Play Que to Teensy Output Audio Mixer modeSelectOutL, port 0
AudioConnection       patchCord14(Q_out_R, 0, modeSelectOutR, 0);               // patch RCV R Chan I2S Audio Play Que to Teensy Output Audio Mixer modeSelectOutR, port 0

AudioConnection       patchCord15(modeSelectOutL, 0, i2s_quadOut, 2);           // patch Teensy RCV L Output Audio Mixer to PCM5102 I2S L channel input
AudioConnection       patchCord16(modeSelectOutR, 0, i2s_quadOut, 3);           // patch Teensy RCV R Output Audio Mixer to PCM5102 I2S R channel input


AudioControlSGTL5000              sgtl5000_1;                                   // although the SGTL5000 is not used, this may be needed to initialize aspects of the Teensy Audio Library

#define IIR_ORDER                 8
#define IIR_NUMSTAGES             (IIR_ORDER / 2)

int NumExBlocks                   = 8;
const uint32_t N_BLOCKS_EX        = 16;

enum
{
  _USB,
  _LSB,
  _CW
};

                                                                                // ********** TRANSMIT SPECIFIC DEFINITIONS AND DECLARATIONS START HERE **********

                                                                                // Decimation and Interpolation Filters
arm_fir_decimate_instance_f32 FIR_dec1_EX_I;
arm_fir_decimate_instance_f32 FIR_dec1_EX_Q;
arm_fir_decimate_instance_f32 FIR_dec2_EX_I;
arm_fir_decimate_instance_f32 FIR_dec2_EX_Q;

arm_fir_interpolate_instance_f32 FIR_int1_EX_I;
arm_fir_interpolate_instance_f32 FIR_int1_EX_Q;
arm_fir_interpolate_instance_f32 FIR_int2_EX_I;
arm_fir_interpolate_instance_f32 FIR_int2_EX_Q;

float32_t DMAMEM float_buffer_L_EX[2048];
float32_t DMAMEM float_buffer_R_EX[2048];
float32_t DMAMEM float_buffer_LTemp[2048];
float32_t DMAMEM float_buffer_RTemp[2048];

float32_t DMAMEM FIR_dec1_EX_I_state[2095];
float32_t DMAMEM FIR_dec1_EX_Q_state[2095];
float32_t DMAMEM FIR_dec2_EX_I_state[535];
float32_t DMAMEM FIR_dec2_EX_Q_state[535];

float32_t DMAMEM FIR_int2_EX_I_state[519];
float32_t DMAMEM FIR_int2_EX_Q_state[519];
float32_t DMAMEM FIR_int1_EX_I_state[279];
float32_t DMAMEM FIR_int1_EX_Q_state[279];

float32_t DMAMEM FIR_int1_EX_coeffs[48];
float32_t DMAMEM FIR_int2_EX_coeffs[48];
                       
float32_t FIR_Hilbert_state_L [100 + 256 - 1];                                  // Hilbert FIR Filters
float32_t FIR_Hilbert_state_R [100 + 256 - 1];

arm_fir_instance_f32 FIR_Hilbert_L;
arm_fir_instance_f32 FIR_Hilbert_R;



                                                                                // ********** RECEIVE SPECIFIC DEFINITIONS AND DECLARATIONS START HERE **********

#define FFT_LENGTH                        512
#define NUM_BANDS                         11

#undef  PI
#undef  HALF_PI
#undef  TWO_PI

#define PI                                3.1415926535897932384626433832795f
#define HALF_PI                           1.5707963267948966192313216916398f
#define TWO_PI                            6.283185307179586476925286766559f
#define TPI                               TWO_PI
#define PIH                               HALF_PI
#define FOURPI                            (2.0f * TPI)
#define SIXPI                             (3.0f * TPI)

uint32_t FFT_length                       = FFT_LENGTH;

const float32_t DF1                       = 4.0;                                // decimation factor
const float32_t DF2                       = 2.0;                                // decimation factor
const float32_t DF                        = DF1 * DF2;                          // decimation factor
const float32_t n_samplerate              = 192.0;                              // samplerate before decimation (was 176.0 in original code)

const uint32_t N_B = FFT_LENGTH / 2 / BUFFER_SIZE * (uint32_t)DF;

uint32_t N_BLOCKS = N_B;

uint32_t m_NumTaps                        = (FFT_LENGTH / 2) + 1;

const float32_t n_att                     = 90.0;                               // need here for later def's
const float32_t n_desired_BW              = 9.0;                                // desired max BW of the filters

const float32_t n_fpass1                  = n_desired_BW / n_samplerate;
const float32_t n_fpass2                  = n_desired_BW / (n_samplerate / DF1);
const float32_t n_fstop1                  = ( (n_samplerate / DF1) - n_desired_BW) / n_samplerate;
const float32_t n_fstop2                  = ((n_samplerate / (DF1 * DF2)) - n_desired_BW) / (n_samplerate / DF1);

const uint16_t n_dec1_taps                = (1 + (uint16_t) (n_att / (22.0 * (n_fstop1 - n_fpass1))));
const uint16_t n_dec2_taps                = (1 + (uint16_t) (n_att / (22.0 * (n_fstop2 - n_fpass2))));


uint8_t FIR_filter_window                 = 1;

const uint32_t N_stages_biquad_lowpass1   = 1;

const arm_cfft_instance_f32 *maskS;
const arm_cfft_instance_f32 *S;
const arm_cfft_instance_f32 *iS;
const arm_cfft_instance_f32 *NR_FFT;
const arm_cfft_instance_f32 *NR_iFFT;
const arm_cfft_instance_f32 *spec_FFT;

int LP_F_help                             = 3500;

float MSinc(int m, float fc);

int8_t first_block                        = 1;

const uint32_t N_DEC_B                    = N_B / (uint32_t)DF;

const int INT1_STATE_SIZE                 = 24 + BUFFER_SIZE * N_B / (uint32_t)DF - 1;
const int INT2_STATE_SIZE                 = 8 + BUFFER_SIZE * N_B / (uint32_t)DF1 - 1;
const int DEC2STATESIZE                   = n_dec2_taps + (BUFFER_SIZE * N_B / (uint32_t)DF1) - 1;

int8_t modeFactor;

int16_t lsb_LoCut                         = -4000;
int16_t lsb_HiCut                         = -100;

int16_t usb_LoCut                         = 100;
int16_t usb_HiCut                         = 4000;

float32_t ifGain                          = 0;
float32_t txGain                          = 10;

float32_t IQ_RxAmpCorrection              = 1.0;
float32_t IQ_RxPhaseCorrection            = 0.0;
float32_t IQ_TxAmpCorrection              = 1.0;
float32_t IQ_TxPhaseCorrection            = 0.0;

arm_fir_decimate_instance_f32 FIR_dec1_I;
arm_fir_decimate_instance_f32 FIR_dec1_Q;
arm_fir_decimate_instance_f32 FIR_dec2_I;
arm_fir_decimate_instance_f32 FIR_dec2_Q;

float32_t DMAMEM float_buffer_L[BUFFER_SIZE * N_B];
float32_t DMAMEM float_buffer_R[BUFFER_SIZE * N_B];
float32_t DMAMEM float_buffer_L2[BUFFER_SIZE * N_B];
float32_t DMAMEM float_buffer_R2[BUFFER_SIZE * N_B];
float32_t float_buffer_L_3[BUFFER_SIZE * N_B];
float32_t float_buffer_R_3[BUFFER_SIZE * N_B];

float32_t sample_meanL = 0.0;
float32_t sample_meanR = 0.0;

float32_t DMAMEM R_BufferOffset[BUFFER_SIZE * N_B];
float32_t DMAMEM L_BufferOffset[BUFFER_SIZE * N_B];

float32_t IQ_amplitude_correction_factor  = 1.0;
float32_t IQ_phase_correction_factor      = 0.0;

float32_t hh1                             = 0.0;
float32_t hh2                             = 0.0;

float32_t DMAMEM FIR_Coef_I[(FFT_LENGTH / 2) + 1];
float32_t DMAMEM FIR_Coef_Q[(FFT_LENGTH / 2) + 1];

float32_t biquad_lowpass1_coeffs[5 * N_stages_biquad_lowpass1] = {0, 0, 0, 0, 0};
float32_t coefficient_set[5]                                   = {0, 0, 0, 0, 0};

float32_t DMAMEM FIR_filter_mask[FFT_LENGTH * 2] __attribute__ ((aligned (4)));

float32_t DMAMEM FIR_dec1_coeffs[n_dec1_taps];
float32_t DMAMEM FIR_dec2_coeffs[n_dec2_taps];
float32_t DMAMEM FIR_int1_coeffs[48];
float32_t DMAMEM FIR_int2_coeffs[32];

float32_t bin_BW                          = 1.0 / (DF * FFT_length) * sampleRate;

float32_t Izero(float32_t x);

float32_t DMAMEM FFT_buffer[FFT_LENGTH * 2] __attribute__ ((aligned (4)));
float32_t DMAMEM iFFT_buffer[FFT_LENGTH * 2 + 1];
float32_t DMAMEM last_sample_buffer_L[BUFFER_SIZE * N_DEC_B];
float32_t DMAMEM last_sample_buffer_R[BUFFER_SIZE * N_DEC_B];

float32_t DMAMEM FIR_int1_I_state[INT1_STATE_SIZE];
float32_t DMAMEM FIR_int1_Q_state[INT1_STATE_SIZE];
float32_t DMAMEM FIR_int2_I_state[INT2_STATE_SIZE];
float32_t DMAMEM FIR_int2_Q_state[INT2_STATE_SIZE]; 

float32_t DMAMEM FIR_dec1_I_state[n_dec1_taps + (uint16_t) BUFFER_SIZE * (uint32_t) N_B - 1];
float32_t DMAMEM FIR_dec1_Q_state[n_dec1_taps + (uint16_t)BUFFER_SIZE * (uint16_t)N_B - 1];
float32_t DMAMEM FIR_dec2_I_state[DEC2STATESIZE];
float32_t DMAMEM FIR_dec2_Q_state[DEC2STATESIZE];

arm_fir_interpolate_instance_f32 FIR_int1_I;
arm_fir_interpolate_instance_f32 FIR_int1_Q;
arm_fir_interpolate_instance_f32 FIR_int2_I;
arm_fir_interpolate_instance_f32 FIR_int2_Q;

arm_biquad_casd_df1_inst_f32 biquad_lowpass1;

float32_t biquad_lowpass1_state[N_stages_biquad_lowpass1 * 4];

float32_t HP_DC_Butter_state[6] = {0, 0, 0, 0, 0, 0};
float32_t HP_DC_Butter_state2[2] = {0, 0};
arm_biquad_cascade_df2T_instance_f32  s1_Receive = {3, HP_DC_Butter_state, HP_DC_Filter_Coeffs};
arm_biquad_cascade_df2T_instance_f32  s1_Receive2 = {1, HP_DC_Butter_state2, HP_DC_Filter_Coeffs2};
                  

DSP_Routines::DSP_Routines(void)
{

}


void DSP_Routines::Begin(void)                                                                                
{
  
  AudioMemory(400); 
  delay(200);

  SetI2SFreq(sampleRate);
  delay(200);

  uint8_t newMode = _USB;

  AudioNoInterrupts();

  if(newMode == _USB)
    {
      modeFactor = 1.0;                                                                                                           // for XMT mode selection
      CalcCplxFIRCoeffs(FIR_Coef_I, FIR_Coef_Q, m_NumTaps, (float32_t)usb_LoCut, (float32_t)usb_HiCut, (float)sampleRate / DF);   // for RCV mode selection
    }
  
  else if(newMode == _LSB) 
    {
      modeFactor = -1.0;
      CalcCplxFIRCoeffs(FIR_Coef_I, FIR_Coef_Q, m_NumTaps, (float32_t)lsb_LoCut, (float32_t)lsb_HiCut, (float)sampleRate / DF);
    }

  else if(newMode == _CW)                                                                                                          // not enabling narrower CW filters at this time
    {                                                                                                                              // use USB filter settings
      modeFactor = 1.0;                                     
      CalcCplxFIRCoeffs(FIR_Coef_I, FIR_Coef_Q, m_NumTaps, (float32_t)usb_LoCut, (float32_t)usb_HiCut, (float)sampleRate / DF);
    }
                                                                                                                                    
  maskS = &arm_cfft_sR_f32_len512;
  
  InitFilterMask();

  AudioInterrupts();

// ********** transmit filter intiialization starts here

  arm_fir_init_f32(&FIR_Hilbert_L, 100, FIR_Hilbert_coeffs_45, FIR_Hilbert_state_L, 256);                     // transmit Hilbert Transforms
  arm_fir_init_f32(&FIR_Hilbert_R, 100, FIR_Hilbert_coeffs_neg45, FIR_Hilbert_state_R, 256);

  arm_fir_decimate_init_f32(&FIR_dec1_EX_I, 48, 4 , coeffs192K_3K_LPF_FIR, FIR_dec1_EX_I_state, 2048);        // transmit Decimation (down sample) by factor of 4
  arm_fir_decimate_init_f32(&FIR_dec1_EX_Q, 48, 4, coeffs192K_3K_LPF_FIR, FIR_dec1_EX_Q_state, 2048) ;

  arm_fir_decimate_init_f32(&FIR_dec2_EX_I, 24, 2, coeffs48K_3K_LPF_FIR, FIR_dec2_EX_I_state, 512);           // transmit Decimation (down sample) by factor of 2
  arm_fir_decimate_init_f32(&FIR_dec2_EX_Q, 24, 2, coeffs48K_3K_LPF_FIR, FIR_dec2_EX_Q_state, 512);

  arm_fir_interpolate_init_f32(&FIR_int1_EX_I, 2, 48, coeffs48K_3K_LPF_FIR, FIR_int1_EX_I_state, 256);        // transmit Interpolation (up sample) by factor of 2 
  arm_fir_interpolate_init_f32(&FIR_int1_EX_Q, 2, 48, coeffs48K_3K_LPF_FIR, FIR_int1_EX_Q_state, 256); 

  arm_fir_interpolate_init_f32(&FIR_int2_EX_I, 4, 32, coeffs192K_3K_LPF_FIR, FIR_int2_EX_I_state, 512);       // transmit Interpolation (up sample) by factor of 4
  arm_fir_interpolate_init_f32(&FIR_int2_EX_Q, 4, 32, coeffs192K_3K_LPF_FIR, FIR_int2_EX_Q_state, 512);

  
// ********** receive filter initialization starts here **********

  S = &arm_cfft_sR_f32_len512;
  iS = &arm_cfft_sR_f32_len512;

  biquad_lowpass1.numStages = N_stages_biquad_lowpass1;                                                       // set number of stages
  biquad_lowpass1.pCoeffs   = biquad_lowpass1_coeffs;                                                         // set pointer to coefficients file

  for (unsigned i = 0; i < 4 * N_stages_biquad_lowpass1; i++)
  {
    biquad_lowpass1_state[i] = 0.0;                                                                           // set state variables to zero
  }
  biquad_lowpass1.pState = biquad_lowpass1_state;                                                             // set pointer to the state variables

  SetIIRCoeffs((float32_t)LP_F_help, 1.3, (float32_t)sampleRate / DF, 0);                                     // 1st stage
  for (int i = 0; i < 5; i++)
  {                                                                                                           // fill coefficients into the right file
    biquad_lowpass1_coeffs[i] = coefficient_set[i];
  }

                                                                                                              // Decimation filter 1, M1 = DF1
  CalcFIRCoeffs(FIR_dec1_coeffs, n_dec1_taps, (float32_t)(n_desired_BW * 1000.0), n_att, 0, 0.0, (float32_t)sampleRate);

  if (arm_fir_decimate_init_f32(&FIR_dec1_I, n_dec1_taps, (uint32_t)DF1 , FIR_dec1_coeffs, FIR_dec1_I_state, BUFFER_SIZE * N_BLOCKS)) 
  {
    while (1);
  }

  if (arm_fir_decimate_init_f32(&FIR_dec1_Q, n_dec1_taps, (uint32_t)DF1, FIR_dec1_coeffs, FIR_dec1_Q_state, BUFFER_SIZE * N_BLOCKS))
  {
    while (1);
  }

                                                                                                              // Decimation filter 2, M2 = DF2
  CalcFIRCoeffs(FIR_dec2_coeffs, n_dec2_taps, (float32_t)(n_desired_BW * 1000.0), n_att, 0, 0.0, (float32_t)(sampleRate / DF1));
  
  if (arm_fir_decimate_init_f32(&FIR_dec2_I, n_dec2_taps, (uint32_t)DF2, FIR_dec2_coeffs, FIR_dec2_I_state, BUFFER_SIZE * N_BLOCKS / (uint32_t)DF1)) 
  {
    while (1);
  }

  if (arm_fir_decimate_init_f32(&FIR_dec2_Q, n_dec2_taps, (uint32_t)DF2, FIR_dec2_coeffs, FIR_dec2_Q_state, BUFFER_SIZE * N_BLOCKS / (uint32_t)DF1)) 
  {
    while (1);
  }


                                                                                                              // Interpolation filter 1, L1 = 2
  CalcFIRCoeffs(FIR_int1_coeffs, 48, (float32_t)(n_desired_BW * 1000.0), n_att, 0, 0.0, sampleRate / 4.0);
  
  if (arm_fir_interpolate_init_f32(&FIR_int1_I, (uint8_t)DF2, 48, FIR_int1_coeffs, FIR_int1_I_state, BUFFER_SIZE * N_BLOCKS / (uint32_t)DF)) 
  {
    while (1);
  }
  
  if (arm_fir_interpolate_init_f32(&FIR_int1_Q, (uint8_t)DF2, 48, FIR_int1_coeffs, FIR_int1_Q_state, BUFFER_SIZE * N_BLOCKS / (uint32_t)DF)) 
  {
    while (1);
  }

                                                                                                              // Interpolation filter 2, L2 = 4
  CalcFIRCoeffs(FIR_int2_coeffs, 32, (float32_t)(n_desired_BW * 1000.0), n_att, 0, 0.0, (float32_t)sampleRate);

  if (arm_fir_interpolate_init_f32(&FIR_int2_I, (uint8_t)DF1, 32, FIR_int2_coeffs, FIR_int2_I_state, BUFFER_SIZE * N_BLOCKS / (uint32_t)DF1)) 
  {
    while (1);
  }
  
  if (arm_fir_interpolate_init_f32(&FIR_int2_Q, (uint8_t)DF1, 32, FIR_int2_coeffs, FIR_int2_Q_state, BUFFER_SIZE * N_BLOCKS / (uint32_t)DF1)) 
  {
    while (1);
  }

  SetDecIntFilters();
}


void DSP_Routines::SetRX_IF_Gain(int16_t gain)
{
  
  ifGain = pow(10.0, (float32_t)gain / 20.0);
}


void DSP_Routines::SetTX_AF_Gain(int16_t gain)
{
  
  txGain = pow(10.0, (float32_t)gain / 20.0);
}


void DSP_Routines::SetRX_AmpCorrection(int16_t amp)
{
  IQ_RxAmpCorrection = (float32_t) amp / 1000;
  #ifdef DEBUG
  Serial.print("RX Amp: ");
  Serial.println(IQ_RxAmpCorrection);
  #endif
}


void DSP_Routines::SetRX_PhsCorrection(int16_t phase)
{
  IQ_RxPhaseCorrection = (float32_t) phase / 1000;
  #ifdef DEBUG
  Serial.print("RX Phs: ");
  Serial.println(IQ_RxPhaseCorrection);
  #endif
}


void DSP_Routines::SetTX_AmpCorrection(int16_t amp)
{
  IQ_TxAmpCorrection = (float32_t) amp / 1000;
  #ifdef DEBUG
  Serial.print("TX Amp: ");
  Serial.println(IQ_TxAmpCorrection);
  #endif
}


void DSP_Routines::SetTX_PhsCorrection(int16_t phase)
{
  IQ_TxPhaseCorrection = (float32_t) phase / 1000;
  #ifdef DEBUG
  Serial.print("TX Phs: ");
  Serial.println(IQ_TxPhaseCorrection);
  #endif
}


void DSP_Routines::XMT_Mode(void)
{                                                                               
  
  Q_in_L.end();                                                                 // end RCV ques
  Q_in_R.end();
                                                                                // Mixer4.gain(Port#, gain) where 1.0 = pass through, 0 = shutoff, >1 = gain
  modeSelectInR.gain(0, 0);                                                     // mute R RCV Input
  modeSelectInL.gain(0, 0);                                                     // mute R RCV Input
  modeSelectOutR.gain(0, 0);                                                    // mute R RCV Output
  modeSelectOutL.gain(0, 0);                                                    // mute L RCV Output

  modeSelectOutExR.gain(0, 1.0);                                                // enable R XMT Output
  modeSelectOutExL.gain(0, 1.0);                                                // enable L XMT Output
  modeSelectInExR.gain(0, 1.0);                                                 // enable R XMT Input
  modeSelectInExL.gain(0, 1.0);                                                 // enable L XMT Input
  
  Q_in_L_Ex.begin();                                                            // start XMT ques
  Q_in_R_Ex.begin();
}


void DSP_Routines::RCV_Mode(void)
{
  Q_in_L_Ex.end();                                                              // end XMT ques
  Q_in_R_Ex.end();

  modeSelectOutExR.gain(0, 0);                                                  // mute R XMT Output
  modeSelectOutExL.gain(0, 0);                                                  // mute L XMT Output
  modeSelectInExR.gain(0, 0);                                                   // mute R XMT Input
  modeSelectInExL.gain(0, 0);                                                   // mute L XMT Input

  modeSelectOutR.gain(0, 1.0);                                                  // enable R RCV Output
  modeSelectOutL.gain(0, 1.0);                                                  // enable L RCV Output
  modeSelectInR.gain(0, 1.0);                                                   // enable R RCV Input                                            
  modeSelectInL.gain(0, 1.0);                                                   // enable L RCV Input
  
  Q_in_L.begin();                                                               // start RCV ques
  Q_in_R.begin();
}


void DSP_Routines::TransmitIQData(void)
{
                                                                                                              // read in 16 blocks of 128 samples each from Teensy L & R Record Que buffers
                                                                                                              // into two 2048 sample arrays
  if ( (uint32_t) Q_in_L_Ex.available() > N_BLOCKS_EX && (uint32_t) Q_in_R_Ex.available() > N_BLOCKS_EX )     // wait for 16 blocks of I and Q samples to be available
  {
    for (unsigned i = 0; i < N_BLOCKS_EX; i++)                                                                // read in 16 blocks x 128 samples into each L and R chan arrays
    {
      sp_L = Q_in_L_Ex.readBuffer();                                                                        
      sp_R = Q_in_R_Ex.readBuffer();

      arm_q15_to_float (sp_L, &float_buffer_L_EX[BUFFER_SIZE * i], BUFFER_SIZE);                              // convert 16-bit int to float32 and normalize to -1 to +1 by dividing by 32768
      arm_q15_to_float (sp_R, &float_buffer_R_EX[BUFFER_SIZE * i], BUFFER_SIZE);                                      
      
      Q_in_L_Ex.freeBuffer();
      Q_in_R_Ex.freeBuffer();
    }
                                                                                                              // Decimation is the process of down sampling the data stream and LP filtering
                                                                                                              // Decimation is done in two stages to prevent reversal of the spectrum, 
                                                                                                              // which occurs with each even Decimation.  
                                                                                                              // First select every 4th sample and then every 2nd sample, yielding 8x down sampling
                                                                                                              // 192KHz/8 = 24KHz, with 8xsmaller sample sizes

    arm_fir_decimate_f32(&FIR_dec1_EX_I, float_buffer_L_EX, float_buffer_L_EX, BUFFER_SIZE * N_BLOCKS_EX );   // 192KHz effective sample rate here, decimation-by-4 in-place
    arm_fir_decimate_f32(&FIR_dec1_EX_Q, float_buffer_R_EX, float_buffer_R_EX, BUFFER_SIZE * N_BLOCKS_EX );
    
    arm_fir_decimate_f32(&FIR_dec2_EX_I, float_buffer_L_EX, float_buffer_L_EX, 512);                          // 48KHz effective sample rate here, decimation-by-2 in-place
    arm_fir_decimate_f32(&FIR_dec2_EX_Q, float_buffer_R_EX, float_buffer_R_EX, 512);

    arm_copy_f32(float_buffer_L_EX, float_buffer_R_EX, 256);                                                  // L chan contains the Xmt audio, so copy to right channel ready for Hilbert Transform

  
                                                                                                              // process L chan through 0 deg and R chan through 90 Hilbert Transforms
                                                                                                              // provides I and Q quadrature data streans for phasing method SSB generation.
                                                                                                              // 0 deg Hilbert Transform is used to preserve timing integrity between I and Q streams
    arm_fir_f32(&FIR_Hilbert_L, float_buffer_L_EX, float_buffer_L_EX, 256);
    arm_fir_f32(&FIR_Hilbert_R, float_buffer_R_EX, float_buffer_R_EX, 256);

    // ********** IQ Amplitude and Phase Correction
                                                                                                              
    arm_scale_f32 (float_buffer_L_EX, modeFactor*IQ_TxAmpCorrection, float_buffer_L_EX, 256);                 // modeFactor = 1.0 for USB, -1.0 for LSB
    IQPhaseCorrection(float_buffer_L_EX, float_buffer_R_EX, IQ_TxPhaseCorrection, 256);                                   
    
    arm_scale_f32 (float_buffer_R_EX, 1.00, float_buffer_R_EX, 256);
                                                                                                              // interpolate (up sample) the data streams by 8X to create the 192KHx sample rate for output
              
    arm_fir_interpolate_f32(&FIR_int1_EX_I, float_buffer_L_EX, float_buffer_LTemp, 256);                      // interpolate-by-2, 24KHz effective sample rate here increased to 48KHz sample rate
    arm_fir_interpolate_f32(&FIR_int1_EX_Q, float_buffer_R_EX, float_buffer_RTemp, 256);

    arm_fir_interpolate_f32(&FIR_int2_EX_I, float_buffer_LTemp, float_buffer_L_EX, 512);                      // interpolation-by-4, 48KHz effective sample rate here increased to 192KHz samle rate
    arm_fir_interpolate_f32(&FIR_int2_EX_Q, float_buffer_RTemp, float_buffer_R_EX, 512);
    
    arm_scale_f32(float_buffer_L_EX, txGain, float_buffer_L_EX, 2048);                                        // 192KHz effective sample rate here
    arm_scale_f32(float_buffer_R_EX, txGain, float_buffer_R_EX, 2048);                                        // adjust Tx Gain here

                                                                                                              
    for (unsigned  i = 0; i < N_BLOCKS_EX; i++)                                                               // convert to integers and send to PCM5102 DAC
    {
      sp_L = Q_out_L_Ex.getBuffer();
      sp_R = Q_out_R_Ex.getBuffer();
      arm_float_to_q15 (&float_buffer_L_EX[BUFFER_SIZE * i], sp_L, BUFFER_SIZE);
      arm_float_to_q15 (&float_buffer_R_EX[BUFFER_SIZE * i], sp_R, BUFFER_SIZE);
      Q_out_L_Ex.playBuffer();
      Q_out_R_Ex.playBuffer();
    }
  }
}


void DSP_Routines::ReceiveIQData(void)
{
/************************************************************************************************************************************************************************************
Teensy 4.1
MCLK      49.152-MHz      Master Clock = 256 x fs = 256 x 192-KHz = 49.152-MHz
BCLK      12.288-MHz      Bit Clock = LRCLK x 32 Bits x 2 channels = 12.288-MHz
LRCLK     192.0-KHz       Left-Right Clock, or Frame Synch Clock, also Sample Frequency fs

PCM1808
SCLKI     49.152-MHz      System Clock Input = 256 x fs = 256 x 192-KHz = 49.152-MHz
BCK       12.288-MHz      Audio Data Bit Clock
LRCK      192.0-KHz       Audio Data Latch Enable

Audio Sampling Frequency is 192-KHz.

SCKI is only used for timing internal functions of the ADC.

LRCK falling edge clocks out 24 bits left channel audio data at BCK frequency.
LRCK rising edge clocks out 24 bits right channel audio data at BCK frequency.

With BCK of 12.288-MHz and LRCK of 192.0-KHz, there is time for 32 bits to be clocked out, but only 24 bits are used in this case.
*************************************************************************************************************************************************************************************/

// ********** Teensy truncates the 24-bit samples from the PCM1808 to 16-bit samples in the Record Que buffers.
// ********** 16 blocks of 128 samples each from Teensy L & R Record Que buffers are read into two 2048 sample arrays.
                                                                                                          
  digitalWrite(31, LOW);

  if ( (uint32_t) Q_in_L.available() > N_BLOCKS && (uint32_t) Q_in_R.available() > N_BLOCKS)   
  {
    for (unsigned i = 0; i < N_BLOCKS; i++)        
    {
      sp_L = Q_in_L.readBuffer();                                                                        
      sp_R = Q_in_R.readBuffer();

      arm_q15_to_float (sp_L, &float_buffer_L[BUFFER_SIZE * i], BUFFER_SIZE);                                 // arm_q15_to_float converts 16-bit int to float32 and normalizes to -1 to +1 by dividing by 32768
      arm_q15_to_float (sp_R, &float_buffer_R[BUFFER_SIZE * i], BUFFER_SIZE);       

      Q_in_L.freeBuffer();
      Q_in_R.freeBuffer();
    }

    arm_scale_f32 (float_buffer_L, ifGain, float_buffer_L, BUFFER_SIZE * N_BLOCKS);                           // scale the data in the buffers by the IF Gain value: -60dB to +30dB converted to voltage ratio (ifGain)
    arm_scale_f32 (float_buffer_R, ifGain, float_buffer_R, BUFFER_SIZE * N_BLOCKS);

    arm_biquad_cascade_df2T_f32(&s1_Receive2, float_buffer_L, float_buffer_L, 2048);                          // 4.79-KHz High Pass Biquad Filter, 1 pole, to remove noise near DC from the 48-KHz IF
    arm_biquad_cascade_df2T_f32(&s1_Receive2, float_buffer_R, float_buffer_R, 2048);
                                                                                
    arm_scale_f32 (float_buffer_L, IQ_RxAmpCorrection, float_buffer_L, BUFFER_SIZE * N_BLOCKS);               // IQ Amplitude & Phase Correction
    IQPhaseCorrection(float_buffer_L, float_buffer_R, IQ_RxPhaseCorrection, BUFFER_SIZE * N_BLOCKS);

    FreqShift1();                                                                                             // Down convert the sampled signals centered at the 48-KHz IF frequency by Fs/4 (48-KHz) to baseband audio signals. 
                                                                                                              // Still at 192-KHz sample rate at this stage.. Refer to Lyons, ch. 13.1.2


// ********** Decimate first by 4 and then by 2 to produce a baseband audio signal with Sample Rate = 24-KHz

    arm_fir_decimate_f32(&FIR_dec1_I, float_buffer_L, float_buffer_L, BUFFER_SIZE * N_BLOCKS);
    arm_fir_decimate_f32(&FIR_dec1_Q, float_buffer_R, float_buffer_R, BUFFER_SIZE * N_BLOCKS);
    
  
    arm_fir_decimate_f32(&FIR_dec2_I, float_buffer_L, float_buffer_L, BUFFER_SIZE * N_BLOCKS / (uint32_t)DF1);
    arm_fir_decimate_f32(&FIR_dec2_Q, float_buffer_R, float_buffer_R, BUFFER_SIZE * N_BLOCKS / (uint32_t)DF1);

                                                                               
// ********** Digital FFT Convolution
// Filtering is accomplished by combinig (multiplying) spectra in the frequency domain.
// basis for this was Lyons, R. (2011): Understanding Digital Processing.
// "Fast FIR Filtering using the FFT", pages 688 - 694.
// Method used here: overlap-and-save.

// First, Create Complex time signal for CFFT routine.
// Fill first block with Zeros
// Then interleave RE and IM parts to create signal for FFT
                                                                                                              // Prepare the audio signal buffers:

    if (first_block)                                                                                          // Fill real & imaginaries with zeros for the first BLOCKSIZE samples
    {
      for (unsigned i = 0; i < BUFFER_SIZE * N_BLOCKS / (uint32_t)(DF / 2.0); i++) 
      {
        FFT_buffer[i] = 0.0;
      }
      first_block = 0;
    } 

    else                                                                                                      // Fill FFT_buffer with last events audio samples for all other FFT instances

      for (unsigned i = 0; i < BUFFER_SIZE * N_BLOCKS / (uint32_t)(DF); i++) 
      {
        FFT_buffer[i * 2] = last_sample_buffer_L[i];                                                          // real
        FFT_buffer[i * 2 + 1] = last_sample_buffer_R[i];                                                      // imaginary
      }

    for (unsigned i = 0; i < BUFFER_SIZE * N_BLOCKS / (uint32_t)(DF); i++)                                    // copy recent samples to last_sample_buffer for next time!
    {
      last_sample_buffer_L [i] = float_buffer_L[i];
      last_sample_buffer_R [i] = float_buffer_R[i];
    }

    for (unsigned i = 0; i < BUFFER_SIZE * N_BLOCKS / (uint32_t)(DF); i++)                                    // now fill recent audio samples into FFT_buffer (left channel: re, right channel: im)
    {
      FFT_buffer[FFT_length + i * 2] = float_buffer_L[i];                                                     // real
      FFT_buffer[FFT_length + i * 2 + 1] = float_buffer_R[i];                                                 // imaginary
    }


// ********** Perform complex FFT on the audio time signals
                                                                                                              // calculation is performed in-place in the FFT_buffer [re, im, re, im, re, im . . .]
    arm_cfft_f32(S, FFT_buffer, 0, 1);


// ********** Continuing FFT Convolution
                                                                                                              // complex multiply filter mask with the frequency domain audio data.
                                                                                                              // FIR_filter_mask removes the unwanted sideband
    arm_cmplx_mult_cmplx_f32 (FFT_buffer, FIR_filter_mask, iFFT_buffer, FFT_length);


// ********** Complex Inverse FFT to return to the Time Domain, IFFT is selected by the IFFT flag=1 in the Arm CFFT function.
    
    arm_cfft_f32(iS, iFFT_buffer, 1, 1);


    //dsp_AGC.AGC();


// ********** SSB Demodulation    
    for (unsigned i = 0; i < FFT_length / 2; i++)                               
    {
          float_buffer_L[i] = iFFT_buffer[FFT_length + (i * 2)];                                              // for SSB copy real part in both outputs; AM demodulation not enabled at this time
          float_buffer_R[i] = float_buffer_L[i];
    }


// ********** Interpolation to restore 24-KHz Sample Rate to 192-KHz
                                                                                                              // interpolation-by-2
    arm_fir_interpolate_f32(&FIR_int1_I, float_buffer_L, iFFT_buffer, BUFFER_SIZE * N_BLOCKS / (uint32_t)(DF));
    arm_fir_interpolate_f32(&FIR_int1_Q, float_buffer_R, FFT_buffer, BUFFER_SIZE * N_BLOCKS / (uint32_t)(DF));

                                                                                                              // interpolation-by-4
    arm_fir_interpolate_f32(&FIR_int2_I, iFFT_buffer, float_buffer_L, BUFFER_SIZE * N_BLOCKS / (uint32_t)(DF1));
    arm_fir_interpolate_f32(&FIR_int2_Q, FFT_buffer, float_buffer_R, BUFFER_SIZE * N_BLOCKS / (uint32_t)(DF1));

    arm_scale_f32(float_buffer_L, 500.0, float_buffer_L, BUFFER_SIZE * N_BLOCKS);
    arm_scale_f32(float_buffer_R, 500.0, float_buffer_R, BUFFER_SIZE * N_BLOCKS);


// ********** Convert to Audio and Play
    
    for (unsigned  i = 0; i < N_BLOCKS; i++) 
    {
      sp_L = Q_out_L.getBuffer();
      sp_R = Q_out_R.getBuffer();
      arm_float_to_q15 (&float_buffer_L[BUFFER_SIZE * i], sp_L, BUFFER_SIZE);
      arm_float_to_q15 (&float_buffer_R[BUFFER_SIZE * i], sp_R, BUFFER_SIZE);
      Q_out_L.playBuffer();
      Q_out_R.playBuffer();
    }
  }
}


void DSP_Routines::FreqShift1(void)                                                                           // Rearrange IQ Data to shift rcv signal at 48-KHz IF to near baseband, this process moves DC noise
{                                                                                                             // away from the baseband by 48-KHz. Refer to Lyons ch. 13.1.2.
  for (unsigned i = 0; i < BUFFER_SIZE * N_BLOCKS; i += 4) 
  {
                                                                                                              // Xnew(0) = Xreal(0) + jXimag(0) - so no change to first sample in sequence

    hh1 = - float_buffer_R[i + 1];                                                                            // Xnew(1) = Ximag(1) - jXreal(1)
    hh2 =   float_buffer_L[i + 1];                                              
    float_buffer_L[i + 1] = hh1;                                                
    float_buffer_R[i + 1] = hh2; 

    hh1 = - float_buffer_L[i + 2];                                                                            // Xnew(2) = Xreal(2) - jXimag(2)
    hh2 = - float_buffer_R[i + 2];
    float_buffer_L[i + 2] = hh1;
    float_buffer_R[i + 2] = hh2;

    hh1 =   float_buffer_R[i + 3];                                                                            // Xnew(3) = -Ximag(3) + jXreal(3)
    hh2 = - float_buffer_L[i + 3];
    float_buffer_L[i + 3] = hh1;
    float_buffer_R[i + 3] = hh2;
  }
}


void DSP_Routines::IQPhaseCorrection(float32_t *I_buffer, float32_t *Q_buffer, float32_t factor, uint32_t blocksize)
{
  float32_t temp_buffer[blocksize];
  if (factor < 0.0) {                                                        
    arm_scale_f32 (I_buffer, factor, temp_buffer, blocksize);
    arm_add_f32 (Q_buffer, temp_buffer, Q_buffer, blocksize);
  } else {                                                           
    arm_scale_f32 (Q_buffer, factor, temp_buffer, blocksize);
    arm_add_f32 (I_buffer, temp_buffer, I_buffer, blocksize);
  }
}


int DSP_Routines::SetI2SFreq(int freq) 
{
  int n1;
  int n2 ;
  int c0;
  int c2;
  int c1;
  double C;

  // PLL between 27*24 = 648MHz und 54*24=1296MHz
  // Fudge to handle 8kHz - El Supremo
  if (freq > 8000) {
    n1 = 4; //SAI prescaler 4 => (n1*n2) = multiple of 4
  } else {
    n1 = 8;
  }
  n2 = 1 + (24000000 * 27) / (freq * 256 * n1);
  C = ((double)freq * 256 * n1 * n2) / 24000000;
  c0 = C;
  c2 = 10000;
  c1 = C * c2 - (c0 * c2);
  set_audioClock(c0, c1, c2, true);
  CCM_CS1CDR = (CCM_CS1CDR & ~(CCM_CS1CDR_SAI1_CLK_PRED_MASK | CCM_CS1CDR_SAI1_CLK_PODF_MASK))
               | CCM_CS1CDR_SAI1_CLK_PRED(n1 - 1) // &0x07
               | CCM_CS1CDR_SAI1_CLK_PODF(n2 - 1); // &0x3f

  CCM_CS2CDR = (CCM_CS2CDR & ~(CCM_CS2CDR_SAI2_CLK_PRED_MASK | CCM_CS2CDR_SAI2_CLK_PODF_MASK))
               | CCM_CS2CDR_SAI2_CLK_PRED(n1 - 1) // &0x07
               | CCM_CS2CDR_SAI2_CLK_PODF(n2 - 1); // &0x3f)
  return freq;
}


void DSP_Routines::CalcCplxFIRCoeffs(float * coeffs_I, float * coeffs_Q, int numCoeffs, float32_t FLoCut, float32_t FHiCut, float SampleRate)
{
                                                                                                              //  calculate some normalized filter parameters
  float32_t nFL = FLoCut / SampleRate;
  float32_t nFH = FHiCut / SampleRate;
  float32_t nFc = (nFH - nFL) / 2.0;                                                                          //  prototype LP filter cutoff
  float32_t nFs = PI * (nFH + nFL);                                                                           //  2 PI times required frequency shift (FHiCut+FLoCut)/2
  float32_t fCenter = 0.5 * (float32_t)(numCoeffs - 1);                                                       //  floating point center index of FIR filter
  float32_t x;
  float32_t z;

  memset(coeffs_I, 0.0, (size_t) sizeof(n_dec1_taps));                                                        //  zero entire buffer, important for variables from DMAMEM
  memset(coeffs_Q, 0.0, (size_t) sizeof(n_dec1_taps));

                                                                                                              //  create LP FIR windowed sinc, sin(x)/x complex LP filter coefficients
  for (int i = 0; i < numCoeffs; i++)  {
    x = (float32_t)i - fCenter;
    if ( abs((float)i - fCenter) < 0.01)                                                                      //  deal with odd size filter singularity where sin(0)/0==1
      z = 2.0 * nFc;
    else
      switch (FIR_filter_window) {
        case 1:                                                                                               // 4-term Blackman-Harris --> this is what Power SDR uses
          z = (float32_t)sinf(TWO_PI * x * nFc) / (PI * x) *
              (0.35875 - 0.48829 * cosf( (TWO_PI * i) / (numCoeffs - 1) )
               + 0.14128 * cosf( (FOURPI * i) / (numCoeffs - 1) )
               - 0.01168 * cosf( (SIXPI * i) / (numCoeffs - 1) ) );
          break;

        case 2:
          z = (float32_t)sinf(TWO_PI * x * nFc) / (PI * x) *
              (0.355768 - 0.487396 * cosf( (TWO_PI * i) / (numCoeffs - 1) )
               + 0.144232 * cosf( (FOURPI * i) / (numCoeffs - 1) )
               - 0.012604 * cosf( (SIXPI * i) / (numCoeffs - 1) ) );
          break;

        case 3: // cosine
          z = (float32_t)sinf(TWO_PI * x * nFc) / (PI * x) *
              cosf((PI * (float32_t)i) / (numCoeffs - 1));
          break;

        case 4: // Hann
          z = (float32_t)sinf(TWO_PI * x * nFc) / (PI * x) *
              0.5 * (float32_t)(1.0 - (cosf(PI * 2 * (float32_t)i / (float32_t)(numCoeffs - 1))));
          break;
        default: // Blackman-Nuttall window
          z = (float32_t)sinf(TWO_PI * x * nFc) / (PI * x) *
              (0.3635819
               - 0.4891775 * cosf( (TWO_PI * i) / (numCoeffs - 1) )
               + 0.1365995 * cosf( (FOURPI * i) / (numCoeffs - 1) )
               - 0.0106411 * cosf( (SIXPI * i) / (numCoeffs - 1) ) );
          break;
      }
                                                                                                              //  shift lowpass filter coefficients in frequency by (hicut+lowcut)/2 to form bandpass filter anywhere in range
    coeffs_I[i]   = z * cosf(nFs * x);
    coeffs_Q[i]   = z * sinf(nFs * x);
  }
}


void DSP_Routines::InitFilterMask()
{                                                                               
// ********** Calculate the FFT of the FIR filter coefficients once to produce the FIR filter mask
// The FIR has exactly m_NumTaps and a maximum of (FFT_length / 2) + 1 taps = coefficients, 
// so we have to add (FFT_length / 2) -1 zeros before the FFT in order to produce a FFT_length point input buffer
// for the FFT. Copy coefficients into real values of first part of buffer, rest is zero.

  for (unsigned i = 0; i < m_NumTaps; i++)
  {
                                                                                                              // try out a window function to eliminate ringing of the filter at the stop frequency
                                                                                                              // sd.FFT_Samples[i] = (float32_t)((0.53836 - (0.46164 * arm_cos_f32(PI*2 * (float32_t)i / (float32_t)(FFT_IQ_BUFF_LEN-1)))) * sd.FFT_Samples[i]);
    FIR_filter_mask[i * 2] = FIR_Coef_I [i];
    FIR_filter_mask[i * 2 + 1] = FIR_Coef_Q [i];
  }

  for (unsigned i = FFT_length + 1; i < FFT_length * 2; i++)
  {
    FIR_filter_mask[i] = 0.0;
  }
                                                                                                              // FFT of FIR_filter_mask
                                                                                                              // perform FFT (in-place), needs only to be done once (or every time the filter coeffs change)
  arm_cfft_f32(maskS, FIR_filter_mask, 0, 1);
}


void DSP_Routines::SetDecIntFilters(void)
{                                                                               
// ********** Calculate RCV decimation and interpolation FIR filters

  CalcFIRCoeffs(FIR_dec1_coeffs, n_dec1_taps, (float32_t)(LP_F_help), n_att, 0, 0.0, (float32_t)(sampleRate));
  CalcFIRCoeffs(FIR_dec2_coeffs, n_dec2_taps, (float32_t)(LP_F_help), n_att, 0, 0.0, (float32_t)(sampleRate / DF1));

  CalcFIRCoeffs(FIR_int1_coeffs, 48, (float32_t)(LP_F_help), n_att, 0, 0.0, (float32_t)(sampleRate / DF1));
  CalcFIRCoeffs(FIR_int2_coeffs, 32, (float32_t)(LP_F_help), n_att, 0, 0.0, (float32_t)(sampleRate));
  
  //bin_BW = 1.0 / (DF * FFT_length) * (float32_t)sampleRate;
}


void DSP_Routines::SetIIRCoeffs(float32_t f0, float32_t Q, float32_t sample_rate, uint8_t filter_type)
                                         
// ********** Cascaded biquad (notch, peak, lowShelf, highShelf) [DD4WH, april 2016]
// DSP Audio-EQ-cookbook for generating the coeffs of the filters on the fly
// www.musicdsp.org/files/Audio-EQ-Cookbook.txt  [by Robert Bristow-Johnson]
// https://www.w3.org/2011/audio/audio-eq-cookbook.html
// the ARM algorithm assumes the biquad form
// y[n] = b0 * x[n] + b1 * x[n-1] + b2 * x[n-2] + a1 * y[n-1] + a2 * y[n-2]
//
// However, the cookbook formulae by Robert Bristow-Johnson AND the Iowa Hills IIR Filter designer
// use this formula:
//
// y[n] = b0 * x[n] + b1 * x[n-1] + b2 * x[n-2] - a1 * y[n-1] - a2 * y[n-2]
//
// Therefore, we have to use negated a1 and a2 for use with the ARM function

{
  if (f0 > sample_rate / 2.0) f0 = sample_rate / 2.0;
  float32_t w0 = f0 * (TPI / sample_rate);
  float32_t sinW0 = sinf(w0);
  float32_t alpha = sinW0 / (Q * 2.0);
  float32_t cosW0 = cosf(w0);
  float32_t scale = 1.0 / (1.0 + alpha);

  if (filter_type == 0) 
  {                                                                             // lowpass coeffs
    coefficient_set[0] = ((1.0 - cosW0) / 2.0) * scale;                         // b0
    coefficient_set[1] = (1.0 - cosW0) * scale;                                 // b1
    coefficient_set[2] = coefficient_set[0];                                    // b2
    coefficient_set[3] = (2.0 * cosW0) * scale;                                 // negated a1
    coefficient_set[4] = (-1.0 + alpha) * scale;                                // negated a2
  } 

  else if (filter_type == 2) 
  {
                                                                                // ??
  } 

  else if (filter_type == 3) 
  {                                                                             // notch
    coefficient_set[0] =  1.0;                                                  // b0 
    coefficient_set[1] =  - 2.0 * cosW0;                                        // b1 
    coefficient_set[2] =  1.0;                                                  // b2 
    coefficient_set[3] =  2.0 * cosW0 * scale;                                  // negated a1
    coefficient_set[4] =  alpha - 1.0;                                          // negated a2
  }
}


void DSP_Routines::CalcFIRCoeffs(float *coeffs_I, int numCoeffs, float32_t fc, float32_t Astop, int type, float dfc, float Fsamprate)

// modified by WMXZ and DD4WH after
// Wheatley, M. (2011): CuteSDR Technical Manual. www.metronix.com, pages 118 - 120, FIR with Kaiser-Bessel Window
// assess required number of coefficients by
//     numCoeffs = (Astop - 8.0) / (2.285 * TPI * normFtrans);
// selecting high-pass, numCoeffs is forced to an even number for better frequency response

{ 
  int nc    = numCoeffs;
  float32_t Beta;
  float32_t izb;
  float fcf = fc;
  float x, w;
  fc        = fc / Fsamprate;
  dfc       = dfc / Fsamprate;

                                                                                // calculate Kaiser-Bessel window shape factor beta from stop-band attenuation
  if (Astop < 20.96) 
  {
    Beta = 0.0;
  } 

  else 
  {
    if (Astop >= 50.0) 
    {
      Beta = 0.1102 * (Astop - 8.71);
    } else {
      Beta = 0.5842 * powf((Astop - 20.96), 0.4) + 0.07886 * (Astop - 20.96);
    }
  }
  
  memset(coeffs_I, 0.0, sizeof(n_dec1_taps));                                   //zero entire buffer, important for variables from DMAMEM

  izb = Izero(Beta);

  if (type == 0)                                                                // low pass filter
  {
    fcf = fc * 2.0;
    nc  =  numCoeffs;
  } 

  else if (type == 1)                                                           // high-pass filter
  {
    fcf = -fc;
    nc  =  2 * (numCoeffs / 2);
  } 

  else if ((type == 2) || (type == 3))                                          // band-pass filter
  {
    fcf = dfc;
    nc  =  2 * (numCoeffs / 2);                                                 // maybe not needed
  } 

  else if (type == 4)                                                           // Hilbert transform
  {
    nc  =  2 * (numCoeffs / 2);
                                                                                // clear coefficients
    for (int ii = 0; ii < 2 * (nc - 1); ii++) 
    {
      coeffs_I[ii] = 0;
    }
    
    coeffs_I[nc] = 1;                                                           // set real delay
    for (int ii = 1; ii < (nc + 1); ii += 2)                                    // set imaginary Hilbert coefficients
    { 
      if (2 * ii == nc) continue;
      x = (float)(2 * ii - nc) / (float)nc;
      w = Izero(Beta * sqrtf(1.0f - x * x)) / izb; // Kaiser window
      coeffs_I[2 * ii + 1] = 1.0f / (PIH * (float)(ii - nc / 2)) * w ;
    }
    return;
  }

  for (int ii = - nc, jj = 0; ii < nc; ii += 2, jj++) 
  {
    x = (float)ii / (float)nc;
    w = Izero(Beta * sqrtf(1.0f - x * x)) / izb;                                // Kaiser window
    coeffs_I[jj] = fcf * MSinc(ii, fcf) * w;
  }

  if (type == 1) 
  {
    coeffs_I[nc / 2] += 1;
  } 

  else if (type == 2) 
  {
    for (int jj = 0; jj < nc + 1; jj++) coeffs_I[jj] *= 2.0f * cosf(PIH * (2 * jj - nc) * fc);
  } 

  else if (type == 3) 
  {
    for (int jj = 0; jj < nc + 1; jj++) coeffs_I[jj] *= -2.0f * cosf(PIH * (2 * jj - nc) * fc);
    coeffs_I[nc / 2] += 1;
  }

}


float DSP_Routines::MSinc(int m, float fc)                                      // calculate sinc function
{
  float x = m * PIH;
  if (m == 0)
    return 1.0f;
  else
    return sinf(x * fc) / (fc * x);
}


float32_t DSP_Routines::Izero(float32_t x)
{
  float32_t x2          = x / 2.0;
  float32_t summe       = 1.0;
  float32_t ds          = 1.0;
  float32_t di          = 1.0;
  float32_t errorlimit  = 1e-9;
  float32_t tmp;

  do
  {
    tmp = x2 / di;
    tmp *= tmp;
    ds *= tmp;
    summe += ds;
    di += 1.0;
  } while (ds >= errorlimit * summe);
  return (summe);
}  // END Izero


void DSP_Routines::MyDelay(unsigned long millisWait)
{
  unsigned long now = millis();

  while (millis() - now < millisWait);
}