# SDR-Transceiver-QSD-QSE-LO-Teensy-4.1-DSP-Audio-Board

The QSD-QSE-LO-Teensy-4.1-DSP-Audio-Board processes RF to audio in the receiver section and audio to RF in the transmitter section of the 2022 SDR Transceiver. Refer to www.ad5gh.com for further details.

This depository contains the Teensy 4.1 software.

The heart of the SDR-Transceiver-QSD-QSE-LO-Teensy-4.1-DSP-Audio-Board is the Teensy 4.1 from PJRC which is based on a Cortex-M7 processor running at 600-MHz. The Teensy 4.1 runs Digital Signal Processing (DSP) software based upon the Convolution SDR software written by Frank Dziock, DD4WH, along with other contributors, that has been significantly modified by Albert Peter, AC8GY, and Jack Purdum, W8TEE for the T41-EP. The software uses routines from the Teensy Audio Library developed by Paul Stoffregen and relies heavily on the CMSIS DSP Software Library for Cortex-M processor based devices. Al and Jack’s code has been significantly refactored to extract the core DSP software necessary for processing receiver 48-KHz In-phase (“I“) and Quadrature (“Q“) IF to audio, and transmit audio to IQ audio; including, in the receiver, IF filtering, frequency conversion from 48-KHz IF to audio, unwanted sideband filtering, and SSB demodulation; and in the transmitter, low pass filtering of the audio, and audio phase shift to provide I and Q transmit audio streams. Refer to AL and Jack’s book for a description of the full software package for the T41-EP.

