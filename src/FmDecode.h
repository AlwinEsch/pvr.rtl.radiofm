#pragma once
/*
 *      Copyright (C) 2013, Joris van Rantwijk.
 *      Copyright (C) 2005-2015 Team KODI (Alwin Esch)
 *      http://kodi.tv
 *
 *  This Program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2, or (at your option)
 *  any later version.
 *
 *  This Program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this Software; see the file COPYING.  If not, write to
 *  the Free Software Foundation, 675 Mass Ave, Cambridge, MA 02139, USA.
 *  http://www.gnu.org/copyleft/gpl.html
 *
 *  Several parts of code are taken from SoftFM created by Joris van Rantwijk
 *  (Copyright 2013) and handled by GNU GPL
 */

#include <stdint.h>
#include <vector>

#include "DownConvert.h"
#include "FirFilter.h"
#include "IirFilter.h"
#include "RDSProcess.h"

#define DEFAULT_DEEMPHASIS         50.0
#define DEFAULT_BANDWIDTH_IF   100000.0
#define DEFAULT_FREQ_DEV        60000.0
#define DEFAULT_BANDWIDTH_PCM   15000.0
#define PILOT_FREQ              19000.0

class cRadioReceiver;

/** Fine tuner which shifts the frequency of an IQ signal by a fixed offset. */
class cFineTuner
{
public:
  /*!
   * Construct fine tuner.
   *
   * table_size :: Size of internal sin/cos tables, determines the resolution
   *               of the frequency shift.
   *
   * freq_shift :: Frequency shift. Signal frequency will be shifted by
   *               (sample_rate * freq_shift / table_size).
   */
  cFineTuner(unsigned int table_size, int freq_shift);
  virtual ~cFineTuner();

  /*! Process samples. */
  void Process(const ComplexType *samples_in, ComplexType *samples_out, unsigned int samples);

private:
  unsigned int    m_index;
  ComplexType    *m_table;
  unsigned int    m_tableSize;
};

/*! Phase-locked loop for stereo pilot. */
class cPilotPhaseLock
{
public:
  /*!
   * Construct phase-locked loop.
   *
   * freq       :: 19 kHz center frequency relative to sample freq
   *               (0.5 is Nyquist)
   * bandwidth  :: bandwidth relative to sample frequency
   * minsignal  :: minimum pilot amplitude
   */
  cPilotPhaseLock(double freq, double bandwidth, double minsignal);

  /*!
   * Process samples and extract 19 kHz pilot tone.
   * Generate phase-locked 38 kHz tone with unit amplitude.
   */
  bool Process(const RealType *samples_in, RealType *samples_out, unsigned int length);

  /*! Return detected amplitude of pilot signal. */
  double GetPilotLevel() const
  {
    return 2 * m_pilotLevel;
  }

private:
  RealType  m_minfreq, m_maxfreq;
  RealType  m_phasor_b0, m_phasor_a1, m_phasor_a2;
  RealType  m_phasor_i1, m_phasor_i2, m_phasor_q1, m_phasor_q2;
  RealType  m_loopfilter_b0, m_loopfilter_b1;
  RealType  m_loopfilter_x1;
  RealType  m_freq, m_phase;
  RealType  m_minsignal;
  RealType  m_pilotLevel;
  int     m_lock_delay;
  int     m_lock_cnt;
};

/*! Complete decoder for FM broadcast signal. */
class cFmDecoder
{
public:
  /*!
   * Construct FM decoder.
   *
   * sample_rate_if   :: IQ sample rate in Hz.
   * tuning_offset    :: Frequency offset in Hz of radio station with respect
   *                     to receiver LO frequency (positive value means
   *                     station is at higher frequency than LO).
   * sample_rate_pcm  :: Audio sample rate.
   * stereo           :: True to enable stereo decoding.
   * freq_dev         :: Full scale carrier frequency deviation
   *                     (75 kHz for broadcast FM)
   * bandwidth_pcm    :: Half bandwidth of audio signal in Hz
   *                     (15 kHz for broadcast FM)
   * downsample       :: Downsampling factor to apply after FM demodulation.
   *                     Set to 1 to disable.
   */
  cFmDecoder(cRadioReceiver *proc,
            double sample_rate_if,
            double tuning_offset,
            double sample_rate_pcm,
            double bandwidth_pcm=DEFAULT_BANDWIDTH_PCM,
            unsigned int downsample=1,
            bool USver=false);

  virtual ~cFmDecoder();

  void Reset();

  /*!
   * Process IQ samples and return audio samples.
   *
   * If the decoder is set in stereo mode, samples for left and right
   * channels are interleaved in the output vector (even if no stereo
   * signal is detected). If the decoder is set in mono mode, the output
   * vector only contains samples for one channel.
   * @param samples_in array which include recieved signal
   * @param samples the amount of samples in data
   * @param audio pointer to return data array (is allocated with samples_in
   * size and is always enough)
   * @return the amount of present samples in audio
   */
  unsigned int ProcessStream(const ComplexType *samples_in, unsigned int samples, float *audio);

  /*!
   * Return true if a stereo signal is detected.
   */
  bool StereoDetected() const
  {
    return m_StereoDetected;
  }

  /*!
   * Return actual frequency offset in Hz with respect to receiver LO.
   * @return frequency
   */
  double GetTuningOffset() const
  {
    double tuned = - m_TuningShift * m_SampleRate_Interface / double(m_TuningTableSize);
    return tuned + m_BasebandMean * m_FrequencyDev;
  }

  /*!
   * Return RMS IF level (where full scale IQ signal is 1.0).
   */
  double GetInterfaceLevel() const
  {
    return m_InterfaceLevel;
  }

  /*!
   * Return RMS baseband signal level (where nominal level is 0.707).
   */
  double GetBasebandLevel() const
  {
    return m_BasebandLevel;
  }

  /*!
   * Return amplitude of stereo pilot (nominal level is 0.1).
   */
  double GetPilotLevel() const
  {
    return m_PilotPLL.GetPilotLevel();
  }

private:
  void InitDeemphasis(RealType Time, RealType SampleRate);

  inline void SamplesMeanRMS(const RealType* samples, double& mean, double& rms, unsigned int n);
  inline ComplexType::value_type RMSLevelApprox(const ComplexType *samples, unsigned int length);
  inline void ProcessDeemphasisFilter(RealType* bufferA, RealType* bufferB, unsigned int length);
  inline void PhaseLockedLoop(ComplexType *signal, RealType *out, unsigned int dataSize);

   cRadioReceiver          *m_proc;
  const double              m_SampleRate_Interface;
  const double              m_SampleRate_Baseband;
  const int                 m_TuningTableSize;
  const int                 m_TuningShift;
  const double              m_FrequencyDev;
  const unsigned int        m_Downsample;
  const double              m_BandwidthInterface;

  bool                      m_StereoDetected;
  double                    m_InterfaceLevel;
  double                    m_BasebandMean;
  double                    m_BasebandLevel;
  double                    m_AudioLevel;
  double                    m_FMDeModGain;

  ComplexType              *m_BufferIfTuned;
  ComplexType              *m_BufferDemod;
  RealType                 *m_BufferBaseband;
  RealType                 *m_BufferMono;
  RealType                 *m_BufferStereo;
  RealType                 *m_BufferRawStereo;

  cFineTuner                m_FineTuner;
  cPilotPhaseLock           m_PilotPLL;
  cDownsampleFilter         m_ReSampleInput;
  cDownsampleFilter         m_ReSampleMono;
  cDownsampleFilter         m_ReSampleStereo;
  cRDSRxSignalProcessor     m_RDSProcess;
  cIirFilter                m_DCBlock;
  cIirFilter                m_NotchFilter;
  cFirFilter                m_LPFilter;
  cFirFilter                m_InputLPFilter;

  RealType                  m_DeemphasisAveRe;
  RealType                  m_DeemphasisAveIm;
  RealType                  m_DeemphasisAlpha;

  RealType                  m_NcoPhase;
  RealType                  m_NcoPhaseIncr;
  RealType                  m_NcoHLimit;
  RealType                  m_NcoLLimit;
  RealType                  m_PLLAlpha;
  RealType                  m_PLLBeta;
  RealType                  m_DemodDCOffset;
};
