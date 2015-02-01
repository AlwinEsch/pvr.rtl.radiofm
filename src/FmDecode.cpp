/*
 *      Copyright (C) 2013, Joris van Rantwijk.
 *      Copyright (C) 2015, Alwin Esch (Team KODI)
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
 *  along with KODI; see the file COPYING.  If not, write to
 *  the Free Software Foundation, 51 Franklin Street, Fifth Floor, Boston,
 *  MA 02110-1301  USA
 *  http://www.gnu.org/copyleft/gpl.html
 *
 *  Several parts of code are taken from SoftFM created by Joris van Rantwijk
 *  (Copyright 2013) and handled by GNU GPL
 */

#include <assert.h>
#include <math.h>

#include "FmDecode.h"
#include "RadioReceiver.h"
#include "RTL_SDR_Source.h"

using namespace std;
using namespace ADDON;

/* ****************  class cFineTuner  **************** */

// Construct finetuner.
cFineTuner::cFineTuner(unsigned int table_size, int freq_shift)
  : m_index(0)
  , m_tableSize(table_size)
{
  m_table = new ComplexType[table_size];

  double phase_step = K_2PI / double(table_size);
  for (unsigned int i = 0; i < table_size; i++)
  {
    double phi = (((int64_t)freq_shift * i) % table_size) * phase_step;
    double pcos = MCOS(phi);
    double psin = MSIN(phi);
    m_table[i] = ComplexType(pcos, psin) * 2.0;
  }
}

cFineTuner::~cFineTuner()
{
  delete[] m_table;
}

// Process samples.
void cFineTuner::Process(const ComplexType *samples_in, ComplexType *samples_out, unsigned int samples)
{
  unsigned int tblidx = m_index;
  unsigned int tblsiz = m_tableSize;

  for (unsigned int i = 0; i < samples; i++)
  {
    samples_out[i] = samples_in[i] * m_table[tblidx];
    tblidx++;
    if (tblidx == tblsiz)
      tblidx = 0;
  }

  m_index = tblidx;
}


/* ****************  class cPilotPhaseLock  **************** */

// Construct phase-locked loop.
cPilotPhaseLock::cPilotPhaseLock(double freq, double bandwidth, double minsignal)
{
  /*!
   * This is a type-2, 4th order phase-locked loop.
   *
   * Open-loop transfer function:
   *   G(z) = K * (z - q1) / ((z - p1) * (z - p2) * (z - 1) * (z - 1))
   *   K  = 3.788 * (bandwidth * 2 * Pi)**3
   *   q1 = exp(-0.1153 * bandwidth * 2*Pi)
   *   p1 = exp(-1.146 * bandwidth * 2*Pi)
   *   p2 = exp(-5.331 * bandwidth * 2*Pi)
   *
   * I don't understand what I'm doing; hopefully it will work.
   */

  //! Set min/max locking frequencies.
  m_minfreq = (freq - bandwidth) * K_2PI;
  m_maxfreq = (freq + bandwidth) * K_2PI;

  //! Set valid signal threshold.
  m_minsignal  = minsignal;
  m_lock_delay = int(20.0 / bandwidth);
  m_lock_cnt   = 0;

  //! Create 2nd order filter for I/Q representation of phase error.
  //! Filter has two poles, unit DC gain.
  double p1 = exp(-1.146 * bandwidth * K_2PI);
  double p2 = exp(-5.331 * bandwidth * K_2PI);
  m_phasor_a1 = - p1 - p2;
  m_phasor_a2 =   p1 * p2;
  m_phasor_b0 = 1 + m_phasor_a1 + m_phasor_a2;

  //! Create loop filter to stabilize the loop.
  m_loopfilter_b0 = 0.62 * bandwidth * K_2PI;
  m_loopfilter_b1 = - m_loopfilter_b0 * exp(-0.1153 * bandwidth * K_2PI);

  /*!
   * After the loop filter, the phase error is integrated to produce
   * the frequency. Then the frequency is integrated to produce the phase.
   * These integrators form the two remaining poles, both at z = 1.
   */

  //! Initialize frequency and phase.
  m_freq  = freq * K_2PI;
  m_phase = 0;

  m_phasor_i1 = 0;
  m_phasor_i2 = 0;
  m_phasor_q1 = 0;
  m_phasor_q2 = 0;
  m_loopfilter_x1 = 0;
  m_pilotLevel = 0;
}


bool cPilotPhaseLock::Process(const RealType *samples_in, RealType *samples_out, unsigned int length)
{
  m_pilotLevel = 1000.0;

  RealType psin;
  RealType pcos;
  for (unsigned int i = 0; i < length; i++)
  {
    //! Generate locked pilot tone.
#if TARGET_WINDOWS
    RealType*  pdCosAns  = &pcos;
    RealType*  pdSinAns  = &psin;
    _asm
    {
      fld QWORD PTR [m_phase]
      fsincos
      mov ebx,[pdCosAns]    ;  get the pointer into ebx
      fstp QWORD PTR [ebx]  ;  store the result through the pointer
      mov ebx,[pdSinAns]
      fstp QWORD PTR [ebx]
    }
#elif (defined(__i386__) || defined(__x86_64__))
    asm volatile ("fsincos" : "=%&t" (pcos), "=%&u" (psin) : "0" (m_phase));
#else
    psin = MSIN(m_phase);    //178ns for sin/cos calc
    pcos = MCOS(m_phase);
#endif

    //! Generate double-frequency output.
    //! sin(2*x) = 2 * sin(x) * cos(x)
    samples_out[i] = 2 * psin * pcos;

    //! Multiply locked tone with input.
    RealType x = samples_in[i];
    RealType phasor_i = psin * x;
    RealType phasor_q = pcos * x;

    //! Run IQ phase error through low-pass filter.
    phasor_i = m_phasor_b0 * phasor_i
               - m_phasor_a1 * m_phasor_i1
               - m_phasor_a2 * m_phasor_i2;
    phasor_q = m_phasor_b0 * phasor_q
               - m_phasor_a1 * m_phasor_q1
               - m_phasor_a2 * m_phasor_q2;
    m_phasor_i2 = m_phasor_i1;
    m_phasor_i1 = phasor_i;
    m_phasor_q2 = m_phasor_q1;
    m_phasor_q1 = phasor_q;

    //! Convert I/Q ratio to estimate of phase error.
    RealType phase_err;
    if (phasor_i > abs(phasor_q))
      phase_err = phasor_q / phasor_i;            //!< We are within +/- 45 degrees from lock. Use simple linear approximation of arctan.
    else if (phasor_q > 0)
      phase_err = 1;            //!< We are lagging more than 45 degrees behind the input.
    else
      phase_err = -1;            //!< We are more than 45 degrees ahead of the input.

    //! Detect pilot level (conservative).
    m_pilotLevel = min(m_pilotLevel, phasor_i);

    //! Run phase error through loop filter and update frequency estimate.
    m_freq += m_loopfilter_b0 * phase_err + m_loopfilter_b1 * m_loopfilter_x1;
    m_loopfilter_x1 = phase_err;

    //! Limit frequency to allowable range.
    m_freq = max(m_minfreq, min(m_maxfreq, m_freq));

    //! Update locked phase.
    m_phase += m_freq;
    if (m_phase > K_2PI)
      m_phase -= K_2PI;
  }

  //! Update lock status.
  if (2 * m_pilotLevel > m_minsignal)
  {
    if (m_lock_cnt < m_lock_delay)
        m_lock_cnt += length;
  }
  else
    m_lock_cnt = 0;

  return m_lock_cnt >= m_lock_delay;
}


/* ****************  class cFmDecoder  **************** */

#define PILOTPLL_FREQ 19000.0  //Centerfreq
#define  RDS_DECIMATOR  8

cFmDecoder::cFmDecoder(cRadioReceiver *proc,
                     double sample_rate_if,
                     double tuning_offset,
                     double sample_rate_pcm,
                     double bandwidth_pcm,
                     unsigned int downsample,
                     bool USver)

  // Initialize member fields
  : m_proc(proc)
  , m_SampleRate_Interface(sample_rate_if)
  , m_SampleRate_Baseband(sample_rate_if / downsample)
  , m_TuningTableSize(64)
  , m_TuningShift(lrint(-64.0 * tuning_offset / sample_rate_if))
  , m_FrequencyDev(DEFAULT_FREQ_DEV)
  , m_Downsample(downsample)
  , m_BandwidthInterface(DEFAULT_BANDWIDTH_IF)
  , m_FMDeModGain(1.0 / (DEFAULT_FREQ_DEV / m_SampleRate_Baseband * K_2PI))
  , m_FineTuner(m_TuningTableSize, m_TuningShift)       //!< Construct FineTuner
  , m_PilotPLL(                                         //!< Construct cPilotPhaseLock
         PILOT_FREQ / m_SampleRate_Baseband,      // freq
         50 / m_SampleRate_Baseband,              // bandwidth
         0.04)                                    // minsignal
  , m_ReSampleInput(8 * downsample, 0.4 / downsample, downsample, true)
  , m_ReSampleMono(                                    //!< Construct DownsampleFilter for mono channel
        int(m_SampleRate_Baseband / 1000.0),               // filter_order
        bandwidth_pcm / m_SampleRate_Baseband,             // cutoff
        m_SampleRate_Baseband / sample_rate_pcm,           // downsample
        false)                                              // integer_factor
  , m_ReSampleStereo(                                  //!< Construct DownsampleFilter for stereo channel
        int(m_SampleRate_Baseband / 1000.0),               // filter_order
        bandwidth_pcm / m_SampleRate_Baseband,             // cutoff
        m_SampleRate_Baseband / sample_rate_pcm,           // downsample
        false)                                              // integer_factor
  , m_RDSProcess(proc, m_SampleRate_Baseband)
{
  m_BufferIfTuned     = new ComplexType[cRtlSdrSource::default_block_length];
  m_BufferDemod       = new ComplexType[cRtlSdrSource::default_block_length];
  m_BufferBaseband    = new RealType[cRtlSdrSource::default_block_length];
  m_BufferMono        = new RealType[cRtlSdrSource::default_block_length];
  m_BufferStereo      = new RealType[cRtlSdrSource::default_block_length];
  m_BufferRawStereo   = new RealType[cRtlSdrSource::default_block_length];

  m_DCBlock.Init(ftHP, 30.0, 2.0, sample_rate_pcm);
  m_NotchFilter.Init(ftBR, PILOTPLL_FREQ, 5, sample_rate_pcm);
  m_LPFilter.InitLPFilter(0, 1.0, 60.0, 15000.0, 1.4*15000.0, sample_rate_pcm);

  /*!
   * create deemphasis filter with 75uSec or 50uSec LP corner
   */
  if (USver)
    InitDeemphasis(75E-6, sample_rate_pcm);
  else
    InitDeemphasis(50E-6, sample_rate_pcm);

  /*!
   * Init phase-locked loop (PLL)
   */
  RealType fac        = K_2PI / m_SampleRate_Baseband;
  RealType bandwidth  = 0.85 * m_SampleRate_Baseband;
  RealType maxFreqDev = 0.95 * (0.5 * m_SampleRate_Baseband);

  m_NcoLLimit          = (-maxFreqDev) * fac;              //!< boundary for changes
  m_NcoHLimit          = (+maxFreqDev) * fac;
  m_PLLAlpha          = 0.125 * bandwidth * fac;          //!< pll bandwidth
  m_PLLBeta           = (m_PLLAlpha * m_PLLAlpha) / 2.0;  //!< second order term
  Reset();
}

cFmDecoder::~cFmDecoder()
{
  delete[] m_BufferIfTuned;
  delete[] m_BufferDemod;
  delete[] m_BufferBaseband;
  delete[] m_BufferMono;
  delete[] m_BufferStereo;
  delete[] m_BufferRawStereo;
}

void cFmDecoder::Reset()
{
  m_StereoDetected    = false;
  m_InterfaceLevel    = 0;
  m_BasebandMean      = 0;
  m_BasebandLevel     = 0;
  m_AudioLevel        = 0;
  m_DemodDCOffset     = 0;
  m_NcoPhaseIncr      = 0.0;                              //!< this will change during runs
  m_NcoPhase          = 0.0;

  m_RDSProcess.Reset();
}

void cFmDecoder::InitDeemphasis(RealType Time, RealType SampleRate)  //!< create De-emphasis LP filter
{
  m_DeemphasisAlpha = (1.0 - MEXP(-1.0 / (SampleRate * Time)));
  m_DeemphasisAveRe = 0.0;
  m_DeemphasisAveIm = 0.0;
}

void cFmDecoder::ProcessDeemphasisFilter(RealType* bufferA, RealType* bufferB, unsigned int length)
{
  for (unsigned int i = 0; i < length; i++)
  {
    m_DeemphasisAveRe = (1.0 - m_DeemphasisAlpha) * m_DeemphasisAveRe + m_DeemphasisAlpha * bufferA[i];
    bufferA[i] = m_DeemphasisAveRe * 2.0;
    m_DeemphasisAveIm = (1.0 - m_DeemphasisAlpha) * m_DeemphasisAveIm + m_DeemphasisAlpha * bufferB[i];
    bufferB[i] = m_DeemphasisAveIm * 2.0;
  }
}

#define  DCAlpha  0.0001
void cFmDecoder::PhaseLockedLoop(ComplexType *signal, RealType *out, unsigned int dataSize)
{
  ComplexType pll_Delay;
  RealType    phzError;
  RealType    phaseIncr;
  RealType    Sin;
  RealType    Cos;

  RealType dcOffset = m_DemodDCOffset;
  for (unsigned int i = 0; i < dataSize; i++)
  {
#if TARGET_WINDOWS
    RealType*  pdCosAns  = &Sin;
    RealType*  pdSinAns  = &Cos;
    _asm
    {
      fld QWORD PTR [m_NcoPhase]
      fsincos
      mov ebx,[pdCosAns]    ;  get the pointer into ebx
      fstp QWORD PTR [ebx]  ;  store the result through the pointer
      mov ebx,[pdSinAns]
      fstp QWORD PTR [ebx]
    }
#elif (defined(__i386__) || defined(__x86_64__))
    asm volatile ("fsincos" : "=%&t" (Cos), "=%&u" (Sin) : "0" (m_NcoPhase));  //126nS
#else
    Sin = MSIN(m_NcoPhase);    //178ns for sin/cos calc
    Cos = MCOS(m_NcoPhase);
#endif

    pll_Delay = ComplexType(Cos, Sin) * signal[i];
    phzError  = -atan2(imag(pll_Delay), real(pll_Delay));

    m_NcoPhaseIncr += m_PLLBeta * phzError;
    if (m_NcoPhaseIncr < m_NcoLLimit)
       m_NcoPhaseIncr = m_NcoLLimit;
    if (m_NcoPhaseIncr > m_NcoHLimit)
       m_NcoPhaseIncr = m_NcoHLimit;

    m_NcoPhase += m_NcoPhaseIncr + m_PLLAlpha * phzError;
    if (m_NcoPhase >= K_2PI)
      m_NcoPhase = fmod(m_NcoPhase, K_2PI);
    while (m_NcoPhase < 0)
      m_NcoPhase += K_2PI;

    phaseIncr = 2 * m_NcoPhaseIncr;
    dcOffset = (1 - DCAlpha) * dcOffset + DCAlpha * phaseIncr; //!<  lowpass the NCO frequency term to get a DC offset
    out[i] = (phaseIncr - dcOffset) * m_FMDeModGain;
  }
  m_DemodDCOffset = dcOffset;
}

unsigned int cFmDecoder::ProcessStream(const ComplexType *samples_in, unsigned int samples, float *audio)
{
  unsigned int dataSize = samples;

  //! Fine tuning.
  m_FineTuner.Process(samples_in, m_BufferIfTuned, dataSize);

  //! Measure IF level.
  m_InterfaceLevel = 0.95 * m_InterfaceLevel + 0.05 * RMSLevelApprox(m_BufferIfTuned, dataSize);

  //! Perform first downsample to match 100000 Hz Format
  dataSize = m_ReSampleInput.Process(m_BufferIfTuned, m_BufferDemod, dataSize);

  //! Extract carrier frequency.
  PhaseLockedLoop(m_BufferDemod, m_BufferBaseband, dataSize);

  //! Handle RDS signal
//  m_RDSProcess.Process(m_BufferBaseband, dataSize);

  //! Measure baseband level.
  double baseband_mean, baseband_rms;
  SamplesMeanRMS(m_BufferBaseband, baseband_mean, baseband_rms, dataSize);
  m_BasebandMean  = 0.95 * m_BasebandMean + 0.05 * baseband_mean;
  m_BasebandLevel = 0.95 * m_BasebandLevel + 0.05 * baseband_rms;

  //! Extract mono audio signal.
  unsigned int monoSize = m_ReSampleMono.Process(m_BufferBaseband, m_BufferMono, dataSize);

  //! Lock on stereo pilot.
  m_StereoDetected = m_PilotPLL.Process(m_BufferBaseband, m_BufferRawStereo, dataSize);

  /*!
   * Just multiply the baseband signal with the double-frequency pilot.
   * And multiply by two to get the full amplitude.
   * That's all.
   */
  for (unsigned int i = 0; i < dataSize; i++)
    m_BufferRawStereo[i] *= 2 * m_BufferBaseband[i];

  /*!
   * Extract audio and downsample.
   * @note This MUST be done even if no stereo signal is detected yet,
   * because the downsamplers for mono and stereo signal must be
   * kept in sync.
   */
  dataSize = m_ReSampleStereo.Process(m_BufferRawStereo, m_BufferStereo, dataSize);

  /*!
   * DC blocking and de-emphasis.
   */
  m_LPFilter.ProcessTwo(m_BufferStereo, m_BufferMono, dataSize);
  ProcessDeemphasisFilter(m_BufferStereo, m_BufferMono, dataSize);
  m_NotchFilter.ProcessTwo(m_BufferStereo, m_BufferMono, dataSize);

  if (m_StereoDetected)
  {
    assert(dataSize == monoSize);

    /*!>
     * Extract left/right channels from mono/stereo signals.
     */
    for (unsigned int i = 0; i < dataSize; i++)
    {
      float m = m_BufferMono[i];
      float s = m_BufferStereo[i];
      audio[2*i]   = (m + s) * 0.5;
      audio[2*i+1] = (m - s) * 0.5;
    }
  }
  else
  {
    /*!>
     * Send only mono signal out
     */
    for (unsigned int i = 0; i < dataSize; i++)
    {
      float m = m_BufferMono[i] * 0.5;
      audio[2*i]   = m;
      audio[2*i+1] = m;
    }
  }

  return 2*dataSize;
}

/** Compute RMS level over a small prefix of the specified sample vector. */
ComplexType::value_type cFmDecoder::RMSLevelApprox(const ComplexType *samples, unsigned int length)
{
  unsigned int n = length;
  n = (n + 63) / 64;

  ComplexType::value_type level = 0;
  for (unsigned int i = 0; i < n; i++)
  {
    const ComplexType& s = samples[i];
    ComplexType::value_type re = s.real(), im = s.imag();
    level += re * re + im * im;
  }

  return sqrt(level / n);
}

/** Compute mean and RMS over a sample vector. */
void cFmDecoder::SamplesMeanRMS(const RealType* samples, double& mean, double& rms, unsigned int n)
{
  RealType vsum = 0;
  RealType vsumsq = 0;

  for (unsigned int i = 0; i < n; i++)
  {
    RealType v = samples[i];
    vsum   += v;
    vsumsq += v * v;
  }

  mean = vsum / n;
  rms  = sqrt(vsumsq / n);
}
