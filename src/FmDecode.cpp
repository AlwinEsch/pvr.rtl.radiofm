/*
 *  Copyright (C) 2013, Joris van Rantwijk.
 *  Copyright (C) 2015-2020, Alwin Esch (Team KODI)
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSE.md for more information.
 */

#include "FmDecode.h"

#include "RTL_SDR_Source.h"
#include "RadioReceiver.h"

#include <assert.h>
#include <math.h>

#define FMDEMOD_GAIN 8000.0

#define HILB_LENGTH 61
const RealType HILBLP_H[HILB_LENGTH] =
    { //LowPass filter prototype that is shifted and "hilbertized" to get 90 deg phase shift
        //and convert to baseband complex domain.
        //kaiser-Bessel alpha 1.4  cutoff 30Khz at sample rate of 250KHz
        -0.000389631665953405, 0.000115430826670992,  0.000945331102222503,  0.001582460677684605,
        0.001370803713784687,  -0.000000000000000002, -0.002077413537668161, -0.003656132107176520,
        -0.003372610825000167, -0.000649815020884706, 0.003583263233560064,  0.006997162933343487,
        0.006990985399916562,  0.002383133886438500,  -0.005324501734543406, -0.012092135317628615,
        -0.013212201698221963, -0.006168904735839018, 0.007082277142635906,  0.020017841466263672,
        0.024271835962039127,  0.014255112728911837,  -0.008597071392140753, -0.034478282954624850,
        -0.048147195828726633, -0.035409729589347565, 0.009623663461671806,  0.080084441681677138,
        0.157278883310078170,  0.217148915611638180,  0.239688166538436750,  0.217148915611638180,
        0.157278883310078170,  0.080084441681677138,  0.009623663461671806,  -0.035409729589347565,
        -0.048147195828726633, -0.034478282954624850, -0.008597071392140753, 0.014255112728911837,
        0.024271835962039127,  0.020017841466263672,  0.007082277142635906,  -0.006168904735839018,
        -0.013212201698221963, -0.012092135317628615, -0.005324501734543406, 0.002383133886438500,
        0.006990985399916562,  0.006997162933343487,  0.003583263233560064,  -0.000649815020884706,
        -0.003372610825000167, -0.003656132107176520, -0.002077413537668161, -0.000000000000000002,
        0.001370803713784687,  0.001582460677684605,  0.000945331102222503,  0.000115430826670992,
        -0.000389631665953405};


/* ****************  class cFineTuner  **************** */

// Construct finetuner.
cFineTuner::cFineTuner(unsigned int table_size, int freq_shift)
  : m_index(0), m_tableSize(table_size)
{
  m_table = new ComplexType[table_size];

  float phase_step = K_2PI / float(table_size);
  for (unsigned int i = 0; i < table_size; ++i)
  {
    float phi = (((int64_t)freq_shift * i) % table_size) * phase_step;
    float pcos = MCOS(phi);
    float psin = MSIN(phi);
    m_table[i] = ComplexType(pcos, psin) * 2.0f;
  }
}

cFineTuner::~cFineTuner()
{
  delete[] m_table;
}

// Process samples.
void cFineTuner::Process(const ComplexType* samples_in,
                         ComplexType* samples_out,
                         unsigned int samples)
{
  unsigned int tblidx = m_index;
  unsigned int tblsiz = m_tableSize;

  for (unsigned int i = 0; i < samples; ++i)
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
cPilotPhaseLock::cPilotPhaseLock(RealType freq, RealType bandwidth, RealType minsignal)
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
  m_minsignal = minsignal;
  m_lock_delay = int(20.0f / bandwidth);
  m_lock_cnt = 0;

  //! Create 2nd order filter for I/Q representation of phase error.
  //! Filter has two poles, unit DC gain.
  RealType p1 = exp(-1.146f * bandwidth * K_2PI);
  RealType p2 = exp(-5.331f * bandwidth * K_2PI);
  m_phasor_a1 = -p1 - p2;
  m_phasor_a2 = p1 * p2;
  m_phasor_b0 = 1 + m_phasor_a1 + m_phasor_a2;

  //! Create loop filter to stabilize the loop.
  m_loopfilter_b0 = 0.62f * bandwidth * K_2PI;
  m_loopfilter_b1 = -m_loopfilter_b0 * exp(-0.1153 * bandwidth * K_2PI);

  /*!
   * After the loop filter, the phase error is integrated to produce
   * the frequency. Then the frequency is integrated to produce the phase.
   * These integrators form the two remaining poles, both at z = 1.
   */

  //! Initialize frequency and phase.
  m_freq = freq * K_2PI;
  m_phase = 0;

  m_phasor_i1 = 0;
  m_phasor_i2 = 0;
  m_phasor_q1 = 0;
  m_phasor_q2 = 0;
  m_loopfilter_x1 = 0;
  m_pilotLevel = 0;
}


bool cPilotPhaseLock::Process(const RealType* samples_in,
                              RealType* samples_out,
                              unsigned int length)
{
  m_pilotLevel = 1000.0f;

  RealType psin;
  RealType pcos;
  for (unsigned int i = 0; i < length; ++i)
  {
    //! Generate locked pilot tone.
#if TARGET_WINDOWS
    RealType* pdCosAns = &pcos;
    RealType* pdSinAns = &psin;
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
    asm volatile("fsincos" : "=%&t"(pcos), "=%&u"(psin) : "0"(m_phase));
#elif defined(__arm__)
    sincos_LP(m_phase, psin, pcos);
#else
    psin = MSIN(m_phase); //178ns for sin/cos calc
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
    phasor_i = m_phasor_b0 * phasor_i - m_phasor_a1 * m_phasor_i1 - m_phasor_a2 * m_phasor_i2;
    phasor_q = m_phasor_b0 * phasor_q - m_phasor_a1 * m_phasor_q1 - m_phasor_a2 * m_phasor_q2;
    m_phasor_i2 = m_phasor_i1;
    m_phasor_i1 = phasor_i;
    m_phasor_q2 = m_phasor_q1;
    m_phasor_q1 = phasor_q;

    //! Convert I/Q ratio to estimate of phase error.
    RealType phase_err;
    if (phasor_i > abs(phasor_q))
      phase_err =
          phasor_q /
          phasor_i; //!< We are within +/- 45 degrees from lock. Use simple linear approximation of arctan.
    else if (phasor_q > 0)
      phase_err = 1; //!< We are lagging more than 45 degrees behind the input.
    else
      phase_err = -1; //!< We are more than 45 degrees ahead of the input.

    //! Detect pilot level (conservative).
    m_pilotLevel = std::min(m_pilotLevel, phasor_i);

    //! Run phase error through loop filter and update frequency estimate.
    m_freq += m_loopfilter_b0 * phase_err + m_loopfilter_b1 * m_loopfilter_x1;
    m_loopfilter_x1 = phase_err;

    //! Limit frequency to allowable range.
    m_freq = std::max(m_minfreq, std::min(m_maxfreq, m_freq));

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

#define PILOTPLL_FREQ 19000.0 //Centerfreq
#define RDS_DECIMATOR 8

cFmDecoder::cFmDecoder(cRadioReceiver* proc,
                       double sample_rate_if,
                       double tuning_offset,
                       double sample_rate_pcm,
                       double bandwidth_pcm,
                       unsigned int downsample,
                       bool USver)

  // Initialize member fields
  : m_proc(proc),
    m_SampleRate_Interface(sample_rate_if),
    m_SampleRate_Baseband(sample_rate_if / downsample),
    m_TuningTableSize(64),
    m_TuningShift(lrint(-64.0 * tuning_offset / sample_rate_if)),
    m_FrequencyDev(DEFAULT_FREQ_DEV),
    m_Downsample(downsample),
    m_BandwidthInterface(DEFAULT_BANDWIDTH_IF),
    m_FMDeModGain(1.0 / (DEFAULT_FREQ_DEV / m_SampleRate_Baseband * K_2PI)),
    m_FineTuner(m_TuningTableSize, m_TuningShift) //!< Construct FineTuner
    ,
    m_PilotPLL( //!< Construct cPilotPhaseLock
        PILOT_FREQ / m_SampleRate_Baseband, // freq
        50 / m_SampleRate_Baseband, // bandwidth
        0.04f) // minsignal
    ,
    m_ReSampleInput(8 * downsample, 0.6 / downsample, downsample, true),
    m_ReSampleMono( //!< Construct DownsampleFilter for mono channel
        int(m_SampleRate_Baseband / 1000.0), // filter_order
        bandwidth_pcm / m_SampleRate_Baseband, // cutoff
        m_SampleRate_Baseband / sample_rate_pcm, // downsample
        false) // integer_factor
    ,
    m_ReSampleStereo( //!< Construct DownsampleFilter for stereo channel
        int(m_SampleRate_Baseband / 1000.0), // filter_order
        bandwidth_pcm / m_SampleRate_Baseband, // cutoff
        m_SampleRate_Baseband / sample_rate_pcm, // downsample
        false) // integer_factor
    ,
    m_RDSProcess(proc, m_SampleRate_Baseband)
{
  m_BufferIfTuned = new ComplexType[cRtlSdrSource::default_block_length];
  m_BufferDemod = new ComplexType[cRtlSdrSource::default_block_length];
  m_BufferBaseband = new RealType[cRtlSdrSource::default_block_length];
  m_BufferMono = new RealType[cRtlSdrSource::default_block_length];
  m_BufferStereo = new RealType[cRtlSdrSource::default_block_length];
  m_BufferRawStereo = new RealType[cRtlSdrSource::default_block_length];

  m_DCBlock.Init(ftHP, 30.0, 2.0, sample_rate_pcm);
  m_NotchFilter.Init(ftBR, PILOTPLL_FREQ, 5, sample_rate_pcm);
  m_LPFilter.InitLPFilter(0, 1.0, 60.0, 15000.0, 1.4 * 15000.0, sample_rate_pcm);

  //create filters to create baseband complex data from real fmdemoulator output
  m_HilbertFilter.InitConstFir(HILB_LENGTH, HILBLP_H, sample_rate_pcm);
  m_HilbertFilter.GenerateHBFilter(
      42000); //shift +/-30KHz LP filter by 42KHz to make 12 to 72KHz bandpass


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
  RealType fac = K_2PI / m_SampleRate_Baseband;
  RealType bandwidth = 0.85f * m_SampleRate_Baseband;
  RealType maxFreqDev = 0.95f * (0.5f * m_SampleRate_Baseband);

  m_NcoLLimit = (-maxFreqDev) * fac; //!< boundary for changes
  m_NcoHLimit = (+maxFreqDev) * fac;
  m_PLLAlpha = 0.125f * bandwidth * fac; //!< pll bandwidth
  m_PLLBeta = (m_PLLAlpha * m_PLLAlpha) / 2.0f; //!< second order term
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
  m_StereoDetected = false;
  m_InterfaceLevel = 0;
  m_BasebandMean = 0;
  m_BasebandLevel = 0;
  m_AudioLevel = 0;
  m_DemodDCOffset = 0;
  m_NcoPhaseIncr = 0.0f; //!< this will change during runs
  m_NcoPhase = 0.0f;

  m_RDSProcess.Reset();
}

void cFmDecoder::InitDeemphasis(RealType Time,
                                RealType SampleRate) //!< create De-emphasis LP filter
{
  m_DeemphasisAlpha = (1.0f - MEXP(-1.0f / (SampleRate * Time)));
  m_DeemphasisAveRe = 0.0f;
  m_DeemphasisAveIm = 0.0f;
}

void cFmDecoder::ProcessDeemphasisFilter(RealType* bufferA, RealType* bufferB, unsigned int length)
{
  for (unsigned int i = 0; i < length; ++i)
  {
    m_DeemphasisAveRe =
        (1.0f - m_DeemphasisAlpha) * m_DeemphasisAveRe + m_DeemphasisAlpha * bufferA[i];
    bufferA[i] = m_DeemphasisAveRe * 2.0f;
    m_DeemphasisAveIm =
        (1.0f - m_DeemphasisAlpha) * m_DeemphasisAveIm + m_DeemphasisAlpha * bufferB[i];
    bufferB[i] = m_DeemphasisAveIm * 2.0f;
  }
}

#define DCAlpha 0.0001
void cFmDecoder::PhaseLockedLoop(ComplexType* signal, RealType* out, unsigned int dataSize)
{
  ComplexType pll_Delay;
  RealType phzError;
  RealType phaseIncr;
  RealType Sin;
  RealType Cos;

  RealType dcOffset = m_DemodDCOffset;
  for (unsigned int i = 0; i < dataSize; ++i)
  {
#if TARGET_WINDOWS
    RealType* pdCosAns = &Sin;
    RealType* pdSinAns = &Cos;
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
    asm volatile("fsincos" : "=%&t"(Cos), "=%&u"(Sin) : "0"(m_NcoPhase)); //126nS
#elif defined(__arm__)
    sincos_LP(m_NcoPhase, Sin, Cos);
#else
    Sin = MSIN(m_NcoPhase); //178ns for sin/cos calc
    Cos = MCOS(m_NcoPhase);
#endif

    pll_Delay = ComplexType(Cos, Sin) * signal[i];
    phzError = -atan2(imag(pll_Delay), real(pll_Delay));

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
    dcOffset = (1 - DCAlpha) * dcOffset +
               DCAlpha * phaseIncr; //!<  lowpass the NCO frequency term to get a DC offset
    out[i] = (phaseIncr - dcOffset) * m_FMDeModGain;
  }
  m_DemodDCOffset = dcOffset;
}

unsigned int cFmDecoder::ProcessStream(const ComplexType* samples_in,
                                       unsigned int samples,
                                       float* audio)
{
  unsigned int dataSize = samples;

  //! Fine tuning.
  m_FineTuner.Process(samples_in, m_BufferIfTuned, dataSize);

  //! Measure IF level.
  m_InterfaceLevel = 0.95f * m_InterfaceLevel + 0.05f * RMSLevelApprox(m_BufferIfTuned, dataSize);

  //! Perform first downsample to match 100000 Hz Format
  dataSize = m_ReSampleInput.Process(m_BufferIfTuned, m_BufferDemod, dataSize);

  //! Extract carrier frequency.
  PhaseLockedLoop(m_BufferDemod, m_BufferBaseband, dataSize);

  //! Handle RDS signal
  m_RDSProcess.Process(m_BufferBaseband, dataSize);

  //! Measure baseband level.
  RealType baseband_mean, baseband_rms;
  SamplesMeanRMS(m_BufferBaseband, baseband_mean, baseband_rms, dataSize);
  m_BasebandMean = 0.95f * m_BasebandMean + 0.05f * baseband_mean;
  m_BasebandLevel = 0.95f * m_BasebandLevel + 0.05f * baseband_rms;

  //! Extract mono audio signal.
  unsigned int monoSize = m_ReSampleMono.Process(m_BufferBaseband, m_BufferMono, dataSize);

  //! Lock on stereo pilot.
  m_StereoDetected = m_PilotPLL.Process(m_BufferBaseband, m_BufferRawStereo, dataSize);

  /*!
   * Just multiply the baseband signal with the double-frequency pilot.
   * And multiply by two to get the full amplitude.
   * That's all.
   */
  for (unsigned int i = 0; i < dataSize; ++i)
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
    for (unsigned int i = 0; i < dataSize; ++i)
    {
      float m = m_BufferMono[i];
      float s = m_BufferStereo[i];
      audio[2 * i] = (m + s) * 0.5f;
      audio[2 * i + 1] = (m - s) * 0.5f;
    }
  }
  else
  {
    /*!>
     * Send only mono signal out
     */
    for (unsigned int i = 0; i < dataSize; ++i)
    {
      float m = m_BufferMono[i] * 0.5f;
      audio[2 * i] = m;
      audio[2 * i + 1] = m;
    }
  }

  return 2 * dataSize;
}

/** Compute RMS level over a small prefix of the specified sample vector. */
ComplexType::value_type cFmDecoder::RMSLevelApprox(const ComplexType* samples, unsigned int length)
{
  unsigned int n = length;
  n = (n + 63) / 64;

  ComplexType::value_type level = 0;
  for (unsigned int i = 0; i < n; ++i)
  {
    const ComplexType& s = samples[i];
    ComplexType::value_type re = s.real(), im = s.imag();
    level += re * re + im * im;
  }

  return sqrt(level / n);
}

/** Compute mean and RMS over a sample vector. */
void cFmDecoder::SamplesMeanRMS(const RealType* samples,
                                RealType& mean,
                                RealType& rms,
                                unsigned int n)
{
  RealType vsum = 0;
  RealType vsumsq = 0;

  for (unsigned int i = 0; i < n; ++i)
  {
    RealType v = samples[i];
    vsum += v;
    vsumsq += v * v;
  }

  mean = vsum / n;
  rms = sqrt(vsumsq / n);
}
