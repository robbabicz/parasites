// Copyright 2026 Babiczstyle.
//
// 128-band linked-stereo spectral disperser for the BABICZ BODY / PRISM
// control.  A 256-point, four-times-overlapped STFT yields 127 moving audio
// bands plus DC/Nyquist.  BODY remains the untouched granular stream; PRISM
// fans the same stream across frequency-dependent stereo positions and short
// phase delays before it is copied into Clouds' feedback memory.

#ifndef CLOUDS_DSP_SPECTRAL_PRISM_H_
#define CLOUDS_DSP_SPECTRAL_PRISM_H_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

#include "clouds/dsp/frame.h"
#include "stmlib/fft/shy_fft.h"

namespace clouds {

class SpectralPrism {
 public:
  static constexpr size_t kFftSize = 256;
  static constexpr size_t kHopSize = 64;
  static constexpr size_t kNumBands = kFftSize / 2;
  static constexpr size_t kOutputRingSize = 1024;

  void Init() {
    fft_.Init();
    for (size_t index = 0; index < kFftSize; ++index) {
      const float phase = static_cast<float>(index)
          / static_cast<float>(kFftSize);
      // Square-root Hann on analysis and synthesis. Four overlaps sum the
      // resulting Hann products to a constant two.
      window_[index] = std::sqrt(std::max(
          0.0f, 0.5f - 0.5f * std::cos(6.283185307179586f * phase)));
    }
    for (size_t band = 0; band < kNumBands; ++band) {
      uint32_t value = static_cast<uint32_t>(band + 1u) * 0x9e3779b9u;
      value ^= value >> 16;
      value *= 0x7feb352du;
      value ^= value >> 15;
      value *= 0x846ca68bu;
      value ^= value >> 16;
      const float random = static_cast<float>(value & 0xffffu)
          * (2.0f / 65535.0f) - 1.0f;
      const float normalized = static_cast<float>(band)
          / static_cast<float>(kNumBands - 1);
      // Keep sub energy central, then open progressively through the mids and
      // highs. Each FFT bin receives a stable position, avoiding cheap random
      // flutter while the actual grains continue to provide motion.
      const float bassRelease = SmoothStep(
          std::max(0.0f, std::min(1.0f, (normalized - 0.012f) / 0.11f)));
      // Adjacent pairs form alternating spectral lanes. Randomness changes
      // their aperture, never their basic audibility; no musically important
      // bin can accidentally land almost in the centre.
      const float direction = ((band / 2u) & 1u) == 0u ? -1.0f : 1.0f;
      const float aperture = 0.62f + 0.38f * std::abs(random);
      bandPan_[band] = direction * aperture * bassRelease
          * (0.58f + 0.42f * std::sqrt(normalized));
    }
    Reset();
  }

  void Reset() {
    for (auto& channel : inputRing_) channel.fill(0.0f);
    for (auto& channel : outputRing_) channel.fill(0.0f);
    for (auto& channel : fftInput_) channel.fill(0.0f);
    for (auto& channel : fftSpectrum_) channel.fill(0.0f);
    for (auto& channel : inverseSpectrum_) channel.fill(0.0f);
    for (auto& channel : inverseOutput_) channel.fill(0.0f);
    inputWrite_ = 0;
    sampleCounter_ = 0;
    amount_ = 0.0f;
    sleeping_ = true;
  }

  void Process(FloatFrame* frames, size_t size, float amount) {
    amount = std::max(0.0f, std::min(1.0f, amount));

    // Keep only the inexpensive history ring warm while BODY is selected.
    // This makes the true bypass path sample-transparent and avoids running
    // either stereo FFT. The recent 256 samples are still ready when PRISM is
    // performed or modulated back in, so waking does not require a silent
    // analysis pre-roll.
    constexpr float kSleepThreshold = 0.00001f;
    if (amount <= kSleepThreshold && amount_ <= kSleepThreshold) {
      amount_ = 0.0f;
      if (!sleeping_) {
        for (auto& channel : outputRing_) channel.fill(0.0f);
        sleeping_ = true;
      }
      for (size_t sample = 0; sample < size; ++sample) {
        inputRing_[0][inputWrite_] = frames[sample].l;
        inputRing_[1][inputWrite_] = frames[sample].r;
        inputWrite_ = (inputWrite_ + 1u) % kFftSize;
      }
      return;
    }

    if (sleeping_) {
      // Re-align the overlap/add scheduler and render the most recent history
      // immediately. The existing amount smoother then crossfades into this
      // primed spectral stream without a discontinuity.
      for (auto& channel : outputRing_) channel.fill(0.0f);
      sampleCounter_ = kFftSize;
      RenderFrame();
      sleeping_ = false;
    }

    for (size_t sample = 0; sample < size; ++sample) {
      const size_t outputIndex = static_cast<size_t>(
          sampleCounter_ % kOutputRingSize);
      const float spectralLeft = outputRing_[0][outputIndex];
      const float spectralRight = outputRing_[1][outputIndex];
      outputRing_[0][outputIndex] = 0.0f;
      outputRing_[1][outputIndex] = 0.0f;

      const float inputLeft = frames[sample].l;
      const float inputRight = frames[sample].r;
      inputRing_[0][inputWrite_] = inputLeft;
      inputRing_[1][inputWrite_] = inputRight;
      inputWrite_ = (inputWrite_ + 1u) % kFftSize;

      // A short morph prevents zippering when BODY / PRISM is performed or
      // modulated. BODY at exactly zero is sample-transparent and latency-free.
      amount_ += 0.015f * (amount - amount_);
      frames[sample].l = inputLeft + (spectralLeft - inputLeft) * amount_;
      frames[sample].r = inputRight + (spectralRight - inputRight) * amount_;

      ++sampleCounter_;
      if (sampleCounter_ >= kFftSize
          && ((sampleCounter_ - kFftSize) % kHopSize) == 0u) {
        RenderFrame();
      }
    }
  }

 private:
  using FFT = stmlib::ShyFFT<
      float, kFftSize, stmlib::RotationPhasor>;

  static float SmoothStep(float value) {
    return value * value * (3.0f - 2.0f * value);
  }

  void RenderFrame() {
    for (size_t channel = 0; channel < 2; ++channel) {
      for (size_t index = 0; index < kFftSize; ++index) {
        const size_t source = (inputWrite_ + index) % kFftSize;
        fftInput_[channel][index] = inputRing_[channel][source]
            * window_[index];
      }
      fft_.Direct(fftInput_[channel].data(),
                  fftSpectrum_[channel].data());
      inverseSpectrum_[channel] = fftSpectrum_[channel];
    }

    // ShyFFT stores real bins in [0, N/2) and imaginary bins in [N/2, N).
    // DC stays centred. The remaining 127 audible bins become a true linked
    // stereo spectrum rather than three broad time-domain filter zones.
    inverseSpectrum_[0][0] = inverseSpectrum_[1][0]
        = 0.5f * (fftSpectrum_[0][0] + fftSpectrum_[1][0]);
    inverseSpectrum_[0][kNumBands] = 0.0f;
    inverseSpectrum_[1][kNumBands] = 0.0f;
    for (size_t band = 1; band < kNumBands; ++band) {
      const float leftReal = fftSpectrum_[0][band];
      const float leftImag = fftSpectrum_[0][band + kNumBands];
      const float rightReal = fftSpectrum_[1][band];
      const float rightImag = fftSpectrum_[1][band + kNumBands];
      const float midReal = 0.5f * (leftReal + rightReal);
      const float midImag = 0.5f * (leftImag + rightImag);

      const float pan = 0.92f * bandPan_[band];
      const float gainLeft = std::sqrt(std::max(0.0f, 1.0f - pan));
      const float gainRight = std::sqrt(std::max(0.0f, 1.0f + pan));
      const float normalized = static_cast<float>(band)
          / static_cast<float>(kNumBands - 1);
      const float phase = pan * (0.14f + 0.72f * std::sqrt(normalized));
      const float cosine = std::cos(phase);
      const float sine = std::sin(phase);

      inverseSpectrum_[0][band]
          = (midReal * cosine + midImag * sine) * gainLeft;
      inverseSpectrum_[0][band + kNumBands]
          = (midImag * cosine - midReal * sine) * gainLeft;
      inverseSpectrum_[1][band]
          = (midReal * cosine - midImag * sine) * gainRight;
      inverseSpectrum_[1][band + kNumBands]
          = (midImag * cosine + midReal * sine) * gainRight;
    }

    constexpr float inverseScale = 1.0f
        / static_cast<float>(kFftSize * kFftSize / kHopSize / 2u);
    const size_t synthesisStart = static_cast<size_t>(
        sampleCounter_ % kOutputRingSize);
    for (size_t channel = 0; channel < 2; ++channel) {
      fft_.Inverse(inverseSpectrum_[channel].data(),
                   inverseOutput_[channel].data());
      for (size_t index = 0; index < kFftSize; ++index) {
        const size_t destination =
            (synthesisStart + index) % kOutputRingSize;
        outputRing_[channel][destination] +=
            inverseOutput_[channel][index] * window_[index] * inverseScale;
      }
    }
  }

  FFT fft_;
  std::array<float, kFftSize> window_ {};
  std::array<float, kNumBands> bandPan_ {};
  std::array<std::array<float, kFftSize>, 2> inputRing_ {};
  std::array<std::array<float, kOutputRingSize>, 2> outputRing_ {};
  std::array<std::array<float, kFftSize>, 2> fftInput_ {};
  std::array<std::array<float, kFftSize>, 2> fftSpectrum_ {};
  std::array<std::array<float, kFftSize>, 2> inverseSpectrum_ {};
  std::array<std::array<float, kFftSize>, 2> inverseOutput_ {};
  size_t inputWrite_ = 0;
  uint64_t sampleCounter_ = 0;
  float amount_ = 0.0f;
  bool sleeping_ = true;
};

}  // namespace clouds

#endif  // CLOUDS_DSP_SPECTRAL_PRISM_H_
