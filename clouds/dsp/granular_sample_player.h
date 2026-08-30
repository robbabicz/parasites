// Copyright 2014 Emilie Gillet.
//
// Author: Emilie Gillet (emilie.o.gillet@gmail.com)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
// 
// See http://creativecommons.org/licenses/MIT/ for more information.
//
// -----------------------------------------------------------------------------
//
// Granular playback of audio stored in a buffer.

#ifndef CLOUDS_DSP_GRANULAR_SAMPLE_PLAYER_H_
#define CLOUDS_DSP_GRANULAR_SAMPLE_PLAYER_H_

#include "stmlib/stmlib.h"

#include <algorithm>
#include <cmath>

#include "stmlib/dsp/atan.h"
#include "stmlib/dsp/units.h"
#include "stmlib/utils/random.h"

#include "clouds/dsp/audio_buffer.h"
#include "clouds/dsp/frame.h"
#include "clouds/dsp/grain.h"
#include "clouds/dsp/parameters.h"

#include "clouds/resources.h"

namespace clouds {

// BABICZ SPACE uses one consistent polyphony in every channel/fidelity mode.
// The original hardware varied this between 26 and 40 voices to fit its MCU.
const int32_t kMaxNumGrains = 64;

inline float GrainSizeSamples(float normalized_size) {
  const float size = std::max(0.0f, std::min(1.0f, normalized_size));
  // 512..32768 samples at Clouds' fixed 32 kHz engine rate: approximately
  // 16 ms..1.024 s. The original five-octave range ended at 512 ms.
  return 512.0f * std::exp2(size * 6.0f);
}

using namespace stmlib;

class GranularSamplePlayer {
 public:
  GranularSamplePlayer() { }
  ~GranularSamplePlayer() { }

  inline float active_grains() const {
    return num_grains_;
  }
  
  void Init(int32_t num_channels, int32_t max_num_grains) {
    max_num_grains_ = max_num_grains;
    num_midfi_grains_ = 3 * max_num_grains / 4;
    gain_normalization_ = 1.0f;
    for (int32_t i = 0; i < kMaxNumGrains; ++i) {
      grains_[i].Init();
    }
    num_grains_ = 0.0f;
    num_channels_ = num_channels;
    grain_size_hint_ = 1024.0f;
    // The module firmware has one global RNG because one hardware unit owns
    // one processor. A DAW can run and scan several plugin instances on
    // different threads, so sharing stmlib::Random makes grain scheduling
    // depend on unrelated instances. Keep the original LCG and uniform
    // distribution, but give each GranularSamplePlayer its own state.
    random_state_ = 0x21u;
    field_sequence_ = 0u;
    field_hold_remaining_ = 0;
    field_phase_ = 0.0f;
    field_phase_target_ = 0.0f;
    field_position_hold_remaining_ = 0;
    field_position_current_ = 0.5f;
    field_position_target_ = 0.5f;
    field_position_initialized_ = false;
  }
  
  template<Resolution resolution>
  void Play(
      const AudioBuffer<resolution>* buffer,
      const Parameters& parameters,
      float* out, size_t size) {
    float overlap = parameters.granular.overlap;
    overlap = (overlap * overlap) * (overlap * overlap);
    float target_num_grains = max_num_grains_ * overlap;
    const float natural_space = grain_size_hint_ / target_num_grains;
    const float minimum_space = std::max(
        0.0f, parameters.granular.minimum_spacing_samples);
    const float space_between_grains = std::max(
        natural_space, minimum_space);
    float p = 1.0f / space_between_grains;
    if (parameters.granular.use_deterministic_seed) {
      p = -1.0f;
    } else {
      grain_rate_phasor_ = -1000.0f;
    }
    
    // Build a list of available grains.
    int32_t num_available_grains = FillAvailableGrainsList();
    
    // Try to schedule new grains.
    bool seed_trigger = parameters.trigger;
    for (size_t t = 0; t < size; ++t) {
      grain_rate_phasor_ += 1.0f;
      bool seed_probabilistic = NextRandomFloat() < p
          && target_num_grains > num_grains_;
      bool seed_deterministic = grain_rate_phasor_ >= space_between_grains;
      bool seed = seed_probabilistic || seed_deterministic || seed_trigger;
      if (num_available_grains && seed) {
        --num_available_grains;
        int32_t index = available_grains_[num_available_grains];
        GrainQuality quality;
        if (num_available_grains < num_midfi_grains_) {
          quality = GRAIN_QUALITY_MEDIUM;
        } else {
          quality = GRAIN_QUALITY_HIGH;
        }
        
        Grain* g = &grains_[index];
        ScheduleGrain(
            g,
            parameters,
            t,
            buffer->size(),
            buffer->head() - size + t,
            quality);
        grain_rate_phasor_ = 0.0f;
        seed_trigger = false;
      }
    }
    
    // Overlap grains.
    std::fill(&out[0], &out[size * 2], 0.0f);
    float* e = envelope_buffer_;
    for (int32_t i = 0; i < max_num_grains_; ++i) {
      Grain* g = &grains_[i];
      if (g->recommended_quality() == GRAIN_QUALITY_HIGH) {
        if (num_channels_ == 1) {
          g->OverlapAdd<1, GRAIN_QUALITY_HIGH>(buffer, out, e, size);
        } else {
          g->OverlapAdd<2, GRAIN_QUALITY_HIGH>(buffer, out, e, size);
        }
      } else if (g->recommended_quality() == GRAIN_QUALITY_MEDIUM) {
        if (num_channels_ == 1) {
          g->OverlapAdd<1, GRAIN_QUALITY_MEDIUM>(buffer, out, e, size);
        } else {
          g->OverlapAdd<2, GRAIN_QUALITY_MEDIUM>(buffer, out, e, size);
        }
      } else {
        if (num_channels_ == 1) {
          g->OverlapAdd<1, GRAIN_QUALITY_LOW>(buffer, out, e, size);
        } else {
          g->OverlapAdd<2, GRAIN_QUALITY_LOW>(buffer, out, e, size);
        }
      }
    }
    
    // Compute normalization factor.
    int32_t active_grains = max_num_grains_ - num_available_grains;
    SLOPE(num_grains_, static_cast<float>(active_grains), 0.9f, 0.2f);

    float gain_normalization = num_grains_ > 2.0f
        ? fast_rsqrt_carmack(num_grains_ - 1.0f)
        : 1.0f;  
    float window_gain = 1.0f + 2.0f * parameters.granular.window_shape;
    CONSTRAIN(window_gain, 1.0f, 2.0f);
    gain_normalization *= Crossfade(
        1.0f, window_gain, parameters.granular.overlap);

    // Apply gain normalization.
    for (size_t t = 0; t < size; ++t) {
      ONE_POLE(gain_normalization_, gain_normalization, 0.01f)
      *out++ *= gain_normalization_;
      *out++ *= gain_normalization_;
    }
  }
  
 private:
  int32_t FillAvailableGrainsList() {
    int32_t num_available_grains = 0;
    for (int32_t i = 0; i < max_num_grains_; ++i) {
      if (!grains_[i].active()) {
        available_grains_[num_available_grains] = i;
        ++num_available_grains;
      }
    }
    return num_available_grains;
  }
  
  void ScheduleGrain(
      Grain* grain,
      const Parameters& parameters,
      int32_t pre_delay,
      int32_t buffer_size,
    int32_t buffer_head,
      GrainQuality quality) {
    float position = parameters.position;
    const bool use_random_field =
        parameters.granular.random_field >= 0.0f;
    const int32_t random_field = static_cast<int32_t>(
        parameters.granular.random_field + 0.5f);
    if (use_random_field) {
      position = RandomFieldPosition(parameters, random_field);
    }
    float pitch = parameters.pitch;
    // Two uniform draws create a centre-weighted triangular distribution:
    // most grains stay close to the played pitch while a few define the outer
    // edge selected by MICRO SPREAD. The result is frozen into this grain's
    // phase increment below, preventing artificial real-time glissandi.
    const float micro_pitch_cents =
        (NextRandomFloat() + NextRandomFloat() - 1.0f)
        * std::max(0.0f, std::min(50.0f,
            parameters.granular.micro_pitch_spread));
    pitch += micro_pitch_cents * 0.01f;
    float window_shape = parameters.granular.window_shape;
    float grain_size = GrainSizeSamples(parameters.size);
    float pitch_ratio = SemitonesToRatio(pitch);
    float inv_pitch_ratio = SemitonesToRatio(-pitch);
    float pan = 0.5f
        + parameters.stereo_spread * (NextRandomFloat() - 0.5f);
    float gain_l, gain_r;
    if (num_channels_ == 1) {
      gain_l = Interpolate(lut_sin, pan, 256.0f);
      gain_r = Interpolate(lut_sin + 256, pan, 256.0f);
    } else {
      if (pan < 0.5f) {
        gain_l = 1.0f;
        gain_r = 2.0f * pan;
      } else {
        gain_r = 1.0f;
        gain_l = 2.0f * (1.0f - pan);
      }
    }
    
    if (pitch_ratio > 1.0f) {
      // The grain's play-head moves faster than the buffer record-head.
      // we must make sure that the grain will not consume too much data.
      // In some situations, it might be necessary to reduce the size of the
      // grain.
      grain_size = std::min(grain_size, buffer_size * 0.25f * inv_pitch_ratio);
    }

    float eaten_by_play_head = grain_size * pitch_ratio;
    float eaten_by_recording_head = grain_size;

    float available = 0.0;
    available += static_cast<float>(buffer_size);
    available -= eaten_by_play_head;
    available -= eaten_by_recording_head;

    bool reverse = parameters.granular.reverse;
    int32_t size = static_cast<int32_t>(grain_size) & ~1;
    int32_t start = buffer_head - static_cast<int32_t>(
        position * available + eaten_by_play_head);
    grain->Start(
        pre_delay,
        buffer_size,
        start,
        size,
        reverse,
        static_cast<uint32_t>(pitch_ratio * 65536.0f),
        window_shape,
        gain_l,
        gain_r,
        quality);
    grain_size_hint_ = grain_size;
  }

  inline float Fract(float value) const {
    return value - std::floor(value);
  }

  inline float ClampUnit(float value) const {
    return std::max(0.0f, std::min(1.0f, value));
  }

  inline float FoldUnit(float value) const {
    value = std::fmod(value, 2.0f);
    if (value < 0.0f) {
      value += 2.0f;
    }
    return value <= 1.0f ? value : 2.0f - value;
  }

  float GaussianField(float u1, float u2) const {
    // Box-Muller converted into a bounded 0..1 field. At approximately
    // +/-3.45 sigma the clamp is reached, leaving a useful focused centre
    // without creating invalid buffer positions.
    u1 = std::max(1.0e-6f, u1);
    const float z = std::sqrt(-2.0f * std::log(u1))
        * std::cos(6.28318530718f * u2);
    return ClampUnit(0.5f + z * 0.145f);
  }

  float ShapeRandomField(
      int32_t mode, float u1, float u2, float phase) const {
    if (mode == 1) {
      // FOCUS: one centre-weighted Gaussian field.
      return GaussianField(u1, u2);
    }
    if (mode == 2) {
      // CLUSTERS: two weighted islands with restrained common drift.
      const bool upper = u1 >= 0.5f;
      const float cluster_u = upper ? (u1 - 0.5f) * 2.0f : u1 * 2.0f;
      const float local = (GaussianField(cluster_u, u2) - 0.5f) * 0.32f;
      const float centre = upper ? 0.72f : 0.28f;
      return ClampUnit(centre + local + phase * 0.16f);
    }
    // EVEN is a uniform probability landscape. COHERENCE determines whether
    // its visits are independent or low-discrepancy. WILD uses the same
    // landscape over a wider physical span below.
    return ClampUnit(u1);
  }

  float RandomFieldPosition(
      const Parameters& parameters, int32_t random_field) {
    const float coherence = ClampUnit(parameters.granular.coherence);
    const float motion = ClampUnit(parameters.granular.field_motion);

    // Sample-and-hold controls the slowly moving orientation of the field,
    // not individual grain amplitude or pitch. Evolve increases the renewal
    // rate while higher coherence lengthens the memory.
    if (field_hold_remaining_ <= 0) {
      field_phase_target_ = (NextRandomFloat() - 0.5f) * 0.72f;
      const float hold = 1.0f
          + (coherence * coherence * 56.0f)
              / (0.25f + motion * 1.75f);
      field_hold_remaining_ = static_cast<int32_t>(hold + 0.5f);
    }
    --field_hold_remaining_;
    const float phase_slew = 0.015f + motion * 0.08f
        + (1.0f - coherence) * 0.25f;
    field_phase_ += (field_phase_target_ - field_phase_) * phase_slew;

    // Independent probability draw.
    const float random_u1 = NextRandomFloat();
    const float random_u2 = NextRandomFloat();
    const float random_value = ShapeRandomField(
        random_field, random_u1, random_u2, field_phase_);

    // A golden-ratio low-discrepancy pair creates a continually moving,
    // non-repeating ordered constellation. It fills the selected probability
    // landscape evenly without becoming a static delay pattern.
    ++field_sequence_;
    const float ordered_u1 = Fract(
        static_cast<float>(field_sequence_) * 0.61803398875f
        + field_phase_);
    const float ordered_u2 = Fract(
        static_cast<float>(field_sequence_) * 0.75487766625f
        + field_phase_ * 0.61f);
    const float ordered_value = ShapeRandomField(
        random_field, ordered_u1, ordered_u2, field_phase_);

    const float coherence_curve =
        coherence * coherence * (3.0f - 2.0f * coherence);
    // ORIGINAL is the deliberately unstructured free cloud. EVEN always
    // leans toward the low-discrepancy constellation so it remains audibly
    // distinct from ORIGINAL even when COHERENCE is at zero.
    const float order_amount = random_field == 3
        ? 0.72f + coherence_curve * 0.28f
        : coherence_curve;
    const float sequence_value = random_value
        + (ordered_value - random_value) * order_amount;

    // COHERENCE must be a musical time gesture, not merely a statistically
    // different ordering of otherwise identical random visits. Keep every
    // grain in the moving constellation, however: holding one absolute read
    // position for dozens of grains turns ORDER into a resonant delay and
    // lets low frequencies add coherently in the feedback path.
    //
    // A short shared drift groups three to five ordered visits into one
    // recognisable gesture. Evolve renews the drift faster, while the golden
    // sequence above continues to distribute every grain across the field.
    float field_value = sequence_value;
    if (coherence_curve > 0.0001f) {
      if (!field_position_initialized_) {
        field_position_current_ = 0.0f;
        field_position_target_ = 0.0f;
        field_position_hold_remaining_ = 0;
        field_position_initialized_ = true;
      }
      if (field_position_hold_remaining_ <= 0) {
        field_position_target_ = (NextRandomFloat() - 0.5f) * 0.24f;
        const float residence = 1.0f
            + coherence_curve * coherence_curve
                * (4.0f - motion * 2.0f);
        field_position_hold_remaining_ =
            static_cast<int32_t>(residence + 0.5f);
      }
      --field_position_hold_remaining_;
      const float position_slew =
          0.48f + motion * 0.22f
          + (1.0f - coherence_curve) * 0.18f;
      field_position_current_ +=
          (field_position_target_ - field_position_current_)
          * position_slew;
      field_value = FoldUnit(
          field_value + field_position_current_ * coherence_curve);
    } else {
      // Preserve the independent field exactly at zero and make the next
      // coherent gesture begin without a stale displacement.
      field_position_current_ = 0.0f;
      field_position_target_ = 0.0f;
      field_position_hold_remaining_ = 0;
      field_position_initialized_ = true;
    }
    float span = 0.06f + ClampUnit(parameters.size) * 0.64f;
    if (random_field == 4) {
      span = std::min(1.0f, span * 1.38f);
    }
    const float offset = (field_value - 0.5f) * span;
    return FoldUnit(parameters.position + offset);
  }
  
  int32_t max_num_grains_;
  inline float NextRandomFloat() {
    random_state_ = random_state_ * 1664525u + 1013904223u;
    return static_cast<float>(random_state_) / 4294967296.0f;
  }
  uint32_t random_state_ = 0x21u;
  uint32_t field_sequence_ = 0u;
  int32_t field_hold_remaining_ = 0;
  float field_phase_ = 0.0f;
  float field_phase_target_ = 0.0f;
  int32_t field_position_hold_remaining_ = 0;
  float field_position_current_ = 0.5f;
  float field_position_target_ = 0.5f;
  bool field_position_initialized_ = false;
  int32_t num_midfi_grains_;
  int32_t num_channels_;

  float num_grains_;
  float gain_normalization_;
  float grain_size_hint_;
  float grain_rate_phasor_;
  
  Grain grains_[kMaxNumGrains];
  int32_t available_grains_[kMaxNumGrains];
  float envelope_buffer_[kMaxBlockSize];
  
  DISALLOW_COPY_AND_ASSIGN(GranularSamplePlayer);
};

}  // namespace clouds

#endif  // CLOUDS_DSP_GRANULAR_SAMPLE_PLAYER_H_
