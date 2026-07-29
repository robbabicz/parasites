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
// Parameters of the granular effect.

#ifndef CLOUDS_DSP_PARAMETERS_H_
#define CLOUDS_DSP_PARAMETERS_H_

#include "stmlib/stmlib.h"

namespace clouds {

struct Parameters {
  float position;
  float size;
  float pitch;
  float density;
  float texture;
  float dry_wet;
  float stereo_spread;
  float feedback;
  float reverb;
  
  bool freeze;
  bool trigger;
  bool gate;

  // BABICZ MEMORY repeat contour. AGE is exactly transparent at zero.
  // DARK and THIN are applied inside Clouds' regenerative path so the
  // spectral loss compounds with each generation in FREE GRAIN.
  float repeat_loss_age;
  float repeat_loss_mode;
  
  struct Granular {
    float overlap;
    float window_shape;
    float stereo_spread;
    // BABICZ extensions. Values 0..4 select the five musical position fields;
    // a negative value preserves Clouds' fixed play-head placement for engine
    // modes which do not expose RANDOM FIELD. Coherence morphs independent
    // draws toward a low-discrepancy constellation. At higher values, short
    // groups share a slowly gliding drift while each grain retains its own
    // distributed read position.
    float random_field;
    float coherence;
    float field_motion;
    bool use_deterministic_seed;
    bool reverse;
  } granular;
  
  struct Spectral {
    float quantization;
    float refresh_rate;
    float phase_randomization;
    float warp;
  } spectral;
};

}  // namespace clouds

#endif  // CLOUDS_DSP_PARAMETERS_H_
