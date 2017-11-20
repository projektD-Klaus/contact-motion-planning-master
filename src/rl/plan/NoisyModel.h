//
// Copyright (c) 2016, Arne Sieverling
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#ifndef _RL_PLAN_NOISYMODEL_H_
#define _RL_PLAN_NOISYMODEL_H_

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/random.hpp>

#include <rl/math/Matrix.h>

#include "DistanceModel.h"

namespace rl
{
  namespace plan
  {
    class NoisyModel : public DistanceModel
    {
    public:
      NoisyModel();
      

      virtual ~NoisyModel();

      void sampleMotionError(::rl::math::Vector &error);
      void sampleInitialError(::rl::math::Vector &error);

      void interpolateNoisy(const ::rl::math::Vector& q1, const ::rl::math::Vector& q2, const ::rl::math::Real& alpha, const ::rl::math::Vector& noise, ::rl::math::Vector& q) const;
      ::rl::math::Matrix updateCovariance(const ::rl::math::Matrix& sigma, const ::rl::math::Vector& q1, const ::rl::math::Vector& q2) const;

      ::rl::math::Vector* motionError;

      ::rl::math::Vector* initialError;

      int seed;

     private:
      ::boost::shared_ptr<::boost::random::mt19937> motionErrorGen;


    };
  }
}

#endif // _RL_PLAN_NOISYMODEL_H_
