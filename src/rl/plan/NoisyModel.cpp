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

#include "NoisyModel.h"

namespace rl
{
  namespace plan
  {
    NoisyModel::NoisyModel() :
      DistanceModel(),
      seed(42)
    {
    }
    
    NoisyModel::~NoisyModel()
    {
    }

    ::rl::math::Matrix
    NoisyModel::updateCovariance(const ::rl::math::Matrix& sigma, const ::rl::math::Vector& q1, const ::rl::math::Vector& q2) const
    {
       ::rl::math::Matrix out = sigma;
      for(int i=0;i<out.rows(); i++)
      {
        out(i,i)+=(q2(i)-q1(i))*(q2(i)-q1(i)) * (*this->motionError)(i) * (*this->motionError)(i);
      }
      return out;


//      //Non-parametric version below:
//      std::vector<Particle> movedParticles;
//      for (int pIdx = 0; pIdx < particles.size(); ++pIdx)
//      {
//        ::rl::math::Vector motionNoise(this->model->getDof());
//        this->model->sampleMotionError(motionNoise);

//        auto particle = particles[pIdx];
//        auto particleOffset = particle.config - mean;
//        auto shiftedChosen = chosen + particleOffset;

//        ::rl::math::Vector dest(this->model->getDof());
//        this->model->interpolateNoisy(particle.config, shiftedChosen, distance, motionNoise, dest);

//        Particle p(dest);
//        movedParticles.push_back(p);
//      }

//      BeliefState g(movedParticles);
//      return g.covariance;
    }

    void
    NoisyModel::interpolateNoisy(const ::rl::math::Vector& q1, const ::rl::math::Vector& q2, const ::rl::math::Real& alpha, const ::rl::math::Vector& noise, ::rl::math::Vector& q) const
    {

      interpolate(q1,q2,alpha,q);
      q=q+alpha*(q2-q1).cwiseProduct(noise);
    }

    void
    NoisyModel::sampleInitialError(::rl::math::Vector &error)
    {
      if (NULL == this->motionErrorGen)
      {
        this->motionErrorGen = ::boost::make_shared<::boost::random::mt19937>(seed);
      }

      if(this->motionError->rows()!=this->getDof())
      {
        std::cout << "warning: did not set initial error - will use default value 0.05" << std::endl;
        this->initialError->setOnes(this->getDof());
        (*this->motionError)*=0.05;
      }

      for(int i=0; i<this->getDof(); i++)
      {
        // sample an initial error
        double sampleError;
        do{
          ::boost::random::normal_distribution<> errorDistr(0, (*this->initialError)(i));
          sampleError = errorDistr(*this->motionErrorGen);
        }while(fabs(sampleError) > 2.0*(*this->initialError)(i)); //truncate errot to 2*sigma
        error[i] = sampleError;
      }


    }

    void
    NoisyModel::sampleMotionError(::rl::math::Vector &error)
    {
      if (NULL == this->motionErrorGen)
      {
        this->motionErrorGen = ::boost::make_shared<::boost::random::mt19937>(seed);
      }

      error.resize(this->getDof());

      if(this->motionError->rows()!=this->getDof())
      {
        std::cout << "warning: did not set motion error - will use default value 0.1" << std::endl;
        this->motionError->setOnes(this->getDof());
        (*this->motionError)*=0.1;
      }

      for(int i=0; i<this->getDof(); i++)
      {
        // sample a step error
        ::boost::random::normal_distribution<> stepDistr(0, (*this->motionError)(i));
        error[i] = stepDistr(*this->motionErrorGen);
        //std::cout << "error joint "<<i<<": " << error[i] << std::endl;
      }

    }
  }
}
