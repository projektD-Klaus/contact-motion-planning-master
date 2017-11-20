//
// Copyright (c) 2009, Markus Rickert
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

#include <rl/util/Timer.h>

#include "GaussianSampler.h"
#include "SimpleModel.h"

namespace rl
{
	namespace plan
	{
		GaussianSampler::GaussianSampler() :
			UniformSampler(),
			sigma(NULL),
			gauss(
				::boost::mt19937(static_cast< ::boost::mt19937::result_type >(::rl::util::Timer::now() * 1000000.0f)),
				::boost::normal_distribution< ::rl::math::Real >(0.0f, 1.0f)
			)
		{
		}
		
		GaussianSampler::~GaussianSampler()
		{
		}
		
		void
		GaussianSampler::generateCollisionFree(::rl::math::Vector& q)
		{
			assert(q.size() == this->model->getDof());
			
			::rl::math::Vector q2(this->model->getDof());
			
			while (true)
			{
				this->generate(q);
				
				for (::std::size_t i = 0; i < this->model->getDof(); ++i)
				{
					q2(i) = this->gauss() * (*this->sigma)(i) + q(i);
				}
				
				this->model->clip(q2);
				
				this->model->setPosition(q);
				this->model->updateFrames();
				
				if (!this->model->isColliding())
				{
					this->model->setPosition(q2);
					this->model->updateFrames();
					
					if (this->model->isColliding())
					{
						return;
					}
				}
				else
				{
					this->model->setPosition(q2);
					this->model->updateFrames();
					
					if (!this->model->isColliding())
					{
						q = q2;
						return;
					}
				}
			}
		}
		
		void
		GaussianSampler::seed(const ::boost::mt19937::result_type& value)
		{
			this->gauss.engine().seed(value);
			this->rand.engine().seed(value);
		}
	}
}
