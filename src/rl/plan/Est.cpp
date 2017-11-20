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

#include <boost/make_shared.hpp>
#include <boost/random.hpp>
#include <boost/graph/random.hpp>

#include <rl/math/Vector.h>

#include "Est.h"
#include "NoisyModel.h"

#include <iostream>
#include <math.h>

namespace rl
{
  namespace plan
  {
    ::std::string 
    Est::getName() const 
    {
      return "EST";
    }

    VectorPtr Est::tryConnect(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen)
    {
      ::rl::math::Real distance = nearest.second;
      ::rl::math::Real step = distance;
      

      bool reached = false;
      
      if (step <= this->delta)
      {
        reached = true;
      }
      else
      {
        step = this->delta;
      }
      
      VectorPtr last = ::boost::make_shared< ::rl::math::Vector >(this->model->getDof());
      
      this->model->interpolate(*tree[nearest.first].q, chosen, step / distance, *last);
      
      if (NULL != this->viewer)
      {
//        this->viewer->drawConfiguration(*last);
      }
      
      this->model->setPosition(*last);
      this->model->updateFrames();
      
      if (this->model->isColliding())
      {
        // return NULL;
        return VectorPtr();
      }
      
      ::rl::math::Vector next(this->model->getDof());
      
      while (!reached)
      {
        distance = this->model->distance(*last, chosen);
        step = distance;
        
        if (step <= this->delta)
        {
          reached = true;
        }
        else
        {
          step = this->delta;
        }
        
        this->model->interpolate(*last, chosen, step / distance, next);
        
        if (NULL != this->viewer)
        {
//          this->viewer->drawConfiguration(next);
        }
        
        this->model->setPosition(next);
        this->model->updateFrames();
        
        if (this->model->isColliding())
        {
          break;
        }
        
        *last = next;
      }
      
      return last;

      // Vertex connected = this->addVertex(tree, last);
      // this->addEdge(nearest.first, connected, tree);
      // return connected;
    }

    bool Est::solve() 
    {
      ::std::cout << "Est solve!" << ::std::endl;
      this->begin[0] = this->addVertex(this->tree[0], ::boost::make_shared< ::rl::math::Vector >(*this->start));
      
      boost::random::mt19937 gen;
      boost::random::uniform_real_distribution<> distr(0, 2*M_PI);

      timer.start();
      timer.stop();

      std::cout << this->uncertaintyThreshold << std::endl;

      // store possible goals in here
      ::std::vector<PossibleGoal> possibleGoals;

      ::rl::math::Vector chosenSample(this->model->getDof());
      
      Vertex chosenVertex = this->begin[0];
      
      // main loop
      while (timer.elapsed() < this->duration)
      {
        int steps = 0;
        ::rl::math::Vector nextStep(*this->tree[0][chosenVertex].q);
        
        float angle = distr(gen);
        ::rl::math::Real stepX = ::std::cos(angle) * this->delta;
        ::rl::math::Real stepY = ::std::sin(angle) * this->delta; 
        
        ::rl::math::Real uncertainty = 0.0;

        // move into sampled direction until we collide
        while (!this->model->isColliding() && uncertainty < this->uncertaintyThreshold)
        {
          nextStep[0] += stepX;
          nextStep[1] += stepY;

          this->model->setPosition(nextStep);
          this->model->updateFrames();
          steps++;
          // right now simply add up uncertainty
          uncertainty += this->stepUncertainty;
        }

        // check if we actually moved through some free space and if we didn't exceed the uncertainty threshold
        if (steps > 1 && uncertainty < this->uncertaintyThreshold) 
        {
          // we had a collision, so reset uncertainty
          uncertainty = 0.0;

          Vertex collision_vertex = this->addVertex(this->tree[0], ::boost::make_shared< ::rl::math::Vector >(nextStep));
          this->addEdge(chosenVertex, collision_vertex, this->tree[0]);

          // store the uncertainty in vertex
          this->tree[0][collision_vertex].uncertainty = uncertainty;
          
          // try to connect the new vertex to the goal
          Neighbor nearest;
          nearest.first = collision_vertex;
          nearest.second = this->model->transformedDistance(*this->tree[0][collision_vertex].q, *this->goal);

          PossibleGoal possibleGoal;
          possibleGoal.neighbor = nearest;
          possibleGoal.q = this->tryConnect(this->tree[0], nearest, *this->goal);

          if (NULL != possibleGoal.q)
          {
            if (this->areEqual(*possibleGoal.q, *this->goal)) 
            {
              // calculate the uncertainty in the connected goal vertex by multiplying it with the connect distance
              possibleGoal.uncertainty = nearest.second / this->delta * this->stepUncertainty;
              ::std::cout 
                << "reached goal with uncertainty of " 
                << possibleGoal.uncertainty 
                << " (" << nearest.second << ")" 
                << ::std::endl;
              // save the vertex as one possible solution
              possibleGoals.push_back(possibleGoal);

              // check if we have collected enough possible goals
              if (possibleGoals.size() == this->nrPossibleGoals) {
                // find the goal vertex with the lowest uncertainty
                PossibleGoal *bestGoal = &possibleGoals[0];
                for (int i = 1; i < possibleGoals.size(); ++i)
                {
                  if (possibleGoals[i].uncertainty < bestGoal->uncertainty)
                  {
                    bestGoal = &possibleGoals[i];
                  }
                }
                // found the best goal, finally add a vertex and edge for it
                Vertex connected = this->addVertex(this->tree[0], bestGoal->q);
                this->addEdge(bestGoal->neighbor.first, connected, this->tree[0]);
                this->end[0] = connected;
                return true;
              }
            }
          }
        }

        // reset model to initial position, which should be collision-free
        this->model->setPosition(*this->tree[0][this->begin[0]].q);
        this->model->updateFrames();

        // choose new vertex randomly
        // chosenVertex = ::boost::random_vertex(this->tree[0], gen);

        // choose new vertex using Voronoi bias
        this->choose(chosenSample);
        chosenVertex = this->nearest(this->tree[0], chosenSample).first;

        timer.stop();
      }
      
      return false;
    }
  }
}
