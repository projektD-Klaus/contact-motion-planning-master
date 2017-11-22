//
// Copyright (c) 2016, Felix Wolff
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
#include <iostream>
#include <cmath>
#include <ctime>
#include <fstream>
#include <queue>
#include <stdexcept>

#include <boost/make_shared.hpp>
#include <boost/random.hpp>

#include <Eigen/Eigenvalues>

#include <rl/sg/Shape.h>
#include <rl/sg/Body.h>

#include "NoisyModel.h"
#include "Viewer.h"
#include "GaussianSampler.h"

#include "Cerrt.h"


#define END_EFFECTOR_RADIUS 0.025
#define CORNER_SAFETY_MARGIN_DISTANCE END_EFFECTOR_RADIUS //< min particle distance to a corner
#define MAX_INITIAL_COLLISION_ESCAPE_STEPS 3
#define MIN_EXTEND_LENGTH 5

#define pc(x) std::cout << #x << ": " << (x) << std::endl;

namespace rl
{
namespace plan
{
::std::string Cerrt::getName() const
{
  return "CERRT";
}

Cerrt::Cerrt() :
  Rrt(),
  nrParticles(20),
  fixedOrientationSlide(true),
  gamma(0.5),
  maxDistance(-1.0),
  maxUncertainty(-1.0),
  segmentation(NULL),
  usePlanes(false),
  maxExhaustion(10),
  ignoreExhaustedNodes(false),
  addAlpha(0.05), //needs tuning
  addLower(0.05),
  addRadius(10.0),
  useAdd(false)
{
  // comment in for random seed
  this->seed = std::time(0);
  this->gen = ::boost::make_shared<boost::random::mt19937>(this->seed);
  std::cout<<"random primitive seed: "<< this->seed<<std::endl;
}

///////////////////////////////////////////////////////////////////
/// Helper Functions for sampling
///////////////////////////////////////////////////////////////////

void Cerrt::sampleDirection(::rl::math::Vector& rd)
{
  boost::random::normal_distribution<> distr(0, 1);  // mean 0  sigma 1
  int dim = rd.rows();
  double rdsum = 0;
  for(int i=0; i<dim; i++)
  {
    rd[i] = distr(*this->gen);
    rdsum += rd[i]*rd[i];
  }
  rdsum = sqrt(rdsum);
  for(int i=0; i<dim; i++)
  {
    rd[i] /= rdsum;
  }
}

void
Cerrt::sampleInitialParticles(::std::vector<Particle>& initialParticles)   // ??
{
  ::rl::math::Vector initialError(this->model->getDof());
  initialParticles.clear();

  for(int i=0; i< this->nrParticles; i++)
  {
    rl::math::Vector sample;
    do
    {
      this->model->sampleInitialError(initialError);
      sample = *this->start + initialError;
      this->model->setPosition(sample);
      this->model->updateFrames();
    }while(this->model->isColliding());

    Particle p(sample);
    initialParticles.push_back(p);
  }
}


///////////////////////////////////////////////////////////////////
/// Overloaded RRT Methods
///////////////////////////////////////////////////////////////////


void
Cerrt::choose(::rl::math::Vector& chosen)
{
  ::boost::uniform_real< ::rl::math::Real > goalDistr(0.0f, 1.0f);
  if (goalDistr(*this->gen) > 0.1)
  {
    Rrt::choose(chosen);
  }
  else
  {
    chosen = *this->goal;
  }
}

Cerrt::Neighbor Cerrt::nearest(const Tree& tree, const ::rl::math::Vector& chosen)
{
  struct NeighborCompare
  {
    bool operator()(const Neighbor& lhs, const Neighbor& rhs) const
    {
      return lhs.second > rhs.second;
    }
  };

  std::priority_queue<Neighbor, std::vector<Neighbor>, NeighborCompare> neighbors;

  QueryItem query(&chosen, nullptr);

  //Check among 10 nearest neighbours
  int queryNum = 10;

  int maxK;


  maxK = this->maxExhaustion;

  do
  {
    for (NearestNeighbors::const_iterator i = tree[::boost::graph_bundle].nn.begin(); i != tree[::boost::graph_bundle].nn.end(); ++i)
    {
      if (NULL != *i)
      {
        NeighborSearch search(
              *i->get(),
              query,
              queryNum,
              0,
              true,
              Distance(this->model)
              );

        for(NeighborSearch::iterator it = search.begin(); it != search.end(); it++)
        {
          if(tree[it->first.second].k<maxK || !ignoreExhaustedNodes)
            neighbors.push(Neighbor(it->first.second, it->second));
        }
      }
    }

    maxK*=2;

  }while(neighbors.size() == 0);

  Neighbor bestNeighbor(nullptr, ::std::numeric_limits<::rl::math::Real>::max());

  int tested = 0;
  while (!neighbors.empty() && tested < queryNum)
  {
    const auto& vertex = (neighbors.top().first);
    const auto& mean = *(tree[vertex].q);
    const ::rl::math::Real distance = this->model->distance(mean, chosen);

    //Compute the covariance of the moved distribution
    ::rl::math::Matrix cov = tree[vertex].beliefState->eeCovariance();

    ::rl::math::Real distanceMetric =  distance;
    //L1
    ::rl::math::Real uncertaintyMetric = sqrt(cov.trace());

    //other options worth checking out
    //L2
    //::rl::math::Real uncertaintyMetric = sqrt(tree[vertex].beliefState->eeGaussian().eigenvalues()(0));
    //volume of ellipsoid
    //::rl::math::Real uncertaintyMetric = sqrt(cov.determinant());

    if(distanceMetric > this->maxDistance)
      this->maxDistance = distanceMetric;

    if(uncertaintyMetric > this->maxUncertainty)
      this->maxUncertainty = uncertaintyMetric;

    ::rl::math::Real metric = this->gamma * uncertaintyMetric / this->maxUncertainty +
        (1.0 - this->gamma) * distanceMetric / this->maxDistance;

    if (metric < bestNeighbor.second)
    {
      bestNeighbor.first = vertex;
      bestNeighbor.second = metric;
    }

    tested++;
    neighbors.pop();
  }


  return bestNeighbor;
}

Cerrt::ActionType
Cerrt::selectAction(const Tree& tree, const Vertex& nearest)
{
  // Check if a slide is possible from the nearest neighbor
  bool slidePossible = false;
  if(this->usePlanes)
    slidePossible = !tree[nearest].beliefState->getParticles()[0].planeIDs.empty();
  else
    slidePossible = tree[nearest].beliefState->isInCollision();

  if(slidePossible)
  {
    // randomly decide to do a slide or not
    boost::random::uniform_01<boost::random::mt19937> doSlideDistr(*this->gen);
    bool doSlide = doSlideDistr() < this->gamma;

    if (doSlide)
      return CONNECTSLIDE;
    else
      return CONNECT;
  }
  else
  {
    // randomly decide to do a slide or not
    boost::random::uniform_01<boost::random::mt19937> doGuardDistr(*this->gen);
    bool doGuardedMove = doGuardDistr() < this->gamma;

    if (doGuardedMove)
      return GUARDED;
    else
      return CONNECT;
  }
}





///////////////////////////////////////////////////////////////////
/// Main planner function
///////////////////////////////////////////////////////////////////

double Cerrt::workSpaceDistance(::boost::shared_ptr<BeliefState> node, rl::math::Transform goal)
{
  ::rl::math::Real maxError = 0;
  for(int i=0; i<node->getParticles().size(); i++)
  {
    this->model->setPosition(node->getParticles()[i].config);
    this->model->updateFrames();
    maxError = std::max(maxError,::rl::math::transform::distance(goal, this->model->forwardPosition(), 0.0));
  }

  return maxError;
}

bool Cerrt::solve()
{
  //update the seed of the noisy model
  this->model->seed = this->seed;

  //reset nn scaling factors
  this->maxDistance = -1.0;
  this->maxUncertainty = -1.0;


  // add the start configuation
  this->begin[0] = this->addVertex(this->tree[0], ::boost::make_shared<::rl::math::Vector>(*this->start));
  ::std::vector<Particle> initialParticles;
  this->sampleInitialParticles(initialParticles);


  this->tree[0][this->begin[0]].beliefState = ::boost::make_shared<BeliefState>(initialParticles, this->model);
  this->model->setPosition(*this->start);
  this->model->updateFrames();
  this->tree[0][this->begin[0]].t = ::boost::make_shared<::rl::math::Transform>(this->model->forwardPosition());


  timer.start();
  timer.stop();

  this->model->setPosition(*this->goal);
  this->model->updateFrames();
  ::rl::math::Transform goalT = this->model->forwardPosition();

  while (timer.elapsed() < this->duration)
  {
    Neighbor n;
    ::rl::math::Vector chosenSample(this->model->getDof());
    do
    {
      // sample random config

      this->choose(chosenSample);
      //Choose node to extend
      n = this->nearest(this->tree[0], chosenSample);
    }while (useAdd && n.second > this->tree[0][n.first].radius);

    //Choose action to execute
    ActionType u = this->selectAction(this->tree[0], n.first);

    ::std::vector<Particle> particles;
    ::rl::math::Vector3 slidingNormal;
    slidingNormal.setZero();

    bool sampleResult = this->simulate(u, n, chosenSample, this->nrParticles, true, particles, slidingNormal);

    if (!sampleResult)
    {
        if(n.first != this->begin[0])
          this->tree[0][n.first].k++; //increment failure counter

        if (this->tree[0][n.first].radius < ::std::numeric_limits< ::rl::math::Real >::max())
        {
          this->tree[0][n.first].radius *= (1.0f - this->addAlpha);
          this->tree[0][n.first].radius = ::std::max(this->addLower, this->tree[0][n.first].radius);
        }
        else
        {
          this->tree[0][n.first].radius = this->addRadius;
        }
    }
    else //add node
    {
      // visualize particles
      //this->drawParticles(particles);

      if (this->tree[0][n.first].radius < ::std::numeric_limits< ::rl::math::Real >::max())
      {
        this->tree[0][n.first].radius *= (1.0f + this->addAlpha);
      }

      ::boost::shared_ptr<BeliefState> belief = ::boost::make_shared<BeliefState>(particles, model, slidingNormal);
      belief->setActionType((int) u);
      Gaussian g = belief->configGaussian();

      // add a new vertex and edge
      VectorPtr mean = ::boost::make_shared<::rl::math::Vector>(particles[0].config);
      Vertex newVertex = this->addVertex(this->tree[0], mean);

      //this->drawEigenvectors(g, 1.0);
      //this->drawParticles(particles);

      this->tree[0][newVertex].beliefState = belief;
      this->model->setPosition(particles[0].config);
      this->model->updateFrames();
      this->tree[0][newVertex].t = ::boost::make_shared<::rl::math::Transform>(this->model->forwardPosition());

      this->addEdge(n.first, newVertex, this->tree[0]);

      //Check if new node is close to goal
      double maxError = workSpaceDistance(belief, goalT);
      if (maxError < this->goalEpsilon)
      {
        // visualize goal connect step
        this->drawParticles(particles);
        this->end[0] = newVertex;
        return true;
      }


      // We will now attempt to do a connect move from the last sample to goal.
      //First shoot one particle, without noise.
      rl::math::Vector zeroNoise(this->model->getDof());
      zeroNoise.setZero();
      Particle goalTestParticle;
      bool goalTest = this->sampleConnectParticle(this->tree[0][newVertex].beliefState->getParticles()[0].config, *this->goal, zeroNoise, false, goalTestParticle);

      // check if connect step reached the goal
      if (goalTest && this->areEqual(goalTestParticle.config, *this->goal)) {

        //Connect was successful - now do full simulation
        // sample particles for the connect step
        Neighbor nearest;
        nearest.first = newVertex;
        nearest.second = this->model->transformedDistance(*this->tree[0][newVertex].q, *this->goal);
        ::std::vector<Particle> goalParticles;
        if (this->simulate(CONNECT, nearest, *this->goal, this->nrParticles, false, goalParticles, slidingNormal))
        {
          ::boost::shared_ptr<BeliefState> goalState = ::boost::make_shared<BeliefState>(goalParticles, model);

          double maxError = workSpaceDistance(goalState, goalT);
          std::cout << "reached goal with error: " << maxError << " (max allowed: " << this->goalEpsilon << ")" << std::endl;

          if (maxError < this->goalEpsilon)
          {
            // visualize goal connect step
            this->drawParticles(goalParticles);
            Gaussian g = goalState->configGaussian();
            VectorPtr mean = ::boost::make_shared<::rl::math::Vector>(goalParticles[0].config);
            // add goal connect step to tree
            Vertex connected = this->addVertex(this->tree[0], mean);
            this->tree[0][connected].beliefState = goalState;
            this->model->setPosition(*mean);
            this->model->updateFrames();
            this->tree[0][connected].t = ::boost::make_shared<::rl::math::Transform>(this->model->forwardPosition());

            this->addEdge(newVertex, connected, this->tree[0]);
            this->end[0] = connected;
            return true;
          }
        }
      }
    }
    timer.stop();
  }

  return false;
}


/**
    */
bool Cerrt::isSensor(Particle& particle)
{
  for(::rl::sg::CollisionMap::iterator it = particle.contacts.begin(); it != particle.contacts.end(); it++)
  {
    if(!it->second.isSensor)
      return false;
  }

  return true;
}

/**
    */
bool Cerrt::isSelfCollision(Particle& particle)
{
  for(::rl::sg::CollisionMap::iterator it = particle.contacts.begin(); it != particle.contacts.end(); it++)
  {
    if(it->second.selfCollision)
      return true;
  }

  return false;
}

bool Cerrt::simulate(ActionType action, const Neighbor& nearest, const ::rl::math::Vector& chosen, int nrParticles, bool consistentContactState, ::std::vector<Particle>& particles, ::rl::math::Vector3& slidingNormal)
{

  particles.clear();
  rl::sg::CollisionMap initialContacts;
  std::vector <int> initialPlanes;


  //For sliding actions we need to sample a sliding surface
  int slidingIdx = 0;
  ::rl::math::Vector3 chosen_proj_point;
  ::rl::math::Transform chosen_proj;
  Particle oldParticle;
  rl::sg::CollisionQueryResult slidingCollisionResult;
  std::pair<std::string, std::string> slidingPair ("default", "default");
  if(action >= CONNECTSLIDE)
  {
    oldParticle = *(this->tree[0][nearest.first].beliefState->getParticles().begin());

    // Use collision info to choose a plane to slide on
    if(!this->usePlanes) {

      if(oldParticle.contacts.size() == 0)
      {
        std::cout<<"Attempt slide from particle not in contact --- This should not happen!!!"<<std::endl;
        return false;
      }

      //Sample a sliding surface
      boost::random::uniform_int_distribution<> chooseSurfaceDistr(0, oldParticle.contacts.size()-1);
      slidingIdx = chooseSurfaceDistr(*this->gen);

      // Get the collision info for the sliding surface
      ::rl::sg::CollisionMap::iterator slidingContactIt = oldParticle.contacts.begin();
      std::advance(slidingContactIt,slidingIdx);
      slidingCollisionResult = slidingContactIt->second;
      slidingPair = slidingContactIt->first;
    }

    // Use geometry info to choose a plane to slide on
    else {

      // Assert that particle is in contact with the surface
      if(oldParticle.planeIDs.empty()) {
        throw std::runtime_error(std::string("Slide from particle not in contact!"));
      }

      // Sample a sliding surface
      boost::random::uniform_int_distribution<> chooseSurfaceDistr(0, oldParticle.planeIDs.size()-1);
      int slidingIdx = chooseSurfaceDistr(*this->gen);
      rl::sg::Plane& plane = segmentation->planes[oldParticle.planeIDs[slidingIdx]];

      // Get the collision info for the sliding surface
      this->model->setPosition(oldParticle.config);
      this->model->updateFrames();
      ::rl::math::Transform eeT = this->model->forwardPosition();
      ::rl::math::Vector4 pos (eeT(0,3), eeT(1,3), eeT(2,3), 1);
      double dist = pos.dot(plane.params);
      slidingCollisionResult.normal = plane.params.block<3,1>(0,0);
      slidingCollisionResult.commonPoint = pos.block<3,1>(0,0) - dist * slidingCollisionResult.normal;
    }

    // We move the EE frame to the contact point. This way the Jacobian
    // can be used to project the contact point back on the sliding surface.
    // Caution: whenever this function exits, model->resetTool must be called!
    this->model->setPosition(oldParticle.config);
    this->model->updateFrames();

    //ee in world frame
    ::rl::math::Transform ee_world = this->model->forwardPosition();
    //contact frame - orientation is same as ee
    ::rl::math::Transform cp_world = ee_world;
    //Translation ist shifted to contact point
    cp_world.translation() = slidingCollisionResult.commonPoint;
    //CP in EE frame
    ::rl::math::Transform cp_ee = ee_world.inverse()*cp_world;
    this->model->updateTool(cp_ee);


    //Now take the chosen  expansion direction and project it on the sliding surface

    //Get the forward pose of the random sample
    this->model->setPosition(chosen);
    this->model->updateFrames();
    ::rl::math::Vector3 chosenForwardPos = this->model->forwardPosition().translation();
    //Project it onto the surface
    double distToSurface = this->projectOnSurface(chosenForwardPos, slidingCollisionResult.commonPoint, slidingCollisionResult.normal, chosen_proj_point);
    chosen_proj.translation() = chosen_proj_point;
    chosen_proj.linear() = this->model->forwardPosition().linear();
  }


  int pIdx = 0;
  while (pIdx < nrParticles)
  {
    // sample a starting point from the gaussian
    ::rl::math::Vector init(this->model->getDof());
    this->tree[0][nearest.first].beliefState->sampleConfigurationFromParticle(init, pIdx);

    // sample noise
    ::rl::math::Vector motionNoise(this->model->getDof());
    this->model->sampleMotionError(motionNoise);

    ::rl::math::Vector mean = *(this->tree[0][nearest.first].q);

    ::rl::math::Vector initialError = init - mean;
    ::rl::math::Vector target = chosen + initialError;

    bool simSuccess;
    Particle particle;

    switch( action ) {
    case CONNECT:
      simSuccess = this->sampleConnectParticle(init, target, motionNoise, false, particle);
      break;
    case GUARDED:
      simSuccess = this->sampleConnectParticle(init, target, motionNoise, true, particle);
      break;
    case CONNECTSLIDE:
      simSuccess = this->sampleSlidingParticle(init, mean, chosen_proj, motionNoise, slidingPair, slidingCollisionResult, false, particle);
      break;
    case GUARDEDSLIDE:
      simSuccess = this->sampleSlidingParticle(init, mean, chosen_proj, motionNoise, slidingPair, slidingCollisionResult, true, particle);
      break;
    default:
      simSuccess = false;
      break;
    }

    if(action >= CONNECTSLIDE)
    {
      slidingNormal = slidingCollisionResult.normal;
    }

    if(!simSuccess)
    {
      if(action >= CONNECTSLIDE)
      {
        this->model->resetTool();
      }
      return false;
    }

    if(action < CONNECTSLIDE && particle.contacts.size()>1)
    {
      std::cout<<"connect/guarded move ended in a state with two collisions"<<std::endl;
      return false;
    }

    // Check that all the particles have the same contacts
    // For instance, in a guarded connect, some of the particles may hit wall A and others
    // wall B. This is not good because later on, we would not which wall to slide from.
    if(consistentContactState) {

      // Save the contact state of the first particle to check later against other
      // If plane information is used, then save the planes; otherwise, the collision info
      if(pIdx == 0) {
        if(this->usePlanes) initialPlanes = particle.planeIDs;
        else initialContacts = particle.contacts;
      }

      // Check for contact consistency between the first particle and the current one
      else {

        // Either use the plane or the collision state
        bool consistencyProblem = 0;
        if(this->usePlanes) {

          // Check if the size of contact planes are the same and if so, whether the planes are the same
          consistencyProblem = ((initialPlanes.size() != particle.planeIDs.size()) ||
                                (!((std::mismatch (initialPlanes.begin(), initialPlanes.end(),
                                                   particle.planeIDs.begin())).first == initialPlanes.end())));
        }
        else
          consistencyProblem = !isEqualCollisionState(particle.contacts,initialContacts);

        // Return if there is an inconsistency
        if(consistencyProblem) {
          if(action >= CONNECTSLIDE)
          {
            this->model->resetTool();
          }
          return false;
        }
      }
    }

    if(!isSensor(particle))
    {
      if(action >= CONNECTSLIDE)
      {
        this->model->resetTool();
      }
      return false;
    }

    if(isSelfCollision(particle))
    {
      if(action >= CONNECTSLIDE)
      {
        this->model->resetTool();
      }
      return false;
    }

    particles.push_back(particle);
    pIdx++;
  }

  if(action >= CONNECTSLIDE)
  {
    this->model->resetTool();
  }
  return true;
}





/**
      Samples a particle for a move through free-space to a target configuration
    */
bool Cerrt::sampleConnectParticle(const ::rl::math::Vector& init, const ::rl::math::Vector& target, const ::rl::math::Vector& motionNoise, bool guardedMove, Particle& particle)
{
  ::rl::math::Vector nextStep = init;

  int steps = 0;
  ::rl::math::Real step = this->delta;
  ::rl::math::Real distance = this->model->distance(init, target);

  rl::sg::CollisionMap allColls, initColls;

  //Obtain initial collisions
  this->model->setPosition(init);
  this->model->updateFrames();
  this->model->isColliding();
  initColls = this->getAllCollidingShapes();
  bool inInitialCollision  = (initColls.size() > 0);
  bool reached = false;
  bool collision = false;

  while (inInitialCollision || (!collision && !reached))
  {
    if (step >= distance && !guardedMove)
    {
      reached = true;
      step = distance;
    }

    this->model->interpolateNoisy(init, target,  step / distance, motionNoise, nextStep);

    //Check for joint limits
    if(!this->model->isValid(nextStep))
      return false;

    this->model->setPosition(nextStep);
    this->model->updateFrames();

    collision = this->model->isColliding();

    if(inInitialCollision)
    {
      allColls = this->getAllCollidingShapes();
      inInitialCollision = isEqualCollisionState(initColls, allColls);
    }

    //We could not move out of the initial collision within three steps - we most likely moved into the wall
    if(inInitialCollision && steps > MAX_INITIAL_COLLISION_ESCAPE_STEPS)
      return false;

    step += this->delta;
    steps++;
  }

  if(steps <= MIN_EXTEND_LENGTH)
    return false;

  //Store contact
  particle.contacts =  this->model->scene->getLastCollisions();

  //Store config
  particle.config = nextStep;

  // Compute the plane this particle is in contact with
  if(this->usePlanes) {
    ::rl::math::Transform ee_world = this->model->forwardPosition();
    ::rl::math::Vector3 pos (ee_world(0,3), ee_world(1,3), ee_world(2,3));
    std::vector <int> planeIDs;
    segmentation->getPlanesForSample(pos, 0.01 + END_EFFECTOR_RADIUS, 0.01, planeIDs);
    particle.planeIDs = planeIDs;
  }
  return true;
}


double Cerrt::projectOnSurface(const ::rl::math::Vector3& point, const ::rl::math::Vector3& pointOnSurface, const ::rl::math::Vector3& normal, ::rl::math::Vector3& out)
{

  double dist = (point-pointOnSurface).dot(normal);
  out = point-dist*normal;
  return dist;
}

bool Cerrt::moveConfigOnSurface(const ::rl::math::Vector& config, const ::rl::math::Vector3& pointOnSurface, const ::rl::math::Vector3& normal, const std::pair<std::string, std::string>& collPair, ::rl::math::Vector& out)
{

  //Step 1: move end-effector on the line given by point an normal

  double dist = std::numeric_limits<double>::infinity();
  out = config;

  this->model->setPosition(out);
  this->model->updateFrames();
  this->model->updateJacobian();
  this->model->updateJacobianInverse();

  ::rl::math::Transform eeT = this->model->forwardPosition();
  ::rl::math::Transform goalT = eeT;
  ::rl::math::Vector3 goaltransl;
  dist = projectOnSurface(eeT.translation(),pointOnSurface, normal, goaltransl);

  int steps = 0;
  while(fabs(dist) > 0.01 && steps++<10)
  {
    goalT.translation() = goaltransl;

    rl::math::Vector6 tdot;
    ::rl::math::transform::toDelta(eeT, goalT, tdot);

    rl::math::Vector qdot;
    this->model->inverseVelocity(tdot, qdot);

    if(qdot.norm() > this->delta)
    {
      qdot.normalize();
      qdot*=this->delta;
    }

    out += qdot;
    this->model->setPosition(out);
    this->model->updateFrames();
    this->model->updateJacobian();
    this->model->updateJacobianInverse();

    eeT = this->model->forwardPosition();
    dist = projectOnSurface(eeT.translation(),pointOnSurface, normal, goaltransl);
  }

  // Use of plane information removes the necessity to do collision checking for cliffs
  if(usePlanes) return true;

  if(fabs(dist)>0.01)
    return false;

  //Step 2: Move end-effector towards surface until colliding
  steps = 0;

  this->model->isColliding();
  rl::sg::CollisionMap colls = this->getAllCollidingShapes();
  bool collision = (colls.find(collPair) != colls.end());

  //Store the configuration before projecting onto collision surface
  ::rl::math::Vector preProjection = out;

  while(!collision && steps++ < 10)
  {
    rl::math::Vector6 tdot;
    tdot.setZero();
    tdot(0) = -normal(0); tdot(1) = -normal(1); tdot(2) = -normal(2);

    rl::math::Vector qdot;
    this->model->inverseVelocity(tdot, qdot);
    qdot.normalize();
    qdot *= this->delta;
    out += qdot;
    this->model->setPosition(out);
    this->model->updateFrames();
    this->model->updateJacobian();
    this->model->updateJacobianInverse();
    this->model->isColliding();

    colls = this->getAllCollidingShapes();
    collision = (colls.find(collPair) != colls.end());

  }



  //There is no contact with the sliding surface - return the configuration from step 1.
  if(!collision)
  {
    out = preProjection;
    return false;
  }

  return true;
}

Cerrt::Edge Cerrt::addEdge(const Vertex& u, const Vertex& v, Tree& tree)
{

  Edge e = ::boost::add_edge(u, v, tree).first;
  VertexBundle vb = tree[v];
  int actionType = vb.beliefState->getActionType();

  if (NULL != this->viewer) {

    // Add a joint space visualization if a normal connect move
    if(actionType == -1 || actionType == CONNECT || actionType == GUARDED)
      this->viewer->drawConfigurationEdge(*tree[u].q, *tree[v].q);
    else if(actionType == CONNECTSLIDE || actionType == GUARDEDSLIDE) {
      this->model->setPosition(*tree[u].q);
      this->model->updateFrames();
      ::rl::math::Transform eeT = this->model->forwardPosition();
      ::rl::math::Vector4 pos (eeT(0,3), eeT(1,3), eeT(2,3), 1);
      this->model->setPosition(*tree[v].q);
      this->model->updateFrames();
      eeT = this->model->forwardPosition();
      ::rl::math::Vector4 pos2 (eeT(0,3), eeT(1,3), eeT(2,3), 1);
      this->viewer->drawLine(pos, pos2);
    }
    else
      throw std::runtime_error(std::string(
                                 "Do not have a visualization for this action type yet!"));
  }

  return e;
}

/**
      Samples a set of particles for a sliding move along a surface.
    */
bool Cerrt::sampleSlidingParticle(const ::rl::math::Vector& init, const ::rl::math::Vector& mean, const ::rl::math::Transform& chosen_proj, const ::rl::math::Vector& motionNoise, const std::pair<std::string, std::string>& slidingPair, const rl::sg::CollisionQueryResult& slidingInfo, bool guardedMove, Particle& particle)
{

  //The sliding velocity -- same for all particles
  ::rl::math::Vector6 tdot;

  int steps = 0;
  //This is where the particle actually is
  ::rl::math::Vector nextStepReal = init;
  //This is where the particle thinks it is
  ::rl::math::Vector nextStepVirtual = mean;

  bool reached = false;
  ::rl::sg::CollisionMap allColls;
  rl::sg::Plane* plane = NULL;
  std::vector <int> planeIDs;

  while (!reached)
  {
    this->model->setPosition(nextStepVirtual);
    this->model->updateFrames();
    this->model->updateJacobian();
    this->model->updateJacobianInverse();

    //guarded slides only compute tdot once, connect slides update it in every step
    if(!guardedMove || steps == 0)
    {
      rl::math::Transform ee_world = this->model->forwardPosition();

      //This is the target pose of our slide
      ::rl::math::Transform goal_world = chosen_proj;

      //Compute a 6d Velocity twist for the task-space controller
      ::rl::math::transform::toDelta(ee_world, goal_world, tdot);

      //2D models do not move in z-direction (this is a convention)
      if(this->model->getDof() <= 2)
        tdot(2) = 0;

      if(this->fixedOrientationSlide || this->model->getDof() <= 2)
        tdot(3) = tdot(4) = tdot(5) = 0.0;

    }

    // Get the plane the particle is in contact with
    // TODO MAJOR This could be a parameter to this function as it is computed in ::simulate
    // Do this after the pull request since it possibly requires a function interface change
    if(this->usePlanes && (steps == 0)) {

      // Get the actual tool position (and not the contact point shifted one)
      rl::math::Transform ee_world = this->model->forwardPosition();
      ::rl::math::Vector3 pos (ee_world(0,3), ee_world(1,3), ee_world(2,3));
      ::rl::math::Vector3 pos2 = pos + slidingInfo.normal * END_EFFECTOR_RADIUS;

      // Get the planes that the EE is in contact with
      segmentation->getPlanesForSample (pos2, 0.01 + END_EFFECTOR_RADIUS, 0.01, planeIDs);

      // Sanity check
      if(planeIDs.empty())
        throw std::runtime_error(std::string("Could not find a plane to slide along"));

      // Choose the plane that is closest to the input parameters (see todo above)
      int pidx = -1;
      for(int i = 0; i < planeIDs.size(); i++) {
        rl::sg::Plane& plane_x = this->segmentation->planes[planeIDs[i]];
        double dist = (plane_x.params.block<3,1>(0,0) - slidingInfo.normal).norm();
        if(dist < 1e-3) {
          pidx = i;
          break;
        }
      }
      if(pidx == -1)
        throw std::runtime_error(std::string(
                                   "Could not find a plane to match the input one from simulate"));
      plane = &(segmentation->planes[planeIDs[pidx]]);
    }

    ::rl::math::Vector qdot(this->model->getDof());
    qdot.setZero();
    this->model->inverseVelocity(tdot, qdot);

    if(qdot.norm() < this->delta)
      reached = true;
    else
    {
      qdot.normalize();
      qdot *= this->delta;
    }

    ::rl::math::Vector newStep(this->model->getDof());
    //The robot thinks it is exactly following the line
    nextStepVirtual = nextStepVirtual+qdot;

    //In reality there is noise
    this->model->interpolateNoisy(nextStepReal, nextStepReal + qdot, 1, motionNoise, newStep);
    //Project back on sliding surface
    moveConfigOnSurface(newStep, slidingInfo.commonPoint, slidingInfo.normal, slidingPair, nextStepReal);

    //Check for joint limits
    if(!this->model->isValid(nextStepReal))
    {
      return false;
    }
    //this->viewer->drawConfiguration(nextStepReal);


    steps++;

    if(steps > 10.0/this->delta)
    {
      std::cout<<"Model left the scene - this should not happen, FIXME!"<<std::endl;
      return false;
    }

    this->model->setPosition(nextStepReal);
    this->model->updateFrames();
    this->model->updateJacobian();
    this->model->updateJacobianInverse();

    //check for singularity
    if(this->model->getDof() > 3 && this->model->getManipulabilityMeasure()  < 1.0e-3f)
    {
      return false;
    }

    //Check for collision
    this->model->isColliding();
    allColls = this->getAllCollidingShapes();

    // Check for the signifance of the collisions (ignore the ones with sliding surface)
    // If there's a collision, we stop sliding and see if it's long enough to count
    // TODO Check that the contact point of EE collision is on the sliding plane and no
    // where else
    if(this->usePlanes) {

      // Check if there is a non-EE or self collision; stop sliding motion if there is one
      for(::rl::sg::CollisionMap::iterator it = allColls.begin(); it != allColls.end();it++){
        std::pair <std::string, std::string> colls = it->first;
        if(!it->second.isSensor || it->second.selfCollision) {
          std::pair <std::string, std::string> colls = it->first;
          break;
        }
      }
    }

    else {

      // Check if initial collision state is preserved
      if (allColls.find(slidingPair) != allColls.end()) {
        if (allColls.size() > 1) {
          // there is another collision, so the sliding ends here
          break;
        }
      }
      else {
        // we lost contact
        break;
      }

    }

    // If guarded motion, check which neighbor surfaces the particle can move onto now
    // For wall collisions, note that the collision check above for usePlanes ignores
    // EE collisions and those are handled here.
    if(this->usePlanes) {
      int reachResult = reachedPolygonEdge(plane, tdot.block<3,1>(0,0));
      if(reachResult == 1) break;
      else if(reachResult == -1) return false;
    }
  }

  // some magic number
  if (steps < MIN_EXTEND_LENGTH)
  {
    // we did not move very far, this is considered failure
    return false;
  }

  // valid particle, store it
  particle.config = nextStepReal;

  // Remove the current plane ID from the new particle's list of IDs
  if(this->usePlanes) {

    // Set the next particle's available planes (assuming the slide lead to a loss with
    // only the sliding plane and not a neighboring one)
    std::vector <int>::iterator it = find(planeIDs.begin(), planeIDs.end(), plane->id);
    if(it == planeIDs.end()) throw std::runtime_error(std::string(
                                                        "The plane that was used to slide should be in the original list!"));
    planeIDs.erase(it);
    particle.planeIDs = planeIDs;
  }
  else particle.contacts = allColls;

  return true;
}

int Cerrt::reachedPolygonEdge (rl::sg::Plane* plane, const ::Eigen::Vector3d& tdotTrans)
{
  // Iterate through the neighbors to see if the particle has crossed to that one
  for(int neighID = 0; neighID < plane->neighs.size(); neighID++) {

    // Get the neighbor
    rl::sg::Plane* neigh = plane->neighs[neighID].second;

    // Change the plane offset based on the wall or cliff nature of the connection
    ::rl::math::Transform eeT = this->model->forwardPosition();
    ::rl::math::Vector4 pos (eeT(0,3), eeT(1,3), eeT(2,3), 1);
    ::rl::math::Vector3 normal = plane->params.block<3,1>(0,0);
    pos.block<3,1>(0,0) += normal * END_EFFECTOR_RADIUS;

    // Test if the particle has crossed the thresholds for the neighboring plane
    // Note that the threshold is different based on wall/cliff connections
    bool crossedPlane = false;
    if(plane->neighs[neighID].first) {
      double dist = neigh->params.dot(pos) - END_EFFECTOR_RADIUS;
      crossedPlane = (dist < 0);
    }
    else {
      double dist = neigh->params.dot(pos) - 1.5 * END_EFFECTOR_RADIUS;
      crossedPlane = (dist > 0);
    }

    // Check the distance to the polygon
    if(crossedPlane) {

      // If a cliff contact, ensure we use the edge crossing and not the final pose
      ::Eigen::Vector3d checkPos = pos.block<3,1>(0,0);
      if(!plane->neighs[neighID].first) {

        // Get the outward edge normal on plane
        ::Eigen::Vector3d v1 = plane->neighEdges[neighID].first;
        ::Eigen::Vector3d v2 = plane->neighEdges[neighID].second;
        ::Eigen::Vector3d d21 = (v2 - v1).normalized();
        ::Eigen::Vector3d dEdge =(d21.cross(plane->params.block<3,1>(0,0))).normalized();

        // Compute distance traveled away from edge
        ::Eigen::Vector3d pp = v1 + (checkPos-v1).dot(d21) * d21;
        double tdot_along = tdotTrans.dot(d21);
        double tdot_perp = tdotTrans.dot(dEdge);
        double particle_along = (checkPos - pp).norm() * (tdot_along / tdot_perp);
        ::Eigen::Vector3d checkPos2 = pp - (particle_along * d21);
        checkPos = checkPos2;
      }

      // Check
      bool safeMargins = checkSafeCrossing(*plane, neighID, checkPos,
                                           CORNER_SAFETY_MARGIN_DISTANCE);
      if(safeMargins) {
        return 1;
      }
      else return -1;
    }
  }

  return 0;
}

bool Cerrt::checkSafeCrossing (const rl::sg::Plane& plane, int neighID,
                               const ::rl::math::Vector3& pos, double margin) {

  // Get the end points
  ::Eigen::Vector3d v1 = plane.neighEdges[neighID].first;
  ::Eigen::Vector3d v2 = plane.neighEdges[neighID].second;
  ::Eigen::Vector3d d21 = (v2 - v1);
  double length = (v2 - v1).norm();
  d21 /= length;

  // Pull the endpoints closer due to the margin
  v2 = v1 + (length - margin) * d21;
  v1 = v2 - (length - 2 * margin) * d21;
  //this->viewer->drawLine(v1 + plane.params.block<3,1>(0,0) * 0.01,
  //  v2 + plane.params.block<3,1>(0,0) * 0.01);
  length -= 2 * margin;

  // Project the point onto the edge
  ::Eigen::Vector3d pp = v1 + (pos-v1).dot(d21) * d21;
  //this->viewer->drawPoint(pp + plane.params.block<3,1>(0,0) * 0.01);

  // Compute the distance to both end-points
  double d1 = (pp - v1).norm();
  double d2 = (pp - v2).norm();
  //printf("n: %lu, ds: (%lf, %lf) vs. %lf\n" plane.neighs[neighID].second->id,d1,d2,length);
  if((d1 > length) || (d2 > length)) return false;
  return true;
}




///////////////////////////////////////////////////////////////////
/// Path output
///////////////////////////////////////////////////////////////////

/**
            Computes a path from start to goal and incorporates motion error if requested.
    */

void Cerrt::getPath(VectorList& path, int count)
{
  path.clear();
  Vertex i = this->end[0];
  ::rl::math::Vector q(this->model->getDof());;
  while (i != this->begin[0])
  {
    ::boost::shared_ptr<BeliefState> gs = this->tree[0][i].beliefState;

    int idx = count % gs->getParticles().size();
    path.push_front(gs->getParticles()[idx].config);
    i = ::boost::source(*::boost::in_edges(i, this->tree[0]).first, this->tree[0]);
  }

  ::boost::shared_ptr<BeliefState> gs = this->tree[0][i].beliefState;
  int idx = count % gs->getParticles().size();
  path.push_front(gs->getParticles()[idx].config);
}

void Cerrt::writeOutCurrentPath(std::string& path_string, rl::plan::VectorList path)
{
  std::stringstream path_ss;
  Vertex i = this->end[0];
  std::list< Vertex > states;
  while (i != this->begin[0])
  {
    states.push_front(i);
    i = ::boost::source(*::boost::in_edges(i, this->tree[0]).first, this->tree[0]);
  }

  states.push_front(i);

  for( std::list< Vertex >::iterator it = states.begin(); it != states.end(); it++)
  {
    ::boost::shared_ptr<BeliefState> gs = this->tree[0][*it].beliefState;
    ::rl::math::Vector qm =*(this->tree[0][*it].q);
    for(int i=0; i<qm.rows(); i++)
      path_ss<<qm(i)<<"\t";
    int inContact = 0;
    if(gs->isInCollision())
      inContact = 1;
    path_ss<<inContact<<"\t";

    if(gs->isSlidingMove())
    {
      path_ss<<gs->getSlidingNormal()(0)<<"\t"<<gs->getSlidingNormal()(1)<<"\t"<<gs->getSlidingNormal()(2)<<"\t";
    }
    else
    {
      path_ss<<"0\t0\t0\t";
    }

    ::rl::math::Transform ee_t = *(this->tree[0][*it].t);
    ::rl::math::Quaternion q(ee_t.linear());
    path_ss<<ee_t(0,3)<<"\t"<<ee_t(1,3)<<"\t"<<ee_t(2,3)<<"\t"<< q.x()<<"\t"<<q.y()<<"\t"<<q.z()<<"\t"<<q.w();
    path_ss<<std::endl;


  }
  path_string = path_ss.str();
  //std::cout<<path_string<<std::endl;
}

bool Cerrt::isEqualCollisionState(::rl::sg::CollisionMap& first, ::rl::sg::CollisionMap& second)
{
  if (first.size() != second.size())
  {
    return false;
  }
  else
  {
    for(::rl::sg::CollisionMap::iterator it = first.begin(); it != first.end(); it++)
    {
      if(second.find(it->first) == second.end())
      {
        return false;
      }
    }

    return true;
  }
}

const rl::sg::CollisionMap&  Cerrt::getAllCollidingShapes()
{
  return this->model->scene->getLastCollisions();
}

void Cerrt::overwriteRandomSeed(int seed_) {
  this->seed = seed_;
  this->gen = ::boost::make_shared<boost::random::mt19937>(this->seed);
}

///////////////////////////////////////////////////////////////////
/// Viewer function
///////////////////////////////////////////////////////////////////


void Cerrt::drawParticles(const ::std::vector<Particle>& particles)
{
  for (int i = 0; i < particles.size(); ++i)
  {
    ::rl::math::Vector3 opPos;

    //        if(particles[i].contacts.size() > 0)
    //        {
    //          for (int j = 0; j < particles[i].contacts.size(); ++j)
    //          {
    //            opPos = particles[i].contacts[j].point;
    //            if(this->viewer)
    //              this->viewer->drawSphere(opPos, 0.02);
    //          }
    //        }
    //        else
    {
      this->model->setPosition(particles[i].config);
      this->model->updateFrames();
      const ::rl::math::Transform& t = this->model->forwardPosition();

      opPos[0] = t.translation().x();
      opPos[1] = t.translation().y();
      opPos[2] = t.translation().z();
      if(this->viewer)
        this->viewer->drawSphere(opPos, 0.02);
    }

  }
}

void Cerrt::drawEigenvectors(Gaussian& gaussian, ::rl::math::Real scale)
{
  ::rl::math::Vector3 start, end1, end2, end3;
  ::rl::math::Matrix vectors = gaussian.eigenvectors();
  ::rl::math::Vector values = gaussian.eigenvalues();
  ::rl::math::Vector eigen1 = vectors.col(0) * sqrt(values(0)) * scale;
  ::rl::math::Vector eigen2 = vectors.col(1) * sqrt(values(1)) * scale;

  start(0) = gaussian.mean(0);
  start(1) = gaussian.mean(1);
  start(2) = gaussian.mean(2);



  if(this->model->getDof()==2)
  {
    this->model->setPosition(gaussian.mean+eigen1);
    this->model->updateFrames();
    const ::rl::math::Transform& t2 = this->model->forwardPosition();
    end1(0) = t2.translation().x();
    end1(1) = t2.translation().y();
    end1(2) = t2.translation().z();

    this->model->setPosition(gaussian.mean+eigen2);
    this->model->updateFrames();
    const ::rl::math::Transform& t3 = this->model->forwardPosition();
    end2(0) = t3.translation().x();
    end2(1) = t3.translation().y();
    end2(2) = t3.translation().z();
  }

  if(this->model->getDof()>=3)
  {
    ::rl::math::Vector eigen3 = vectors.col(2) * sqrt(values(2)) * scale;
    end1 = start+eigen1;
    end2 = start+eigen2;
    end3 = start+eigen3;
    this->viewer->drawLine(start, end3);
  }

  this->viewer->drawLine(start, end1);
  this->viewer->drawLine(start, end2);

}

void Cerrt::drawSurfaceNormal(::rl::math::Vector3& startPoint, ::rl::math::Vector3& normal, ::rl::math::Real scale)
{
  this->viewer->drawLine(startPoint, startPoint+normal*scale);
}

}
}
