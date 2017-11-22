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

#ifndef _RL_PLAN_CERRT_H_
#define _RL_PLAN_CERRT_H_

#include <vector>
#include <map>
#include <memory>
#include <numeric>
#include <cmath>
#include <tuple>

#include <boost/shared_ptr.hpp>
#include <boost/random.hpp>

#include <rl/sg/Scene.h>
#include <rl/sg/PlanarSegmentation.h>

#include <Eigen/LU>

#include "Rrt.h"

namespace rl
{
namespace plan
{
/**
     * Contact Exploiting RRT
     */
class Cerrt : public Rrt {
public:
  Cerrt();
  ::std::string getName() const;

  int nrParticles;
  ::rl::math::Real gamma; // no :: 

  int seed;
  ::rl::math::Real goalEpsilon;

  //scaling for nn computation
  ::rl::math::Real maxDistance;
  ::rl::math::Real maxUncertainty;

  /// Description of the planes and their connectivity in the scene
  ::rl::sg::PlanarSegmentation* segmentation;

  /// If true, the planner uses plane information to guide slides instead of collision
  bool usePlanes;

  /// If true, the planner will only slide with constant orientation. Otherwise it will chenge it's
  /// orientation towards the sample while sliding.
  bool fixedOrientationSlide;

  /// Updates the seed and the random number generator
  void overwriteRandomSeed(int seed);

  void getPath(VectorList& path, int i);
  void writeOutCurrentPath(std::string& path_string, rl::plan::VectorList path);

  bool ignoreExhaustedNodes;
  int maxExhaustion;

  bool useAdd;
  ::rl::math::Real addAlpha;
  ::rl::math::Real addLower;
  ::rl::math::Real addRadius;
protected:
  virtual bool solve();

  /// Adds an edge to the tree and if a viewer is available, visualizes the connections
  /// (using straight edges for slides)
  Edge addEdge(const Vertex& u, const Vertex& v, Tree& tree);

  /**
       * @brief samples random direction in R^n
       * @param rd - vector to fill with random direction - must be of size (n x 1)
       *
       * algorithm is from http://mathoverflow.net/questions/24688/efficiently-sampling-points-uniformly-from-the-surface-of-an-n-sphere
       */
  void sampleDirection(::rl::math::Vector& rd);

  void sampleInitialParticles(::std::vector<Particle>& initialParticles);

  bool isSensor(Particle& particle);
  bool isSelfCollision(Particle& particle);

  bool sampleConnectParticle(const ::rl::math::Vector& init, const ::rl::math::Vector& target, const ::rl::math::Vector& motionNoise, bool guardedMove, Particle& particle);
  bool sampleSlidingParticle(const ::rl::math::Vector& init, const ::rl::math::Vector& mean, const ::rl::math::Transform& chosen_proj, const ::rl::math::Vector& motionNoise, const std::pair<std::string, std::string>& slidingPair, const rl::sg::CollisionQueryResult& slidingInfo, bool guardedMove, Particle& particle);

  bool isEqualCollisionState(::rl::sg::CollisionMap& first, ::rl::sg::CollisionMap& second);

  /// Check if a guarded move should end because the particle has reached a plane edge
  /// Returns 0 if did not reach the polygon edge at all
  /// Returns -1 if reached the edge but too close to a corner
  /// Returns 1 if reached the edge safely
  int reachedPolygonEdge (rl::sg::Plane* plane, const ::Eigen::Vector3d& tdotTrans);

  /// Checks if the particle has reached the neighbor plane with safe margins at the edge
  bool checkSafeCrossing (const rl::sg::Plane& plane, int neighID,
                          const ::rl::math::Vector3& pos, double margin);

  double projectOnSurface(const ::rl::math::Vector3& point, const ::rl::math::Vector3& pointOnSurface, const ::rl::math::Vector3& normal, ::rl::math::Vector3& out);
  bool moveConfigOnSurface(const ::rl::math::Vector& config, const ::rl::math::Vector3& pointOnSurface, const ::rl::math::Vector3& normal, const std::pair<std::string, std::string>& collPair, ::rl::math::Vector& out);
  const rl::sg::CollisionMap&  getAllCollidingShapes();

  //bool getNormal(const Vertex& vertex, ::rl::math::Vector& normal);
  void drawParticles(const ::std::vector<Particle>& particles);
  void drawEigenvectors(Gaussian& gaussian, ::rl::math::Real scale);
  void drawSurfaceNormal(::rl::math::Vector3& startPoint, ::rl::math::Vector3& normal, ::rl::math::Real scale = 1.0);

  Cerrt::Neighbor nearest(const Tree& tree, const ::rl::math::Vector& chosen);

  enum ActionType{ CONNECT, GUARDED, CONNECTSLIDE, GUARDEDSLIDE};

  ActionType selectAction(const Tree& tree, const Vertex& nearest);

  bool simulate(ActionType action,
                const Neighbor& nearest,
                const ::rl::math::Vector& chosen,
                int nrParticles,
                bool consistentContactState,
                ::std::vector<Particle>& particles,
                ::rl::math::Vector3& slidingNormal);

  double workSpaceDistance(::boost::shared_ptr<BeliefState> node, rl::math::Transform goal);
  /**
       * @brief chooses a random configuration to extand towards
       * @param chosen - returned random config
       * Currently implements uniform sampling with goal bias: returns this->goal with probability .1
       */
  void choose(::rl::math::Vector& chosen);

  typedef ::std::vector<Particle> ParticleSet;


  struct PossibleGoal
  {
    VectorPtr q;
    Neighbor neighbor;
  };



private:
  ::boost::shared_ptr<boost::random::mt19937> gen;
};
}
}

#endif // _RL_PLAN_CERRT_H_
