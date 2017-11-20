#ifndef _RL_PLAN_KMEANS_H_
#define _RL_PLAN_KMEANS_H_

#include <rl/math/Vector>
#include <rl/math/Matrix>

namespace rl 
{
  namespace plan
  {
    void kMeans(const ::rl::math::Matrix& data, const int k, ::std::vector<::rl::math::Matrix>& clusters);
  }
}

#endif // _RL_PLAN_KMEANS_H_