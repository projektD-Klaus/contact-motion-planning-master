#include "Kmeans.h"

namespace rl 
{
  namespace plan
  {
    void kMeans(const ::rl::math::Matrix& data, const int k, ::std::vector<::rl::math::Matrix>& clusters)
    {
      // just to be sure
      assert(data.rows() >= k && clusters.size() == k);

      // store cluster means in here
      ::std::vector<::rl::math::Vector> means(k);
      // init means with data points
      for (int i = 0; i < k; ++i)
      {
        means[i] = data.row(i);
      }

      // assign data to means
      for (int dataIdx = 0; dataIdx < data.rows(); ++dataIdx)
      {
        for (int i = 0; i < k; ++i)
        {

        }
      }
    }
  }
}
