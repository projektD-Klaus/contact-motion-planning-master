/**
 * @author Can Erdogan
 * @date 2017-05-31
 * @brief Describes the planar segmentation of a scene, plane parameterization and connectivity
 * of the planes.
 */

#ifndef _PLANARSEGMENTATION_H_
#define _PLANARSEGMENTATION_H_

#include <vector>
#include <sstream>

#include <Eigen/Geometry>

#include <rl/math/Vector.h>
#include <rl/xml/Document.h>
#include <rl/xml/DomParser.h>
#include <rl/xml/Path.h>

namespace rl
{
  namespace sg
  {

    /// Definition of a plane, deduced from a perceived polygon in the visual scene
    struct Plane {
      size_t id; //< id of the plane in the segmentation
      ::rl::math::Vector4 params; //< (a,b,c,d) such that ax + by + cz + d = 0 is plane
      std::vector <::rl::math::Vector3> points; //< polygon points that define this plane wrt mean
      ::rl::math::Vector3 mean; //< mean of the polygon points

      /// Neighbors (i.e. those who share an edge). bool is 1 if they're wall contacts, 0 if cliff
      std::vector <std::pair<bool, Plane*>> neighs;
      std::vector <std::pair<::rl::math::Vector3, ::rl::math::Vector3>> neighEdges;

      Plane () {}

      /// Constructor
      Plane (size_t id_, const ::rl::math::Vector4& params_,
        const std::vector <::rl::math::Vector3>&
        points_, const ::rl::math::Vector3& mean_) : id(id_), params(params_), points(points_),
        mean(mean_) {}

      /// Returns the max distance between projection of a given point onto the polygon plane and
      /// the edges of the polygon (with outward normals). That is, if result is positive, then
      /// the point is out of the polygon.
      double maxDistToPoly(const ::rl::math::Vector3& point) const;
    };

    /// Planar segmentation of the perceived world with connections between planes
    struct PlanarSegmentation {

      std::vector <Plane> planes;               //< planes in the world
      std::vector <::rl::math::Vector > connections;  //< the indices of the connected planes
      PlanarSegmentation () {}
      PlanarSegmentation (const std::string& filePath);  //< reads a .xml file

      /// Returns the planes of which the given point is in close proximity.
      /// \details The polygon edges are inflated by some edge offset so that transitions between
      /// polygons can take place (rotations around edges).
      /// Also, a distance offset is used to create a buffer around the plane.
      void getPlanesForSample (const ::rl::math::Vector3& center, double planeOffset,
        double edgeOffset, std::vector <int>& planes) const;

      /// Helper function to read double vectors
      template <class C, int N> ::Eigen::Matrix <C, N, 1> stringToVector
        (const std::string& str);
    };
  }
}

#endif // _PLANARSEGMENTATION_H_

