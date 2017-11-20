/**
 * @author Can Erdogan
 * @date 2017-05-31
 * @brief Describes the planar segmentation of a scene, plane parameterization and connectivity
 * of the planes.
 */

#include <stdexcept>
#include <iostream>

#include "PlanarSegmentation.h"

#define pv(x) std::cout << #x << ": " << (x).transpose() << std::endl;
#define pc(x) std::cout << #x << ": " << (x) << std::endl;

namespace rl
{
  namespace sg
  {
    PlanarSegmentation::PlanarSegmentation(const std::string& filePath) 
    {
      // Get the path to the xml file
      rl::xml::DomParser parser;
      rl::xml::Document doc = parser.readFile(filePath.c_str(), "", 
        XML_PARSE_NOENT | XML_PARSE_XINCLUDE);
      doc.substitute(XML_PARSE_NOENT | XML_PARSE_XINCLUDE);
      rl::xml::Path path(doc);

      // Read the planes
      int planeId = 0;
      char buf [256];
      while(true) {
        
        // Check if plane exists
        sprintf(buf, "count(//plane%d) > 0", planeId);
        if(!path.eval(buf).getBoolval()) break;

        // Get the params (and renormalize normal in case of printing errors)
        sprintf(buf, "//plane%d/params", planeId);
        rl::xml::Object obj = path.eval(buf);
        xmlNodePtr node = obj()->nodesetval->nodeTab[0];
        sprintf(buf, "%s", node->children[0].content);
        ::rl::math::Vector4 params = stringToVector<double, 4>(buf);
        params.block<3,1>(0,0) = params.block<3,1>(0,0).normalized();
        // pv(params);
        
        // Get the polygon points
        int vertId = 0;
        std::vector <::rl::math::Vector3> points;
        while(true) {

          // Check if vertex exists
          sprintf(buf, "count(//plane%d/poly_points_local/v%d) > 0", planeId, vertId);
          if(!path.eval(buf).getBoolval()) break;

          // Get the point position (wrt mean)
          sprintf(buf, "//plane%d/poly_points_local/v%d", planeId, vertId);
          obj = path.eval(buf);
          node = obj()->nodesetval->nodeTab[0];
          sprintf(buf, "%s", node->children[0].content);
          ::rl::math::Vector3 point = stringToVector<double, 3>(buf);
          points.push_back(point);
          vertId++;
        }

        // Get the mean point of the polygon
        sprintf(buf, "//plane%d/mean_point_world", planeId);
        obj = path.eval(buf);
        node = obj()->nodesetval->nodeTab[0];
        sprintf(buf, "%s", node->children[0].content);
        ::rl::math::Vector3 mean = stringToVector<double, 3>(buf);

        // Check that plane parameters and polygon locations match
        ::rl::math::Vector3 p1 = points[0] + mean;
        double dist = params.block<3,1>(0,0).dot(p1) + params(3);
        if(fabs(dist) > 1e-5) {
          printf("bad distance (%lf) from first vertex to the plane %d\n", dist, planeId);
          throw std::runtime_error(std::string("Plane parameters are not correct!"));
        }

        // Create the plane structure
        Plane plane (planeId, params, points, mean);
        planes.push_back(plane);
        planeId++;
      }
      
      // Read the connectivity of the planes
      int pairId = 0;
      while(true) {
        
        // Check if plane exists
        sprintf(buf, "count(//pair%d) > 0", pairId);
        if(!path.eval(buf).getBoolval()) break;

        // Get the point position (wrt mean)
        sprintf(buf, "//pair%d", pairId);
        rl::xml::Object obj = path.eval(buf);
        xmlNodePtr node = obj()->nodesetval->nodeTab[0];
        sprintf(buf, "%s", node->children[0].content);
        ::rl::math::Vector con = stringToVector<double, 4>(buf);
        // pv(con);
        
        // Add the pair to the high-level connectivity set
        connections.push_back(con);
        int id1 = (int) con(0), id2 = (int) (con(1));

        // Fill in the connecting edges
        int vid1 = (int) floor(con(2));
        ::rl::math::Vector3 edgeV1 = planes[id1].points[vid1] + planes[id1].mean;
        ::rl::math::Vector3 edgeV2 = planes[id1].points[(vid1+1)%(planes[id1].points.size())] 
          + planes[id1].mean;
        ::rl::math::Vector3 v1 = edgeV1 + (con(2) - vid1) * (edgeV2 - edgeV1);
        ::rl::math::Vector3 v2 = edgeV1 + (con(3) - vid1) * (edgeV2 - edgeV1);

        // Compute the angle between the planes to determine if they are wall or cliff contacts
        ::rl::math::Vector3 n1 = planes[id1].params.block<3,1>(0,0);
        ::rl::math::Vector3 n2 = planes[id2].params.block<3,1>(0,0);
        ::rl::math::Vector3 d21 = (v2 - v1).normalized();
        ::rl::math::Vector3 vy = (d21.cross(n1)).normalized();
        ::rl::math::Vector2 n1p (1, 0);
        ::rl::math::Vector2 n2p (n2.dot(n1), n2.dot(vy));
        n2p = n2p.normalized();
        double dot = n1p.dot(n2p);
        double det = n1p(0) * n2p(1) - n1p(0) * n1p(1);
        double angle = atan2(det, dot);
        bool wall = (angle < 0);

        // Update the neighbors of each plane
        planes[id1].neighs.push_back(std::make_pair(wall, &(planes[id2])));
        planes[id2].neighs.push_back(std::make_pair(wall, &(planes[id1])));

        // Update the edge information
        planes[id1].neighEdges.push_back(std::make_pair(v1,v2));
        planes[id2].neighEdges.push_back(std::make_pair(v1,v2));

        pairId++;
      }
    }

    void
    PlanarSegmentation::getPlanesForSample (const ::rl::math::Vector3& pos, double planeOffset, 
        double edgeOffset, std::vector <int>& planeIDs) const
    {
      static const bool dbg = 0;

      // For each polygon, check the distance to the plane and the edges
      if(dbg) printf("\ngetPlanesForSample ---------------------------\n");
      for(int i = 0; i < this->planes.size(); i++) {

        // Get the plane parameters
        const rl::sg::Plane& plane = this->planes[i];
        ::rl::math::Vector3 planeNormal = plane.params.block<3,1>(0,0);
        double dOff = plane.params(3);

        // Check distance to plane
        double planeDist = planeNormal.dot(pos) + dOff;
        if(dbg) pc(planeDist);
        if(fabs(planeDist) > planeOffset) continue;
        //if((planeDist > planeOffset) || (planeDist < )) continue;

        // Check distance to edges
        double maxEdgeDist = plane.maxDistToPoly(pos);
        if(dbg) pc(maxEdgeDist);
        if(maxEdgeDist > edgeOffset) continue;

        // Add the plane to the list
        // pv(planeNormal);
        if(dbg) printf("accepted with plane dist %lf and edge dist %lf\n", planeDist,maxEdgeDist);
        planeIDs.push_back(i);
      }
    }

    double
    Plane::maxDistToPoly(const ::rl::math::Vector3& point) const
    {
      // First project the point
      ::rl::math::Vector3 planeNormal = params.block<3,1>(0,0);
      double dist = point.dot(planeNormal) + params(3);
      ::rl::math::Vector3 pp = point - dist * planeNormal;

      // For each edge, compute the outward normal
      double maxDist = -100;
      size_t numPoints = points.size();
      for(int i = 0; i < numPoints; i++) {

        // Get the vector from one vertex to another
        ::rl::math::Vector3 q1 = points[i] + mean;
        ::rl::math::Vector3 q2 = points[(i+1) % numPoints] + mean;
        ::rl::math::Vector3 d21 = (q2 - q1).normalized();

        // Get edge normal vector using the plane normal
        ::rl::math::Vector3 edgeNormal = (d21.cross(planeNormal)).normalized();

        // Get the dist to the edge
        double dist = (pp - q1).dot(edgeNormal);
        maxDist = std::max(dist, maxDist);
      }
      
      return maxDist;
    }

    template <class C, int N> ::Eigen::Matrix <C, N, 1> 
    PlanarSegmentation::stringToVector (const std::string& str) {
      ::Eigen::Matrix <C, N, 1> val;
      std::istringstream iss(str);
      for(int i = 0; i < N; i++) iss >> val(i);
      return val;
    }
  }
}
