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

#ifndef _RL_SG_SCENE_H_
#define _RL_SG_SCENE_H_

#include <string>
#include <vector>
#include <map>
#include <Inventor/actions/SoCallbackAction.h>
#include <Inventor/VRMLnodes/SoVRMLGroup.h>

#include "Exception.h"

#include <rl/math/Vector.h>

#include "Shape.h"

namespace rl
{
	namespace sg
	{
		class Model;


    struct CollisionQueryResult
    {
      rl::math::Vector3 commonPoint;
      bool isSensor;
      rl::math::Vector3 normal;
      bool selfCollision;
    };

    /**
     * We store the outcome of a collision query in this map.
     * The key is a pair of strings that id the two colliding shapes.
     * The value Contains commonPoint, normal and a flag if the collision id percievable by the robot.
     */
    typedef std::map<std::pair<std::string, std::string> , CollisionQueryResult > CollisionMap;

		
		class Scene
		{
		public:
			typedef ::std::vector< Model* >::iterator Iterator;
			
			Scene();
			
			virtual ~Scene();
			
			virtual void add(Model* model);
			
			Iterator begin();
			
			virtual Model* create() = 0;
			
			Iterator end();
			
			Model* getModel(const ::std::size_t& i) const;
			
			virtual ::std::string getName() const;
			
			::std::size_t getNumModels() const;
			
			void load(const ::std::string& filename, const bool& doBoundingBoxPoints = false, const bool& doPoints = false);
			
			virtual void remove(Model* model);
			
			virtual void setName(const ::std::string& name);

      virtual void lastCollidingShapes(::std::string& first, ::std::string& second, ::rl::math::Vector3& first_vec, ::rl::math::Vector3& second_vec);

      virtual bool getCollisionSurfaceNormal(const ::rl::math::Vector3& from, const ::rl::math::Vector3& target, const std::string& eeId, const std::string& obstId, ::rl::math::Vector3& normalVector)
      {
        assert(0); //Not implemented here!
        return false;
      }

      virtual const CollisionMap& getLastCollisions(){
        return this->lastCollisions;}

        void resetCollisionBuffer(){lastCollisions.clear();}

		protected:
			bool isScalingSupported;
			
			::std::vector< Model* > models;

      CollisionMap lastCollisions;

			// int lastCollidingShape1;
			
		private:
			static void triangleCallback(void* userData, SoCallbackAction* action, const SoPrimitiveVertex* v1, const SoPrimitiveVertex* v2, const SoPrimitiveVertex* v3);

			
			::std::string name;
		};
	}
}

#endif // _RL_SG_SCENE_H_
