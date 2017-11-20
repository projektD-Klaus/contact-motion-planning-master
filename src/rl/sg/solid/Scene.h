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

#ifndef _RL_SG_SOLID_SCENE_H_
#define _RL_SG_SOLID_SCENE_H_

#include <SOLID/SOLID.h>
#include <SOLID/SOLID_broad.h>

#include <string>
#include <map>

#include "../DepthScene.h"
#include "../DistanceScene.h"
#include "../RaycastScene.h"
#include "../SimpleScene.h"

namespace rl
{
	namespace sg
	{
		namespace solid
		{
			class Shape;
			
			class Scene : public ::rl::sg::DepthScene, public ::rl::sg::DistanceScene, public ::rl::sg::RaycastScene, public ::rl::sg::SimpleScene
			{
			public:
        //EIGEN_MAKE_ALIGNED_OPERATOR_NEW

				Scene();
				
				virtual ~Scene();
				
				bool areColliding(::rl::sg::Shape* first, ::rl::sg::Shape* second);

        //void lastCollidingShapes(::std::string& first, ::std::string& second, ::rl::math::Vector3& point);
				
				::rl::sg::Model* create();
				
				bool depth(::rl::sg::Shape* first, ::rl::sg::Shape* second, ::rl::math::Vector3& point1, ::rl::math::Vector3& point2);
				
				using ::rl::sg::DistanceScene::distance;
				
				::rl::math::Real distance(::rl::sg::Shape* first, ::rl::sg::Shape* second, ::rl::math::Vector3& point1, ::rl::math::Vector3& point2);
				
				::rl::math::Real distance(::rl::sg::Shape* shape, const ::rl::math::Vector3& point, ::rl::math::Vector3& point1, ::rl::math::Vector3& point2);
				
				::rl::sg::Shape* raycast(const ::rl::math::Vector3& source, const ::rl::math::Vector3& target, ::rl::math::Real& distance);
				
				bool raycast(::rl::sg::Shape* shape, const ::rl::math::Vector3& source, const ::rl::math::Vector3& target, ::rl::math::Real& distance);
				
				void setMargin(const ::rl::math::Real& margin);

        bool getCollisionSurfaceNormal(const ::rl::math::Vector3& from, const ::rl::math::Vector3& target, const std::string& eeId, const std::string& obstId, ::rl::math::Vector3& normalVector);

				BP_SceneHandle broad;
				
				DT_SceneHandle scene;
				
			protected:
				
			private:
				static void beginOverlap(void* clientData, void* object1, void* object2);
				
				static void endOverlap(void* clientData, void* object1, void* object2);

        ::rl::math::Vector3 lastCollisionPoint;


         std::map<std::string, ::rl::sg::solid::Shape*> shapeMap;
			};

		}
	}
}

#endif // _RL_SG_SOLID_SCENE_H_