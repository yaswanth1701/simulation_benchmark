/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <iostream> 
#include <vector>
#include <gz/math/Vector3.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <sdf/Root.hh>

#include "boxes.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Pose.hh"
using namespace gz;
using namespace benchmark;

BoxesTest::Boxes(const std::string &_physicsEngine, double _dt,
                 int _modelCount, bool _collision, bool _complex){

    sdf::root root;
    root.Load(common::joinPaths(PROJECT_SOURCE_PATH,
      "test", "worlds", "shapes.sdf"));

    ASSERT_EQ(1u, root.WorldCount());

    auto systemLoader = std::make_shared<SystemLoader>();
    SimulationRunner runner(root.WorldByIndex(0), systemLoader);

    int worldCount = 0;

    runner.EntityCompMgr().Each<World,components::Name>(
                                [&](const Entity &_entity,
                                 const World *_world,
                                const components::Name *_name)->bool
    {
      EXPECT_NE(nullptr, _world);
      EXPECT_NE(nullptr, _name);

      EXPECT_EQ("default", _name->Data());

      worldCount++;

      worldEntity = _entity;
      return true;
    }

    EXPECT_NE(kNullEntity, worldEntity);
    EXPECT_EQ(worldCount, 1);






    

                 

}

