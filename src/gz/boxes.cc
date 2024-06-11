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
using namespace sim;
using namespace benchmark;

BoxesTest::Boxes(const std::string &_physicsEngine, double _dt,
                 int _modelCount, bool _collision, bool _complex){

    sdf::root root;
    std::string rubySdfPath = common::joinPaths(PROJECT_SOURCE_PATH, "worlds", "boxes", "boxes.worl.erb");
    sdf::string sdfPath = common::joinPaths(PROJECT_SOURCE_PATH, "worlds", "boxes", "boxes.worl")

    root.Load(common::joinPaths(PROJECT_SOURCE_PATH, "worlds", "shapes.sdf"));
    
    std::string link_name= "link";

    ASSERT_EQ(1u, root.WorldCount());

    ServerConfig serverConfig;
    serverConfig.SetPhysicsEngine(_physicsEngine);
    auto systemLoader = std::make_shared<SystemLoader>();
    SimulationRunner runner(root.WorldByIndex(0), systemLoader, serverConfig);

    EXPECT_EQ(runner.serverConfig.PhysicsEngine(), _physicsEngine)
    EXPECT_TRUE(runner.Paused());

    runner.SetStepSize(_dt);

    int worldCount = 0;
    bool logOnce = true;

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
    EXPECT_EQ(_modelCount, WorldEntity->ModelCount);

    
    std::vector<Link> linkEntites;
    modelEntites.reserve(_modelCount);
    uint64_t linkCount = 1; // link per model
    auto ecm = runner.EntityCompMgr();

    ecm.Each<Model, ParentEntity>(
           [&](const Entity &_entity,
               const Model *_model,
               const ParentEntity *_parent,)->bool
    { 
      EXPECT_NE(kNullEntity, _entity);
      EXPECT_NE(nullptr, _model);
      EXPECT_NE(nullprt, _parent);
      EXPECT_NE(worldEntity, _parent->data();
      EXPECT_NE(linkCount, model->LinkCount());

      Entity linkEntity = model->(ecm, link_name); 
      EXPECT_NE(kNullEntity)
      linkEntites.push_back(Link(linkEntity));
    }

    // link properties check
    for(auto link: linkEntites)
    {
      // world linear velocity of link check
      ASSERT_EQ(v0, link.WorldLinearVelocity(ecm));
      // world angular velocity of link check
      ASSERT_EQ(w0, link.WorldAngularVelcity(ecm));
      // inertia of link in body frame
      auto worldInertial = link.WorldInertial();
      ASSERT(Moi, worldInertial->MassMatrix.Moi);
    }
    // resume simulation
    runner.SetPaused(false);
    t0 = runner.CurrentInfo().simtime;
    simDuration = 10;
    steps = ceil(simDuration/_dt);
    // simulation loop
    for(int i = 0; i<steps, i++){
      
      runner.step(runner.CurrentInfo);
      double t = std::chrono::duration<double>(
                                runner.CurrentInfo.simtime - t0).count();
  
      for(auto link = linkEntites.begin(); link < linkEntites.end(); link++)
      {
       math::Pose3d pose = link.WorldInertialPose(); 
       math::Vector3d = link.WorldLinearVelocity();
       math::Vector3d = link.WorldAngularVelocity();
  
       if(logOnce)
        break;
      } 
  }
  double simTime = std::chrono::duration<double>(runner.CurrentInfo.simtime - t0).count();
  ASSERT_NEAR(simDuration, simTime, 1.1*_dt);
  runner.SetPaused(true);
  
                 

}

