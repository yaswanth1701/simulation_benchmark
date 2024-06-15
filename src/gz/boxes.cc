/*
 * Copyright (C) 2024 Open Source Robotics Foundation
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
#include <iomanip>
#include <sstream>
#include <vector>

#include <gz/math/Vector3.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <sdf/Root.hh>
#include "gz/sim/TestFixture.hh"

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
                 int _modelCount, bool _collision, bool _complex)
{
      // Entry point to simulation
    sdf::root root;

    // World creation based on test parameters using boxes.world.erb file.
    std::string sdfRubyPath = common::joinPaths(PROJECT_SOURCE_PATH, "worlds", 
                                                "boxes", "boxes.worl.erb");

    std::stringstream worldFileName;
    worldFieName << "boxes" << "_collision" << _collision << "_complex"
                 << _complex << "_dt" << std::setprecision(2) << std::scientific
                 << _dt << "_modelCount" << _modelCount << ".world";
    // Final world sdf path
    std::string sdfPath = common::joinPaths(PROJECT_SOURCE_PATH, "worlds", "boxes",
                                            RESULT_FOLDER, worldFileName.str());

    // boxes.world.erb to .world conversion command
    std::stringstream command;
    command << "erb" << " collision=" << _collision << " complex=" << _complex
            << " dt=" << _dt << " modelCount=" << _modelCount << " " << sdfRubyPath 
            << " > " << " " << sdfPath;

    std::cout << sdfRubyPath << " " << sdfPath << " " << command.str() << std::endl;

    // execute command
    auto commandCheck =  system(comamnd.str());
    ASSERT_EQ(command, 0);

    root.Load(sdfPath);
    
    // Link name in model
    std::string link_name= "box_link";

    ASSERT_EQ(1u, root.WorldCount());
    
    // Server config to set physic engine (DART, Bullet, BulletFeatherstone)
    ServerConfig serverConfig;
    serverConfig.SetSdfRoot(sdfPath); 
    serverConfig.SetPhysicsEngine(_physicsEngine);

    // initial linear velocity in global frame
    math::Vector3d v0;

    // initial angular velocity in global frame
    math::Vector3d w0;
    
    if (!_complex) 
    {
      v0.Set(-0.9, 0.4, 0.1);
      // Use angular velocity with one non-zero component
      // to ensure linear angular trajectory
      w0.Set(0.5, 0, 0);
    } 
    else 
    {
      v0.Set(-2.0, 2.0, 8.0);
      // Since Ixx > Iyy > Izz,
      // angular velocity with large y component
      // will cause gyroscopic tumbling
      w0.Set(0.1, 5.0, 0.1);
    }

    TestFixture testFixture(serverConfig);
    ASSERT_NE(nullptr, testFixture.Server());
 
    double simTime;
    bool logMultiple = false;
    std::vector<Link> linkEntites;
    modelEntites.reserve(_modelCount);

    // link per model
    uint64_t linkCount = 1; 
    bool addLink = true;

    testFixture.
    OnConfigure([&](const Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        EntityComponentManager &_ecm,
        EventManager &_eventMgr)
    {

     Entity worldEntity = _ecm.EntityByComponents(components::Name("default"));
     EXPECT_EQ(worldEntity, _entity);
     World world(worldEntity);
     EXPECT_EQ(_modelCount, world.ModelCount());

     ecm.Each<Model, ParentEntity>(
           [&](const Entity &_entity,
               const Model *_model,
               const ParentEntity *_parent,)->bool
     { 
       EXPECT_NE(kNullEntity, _entity);
       EXPECT_NE(nullptr, _model);
       EXPECT_NE(nullptr, _parent);
       EXPECT_NE(worldEntity, _parent->data();
       EXPECT_NE(linkCount, model->LinkCount());
 
       Entity linkEntity = model->(ecm, link_name); 
       EXPECT_NE(kNullEntity, linkEntity);
       if(logMultiple || addLink)
       {
       links.push_back(Link(linkEntity));
       addLink = false;
       }
       return true;
     }
        // link properties check
     for(auto link: links)
     {
       // world linear velocity of link check
       ASSERT_EQ(v0, link.WorldLinearVelocity(ecm));
       // world angular velocity of link check
       ASSERT_EQ(w0, link.WorldAngularVelcity(ecm));
       // inertia of link in body frame
       auto worldInertial = link.WorldInertial();
       ASSERT_EQ(Moi, worldInertial->MassMatrix.Moi);
     }
 
     configures++;
    }).
    OnPostUpdate([&](const UpdateInfo &_info,
        const EntityComponentManager &_ecm)
    {
      postUpdates++;
      EXPECT_EQ(postUpdates, _info.iterations);

      simTime = std::chrono::duration_cast<std::chrono::duration<double>>(
                              _info.simTime - t0).count();

      for(int i = 0; i < links.size(); i++)
      {
       auto link = links[i];
       math::Pose3d pose = link.WorldInertialPose(); 
       math::Vector3d linearVelocity = link.WorldLinearVelocity();
       math::Vector3d angularVelocity = link.WorldAngularVelocity();
      } 
    }).
    Finalize();

  int simDuration = 10;
  unsigned int  steps = ceil(simDuration/_dt);
  // simulation loop
  testFixture.Server()->Run(true, steps, false);

  EXPECT_EQ(steps, postUpdates);
  ASSERT_NEAR(simDuration, simTime, 1.1*_dt);
  
}

/////////////////////////////////////////////////
TEST_P(BoxesTest, Boxes) {
  std::string physicsEngine = std::tr1::get<0>(GetParam());
  double dt = std::tr1::get<1>(GetParam());
  int modelCount = std::tr1::get<2>(GetParam());
  bool collision = std::tr1::get<3>(GetParam());
  bool isComplex = std::tr1::get<4>(GetParam());
  gzdbg << physicsEngine << ", dt: " << dt << ", modelCount: " << modelCount
        << ", collision: " << collision << ", isComplex: " << isComplex
        << std::endl;
  Boxes(physicsEngine, dt, modelCount, collision, isComplex);
}
