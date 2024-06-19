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
#include <gz/math/MassMatrix3.hh>
#include "gz/math/Matrix3.hh"
#include <gz/math/Inertial.hh>
#include <sdf/Root.hh>
#include "gz/sim/TestFixture.hh"

#include "benchmark_boxes.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/World.hh"
#include "gz/sim/TestFixture.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Pose.hh"
#include "log.hh"
using namespace gz;
using namespace sim;
using namespace benchmark;

void BoxesTest::Boxes(const std::string &_physicsEngine, double _dt,
                 int _modelCount, bool _collision, bool _complex)
{
      // Entry point to simulation
    sdf::Root root;

    // World creation based on test parameters using boxes.world.erb file.
    std::string sdfRubyPath = common::joinPaths(PROJECT_SOURCE_DIR, "worlds", 
                                                "boxes", "boxes.world.erb");

    std::stringstream worldFileName;
    worldFileName << "boxes" << "_collision" << _collision << "_complex"
                 << _complex << "_dt" << std::setprecision(2) << std::scientific
                 << _dt << "_modelCount" << _modelCount << ".world";
    // Final world sdf path
    std::string sdfPath = common::joinPaths(PROJECT_SOURCE_DIR, "worlds", "boxes", TEST_NAME, worldFileName.str());

    // boxes.world.erb to .world conversion command
    std::stringstream command;
    command << "erb" << " collision=" << _collision << " complex=" << _complex
            << " dt=" << _dt << " modelCount=" << _modelCount << " " << sdfRubyPath 
            << " > " << " " << sdfPath;

    //result directory location
    std::stringstream resultFolderName;
    resultFolderName << PROJECT_SOURCE_DIR <<"/test_results/" << TEST_NAME << "/MCAP" <<"/boxes" << "_collision" << _collision << "_complex"
                 << _complex << "_dt" << std::setprecision(2) << std::scientific
                 << _dt << "_modelCount" << _modelCount << _physicsEngine << ".mcap";
    std::cout << resultFolderName.str() << std::endl;

    Log<benchmark_proto::BoxesMsg> log(resultFolderName.str());
    bool logMultiple = false;

    log.setBoxMsg(_physicsEngine, _dt, _collision, _complex, _modelCount, logMultiple);

    // execute command
    auto commandCheck =  system(command.str().c_str());
    ASSERT_EQ(commandCheck, 0);

    root.Load(sdfPath);
    
    // Link name in model
    std::string link_name = "box_link";

    ASSERT_EQ(1u, root.WorldCount());

    unsigned int configures{0u};
    unsigned int postUpdates{0u};

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

    const double Ixx = 0.80833333;
    const double Iyy = 0.68333333;
    const double Izz = 0.14166667;
    const math::Matrix3d I0(Ixx, 0.0, 0.0,
                            0.0, Iyy, 0.0, 
                            0.0, 0.0, Izz);
    
    // Server config to set physic engine (DART, Bullet, BulletFeatherstone)
    ServerConfig serverConfig;
    serverConfig.SetSdfRoot(root); 
    serverConfig.SetPhysicsEngine(_physicsEngine);

    TestFixture testFixture(serverConfig);
    ASSERT_NE(nullptr, testFixture.Server());
 
    double simTime;
    
    std::vector<Link> links;
    links.reserve(_modelCount);

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
     EXPECT_EQ(_modelCount, world.ModelCount(_ecm));

     _ecm.Each<components::Model, components::ParentEntity>(
           [&](const Entity &_entity,
               const components::Model *_model,
               const components::ParentEntity *_parent)->bool
     { 
       EXPECT_NE(kNullEntity, _entity);
       EXPECT_NE(nullptr, _model);
       EXPECT_NE(nullptr, _parent);
      //  EXPECT_NE(worldEntity, _parent->data());
       Model model(_entity);
       EXPECT_EQ(linkCount, model.LinkCount(_ecm));
 
       Entity linkEntity = model.LinkByName(_ecm, link_name); 
       EXPECT_NE(kNullEntity, linkEntity);
       if(logMultiple || addLink)
       {
       links.push_back(Link(linkEntity));
       addLink = false;
       }
       return true;
     });
     for(const auto link: links)
      {
       link.EnableVelocityChecks(_ecm);
        // // world linear velocity of link check
        // ASSERT_EQ(v0, link.WorldLinearVelocity(_ecm).value());
        // // world angular velocity of link check
        // ASSERT_EQ(w0, link.WorldAngularVelocity(_ecm).value());
        // // inertia of link in body frame
        auto worldInertial = link.WorldInertial(_ecm);
 
        math::Matrix3d Moi;
        if(worldInertial.has_value())
         Moi = worldInertial.value().MassMatrix().Moi();
 
        ASSERT_EQ(I0, Moi);
      }
 
     configures++;
    }).
    OnPostUpdate([&](const UpdateInfo &_info,
        const EntityComponentManager &_ecm)
    {
      postUpdates++;
      EXPECT_EQ(postUpdates, _info.iterations);

      simTime = std::chrono::duration_cast<std::chrono::duration<double>>(
                              _info.simTime).count();
      log.recordSimTime(simTime);
      log.recordComputationTime(simTime);

      for(int i = 0; i < links.size(); i++)
      {
       auto link = links[i];
       auto pose = link.WorldInertialPose(_ecm); 
       auto linearVelocity = link.WorldLinearVelocity(_ecm);
       auto angularVelocity = link.WorldAngularVelocity(_ecm);

       ASSERT_TRUE(pose.has_value());
       ASSERT_TRUE(linearVelocity.has_value());
       ASSERT_TRUE(angularVelocity.has_value());

       log.recordPose(i, pose.value());
       log.recordTwist(i, linearVelocity.value(), angularVelocity.value());
      } 
    }).
    Finalize();

  int simDuration = 10;
  unsigned int  steps = ceil(simDuration/_dt);
  // simulation loop
  testFixture.Server()->Run(true, steps, false);
  log.stop();

  EXPECT_EQ(1, configures);
  EXPECT_EQ(steps, postUpdates);
  ASSERT_NEAR(simDuration, simTime, 1.1*_dt);
  
}

/////////////////////////////////////////////////

TEST_P(BoxesTest, Boxes){
  std::string physicsEngine = std::get<0>(GetParam());
  double dt = std::get<1>(GetParam());
  int modelCount = std::get<2>(GetParam());
  bool collision = std::get<3>(GetParam());
  bool isComplex = std::get<4>(GetParam());
  gzdbg << physicsEngine << ", dt: " << dt << ", modelCount: " << modelCount
        << ", collision: " << collision << ", isComplex: " << isComplex
        << std::endl;
  Boxes(physicsEngine, dt, modelCount, collision, isComplex);
}
