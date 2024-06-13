#include "log.hh"

using namespace gz; 
using namespace benchmark;


Log::Log(const std::string &_filePath)
{
  auto options = mcap::McapWriterOptions("");
  const auto s = this->writer.open(file.c_str(), options);
}

/////////////////////////////////////////////////
Log::setBoxMsg(const std::string &_physicsEngine, 
               double &_dt, bool &_complex, bool &_collision,
               int &_modelCount, bool &_logMultiple)
{
  mcap::Schema schema("benchmark_proto.BoxesMsg", "protobuf",
                     foxglove::BuildFileDescriptorSet(benchmark_proto::Boxes_msg::descriptor()).SerializeAsString());

  this->writer.addSchema(schema);
  mcap::Channel channel("model_states", "protobuf", schema.id);
  this->writer.addChannel(channel);
  this->channelId = channel.id;

  this->msg.set_physics_engine(_physicsEngine);
  this->msg.set_dt(_dt);
  this->msg.set_complex(_complex);
  this->msg.set_collision(_collision);
  this->msg.set_model_count(_modelCount);

  if(_logMultiple)
  {
    for(int model_no = 1; model_no<=_modelCount; model_no++)
    {
      this->msg.add_data()->set_model_no(model_no);
    }
  }

  else
    this->msg.add_data()->set_model_no(1);
}

/////////////////////////////////////////////////
void Log::stop()
{
  std::string serialized = this->msg.SerializeAsString();
  mcap::Message mcapMsg;
  mcapMsg.channelId = this->channelId;
  mcapMsg.data = reinterpret_cast<const std::byte*>(serialized.data());
  mcapMsg.dataSize = serialized.size();
  const auto res = writer.write(mcapMsg);
}

/////////////////////////////////////////////////
void log::recordPose(int &_modelIdx, const std::vector<double> _position, 
                            const std::vector<double> _quaternion)
{
  auto pose = this->msg.mutable_data(modelIdx)->add_poses();   

  pose->mutable_position()->set_x(_position[0]);
  pose->mutable_position()->set_y(_position[1]);
  pose->mutable_position()->set_z(_position[3]);

  pose->mutable_orientation()->set_w(_quaternion[0]);
  pose->mutable_orientation()->set_x(_quaternion[1]);
  pose->mutable_orientation()->set_y(_quaternion[2]);
  pose->mutable_orientation()->set_z(_quaternion[3];                           
}

/////////////////////////////////////////////////
void Log::recordPose(int _modelNo, const math::Pose3d &_pose)
 {  
  math::Vector3d p = _pose.Pos();
  math::Quaternion3d r = _pos.Rot(); 

  auto pose = this->msg.mutable_data(model_no)->add_poses();   
   
  pose->mutable_position()->set_x(p.X());
  pose->mutable_position()->set_y(p.Y());
  pose->mutable_position()->set_z(p.Z());

  pose->mutable_orientation()->set_w(r.W());
  pose->mutable_orientation()->set_x(r.X());
  pose->mutable_orientation()->set_y(r.Y());
  pose->mutable_orientation()->set_z(r.Z());
}

/////////////////////////////////////////////////
void Log::recordTwist(int &_modelIdx, const std::vector<double> &_linVelocity,
                             const std::vector<double> &_angVelocity)
{
  auto twist = this->msg.mutable_data(_modelIdx)->add_twists();

  twist->mutable_linear()->set_x(_linVelocity[0]);
  twist->mutable_linear()->set_y(_linVelocity[1]);
  twist->mutable_linear()->set_z(_linVelocity[2]);

  twist->mutable_angular()->set_x(_angVelocity[0]);
  twist->mutable_angular()->set_y(_angVelocity[1]);
  twist->mutable_angular()->set_z(_angVelocity[2]);
}

/////////////////////////////////////////////////
void Log::recordTwist(int &_modelIdx, const math::Vectot3d &_linVelocity,
                    const math::Vector3d &_angVelocity)
{
  auto twist = this->msg.mutable_data(model_no)->add_twists();

  twist->mutable_linear()->set_x(_linVelocity.X());
  twist->mutable_linear()->set_y(_linVelocity.Y());
  twist->mutable_linear()->set_z(_linVelocity.Z());

  twist->mutable_angular()->set_x(_angVelocity.X());
  twist->mutable_angular()->set_y(_angVelocity.Y());
  twist->mutable_angular()->set_z(_angVelocity.Z());           
}

