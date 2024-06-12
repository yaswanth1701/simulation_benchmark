#define MCAP_IMPLEMENTATION
#include "mcap/writer.hpp"
#include "protobuf/BuildFileDescriptorSet.h"
#include <boxes_msg.pb.h>
#include <iostream>
#include <gz/math/Vector3.hh>
#include <gz/math/Pose3.hh>

using  namespace gz;

namespace benchmark
{
    class Log
    {
    public: explicit Log(std::string filePath);

    public: void setBenchmarkMsg(std::string mgsName);
    
    public: void endLogging();

    public: void recordPose(const int _modelNo, const std::vector<double> _pos, 
                            const std::vector<double> _quat);

    public: void recordPose(int _modelNo, const math::Pose3d &_pose);

    public: void recordTwist(const int _modelNo, const std::vector<double> &_linVelocity,
                             const std::vector<double> &_angVelocity);

    public: void recordTwist(const int _modelNo, const math::Vectot3d &_linVelocity,
                             const math::Vector3d &_angVelocity );

    private: mcap::McapWriter writer;
    private: mcap::ChannelId channelId;
    private: benchmark::BoxMsg boxMsg;

    }
}