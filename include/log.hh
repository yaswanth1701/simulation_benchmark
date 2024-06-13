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
    template <class T>
    class Log
    {
    public: explicit Log(std::string filePath);

    public: void setBenchmarkMsg(std::string msgName, T &msg)
    {
        this->benchmarkMsg = msg;

        if (msgName == "boxes benchmark")
        {
          mcap::Schema schema("benchmark_proto.Boxes_msg", "protobuf", foxglove::BuildFileDescriptorSet
                                        (benchmark_proto::Boxes_msg::descriptor()).SerializeAsString());
        }

        this->writer.addSchema(schema);
        mcap::Channel channel("boxes_states", "protobuf", schema.id);
        this->writer.addChannel(channel);
        this->channelId = channel.id;
    }

    public: void endLogging();

    public:  void recordPose(const int _modelNo, const std::vector<double> _position, 
                            const std::vector<double> _quaternion);

    public: void recordPose(int _modelNo, const math::Pose3d &_pose);

    public: void recordTwist(const int _modelNo, const std::vector<double> &_linVelocity,
                             const std::vector<double> &_angVelocity);

    public: void recordTwist(const int _modelNo, const math::Vectot3d &_linVelocity,
                             const math::Vector3d &_angVelocity );

    private: mcap::McapWriter writer;
    private: mcap::ChannelId channelId;
    private: T benchmarkMsg;

    }
}