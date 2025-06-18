#ifndef CREATE_MODEL_HPP
#define CREATE_MODEL_HPP

#include <unistd.h>
#include <iostream>
#include <filesystem>

#include <gz/transport/Node.hh>

#include <gz/msgs/navsat.pb.h>
#include <gz/msgs/pose_v.pb.h>

#include <gz/plugin/Register.hh>

#include <gz/sim/Model.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/components/NavSat.hh>

#include <gz/math.hh>
#include <gz/msgs.hh>
#include <gz/transport.hh>

#include "udp.hpp"
#include <mavlink/common/mavlink.h>

namespace create_model
{
    class CreateModelPlugin : public gz::sim::System,
                              public gz::sim::ISystemConfigure
    {
    public:
        void Configure(
            const gz::sim::Entity &entity,
            const std::shared_ptr<const sdf::Element> &sdf,
            gz::sim::EntityComponentManager &ecm,
            gz::sim::EventManager &eventMgr) override;

        void init();
        void findModelPath();
        void createModel();
        void setModelPose(const std::vector<double> &pose);

    private:
        std::unique_ptr<Udp> _port;

        int local_port{5010};
        int remote_port{5011};
        uint8_t system_id{5};
        uint8_t component_id{1};
        std::string ip = "127.0.0.1";

        gz::transport::Node _node;
        std::string world_name;
        std::string model_name = "target_stop";
        std::string model_file;

        std::vector<double> model_pose;
        double model_quaternion[4];
        std::vector<double> model_euler;

        std::filesystem::path file_path;
        std::filesystem::path target_path;
    };
}
#endif