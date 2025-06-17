#ifndef MAVLINK_GPS_PUBLISHER_PLUGIN_HPP
#define MAVLINK_GPS_PUBLISHER_PLUGIN_HPP

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

#include "udp.hpp"
#include <mavlink/common/mavlink.h> 

#define RAD2DEG(x) ((x) * 180.0 / M_PI)

namespace mavlink_gps_plugin
{
    class MavlinkGpsPublisherPlugin : public gz::sim::System,
                                      public gz::sim::ISystemConfigure
    {
    public:
        int local_port {5000};
        int remote_port {5001};
        uint8_t system_id {5};
        uint8_t component_id {1};
        std::string ip = "127.0.0.1";
    public:
        void Configure(
            const gz::sim::Entity &entity,
            const std::shared_ptr<const sdf::Element> &sdf,
            gz::sim::EntityComponentManager &ecm,
            gz::sim::EventManager &eventMgr) override;

        void gpsCallback(const gz::msgs::NavSat &_msg);
        void poseCallback(const gz::msgs::Pose_V &_msg);
        bool sendMessage(const mavlink_message_t& msg);

    private:
        std::unique_ptr<Udp> _port;
        gz::transport::Node _node;

        mavlink_gps2_raw_t _gps2_raw;
        uint16_t _yaw;
    };
}

#endif