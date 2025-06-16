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

#define RAD2DEG(x) ((x) * 180.0 / M_PI)

namespace mavlink_gps_plugin
{
    class MavlinkGpsPublisherPlugin : public gz::sim::System,
                                      public gz::sim::ISystemConfigure,
                                      public gz::sim::ISystemUpdate
    {
    public:
        void Configure(
            const gz::sim::Entity &entity,
            const std::shared_ptr<const sdf::Element> &sdf,
            gz::sim::EntityComponentManager &ecm,
            gz::sim::EventManager &eventMgr) override;

        void Update(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override;
        void gpsCallback(const gz::msgs::NavSat &_msg);
        void poseCallback(const gz::msgs::Pose_V &_msg);

    private:
        gz::transport::Node node_;
        // gz::sim::Model model_;
        // gz::sim::Entity stand_link_entity_ {gz::sim::kNullEntity};
    };
}

#endif