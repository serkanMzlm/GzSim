#include <iomanip>

#include "MavlinkGpsPublisherPlugin.hpp"

using namespace gz;
using namespace sim;
using namespace systems;
using namespace mavlink_gps_plugin;

void MavlinkGpsPublisherPlugin::Configure(
    const gz::sim::Entity &entity,
    const std::shared_ptr<const sdf::Element> &sdf,
    gz::sim::EntityComponentManager &ecm,
    gz::sim::EventManager &eventMgr)
{
    // gzmsg << "[MavlinkGpsPublisherPlugin] Configure started." << std::endl;
    std::cout << "[MavlinkGpsPublisherPlugin] Configure started." << std::endl;

    // this->model_ = Model(entity);
    // this->stand_link_entity_ = this->model_.LinkByName(ecm, "stand_link");

    this->node_.Subscribe("/stand/gps", &MavlinkGpsPublisherPlugin::gpsCallback, this);
    this->node_.Subscribe("/model/stand/pose", &MavlinkGpsPublisherPlugin::poseCallback, this);
}

void MavlinkGpsPublisherPlugin::Update(const UpdateInfo &info, EntityComponentManager &ecm)
{
}

void MavlinkGpsPublisherPlugin::gpsCallback(const gz::msgs::NavSat &_msg)
{
    // std::cout << "[MavlinkGpsPublisherPlugin] GPS Callback: "
    //           << "Latitude: " << _msg.latitude_deg()
    //           << ", Longitude: " << _msg.longitude_deg()
    //           << ", Altitude: " << _msg.altitude() << std::endl;
}
void MavlinkGpsPublisherPlugin::poseCallback(const gz::msgs::Pose_V &_msg)
{
    const auto &pose = _msg.pose(0);
    const auto &pos = pose.position();
    const auto &orientation = pose.orientation();

    gz::math::Quaterniond quat(
        orientation.w(),
        orientation.x(),
        orientation.y(),
        orientation.z());

    gz::math::Vector3d euler = quat.Euler();

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "  Position: (" << pos.x() << ", " << pos.y() << ", " << pos.z() << ")\n";
    std::cout << "  Orientation (quaternion): ("
              << orientation.x() << ", "
              << orientation.y() << ", "
              << orientation.z() << ", "
              << orientation.w() << ")\n";

    double roll_deg = RAD2DEG(euler.X());
    double pitch_deg = RAD2DEG(euler.Y());
    double yaw_deg = RAD2DEG(euler.Z());

    std::cout << "  Orientation (Euler): "
              << "roll=" << roll_deg << ", "
              << "pitch=" << pitch_deg << ", "
              << "yaw=" << yaw_deg << std::endl;
}

GZ_ADD_PLUGIN(
    mavlink_gps_plugin::MavlinkGpsPublisherPlugin,
    gz::sim::System,
    mavlink_gps_plugin::MavlinkGpsPublisherPlugin::ISystemConfigure,
    mavlink_gps_plugin::MavlinkGpsPublisherPlugin::ISystemUpdate)

GZ_ADD_PLUGIN_ALIAS(mavlink_gps_plugin::MavlinkGpsPublisherPlugin, "gz::sim::systems::MavlinkGpsPublisherPlugin")