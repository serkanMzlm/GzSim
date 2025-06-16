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
    _port = std::make_unique<Udp>(local_port, ip, remote_port);
    _port->openPort();

    if (!_port->isOpenPort()) {
        std::cerr << "Failed to open MAVLink sender port" << std::endl;
    }

    gzmsg << "[MavlinkGpsPublisherPlugin] Configure started." << std::endl;
    this->_node.Subscribe("/stand/gps", &MavlinkGpsPublisherPlugin::gpsCallback, this);
    this->_node.Subscribe("/model/stand/pose", &MavlinkGpsPublisherPlugin::poseCallback, this);
}

void MavlinkGpsPublisherPlugin::gpsCallback(const gz::msgs::NavSat &_msg)
{
    _gps2_raw.lat = static_cast<int32_t>(_msg.latitude_deg() * 1e7);
    _gps2_raw.lon = static_cast<int32_t>(_msg.longitude_deg() * 1e7);
    _gps2_raw.alt = static_cast<int32_t>(_msg.altitude() * 1e3); 

    mavlink_message_t msg;
    mavlink_msg_gps2_raw_encode(system_id, component_id, &msg, &_gps2_raw); 
    sendMessage(msg);
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
    _gps2_raw.yaw = static_cast<uint16_t>(RAD2DEG(euler.Z()) + 180.0); 

}

bool MavlinkGpsPublisherPlugin::sendMessage(const mavlink_message_t& msg)
{
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);

    try {
        if (_port->isOpenPort())
        {
            _port->writeData(buffer, len);
            return true;
        }

        return false;
    } catch (const std::exception& e) {
        std::cerr << "Failed to send MAVLink message: " << e.what() << std::endl;
        return false;
    }
}

GZ_ADD_PLUGIN(
    mavlink_gps_plugin::MavlinkGpsPublisherPlugin,
    gz::sim::System,
    mavlink_gps_plugin::MavlinkGpsPublisherPlugin::ISystemConfigure
)

GZ_ADD_PLUGIN_ALIAS(mavlink_gps_plugin::MavlinkGpsPublisherPlugin, "gz::sim::systems::MavlinkGpsPublisherPlugin")