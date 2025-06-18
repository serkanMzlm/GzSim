#include "CreateModelPlugin.hpp"

using namespace gz;
using namespace sim;
using namespace systems;
using namespace create_model;

void CreateModelPlugin::Configure(
    const gz::sim::Entity &entity,
    const std::shared_ptr<const sdf::Element> &sdf,
    gz::sim::EntityComponentManager &ecm,
    gz::sim::EventManager &eventMgr)
{
    _port = std::make_unique<Udp>(local_port, ip, remote_port);
    _port->openPort();

    if (!_port->isOpenPort())
    {
        std::cerr << "Failed to open MAVLink sender port" << std::endl;
    }

    init();
    this->_node.Subscribe("/stand/gps", &CreateModelPlugin::gpsCallback, this);
}

void CreateModelPlugin::gpsCallback(const gz::msgs::NavSat &_msg)
{
    cb();
}

void CreateModelPlugin::cb()
{
    try
    {
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        mavlink_message_t msg;
        mavlink_status_t status;

        int bytes_received = _port->readData(buffer, sizeof(buffer));
        if (bytes_received > 0)
        {
            for (int i = 0; i < bytes_received; ++i)
            {
                if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status))
                {
                    handleMavlinkMessage(msg);
                }
            }
        }
        if (is_new_setpoint)
        {
            createModel();
            is_new_setpoint = false;
        }
    }
    catch (const std::system_error &e)
    {
        std::cerr << "[EXCEPTION] readData failed: " << e.what() << std::endl;
    }
}

void CreateModelPlugin::handleMavlinkMessage(const mavlink_message_t &msg)
{
    switch (msg.msgid)
    {
    case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
    {
        mavlink_local_position_ned_t lp;
        mavlink_msg_local_position_ned_decode(&msg, &lp);
        // std::cout << "[RECV] LOCAL_POSITION_NED | X: " << lp.x
        //           << " Y: " << lp.y
        //           << " Z: " << lp.z
        //           << " Vx: " << lp.vx
        //           << " Vy: " << lp.vy
        //           << " Vz: " << lp.vz << std::endl;
        model_pose[0] = lp.x;
        model_pose[1] = lp.y;
        is_new_setpoint = true;
        break;
    }
    default:
        std::cout << "[RECV] Unknown message ID: " << msg.msgid
                  << " from SysID: " << (int)msg.sysid << std::endl;
    }
}

void CreateModelPlugin::init()
{
    findModelPath();
    world_name = "test";
    model_file = target_path.string() + "/model.sdf";

    model_pose = {0.0, 20.0, 0.0};
    model_quaternion[0] = 1.0; // w
    model_quaternion[1] = 0.0; // x
    model_quaternion[2] = 0.0; // y
    model_quaternion[3] = 0.0; // z
}

void CreateModelPlugin::findModelPath()
{
    file_path = std::filesystem::path(__FILE__).parent_path().parent_path();
    file_path = file_path.parent_path().parent_path();
    target_path = file_path / "models" / model_name;
}

void CreateModelPlugin::createModel()
{
    static bool is_create_model = false;
    if (is_create_model)
    {
        changeModelPose();
        return;
    }

    is_create_model = true;
    gz::msgs::EntityFactory req{};
    req.set_sdf_filename(model_file);
    req.set_name(model_name);
    req.set_allow_renaming(false);

    auto pose = req.mutable_pose();
    pose->mutable_position()->set_x(model_pose[0]);
    pose->mutable_position()->set_y(model_pose[1]);
    pose->mutable_position()->set_z(model_pose[2]);
    pose->mutable_orientation()->set_w(model_quaternion[0]);
    pose->mutable_orientation()->set_x(model_quaternion[1]);
    pose->mutable_orientation()->set_y(model_quaternion[2]);
    pose->mutable_orientation()->set_z(model_quaternion[3]);

    gz::msgs::Boolean rep;
    std::string create_service = "/world/" + world_name + "/create";

    bool result;
    bool gz_called = true;

    while (gz_called)
    {
        if (_node.Request(create_service, req, 1000, rep, result))
        {
            if (!rep.data() || !result)
            {
                std::cout << "EntityFactory service call failed\n";
                return;
            }
            else
            {
                gz_called = false;
            }
        }
        else
        {
            std::cout << "Service call timed out as Gazebo has not been detected.\n";
            usleep(2000000);
        }
    }
}

void CreateModelPlugin::changeModelPose()
{
    gz::msgs::Pose req;
    req.set_name(model_name);
    req.mutable_position()->set_x(model_pose[0]);
    req.mutable_position()->set_y(model_pose[1]);
    req.mutable_position()->set_z(model_pose[2]);
    req.mutable_orientation()->set_w(model_quaternion[0]);
    req.mutable_orientation()->set_x(model_quaternion[1]);
    req.mutable_orientation()->set_y(model_quaternion[2]);
    req.mutable_orientation()->set_z(model_quaternion[3]);

    gz::msgs::Boolean rep;
    bool result;
    std::string service = "/world/" + world_name + "/set_pose";

    if (!_node.Request(service, req, 1000, rep, result) || !result || !rep.data())
    {
        std::cerr << "Failed to set model pose\n";
    }
}

void CreateModelPlugin::setModelPose(const std::vector<double> &pose)
{
    if (pose.size() != 3)
    {
        std::cerr << "Invalid pose size. Expected 3 values." << std::endl;
        return;
    }
    model_pose = pose;

    model_quaternion[0] = 1.0; // w
    model_quaternion[1] = 0.0; // x
    model_quaternion[2] = 0.0; // y
    model_quaternion[3] = 0.0; // z
}

GZ_ADD_PLUGIN(
    create_model::CreateModelPlugin,
    gz::sim::System,
    create_model::CreateModelPlugin::ISystemConfigure)

GZ_ADD_PLUGIN_ALIAS(create_model::CreateModelPlugin, "gz::sim::systems::CreateModelPlugin")