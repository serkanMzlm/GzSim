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

    if (!_port->isOpenPort()) {
        std::cerr << "Failed to open MAVLink sender port" << std::endl;
    }

    init();
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

    model_euler = {0.0, 0.0, 0.0};

    createModel();
}

void CreateModelPlugin::findModelPath()
{
    file_path = std::filesystem::path(__FILE__).parent_path().parent_path();
    file_path = file_path.parent_path().parent_path();
    target_path = file_path / "models" / model_name;
}

void CreateModelPlugin::createModel()
{
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

void CreateModelPlugin::setModelPose(const std::vector<double> &pose)
{
    if (pose.size() != 3) {
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
    create_model::CreateModelPlugin::ISystemConfigure
)

GZ_ADD_PLUGIN_ALIAS(create_model::CreateModelPlugin, "gz::sim::systems::CreateModelPlugin")