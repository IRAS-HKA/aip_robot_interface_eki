#include "robot_interface_eki/Server.h"

Server::Server() : Node("Server")
{
    callback_group_ = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant);


    move_srv_ = this->create_service<robot_interface_eki::srv::Move>("move_srv",
        std::bind(&Server::move_cb, this, std::placeholders::_1, std::placeholders::_2),
        ::rmw_qos_profile_default,
        callback_group_);

    grip_srv_ = this->create_service<robot_interface_eki::srv::Grip>("grip_srv",
        std::bind(&Server::grip_cb, this, std::placeholders::_1, std::placeholders::_2),
        ::rmw_qos_profile_default,
        callback_group_);

    run_srv_ = this->create_service<std_srvs::srv::Trigger>("run_srv",
        std::bind(&Server::run_cb, this, std::placeholders::_1, std::placeholders::_2),
        ::rmw_qos_profile_default,
        callback_group_);

    this->declare_parameter("host");
    this->declare_parameter("port");
    this->declare_parameter("meta_port");

    rclcpp::Parameter host_param = this->get_parameter("host");
    rclcpp::Parameter port_param = this->get_parameter("port");
    rclcpp::Parameter meta_param = this->get_parameter("meta_port");

    //RCLCPP_INFO(get_logger(), "Trying to connect to the host"); 

    RCLCPP_INFO(get_logger(), "Trying to connect to the host: [%s], port: [%d], meta_port: [%d]", host_param.value_to_string().c_str(), port_param.as_int(), meta_param.as_int());
    
    robot.connect_async(host_param.as_string(), port_param.as_int(), meta_param.as_int());

    robot.listener = [this](rbt::RobotEvent event, rbt::Robot *robot) -> void {
        switch (event)
        {
        case rbt::RobotEvent::CONNECT:
            RCLCPP_INFO(get_logger(), "Robot connected"); 
            break;
        case rbt::RobotEvent::RUN:
            RCLCPP_INFO(get_logger(), "Robot running"); 
            break;
        case rbt::RobotEvent::STATE:
            break;
        }
    };


    RCLCPP_INFO(get_logger(), "Waiting for a command..."); 

}

Server::~Server() 
{
    RCLCPP_INFO(get_logger(), "Disconnecting..."); 
    robot.disconnect();
}

void Server::move_cb(robot_interface_eki::srv::Move::Request::SharedPtr req, robot_interface_eki::srv::Move::Response::SharedPtr res)
{
    rbt::MoveCommand command;

    switch(req->type) {
        case robot_interface_eki::srv::Move::Request::JOINTS:
            command = rbt::MoveCommand(rbt::PoseJoints(req->a1,req->a2,req->a3,req->a4,req->a5,req->a6,req->a7));
            RCLCPP_INFO(get_logger(), "Running JOINT Movement"); 
            break;
        case robot_interface_eki::srv::Move::Request::CARTESIAN:
            command = rbt::MoveCommand(rbt::PoseCartesian(req->x,req->y,req->z,req->a,req->b,req->c), req->lin);
            RCLCPP_INFO(get_logger(), "Running CARTESIAN Movement"); 
            break;
        case robot_interface_eki::srv::Move::Request::TARGET:
            command = rbt::MoveCommand(req->target);
            RCLCPP_INFO(get_logger(), "Running TARGET"); 
            break;
        default:
            return;
    }

    command.base_index = req->base_index;
    command.tool_index = req->tool_index;

    robot.perform(command);
    res->message = "moving";
}

void Server::grip_cb(robot_interface_eki::srv::Grip::Request::SharedPtr req, robot_interface_eki::srv::Grip::Response::SharedPtr res)
{
    switch(req->type) {
        case robot_interface_eki::srv::Grip::Request::JAW:
            robot.perform(rbt::GripCommand(req->item_size, req->close));
            RCLCPP_INFO(get_logger(), "Running JAW gripping"); 
            break;
        case robot_interface_eki::srv::Grip::Request::VACUUM:
            robot.perform(rbt::GripCommand(req->suction_active, req->cylinder_position));
            RCLCPP_INFO(get_logger(), "Running VACUUM Movement"); 
            break;
        case robot_interface_eki::srv::Grip::Request::COMBINED:
            robot.perform(rbt::GripCommand(req->item_size, req->close, req->suction_active, req->cylinder_position));
            RCLCPP_INFO(get_logger(), "Running COMBINED Movement"); 
            break;
        default:
            break;
    }
    res->message = "gripping";
}

void Server::run_cb(std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res)
{
    robot.run();
    res->success = true;
    res->message = "run commands";
}



int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Server>();

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
}
