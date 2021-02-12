/** *******************************************************
 * AIP - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "Server interface"
 * Purpose : Provides ROS2-Service multiple service
 *           in a single node to use an easy
 *           client with python to send commands to KUKA 
 * 
 * @author Javier Moviglia
 * @since 1.0.0 (2021.01.15)
 *********************************************************/
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <robot_interface_eki/srv/grip.hpp>
#include <robot_interface_eki/srv/move.hpp>

#include <aip/rbt/Robot.h>

class Server : public rclcpp::Node
{
public:
    Server();
    ~Server();

    void move_cb(robot_interface_eki::srv::Move::Request::SharedPtr req, robot_interface_eki::srv::Move::Response::SharedPtr res);
    void grip_cb(robot_interface_eki::srv::Grip::Request::SharedPtr req, robot_interface_eki::srv::Grip::Response::SharedPtr res);
    void run_cb(std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res);

    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_;
    rclcpp::Service<robot_interface_eki::srv::Move>::SharedPtr move_srv_;
    rclcpp::Service<robot_interface_eki::srv::Grip>::SharedPtr grip_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr run_srv_;

    rbt::Robot robot;

};
