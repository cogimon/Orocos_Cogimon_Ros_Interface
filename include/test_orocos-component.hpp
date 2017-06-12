#ifndef OROCOS_TEST_OROCOS_COMPONENT_HPP
#define OROCOS_TEST_OROCOS_COMPONENT_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <rtt/Component.hpp>
#include <std_srvs/Empty.h>
#include <ros/ros.h>
#include <rst-rt/kinematics/JointAngles.hpp>
#include <rst-rt/robot/JointState.hpp>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"

using namespace std;

#define COMAN_ARM_DOF_SIZE 7
class Test_orocos : public RTT::TaskContext{
public:
	Test_orocos(std::string const& name);
	bool configureHook();
	bool startHook();
	void updateHook();
	void stopHook();
	void cleanupHook();
private:
	double prop_counter_step;
	double prop_service_call_counter;



	std_msgs::Float64MultiArray q_left_Arm;
	std_msgs::Float64MultiArray q_right_Arm;

	std_msgs::Float64MultiArray q_left_Arm_current;
	std_msgs::Float64MultiArray q_right_Arm_current;

	std_msgs::Float64MultiArray Dq_left_Arm_current;
	std_msgs::Float64MultiArray Dq_right_Arm_current;

	std_msgs::Float64MultiArray T_left_Arm_current;
	std_msgs::Float64MultiArray T_right_Arm_current;

	std_msgs::Float64MultiArray Object_Pos_Desired;

	std_msgs::Float64MultiArray Object_Pos_current;
	std_msgs::Float64MultiArray Object_Vel_current;

	RTT::InputPort<std_msgs::Float64MultiArray > inport_left;
	RTT::InputPort<std_msgs::Float64MultiArray > inport_right;

	RTT::InputPort<std_msgs::Float64MultiArray > inport_object;

	RTT::OutputPort<std_msgs::Float64MultiArray> outport_q_left;
	RTT::OutputPort<std_msgs::Float64MultiArray> outport_q_right;

	RTT::OutputPort<std_msgs::Float64MultiArray> outport_Dq_left;
	RTT::OutputPort<std_msgs::Float64MultiArray> outport_Dq_right;

	RTT::OutputPort<std_msgs::Float64MultiArray> outport_T_left;
	RTT::OutputPort<std_msgs::Float64MultiArray> outport_T_right;

	RTT::OutputPort<std_msgs::Float64MultiArray> outport_Object_Pos;
	RTT::OutputPort<std_msgs::Float64MultiArray> outport_Object_Vel;
//	RTT::OutputPort<Eigen::VectorXf> outport_Object_Pos;


	// Declare ports and their datatypes
	RTT::OutputPort<rstrt::kinematics::JointAngles> joint_position_left_arm_output_port;
	RTT::OutputPort<rstrt::kinematics::JointAngles> joint_position_right_arm_output_port;

	RTT::OutputPort<Eigen::VectorXf> Object_position_output_port; // It is the desired position!

	RTT::InputPort<rstrt::robot::JointState> joint_position_left_arm_input_port;
	RTT::InputPort<rstrt::robot::JointState> joint_position_right_arm_input_port;

	RTT::InputPort<Eigen::VectorXf> Object_position_input_port; // It is the measured data!
	RTT::InputPort<Eigen::VectorXf> Object_velocity_input_port; // It is the measured data!

	// Actuall joint command to be sent over port:
	rstrt::kinematics::JointAngles joint_position_left_arm_command;
	rstrt::kinematics::JointAngles joint_position_right_arm_command;

	rstrt::robot::JointState joint_position_left_arm;
	rstrt::robot::JointState joint_position_right_arm;

	Eigen::VectorXf Object_position_desired;

	Eigen::VectorXf  Object_position_measured;
	Eigen::VectorXf  Object_velocity_measured;
};
#endif
