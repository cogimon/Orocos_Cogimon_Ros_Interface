#include "test_orocos-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

Test_orocos::Test_orocos(std::string const& name) : TaskContext(name), inport_left_pos("Left_arm_in_pos"), inport_left_vel("Left_arm_in_vel"), inport_left_acc("Left_arm_in_acc")
, inport_right_pos("Right_arm_in_pos") , inport_right_vel("Right_arm_in_vel") , inport_right_acc("Right_arm_in_acc")
, inport_object("Object_Position_in")
, outport_q_left("Left_q_arm_out"), outport_q_right("Right_q_arm_out")
, outport_Dq_left("Left_Dq_arm_out"), outport_Dq_right("Right_Dq_arm_out")
, outport_T_left("Left_T_arm_out"), outport_T_right("Right_T_arm_out")
,outport_Object_Pos("Object_Position_out"), outport_Object_Vel("Object_Vel_out"){

	prop_counter_step=0.001;
	prop_service_call_counter=0.001;
	this->addEventPort(inport_left_pos).doc("Receiving the joint's position of the left arm and they will wake up this component.");
	this->addEventPort(inport_left_vel).doc("Receiving the joint's velocity of the left arm and they will wake up this component.");
	this->addEventPort(inport_left_acc).doc("Receiving the joint's acceleration of the left arm and they will wake up this component.");

	this->addEventPort(inport_right_pos).doc("Receiving the joint's position of the right arm and they will wake up this component.");
	this->addEventPort(inport_right_vel).doc("Receiving the joint's velocity of the right arm and they will wake up this component.");
	this->addEventPort(inport_right_acc).doc("Receiving the joint's acceleration of the right arm and they will wake up this component.");

	this->addEventPort(inport_object).doc("Receiving the desired position of the object will wake up this component.");


	this->addPort(outport_q_left).doc("Sends out the actual joints' position of the left arm ");
	this->addPort(outport_q_right).doc("Sends out the actual joints' position of the right arm");

	this->addPort(outport_Dq_left).doc("Sends out the actual joints' velocity of the left arm ");
	this->addPort(outport_Dq_right).doc("Sends out the actual joints' velocity of the right arm");

	this->addPort(outport_T_left).doc("Sends out the actual joints' torque of the left arm ");
	this->addPort(outport_T_right).doc("Sends out the actual joints' torque of the right arm");


	this->addPort(outport_Object_Pos).doc("Sends out the actual position of the object");
	this->addPort(outport_Object_Vel).doc("Sends out the actual velocity of the object");


	joint_position_left_arm_command = Eigen::VectorXf(COMAN_ARM_DOF_SIZE);
	joint_velocity_left_arm_command = Eigen::VectorXf(COMAN_ARM_DOF_SIZE);
	joint_acceleration_left_arm_command = Eigen::VectorXf(COMAN_ARM_DOF_SIZE);
	joint_position_left_arm_command.setZero();
	joint_velocity_left_arm_command.setZero();
	joint_acceleration_left_arm_command.setZero();

	joint_position_right_arm_command= Eigen::VectorXf(COMAN_ARM_DOF_SIZE);
	joint_velocity_right_arm_command= Eigen::VectorXf(COMAN_ARM_DOF_SIZE);
	joint_acceleration_right_arm_command= Eigen::VectorXf(COMAN_ARM_DOF_SIZE);
	joint_position_right_arm_command.setZero();
	joint_velocity_right_arm_command.setZero();
	joint_acceleration_right_arm_command.setZero();

	joint_state_left_arm = rstrt::robot::JointState(COMAN_ARM_DOF_SIZE);
	joint_state_left_arm.angles.setZero();

	joint_state_right_arm= rstrt::robot::JointState(COMAN_ARM_DOF_SIZE);
	joint_state_right_arm.angles.setZero();

	Object_position_desired = Eigen::VectorXf(7);
	Object_position_desired.setZero();

	joint_position_left_arm_output_port.setName("JointPositionOutputPort_left_arm");
	joint_position_left_arm_output_port.setDataSample(joint_position_left_arm_command);

	joint_velocity_left_arm_output_port.setName("JointVelocityOutputPort_left_arm");
	joint_velocity_left_arm_output_port.setDataSample(joint_position_left_arm_command);

	joint_acceleration_left_arm_output_port.setName("JointAccelerationOutputPort_left_arm");
	joint_acceleration_left_arm_output_port.setDataSample(joint_position_left_arm_command);

	this->addPort(joint_position_left_arm_output_port).doc("Output port for sending left arm reference joint positions");
	this->addPort(joint_velocity_left_arm_output_port).doc("Output port for sending left arm reference joint velocity");
	this->addPort(joint_acceleration_left_arm_output_port).doc("Output port for sending left arm reference joint acceleration");

	joint_position_right_arm_output_port.setName("JointPositionOutputPort_right_arm");
	joint_position_right_arm_output_port.setDataSample(joint_position_right_arm_command);

	joint_velocity_right_arm_output_port.setName("JointVelocityOutputPort_right_arm");
	joint_velocity_right_arm_output_port.setDataSample(joint_position_right_arm_command);

	joint_acceleration_right_arm_output_port.setName("JointAccelerationOutputPort_right_arm");
	joint_acceleration_right_arm_output_port.setDataSample(joint_position_right_arm_command);

	this->addPort(joint_position_right_arm_output_port).doc("Output port for sending right arm reference joint position");
	this->addPort(joint_velocity_right_arm_output_port).doc("Output port for sending right arm reference joint velocity");
	this->addPort(joint_acceleration_right_arm_output_port).doc("Output port for sending right arm reference joint acceleration");


	joint_state_left_arm_input_port.setName("JointPositionInputPort_left_arm");

	this->addPort(joint_state_left_arm_input_port).doc("Input port for receiving left arm actual joint values");

	joint_state_right_arm_input_port.setName("JointPositionInputPort_right_arm");

	this->addPort(joint_state_right_arm_input_port).doc("Input port for receiving right arm actual joint values");

	Object_position_output_port.setName("ObjectPositionOutputPort");
	Object_position_output_port.setDataSample(Object_position_desired);

	this->addPort(Object_position_output_port).doc("Output port for sending object's reference postion and orientation values");


	Object_position_input_port.setName("ObjectPositionInputPort");
	this->addPort(Object_position_input_port).doc("Input port for receiving  the measured position of the object");

	Object_velocity_input_port.setName("ObjectVelocityInputPort");
	this->addPort(Object_velocity_input_port).doc("Input port for receiving  the measured velocity of the object");

	q_left_Arm_current.data.resize(COMAN_ARM_DOF_SIZE);
	q_right_Arm_current.data.resize(COMAN_ARM_DOF_SIZE);

	Dq_left_Arm_current.data.resize(COMAN_ARM_DOF_SIZE);
	Dq_right_Arm_current.data.resize(COMAN_ARM_DOF_SIZE);

	T_left_Arm_current.data.resize(COMAN_ARM_DOF_SIZE);
	T_right_Arm_current.data.resize(COMAN_ARM_DOF_SIZE);


	Object_Pos_Desired.data.resize(7);
	Object_Pos_current.data.resize(7);
	Object_Vel_current.data.resize(7);
	Object_position_measured.resize(7);
	Object_velocity_measured.resize(6);

}

bool Test_orocos::configureHook(){
	if (!joint_position_left_arm_output_port.connected())
		return false;
	else
		return true;
	if (!joint_position_right_arm_output_port.connected())
		return false;
	else
		return true;
	std::cout << "Test_orocos configured !" <<std::endl;
	return true;
}

bool Test_orocos::startHook(){
	std::cout << "Test_orocos started !" <<std::endl;
	return true;
}

void Test_orocos::updateHook(){
	inport_left_pos.read(q_left_Arm_pos);
	inport_left_vel.read(q_left_Arm_vel);
	inport_left_acc.read(q_left_Arm_acc);
	inport_right_pos.read(q_right_Arm_pos);
	inport_right_vel.read(q_right_Arm_vel);
	inport_right_acc.read(q_right_Arm_acc);

	inport_object.read(Object_Pos_Desired);
	joint_state_left_arm_input_port.read(joint_state_left_arm);
	joint_state_right_arm_input_port.read(joint_state_right_arm);
	Object_position_input_port.read(Object_position_measured);
	Object_velocity_input_port.read(Object_velocity_measured);


	if(inport_left_pos.read(q_left_Arm_pos)){
		for(int i=0; i<COMAN_ARM_DOF_SIZE; ++i)
		{
			joint_position_left_arm_command(i) = q_left_Arm_pos.data[i];
		}
		joint_position_left_arm_output_port.write(joint_position_left_arm_command);
	//	inport_left.clear();
	}
	if(inport_left_vel.read(q_left_Arm_vel)){
		for(int i=0; i<COMAN_ARM_DOF_SIZE; ++i)
		{
			joint_velocity_left_arm_command(i) = q_left_Arm_vel.data[i];
		}
		joint_velocity_left_arm_output_port.write(joint_velocity_left_arm_command);
	}

	if(inport_left_acc.read(q_left_Arm_acc)){
		for(int i=0; i<COMAN_ARM_DOF_SIZE; ++i)
		{
			joint_acceleration_left_arm_command(i) = q_left_Arm_acc.data[i];
		}
		joint_acceleration_left_arm_output_port.write(joint_acceleration_left_arm_command);
	}

	if(inport_right_pos.read(q_right_Arm_pos)){
		for(int i=0; i<COMAN_ARM_DOF_SIZE; ++i)
		{
			joint_position_right_arm_command(i) = q_right_Arm_pos.data[i];
		}
		joint_position_right_arm_output_port.write(joint_position_right_arm_command);
//		inport_right.clear();
	}
	if(inport_right_vel.read(q_right_Arm_vel)){
		for(int i=0; i<COMAN_ARM_DOF_SIZE; ++i)
		{
			joint_velocity_right_arm_command(i) = q_right_Arm_vel.data[i];
		}
		joint_velocity_right_arm_output_port.write(joint_velocity_right_arm_command);
//		inport_right.clear();
	}
	if(inport_right_acc.read(q_right_Arm_acc)){
		for(int i=0; i<COMAN_ARM_DOF_SIZE; ++i)
		{
			joint_acceleration_right_arm_command(i) = q_right_Arm_acc.data[i];
		}
		joint_acceleration_right_arm_output_port.write(joint_acceleration_right_arm_command);
//		inport_right.clear();
	}



	if(inport_object.read(Object_Pos_Desired)){
		for(int i=0; i<7; ++i)
		{
			Object_position_desired(i) = Object_Pos_Desired.data[i];
		}
	//	std::cout<<Object_position_desired<<std::endl;
		Object_position_output_port.write(Object_position_desired);
		inport_object.clear();
	}


	if (joint_state_left_arm_input_port.read(joint_state_left_arm))
	{
		for(int i=0; i<COMAN_ARM_DOF_SIZE; ++i)
		{
			q_left_Arm_current.data[i]=joint_state_left_arm.angles(i);
			Dq_left_Arm_current.data[i]=joint_state_left_arm.velocities(i);
			T_left_Arm_current.data[i]=joint_state_left_arm.torques(i);
		}
		outport_q_left.write(q_left_Arm_current);
		outport_Dq_left.write(Dq_left_Arm_current);
		outport_T_left.write(T_left_Arm_current);
	}

	if (joint_state_right_arm_input_port.read(joint_state_right_arm))
	{
		for(int i=0; i<COMAN_ARM_DOF_SIZE; ++i)
		{
			q_right_Arm_current.data[i]=joint_state_right_arm.angles(i);
			Dq_right_Arm_current.data[i]=joint_state_right_arm.velocities(i);
			T_right_Arm_current.data[i]=joint_state_right_arm.torques(i);
		}
		outport_q_right.write(q_right_Arm_current);
		outport_Dq_right.write(Dq_right_Arm_current);
		outport_T_right.write(T_right_Arm_current);
//		joint_position_right_arm_input_port.clear();
	}

	if (Object_position_input_port.read(Object_position_measured))
	{

		for(int i=0; i<7; ++i)
		{
			Object_Pos_current.data[i]=Object_position_measured(i);
		}
		outport_Object_Pos.write(Object_Pos_current);
		Object_position_input_port.clear();
	}

	if (Object_velocity_input_port.read(Object_velocity_measured))
	{
		for(int i=0; i<6; ++i)
		{

			Object_Vel_current.data[i]=Object_velocity_measured(i);
		}
		outport_Object_Vel.write(Object_Vel_current);
		Object_velocity_input_port.clear();
	}
}

void Test_orocos::stopHook() {
	std::cout << "Test_orocos executes stopping !" <<std::endl;
}

void Test_orocos::cleanupHook() {
	std::cout << "Test_orocos cleaning up !" <<std::endl;
}


ORO_CREATE_COMPONENT(Test_orocos)
