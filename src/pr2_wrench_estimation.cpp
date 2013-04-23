/*
 * pr2_wrench_estimation
 *
 *  Created on: Feb 19, 2013
 *  Authors:   Francisco Viña           Alexis Maldonado
 *            fevb <at> kth.se     amaldo <at> cs.uni-bremen.de
 */

/* Copyright (c) 2013, Francisco Vina, KTH
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the KTH nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL KTH BE LIABLE FOR ANY
   DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include <ros/ros.h>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>
#include <urdf/model.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <eigen3/Eigen/Core>
#include <Eigen/Dense>

class PR2WrenchEstimationNode
{

public:

	ros::NodeHandle n_;


	// force estimation from joint efforts
	ros::Publisher topicPub_EstimatedWrench_;
	ros::Publisher topicPub_FilteredWrench_;

	ros::Publisher topicPub_TwistError_;
	ros::Publisher topicPub_FilteredTwistError_;

	ros::Subscriber topicSub_JointStates_;

	ros::Subscriber topicSub_JointTrajControllerState_;


	PR2WrenchEstimationNode()
	{
		n_=  ros::NodeHandle("~");
		topicPub_EstimatedWrench_ = n_.advertise<geometry_msgs::WrenchStamped>("estimated_wrench", 1);
		topicPub_FilteredWrench_ = n_.advertise<geometry_msgs::WrenchStamped>("filtered_wrench", 1);

		topicPub_TwistError_ = n_.advertise<geometry_msgs::TwistStamped>("twist_error", 1);
		topicPub_FilteredTwistError_ = n_.advertise<geometry_msgs::TwistStamped>("filtered_twist_error", 1);

		topicSub_JointStates_ = n_.subscribe("/joint_states", 10, &PR2WrenchEstimationNode::topicCallback_JointStates, this);
		topicSub_JointTrajControllerState_ = n_.subscribe("controller_state", 10,
				&PR2WrenchEstimationNode::topicCallback_JointTrajControllerState, this);

		m_initialized = false;
		m_received_js = false;
		m_received_jcs = true;
	}

	~PR2WrenchEstimationNode()
	{
		delete m_JntToJacSolver;
	}

	void getROSParameters()
	{
		n_.param("chain_tip_frame", m_chain_tip_frame, std::string("l_gripper_tool_frame"));
		n_.param("chain_root_frame", m_chain_root_frame, std::string("torso_lift_link"));

		{
			XmlRpc::XmlRpcValue filter_coeff_b_xml;
			if(n_.hasParam("filter_coeff_b_effort"))
			{
				n_.getParam("filter_coeff_b_effort", filter_coeff_b_xml);

				if(filter_coeff_b_xml.size()!=3)
				{
					ROS_WARN("filter_coeff_b_effort parameter invalid size, setting default value:");
					m_filter_coeff_b_effort = Eigen::Vector3d(0.0201,    0.0402,    0.0201);
					std::cout << m_filter_coeff_b_effort << std::endl;
				}
				else
				{
					for(unsigned int i=0; i<filter_coeff_b_xml.size(); i++)
						m_filter_coeff_b_effort(i) = (double) filter_coeff_b_xml[i];
				}
			}
			else
			{
				ROS_WARN("filter_coeff_b_effort parameter not found, setting default value");
				m_filter_coeff_b_effort = Eigen::Vector3d(0.0201,    0.0402,    0.0201);
				std::cout << m_filter_coeff_b_effort << std::endl;
			}

			XmlRpc::XmlRpcValue filter_coeff_a_xml;
			if(n_.hasParam("filter_coeff_a_effort"))
			{
				n_.getParam("filter_coeff_a_effort", filter_coeff_a_xml);

				if(filter_coeff_a_xml.size()!=3)
				{
					ROS_WARN("filter_coeff_a_effort parameter invalid size, setting default value:");
					m_filter_coeff_a_effort = Eigen::Vector3d(1.0000,   -1.5610,    0.6414);
					std::cout << m_filter_coeff_a_effort << std::endl;
				}
				else
				{
					for(unsigned int i=0; i<filter_coeff_a_xml.size(); i++)
						m_filter_coeff_a_effort(i) = (double) filter_coeff_a_xml[i];
				}
			}
			else
			{
				ROS_WARN("filter_coeff_a_effort parameter not found, setting default value:");
				m_filter_coeff_a_effort = Eigen::Vector3d(1.0000,   -1.5610,    0.6414);
				std::cout << m_filter_coeff_a_effort << std::endl;
			}
		}

		{
			XmlRpc::XmlRpcValue filter_coeff_b_xml;
			if(n_.hasParam("filter_coeff_b_vel"))
			{
				n_.getParam("filter_coeff_b_vel", filter_coeff_b_xml);

				if(filter_coeff_b_xml.size()!=3)
				{
					ROS_WARN("filter_coeff_b_vel parameter invalid size, setting default value:");
					m_filter_coeff_b_vel = Eigen::Vector3d(0.0201,    0.0402,    0.0201);
					std::cout << m_filter_coeff_b_vel << std::endl;
				}
				else
				{
					for(unsigned int i=0; i<filter_coeff_b_xml.size(); i++)
						m_filter_coeff_b_vel(i) = (double) filter_coeff_b_xml[i];
				}
			}
			else
			{
				ROS_WARN("filter_coeff_b_vel parameter not found, setting default value:");
				m_filter_coeff_b_vel = Eigen::Vector3d(0.0201,    0.0402,    0.0201);
				std::cout << m_filter_coeff_b_vel << std::endl;
			}

			XmlRpc::XmlRpcValue filter_coeff_a_xml;
			if(n_.hasParam("filter_coeff_a_vel"))
			{
				n_.getParam("filter_coeff_a_vel", filter_coeff_a_xml);

				if(filter_coeff_a_xml.size()!=3)
				{
					ROS_WARN("filter_coeff_a_vel parameter invalid size, setting default value:");
					m_filter_coeff_a_vel = Eigen::Vector3d(1.0000,   -1.5610,    0.6414);
					std::cout << m_filter_coeff_a_vel << std::endl;
				}
				else
				{
					for(unsigned int i=0; i<filter_coeff_a_xml.size(); i++)
						m_filter_coeff_a_vel(i) = (double) filter_coeff_a_xml[i];
				}
			}
			else
			{
				ROS_WARN("filter_coeff_a_vel parameter not found, setting default value:");
				m_filter_coeff_a_vel = Eigen::Vector3d(1.0000,   -1.5610,    0.6414);
				std::cout << m_filter_coeff_a_vel << std::endl;
			}
		}

	}

	void Init()
	{

		ROS_INFO("---------------------");
		ROS_INFO("Initializing chain from %s to %s", m_chain_root_frame.c_str(),
				m_chain_tip_frame.c_str());
		if(!GetKDLChain())
		{
			return;
		}

		std::cout << std::endl;
		ROS_INFO("DOF in the chain: %d", m_PR2_chain.getNrOfJoints());
		ROS_INFO("Joint names: ");
		for(unsigned int i=0; i<m_PR2_chain.getNrOfSegments(); i++)
		{
			if( m_PR2_chain.getSegment(i).getJoint().getType() != KDL::Joint::None)
			{
				m_joint_names.push_back(m_PR2_chain.getSegment(i).getJoint().getName());
				ROS_INFO("%s", m_joint_names.back().c_str());
			}

		}
		ROS_INFO("Chain initialized");
		ROS_INFO("----------------------");

		m_DOF = m_joint_names.size();

		m_joint_pos = std::vector<double>(m_DOF, 0.0);
		m_joint_efforts = std::vector<double>(m_DOF, 0.0);

		m_joint_controller_pos = std::vector<double>(m_DOF, 0.0);
		m_joint_vel_error = std::vector<double>(m_DOF, 0.0);


		m_JntToJacSolver = new KDL::ChainJntToJacSolver(m_PR2_chain);

		m_initialized = true;
	}

	bool GetKDLChain()
	{
		// first get the tree from the URDF
		KDL::Tree tree;


		/// Get robot_description from ROS parameter server
		std::string param_name = "robot_description";
		std::string full_param_name;
		std::string xml_string;

		n_.searchParam(param_name, full_param_name);
		if (n_.hasParam(full_param_name))
		{
			n_.getParam(full_param_name.c_str(), xml_string);
		}

		else
		{
			ROS_ERROR("Robot description parameter not set, shutting down node...");
			n_.shutdown();
			return false;
		}

		if (xml_string.size() == 0)
		{
			ROS_ERROR("Unable to load robot model from parameter %s",full_param_name.c_str());
			n_.shutdown();
			return false;
		}
		ROS_DEBUG("%s content\n%s", full_param_name.c_str(), xml_string.c_str());

		/// Get urdf model out of robot_description
		urdf::Model model;
		if (!model.initString(xml_string))
		{
			ROS_ERROR("Failed to parse urdf file");
			n_.shutdown();
			return false;
		}
		ROS_DEBUG("Successfully parsed urdf file");

		if (!kdl_parser::treeFromUrdfModel(model, tree)){
			ROS_ERROR("Failed to construct kdl tree");
			n_.shutdown();
			return false;
		}

		if(!tree.getChain(m_chain_root_frame, m_chain_tip_frame, m_PR2_chain))
		{
			ROS_ERROR("Error getting chain from %s to %s, shutting down node ...", m_chain_root_frame.c_str(), m_chain_tip_frame.c_str());
			n_.shutdown();
			return false;
		}

		return true;
	}

	void topicCallback_JointStates(const sensor_msgs::JointStateConstPtr &msg)
	{
		if(!m_initialized)
		{
			ROS_WARN("Node not initialized");
			return;
		}

		ROS_DEBUG("In joint state callback");
		unsigned int joint_count = 0;

		for(unsigned int i=0; i<m_DOF; i++)
		{
			for(unsigned int j=0; j<msg->name.size(); j++)
			{
				if(m_joint_names[i] == (std::string)(msg->name[j].data()))
				{
					m_joint_pos[i] = msg->position[j];
					m_joint_efforts[i] = -1*msg->effort[j];
					joint_count++;
				}
			}
		}

		if(joint_count==m_DOF)
		{
			m_received_js = true;
			SendEstWrench();
		}

		else
		{
			ROS_WARN("Expected %d joint positions in joint_states callback, got %d ",
					m_DOF, joint_count);
		}
	}

	void topicCallback_JointTrajControllerState(const pr2_controllers_msgs::JointTrajectoryControllerStateConstPtr &msg)
	{

		if(!m_initialized)
		{
			ROS_WARN("Node not initialized");
			return;
		}

		ROS_DEBUG("In joint trajectory controller state callback");
		if(msg->actual.positions.size()!=m_DOF)
		{
			ROS_ERROR("In joint trajectory controller state CB: expected joint position msg size %d, got %d.",
					m_DOF, msg->actual.positions.size());
		}

		if(msg->error.velocities.size()!=m_DOF)
		{
			ROS_ERROR("In joint trajectory controller state CB: expected joint velocity error msg size %d, got %d.",
					m_DOF, msg->actual.positions.size());
		}

		if(msg->joint_names.size()!=m_DOF)
		{
			ROS_ERROR("In joint trajectory controller state CB: expected joint names msg size %d, got %d.",
					m_DOF, msg->joint_names.size());
		}

		unsigned int joint_count = 0;
		for(unsigned int i=0; i<m_DOF; i++)
		{
			if(msg->joint_names[i] != m_joint_names[i])
			{
				ROS_ERROR("In joint trajectory controller state CB: expected joint name %s, got %s.",
						m_joint_names[i].c_str(), msg->joint_names[i].c_str());
				break;
			}
			joint_count++;
		}

		if(joint_count == m_DOF)
		{
			m_joint_controller_pos = msg->actual.positions;
			m_joint_vel_error = msg->error.velocities;

			m_received_jcs = true;
			SendTwistError();
		}

		else
		{
			return;
		}

	}

	bool isInitialized()
	{
		return m_initialized;
	}


	// expressed in the base frame
	bool SendEstWrench()
	{
		// first get the jacobian
		KDL::Jacobian J(m_DOF);

		KDL::JntArray q_in;
		q_in.resize(m_DOF);

		for(unsigned int i=0; i<m_DOF; i++)
		{
			q_in(i) = m_joint_pos[i];
		}

		m_JntToJacSolver->JntToJac(q_in, J);

		Eigen::MatrixXd joint_effort;
		joint_effort.resize(m_DOF, 1);

		for(unsigned int i=0; i<m_DOF; i++)
			joint_effort(i) = m_joint_efforts[i];

		Eigen::MatrixXd JT = J.data.transpose();
		Eigen::Matrix<double, 6, 1> estimated_wrench =
				JT.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(joint_effort);

		geometry_msgs::WrenchStamped estimated_wrench_msg;
		estimated_wrench_msg.header.stamp = ros::Time::now();
		estimated_wrench_msg.header.frame_id = m_chain_root_frame;

		estimated_wrench_msg.wrench.force.x = estimated_wrench(0);
		estimated_wrench_msg.wrench.force.y = estimated_wrench(1);
		estimated_wrench_msg.wrench.force.z = estimated_wrench(2);

		estimated_wrench_msg.wrench.torque.x = estimated_wrench(3);
		estimated_wrench_msg.wrench.torque.y = estimated_wrench(4);
		estimated_wrench_msg.wrench.torque.z = estimated_wrench(5);

		topicPub_EstimatedWrench_.publish(estimated_wrench_msg);

		Eigen::Matrix<double, 6, 1> filtered_wrench;
		FilterWrench(estimated_wrench, filtered_wrench);

		geometry_msgs::WrenchStamped filtered_wrench_msg;
		filtered_wrench_msg.header = estimated_wrench_msg.header;
		filtered_wrench_msg.wrench.force.x = filtered_wrench(0);
		filtered_wrench_msg.wrench.force.y = filtered_wrench(1);
		filtered_wrench_msg.wrench.force.z = filtered_wrench(2);

		filtered_wrench_msg.wrench.torque.x = filtered_wrench(3);
		filtered_wrench_msg.wrench.torque.y = filtered_wrench(4);
		filtered_wrench_msg.wrench.torque.z = filtered_wrench(5);

		topicPub_FilteredWrench_.publish(filtered_wrench_msg);

		return true;
	}

	bool SendTwistError()
	{
		// first get the jacobian
		KDL::Jacobian J(m_DOF);

		KDL::JntArray q_in;
		q_in.resize(m_DOF);

		for(unsigned int i=0; i<m_DOF; i++)
		{
			q_in(i) = m_joint_pos[i];
		}

		m_JntToJacSolver->JntToJac(q_in, J);

		Eigen::Matrix<double, 6, 1> twist_error;
		Eigen::MatrixXd joint_vel_error;
		joint_vel_error.resize(m_DOF, 1);

		for(unsigned int i=0; i<m_DOF; i++)
			joint_vel_error(i) = m_joint_vel_error[i];

		twist_error = J.data*joint_vel_error;


		geometry_msgs::TwistStamped twist_error_msg;
		twist_error_msg.header.stamp = ros::Time::now();
		twist_error_msg.header.frame_id = m_chain_root_frame;

		twist_error_msg.twist.linear.x = twist_error(0);
		twist_error_msg.twist.linear.y = twist_error(1);
		twist_error_msg.twist.linear.z = twist_error(2);

		twist_error_msg.twist.angular.x = twist_error(3);
		twist_error_msg.twist.angular.y = twist_error(4);
		twist_error_msg.twist.angular.z = twist_error(5);

		topicPub_TwistError_.publish(twist_error_msg);

		Eigen::Matrix<double, 6, 1> filtered_twist_error;
		FilterTwist(twist_error, filtered_twist_error);

		geometry_msgs::TwistStamped filtered_twist_error_msg;
		filtered_twist_error_msg.header = twist_error_msg.header;
		filtered_twist_error_msg.twist.linear.x = filtered_twist_error(0);
		filtered_twist_error_msg.twist.linear.y = filtered_twist_error(1);
		filtered_twist_error_msg.twist.linear.z = filtered_twist_error(2);

		filtered_twist_error_msg.twist.angular.x = filtered_twist_error(3);
		filtered_twist_error_msg.twist.angular.y = filtered_twist_error(4);
		filtered_twist_error_msg.twist.angular.z = filtered_twist_error(5);

		topicPub_FilteredTwistError_.publish(filtered_twist_error_msg);


		return true;
	}


	void FilterWrench(const Eigen::Matrix<double, 6, 1> &wrench,
			Eigen::Matrix<double, 6, 1> &filtered_wrench)
	{
		static std::vector<Eigen::Matrix<double, 6, 1> > past_wrench(2, Eigen::Matrix<double, 6, 1>::Zero());
		static std::vector<Eigen::Matrix<double, 6, 1> > past_filtered_wrench(2, Eigen::Matrix<double, 6, 1>::Zero());

		// coefficients of the filter
		// cutting at 0.25 the sampling frequency
		Eigen::Vector3d b = m_filter_coeff_b_effort;
		Eigen::Vector3d a = m_filter_coeff_a_effort;

		filtered_wrench = b(0)*wrench + b(1)*past_wrench[0] + b(2)*past_wrench[1]
		                                                                       - a(1)*past_filtered_wrench[0] - a(2) * past_filtered_wrench[1];


		past_wrench[1] = past_wrench[0];
		past_wrench[0] = wrench;


		past_filtered_wrench[1] = past_filtered_wrench[0];
		past_filtered_wrench[0] = filtered_wrench;

	}

	void FilterTwist(const Eigen::Matrix<double, 6, 1> &twist,
			Eigen::Matrix<double, 6, 1> &filtered_twist)
	{
		static std::vector<Eigen::Matrix<double, 6, 1> > past_twist(2, Eigen::Matrix<double, 6, 1>::Zero());
		static std::vector<Eigen::Matrix<double, 6, 1> > past_filtered_twist(2, Eigen::Matrix<double, 6, 1>::Zero());

		// coefficients of the filter
		// cutting at 0.25 the sampling frequency
		Eigen::Vector3d b = m_filter_coeff_b_vel;
		Eigen::Vector3d a = m_filter_coeff_a_vel;

		filtered_twist = b(0)*twist + b(1)*past_twist[0] + b(2)*past_twist[1]
		                                                                   - a(1)*past_filtered_twist[0] - a(2) * past_filtered_twist[1];


		past_twist[1] = past_twist[0];
		past_twist[0] = twist;


		past_filtered_twist[1] = past_filtered_twist[0];
		past_filtered_twist[0] = filtered_twist;

	}

private:

	std::vector<double> m_joint_pos;
	std::vector<double> m_joint_efforts;

	std::vector<double> m_joint_controller_pos;
	std::vector<double> m_joint_vel_error;

	std::string m_chain_root_frame;
	std::string m_chain_tip_frame;

	std::vector<std::string> m_joint_names;

	KDL::Chain m_PR2_chain;
	KDL::ChainJntToJacSolver *m_JntToJacSolver;

	Eigen::Vector3d m_filter_coeff_b_effort;
	Eigen::Vector3d m_filter_coeff_a_effort;

	Eigen::Vector3d m_filter_coeff_b_vel;
	Eigen::Vector3d m_filter_coeff_a_vel;

	bool m_received_js;
	bool m_received_jcs;


	bool m_initialized;
	unsigned int m_DOF;

};



int main(int argc, char **argv) {

	ros::init(argc, argv, "pr2_wrench_estimation");

	PR2WrenchEstimationNode wrench_estimation_node;
	wrench_estimation_node.getROSParameters();
	wrench_estimation_node.Init();

	if(!wrench_estimation_node.isInitialized())
	{
		ROS_ERROR("The wrench estimation node is not initialized, shutting down node...");
		wrench_estimation_node.n_.shutdown();
	}

	double loop_frequency;
	wrench_estimation_node.n_.param("loop_frequency", loop_frequency, 100.0);

	ros::Rate loop_rate(loop_frequency);


	while(wrench_estimation_node.n_.ok())
	{

		ros::spinOnce();
		loop_rate.sleep();
	}

}




