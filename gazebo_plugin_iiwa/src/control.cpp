

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include "ros/ros.h"
#include "boost/thread.hpp"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose.h"
#include <std_msgs/Float64.h>

#include <kdl_parser/kdl_parser.hpp> //to get robot model

#include <kdl/chainfksolverpos_recursive.hpp> //to solve inv kinem
#include <kdl/chainiksolvervel_pinv.hpp> 
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>

#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/trajectory.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_stationary.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/rotational_interpolation_sa.hpp>

#include <kdl/jntarrayvel.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include "geometry_msgs/Point.h"



using namespace std;
	
namespace gazebo
{
  

  class click_iiwaPlugin : public ModelPlugin
  {
  	private: 
  
		physics::ModelPtr model;
	      
		physics::JointPtr joint_1;
		physics::JointPtr joint_2;
		physics::JointPtr joint_3;
		physics::JointPtr joint_4;
		physics::JointPtr joint_5;
		physics::JointPtr joint_6;
		physics::JointPtr joint_7;
		

	
	private:
	
		bool new_trajectory=false; //tells when a new desired trajectory is avaiable 
		ros::NodeHandle _nh;
		KDL::Tree iiwa_tree;
	
		KDL::ChainFkSolverPos_recursive *_fksolver; //Forward position solver	
		KDL::ChainIkSolverVel_pinv *_ik_solver_vel;   	//Inverse velocity solver
		KDL::ChainIkSolverPos_NR *_ik_solver_pos;

		KDL::Chain _k_chain;
		KDL::JntArray *_q_now; // actual joints position
		
		
		ros::Publisher _cmd_pub[7];
		KDL::Frame _p_now; // actual end-effector position
		
		
    
	      unsigned int n_dof_=7;

		
	
		KDL::Frame _F_des;
		ros::Subscriber _p_des_sub;
		ros::Subscriber _p_des_sub_relative;


		KDL::Trajectory* traject;
		//define vel max and acc max trajectory
		double trap_max_vels_ = 0.5;
  	      double trap_max_accs_ = 0.1;
  	      
  	      
  	      
  	     
	public:
		void KUKA_INVKIN();
		void run();
		bool init_robot_model();
		void get_dirkin();
		void ctrl_loop();


		void updateJointPosition();
		
		void computeCartTr();
		void update_p_des_cb( geometry_msgs::Point );
		void update_p_des_relative_cb( geometry_msgs::Point);
		bool checkPosition();	
		
		void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {	
		                                         
		      model = _parent;

			joint_1 = this->model->GetJoint("lbr_iiwa_joint_1");
			joint_2 = this->model->GetJoint("lbr_iiwa_joint_2");
			joint_3 = this->model->GetJoint("lbr_iiwa_joint_3");
			joint_4 = this->model->GetJoint("lbr_iiwa_joint_4");
			joint_5 = this->model->GetJoint("lbr_iiwa_joint_5");
			joint_6 = this->model->GetJoint("lbr_iiwa_joint_6");
			joint_7 = this->model->GetJoint("lbr_iiwa_joint_7");
		        
		                                                                                                               
	    		if (!ros::isInitialized())
	    		{
				ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
		  		<< "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
	    	  		return;
	   	      }

			_p_des_sub = _nh.subscribe("/gazebo_plugin/des_position", 0, &click_iiwaPlugin::update_p_des_cb, this);
			_p_des_sub_relative = _nh.subscribe("/gazebo_plugin/des_position_relative", 0, &click_iiwaPlugin::update_p_des_relative_cb, this);
			
			KUKA_INVKIN();
			ROS_INFO("end initialization!");

			run();
		}
		
  };
  
  
  
  
  
void click_iiwaPlugin::updateJointPosition(){
	 
	_q_now->data[0] = joint_1->Position(0);
	_q_now->data[1] = joint_2->Position(0);
	_q_now->data[2] = joint_3->Position(0);
	_q_now->data[3] = joint_4->Position(0);
	_q_now->data[4] = joint_5->Position(0);
	_q_now->data[5] = joint_6->Position(0);
	_q_now->data[6] = joint_7->Position(0);
	
}
  
// HOW TO SEND POSITION:  
//  rostopic pub -1 /gazebo_plugin/des_position geometry_msgs/Point -- '0.5' '0.3' '0.2'

void click_iiwaPlugin::update_p_des_relative_cb( geometry_msgs::Point p_des_in){
//	ROS_INFO("update position desid end effector!");

      updateJointPosition();
      
	if(_fksolver->JntToCart(*_q_now, _p_now)<0){
		std::cout << _p_now <<std::endl;
		printf("%s \n","Error: could not calculate forward kinematics :(");
	}
	ROS_INFO("Actual position! : [ %f , %f , %f ]\n", _p_now.p.x(), _p_now.p.y() , _p_now.p.z());
	
	_F_des.p.data[0] = _p_now.p.x() + p_des_in.x; 
	_F_des.p.data[1] = _p_now.p.y() + p_des_in.y;      
	_F_des.p.data[2] = _p_now.p.z() + p_des_in.z;
      
      ROS_INFO("New desired position! : [ %f , %f , %f ]\n", _F_des.p.data[0], _F_des.p.data[1] , _F_des.p.data[2]);
	for(int i=0; i<9; i++ )
		_F_des.M.data[i] = _p_now.M.data[i]; 
	
	computeCartTr();
}

// HOW TO SEND POSITION:
//  rostopic pub -1 /gazebo_plugin/des_position_relative geometry_msgs/Point -- '0.5' '0.3' '0.2'

void click_iiwaPlugin::update_p_des_cb( geometry_msgs::Point p_des_in){
//	ROS_INFO("update position desid end effector!");

      updateJointPosition();
      
	if(_fksolver->JntToCart(*_q_now, _p_now)<0){
		std::cout << _p_now <<std::endl;
		printf("%s \n","Error: could not calculate forward kinematics :(");
	}
	ROS_INFO("Actual position : [ %f , %f , %f ]\n", _p_now.p.x(), _p_now.p.y() , _p_now.p.z());
	
	_F_des.p.data[0] = p_des_in.x; 
	_F_des.p.data[1] = p_des_in.y;      
	_F_des.p.data[2] = p_des_in.z;
      
      ROS_INFO("New desired position : [ %f , %f , %f ]\n", _F_des.p.data[0], _F_des.p.data[1] , _F_des.p.data[2]);
	for(int i=0; i<9; i++ )
		_F_des.M.data[i] = _p_now.M.data[i]; 
	
	computeCartTr();
}


void click_iiwaPlugin::computeCartTr() { 
	
	ROS_INFO("Start computing cart trajectories...");
	
	new_trajectory = true;
	double roll;
	double pitch;
	double yaw;

	_F_des.M.GetRPY(roll, pitch, yaw); 

	if(_fksolver->JntToCart(*_q_now, _p_now)<0){
		std::cout << _p_now <<std::endl;
		printf("%s \n","Error: could not calculate forward kinematics :(");
	}

	KDL::Path_RoundedComposite* path = new KDL::Path_RoundedComposite(0.2,0.01,new KDL::RotationalInterpolation_SingleAxis());
	
	path->Add(KDL::Frame(KDL::Rotation::RPY(roll,pitch,yaw), KDL::Vector(_p_now.p.x(),_p_now.p.y(),_p_now.p.z())));
	path->Add(KDL::Frame(KDL::Rotation::RPY(roll,pitch,yaw), KDL::Vector(_F_des.p.data[0],_F_des.p.data[1],_F_des.p.data[2])));
	path->Finish();

      // Trajectory defines a motion of the robot along a path.
      // This defines a trapezoidal velocity profile.
	KDL::VelocityProfile* velpref = new KDL::VelocityProfile_Trap(trap_max_vels_,trap_max_accs_); //max_vel max_acc
	velpref->SetProfile(0,path->PathLength());  
	traject = new KDL::Trajectory_Segment(path, velpref);	
	
      ROS_INFO("End computing cart trajectories");
	
}



/* check if the desired position is correctly reached*/
bool click_iiwaPlugin::checkPosition(){ 
	
      updateJointPosition();
	
	if(_fksolver->JntToCart(*_q_now, _p_now)<0){
		std::cout << _p_now <<std::endl;
		printf("%s \n","Error: could not calculate forward kinematics :(");
	}
	
	ROS_INFO("Actual position : [ %f , %f , %f ]\t Position desired : [ %f , %f , %f ]\n", _p_now.p.x(), _p_now.p.y() , _p_now.p.z(), _F_des.p.data[0], _F_des.p.data[1] , _F_des.p.data[2]);
//	ROS_INFO("Error : [ %f , %f , %f ]\n",_p_now.p.x() - _F_des.p.data[0], _p_now.p.y() - _F_des.p.data[1], _p_now.p.z() - _F_des.p.data[2] );
	

//	double squared_error_norm = (_p_now.p.x() - _F_des.p.data[0])*(_p_now.p.x() - _F_des.p.data[0]) + (_p_now.p.y() - _F_des.p.data[1])*(_p_now.p.y() - _F_des.p.data[1]) + (_p_now.p.z() - _F_des.p.data[2])*(_p_now.p.z() - _F_des.p.data[2]);	

//	ROS_INFO("Squared error norm : %f", squared_error_norm);
//	if( square_error_norm < 0.01 ){
//	 	ROS_INFO("Destination correctly reached");   
//		return true;
//	}

	if( (fabs(_p_now.p.x() - _F_des.p.data[0]) < 0.01) &&
	    (fabs(_p_now.p.y() - _F_des.p.data[1]) < 0.01) &&
	    (fabs(_p_now.p.z() - _F_des.p.data[2]) < 0.01) ){
	 	ROS_INFO("Destination correctly reached");   
		return true;
	}

	ROS_INFO("Another iteration");
	computeCartTr();	       
	
}


void	click_iiwaPlugin::KUKA_INVKIN() {

	if (!init_robot_model()) exit(1); 
	ROS_INFO("Robot tree correctly loaded from parameter server!");

	cout << "Joints and segments: " << iiwa_tree.getNrOfJoints() << " - " << iiwa_tree.getNrOfSegments() << endl;
 

	_cmd_pub[0] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/joint1_position_controller/command", 0);
	_cmd_pub[1] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/joint2_position_controller/command", 0);
	_cmd_pub[2] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/joint3_position_controller/command", 0);
	_cmd_pub[3] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/joint4_position_controller/command", 0);
	_cmd_pub[4] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/joint5_position_controller/command", 0);
	_cmd_pub[5] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/joint6_position_controller/command", 0);
	_cmd_pub[6] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/joint7_position_controller/command", 0);

	
	
	
}



bool click_iiwaPlugin::init_robot_model() {
	std::string robot_desc_string;
	_nh.param("robot_description", robot_desc_string, std::string()); ////////////
	if (!kdl_parser::treeFromString(robot_desc_string, iiwa_tree)){ //def iiwa_tree    <-- voglio farlo con l'sdf
		ROS_ERROR("Failed to construct kdl tree");
		return false;
	}

	std::string base_link = "lbr_iiwa_link_0";
	std::string tip_link  = "lbr_iiwa_link_7";
	if ( !iiwa_tree.getChain(base_link, tip_link, _k_chain) ) return false;
	_fksolver = new KDL::ChainFkSolverPos_recursive( _k_chain );

	_ik_solver_vel = new KDL::ChainIkSolverVel_pinv( _k_chain );
	_ik_solver_pos = new KDL::ChainIkSolverPos_NR( _k_chain, *_fksolver, *_ik_solver_vel, 100, 1e-6 ); 

	_q_now = new KDL::JntArray( _k_chain.getNrOfJoints() );
	
	return true;
}






void click_iiwaPlugin::ctrl_loop() {


	ros::Rate r(100);
	updateJointPosition();
		    
	double t;

	bool correctly_position;
	double dt = 0.01;
	std_msgs::Float64 cmd[7];

	ROS_INFO("Ready to control!");	
	while( ros::ok()) {				
	

		if(new_trajectory){
			new_trajectory = false;
			for (double t=0.0; (t <= traject->Duration()) && (!new_trajectory); t+= dt) {
				
				KDL::JntArray q_des(_k_chain.getNrOfJoints());
				KDL::Frame next_pose;
				next_pose = traject->Pos(t);
				updateJointPosition();
				if( _ik_solver_pos->CartToJnt(*_q_now, next_pose, q_des) != KDL::SolverI::E_NOERROR ) 
					cout << "failing in ik!" << endl;
				
				//// just for debug
//				if(_fksolver->JntToCart(*_q_now, _p_now)<0){
//					std::cout << _p_now <<std::endl;
//					printf("%s \n","Error: could not calculate forward kinematics :(");
//				}		
//				ROS_INFO("Actual position! : [ %f , %f , %f ]\t Next position! : [ %f , %f , %f ]\n", _p_now.p.x(), _p_now.p.y() , _p_now.p.z(), next_pose.p.x(), next_pose.p.y() , next_pose.p.z());
				////
				
				for(int i=0; i < n_dof_; i++) {
					cmd[i].data = q_des.data[i];
				}
				for(int i=0; i < n_dof_; i++) {
					_cmd_pub[i].publish(cmd[i]);
				}	           			
		     		
	 		 
				r.sleep(); 
			}
			ROS_INFO("End trajectories!");
			checkPosition(); 
		}		
		r.sleep();			
	}
}



void click_iiwaPlugin::run() {

	boost::thread ctrl_loop_t ( &click_iiwaPlugin::ctrl_loop, this);
}





  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(click_iiwaPlugin)
}





