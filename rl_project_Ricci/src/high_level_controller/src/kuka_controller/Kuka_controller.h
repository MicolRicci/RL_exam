#include "ros/ros.h"
#include "boost/thread.hpp"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose.h"
#include <std_msgs/Float64.h>
#include "std_msgs/Int32.h"
#include "aruco_msgs/MarkerArray.h"
#include <cmath>
#include <iostream>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>



using namespace std;
using namespace KDL;
using namespace ros;

namespace kuka_control{

class KUKA_INVKIN {
	public:
		KUKA_INVKIN();
		void run();
		bool init_robot_model();
		
		void get_dirkin();
		
		void joint_states_cb( sensor_msgs::JointState );
		void ctrl_loop();
		
		void goto_initial_position( float dp[7] );

	private:
		NodeHandle _nh;
		Tree iiwa_tree;
	
		ChainFkSolverPos_recursive *_fksolver; //Forward position solver	
		ChainIkSolverVel_pinv *_ik_solver_vel; //Inverse velocity solver
		ChainIkSolverPos_NR *_ik_solver_pos;

		Chain _k_chain;
	
		Subscriber _js_sub;
		Publisher _cartpose_pub;
		
		JntArray *_q_in;
		
		bool _first_js;
		bool _first_fk;
		
		Publisher _cmd_pub[7];
		Frame _p_out;

		Subscriber _kuka_coord_sub;
		Subscriber _aruco_sub;
		geometry_msgs::Pose _target_pose;
		
		bool _marker_found;
		bool _obj_delivered;		 //signals the tool has been found and delibvered
		bool _obj_not_delivered; 	///signals it was not possible to find the object

		void get_marker_position(aruco_msgs::MarkerArrayConstPtr);
		void ready_for_action(std_msgs::Int32ConstPtr);
};

}
