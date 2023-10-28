#include "Kuka_controller.h"

namespace kuka_control{

bool KUKA_INVKIN::init_robot_model() {
	std::string robot_desc_string;
	_nh.param("robot_description", robot_desc_string, std::string());
	if (!kdl_parser::treeFromString(robot_desc_string, iiwa_tree)){
		ROS_ERROR("Failed to construct kdl tree");
		return false;
	}

	std::string base_link = "lbr_iiwa_link_0";
	std::string tip_link  = "lbr_iiwa_link_7";
	if ( !iiwa_tree.getChain(base_link, tip_link, _k_chain) ) return false;
	_fksolver = new KDL::ChainFkSolverPos_recursive( _k_chain );

	_ik_solver_vel = new KDL::ChainIkSolverVel_pinv( _k_chain );
	_ik_solver_pos = new KDL::ChainIkSolverPos_NR( _k_chain, *_fksolver, *_ik_solver_vel, 100, 1e-6 );

	_q_in = new KDL::JntArray( _k_chain.getNrOfJoints() );
	return true;
}


KUKA_INVKIN::KUKA_INVKIN() {

	if (!init_robot_model()) exit(1); 
	ROS_INFO("Robot tree correctly loaded from parameter server!");

	cout << "Joints and segments: " << iiwa_tree.getNrOfJoints() << " - " << iiwa_tree.getNrOfSegments() << endl;
 
	_cartpose_pub = _nh.advertise<geometry_msgs::Pose>("/lbr_iiwa/eef_pose", 0);
	_js_sub = _nh.subscribe("/lbr_iiwa/joint_states", 0, &KUKA_INVKIN::joint_states_cb, this);
	_aruco_sub = _nh.subscribe("/aruco_marker_publisher_wall_camera/markers", 1, &KUKA_INVKIN::get_marker_position, this);
	_kuka_coord_sub = _nh.subscribe( "/kuka_sem", 1, &KUKA_INVKIN::ready_for_action, this);
	
	_cmd_pub[0] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/joint1_position_controller/command", 0);
	_cmd_pub[1] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/joint2_position_controller/command", 0);
	_cmd_pub[2] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/joint3_position_controller/command", 0);
	_cmd_pub[3] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/joint4_position_controller/command", 0);
	_cmd_pub[4] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/joint5_position_controller/command", 0);
	_cmd_pub[5] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/joint6_position_controller/command", 0);
	_cmd_pub[6] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/joint7_position_controller/command", 0);

	_target_pose.position.z = 0.2;
	_target_pose.orientation.x = 0;
	_target_pose.orientation.y = 1;
	_target_pose.orientation.z = 0;
	_target_pose.orientation.w = 0;
	_first_js = false;
	_first_fk = false;
	_marker_found = false;
	_obj_delivered = false;
	_obj_not_delivered = false;
}


void KUKA_INVKIN::joint_states_cb( sensor_msgs::JointState js ) {

	for(int i=0; i<7; i++ ) 
		_q_in->data[i] = js.position[i];

	_first_js = true;
}


void KUKA_INVKIN::goto_initial_position( float dp[7] ) {
	
	ros::Rate r(10);
	float min_e = 1000;

	std_msgs::Float64 cmd[7];

	float max_e = 1000;
	while( max_e > 0.002 ) {
 		max_e = -1000;
		for(int i=0; i<7; i++) {
 			cmd[i].data = dp[i];
			_cmd_pub[i].publish (cmd[i]);
			float e = fabs( cmd[i].data - _q_in->data[i] );
			max_e = ( e > max_e ) ? e : max_e;
		}
		r.sleep();
	}

	sleep(2);
}


void KUKA_INVKIN::get_dirkin() {

	ros::Rate r(50);
	geometry_msgs::Pose cpose;

	KDL::JntArray q_curr(_k_chain.getNrOfJoints());


	while( !_first_js ) usleep(0.1);


	while(ros::ok()) {


		_fksolver->JntToCart(*_q_in, _p_out);

		cpose.position.x = _p_out.p.x();
		cpose.position.y = _p_out.p.y();
		cpose.position.z = _p_out.p.z();

		double qx, qy, qz, qw;
		_p_out.M.GetQuaternion( qx, qy, qz, qw);
		cpose.orientation.w = qw;
		cpose.orientation.x = qx;
		cpose.orientation.y = qy;
		cpose.orientation.x = qz;

		_cartpose_pub.publish( cpose );		
		_first_fk = true;
	
		r.sleep();
	}
}



void KUKA_INVKIN::ctrl_loop() {

	std_msgs::Float64 d;
	

	while( !_first_fk ) usleep(0.1);
	
	float i_cmd[7];
	i_cmd[0] = 0.0;
	i_cmd[1] = i_cmd[2] = i_cmd[4] = i_cmd[6] = 0.0;
	i_cmd[3] = 1.57;
	i_cmd[5] = -1.57;
	
	goto_initial_position( i_cmd );

	
	ros::Rate r(50);

	KDL::Frame F_dest;
	
	KDL::JntArray q_out(_k_chain.getNrOfJoints());

	F_dest.p.data[0] = _target_pose.position.x;
	F_dest.p.data[1] = _target_pose.position.y;
	F_dest.p.data[2] = _target_pose.position.z;

	F_dest.M = KDL::Rotation::Quaternion(_target_pose.orientation.x, _target_pose.orientation.y, _target_pose.orientation.z, _target_pose.orientation.w);

	std_msgs::Float64 cmd[7];

	ROS_INFO("Waiting for turtlebot delivery");

	while(!_obj_delivered && !_obj_not_delivered) {} //do nothing until the turtlebot signals the results of its task

	if(_obj_delivered) ROS_INFO("Tool delivered, grasping...");
	else if(_obj_delivered)
	{
		ROS_INFO("Tool not delivered, shutting down...");
		return;
	}

	while(ros::ok() && _obj_delivered && _marker_found) {		

		if( _ik_solver_pos->CartToJnt(*_q_in, F_dest, q_out) != KDL::SolverI::E_NOERROR ) 
			cout << "failing in ik!" << endl;

		//Remapping joint position in [-pi, pi]
		for(int i = 0; i < 7; i ++){
			if(q_out.data[i] > 3.14) q_out.data[i] = q_out.data[i] - ceil(q_out.data[i]/6.28) * 6.28;
			if(q_out.data[i] < -3.14) q_out.data[i] = q_out.data[i] + ceil(q_out.data[i]/6.28) * 6.28;
		}

		float K_p = 0.01;
		float max_e = 1000;
		while( max_e > 0.002 && ok()) 
		{
			max_e = -1000;
			for(int i=0; i<7; i++) 
			{
				//Definition of i-th error component
				float e = q_out.data[i] - _q_in->data[i];

				/*Position is provided in a cumulative way starting from initial command value and increasing 
				it in a way proportional to the error*/
				cmd[i].data = cmd[i].data + K_p*e;

				//Publishing message
				_cmd_pub[i].publish (cmd[i]);

				//Find the maximum error: if the i-th joint error is greater than the previous maximum error, update max_e
				float abs_e = fabs( e );
				max_e = ( abs_e > max_e ) ? abs_e : max_e;
				
			}

		r.sleep();
		}

		_obj_delivered = false;
	}
	if(_marker_found)
		ROS_INFO("Tool grasped");
	else
		ROS_INFO("Nothing to pick up");
}

void KUKA_INVKIN::run() {

	ROS_INFO("Starting up kuka...");

	boost::thread get_dirkin_t( &KUKA_INVKIN::get_dirkin, this);
	boost::thread ctrl_loop_t ( &KUKA_INVKIN::ctrl_loop, this);
	spin();
		
}

void KUKA_INVKIN::get_marker_position(aruco_msgs::MarkerArrayConstPtr mark)
{
	if(!_marker_found)
	{
		ROS_INFO("Target position acquired");
		_target_pose.position.x = mark->markers[0].pose.pose.position.x;
		_target_pose.position.y = mark->markers[0].pose.pose.position.y;
		_marker_found = true;
	}
}

void KUKA_INVKIN::ready_for_action(std_msgs::Int32ConstPtr msg)
{
	if(!_obj_delivered && !_obj_not_delivered)
	{
		if(msg->data == 1)
			_obj_delivered = true;
		else if(msg->data == 2)
			_obj_not_delivered == true;
	}
}

}


int main(int argc, char** argv) {

	init(argc, argv, "Kuka_control");
	kuka_control::KUKA_INVKIN kuka;
	kuka.run();
	return 0;
}
