#include "Turtlebot_controller.h"

namespace tbot_ctrl
{

Turtlebot_controller::Turtlebot_controller() : _mv_base("move_base", true) 
{ 
    _aruco_sub = _nH.subscribe("/aruco_marker_publisher/markers", 1, &Turtlebot_controller::look_for_marker, this);
    _cmd_vel_pub = _nH.advertise<geometry_msgs::Twist> ("/cmd_vel", 1);
    _kuka_coord_pub = _nH.advertise<std_msgs::Int32> ("/kuka_sem", 1);

    _w_p[0].x = -7;
    _w_p[0].y = -7;
    _w_p[1].x = -7;
    _w_p[1].y = 7;
    _w_p[2].x = 7;
    _w_p[2].y = 5;
    _w_p[3].x = 7;
    _w_p[3].y = -6.5;

    ROS_INFO("Waiting for move_base server");
    _mv_base.waitForServer();
    ROS_INFO("The connection to the move_base server has been established successfully");

    _desired_marker = 0;
    _marker_detected = 0;
    _marker_identified = false;
    _waypoint_reached = false;
    _marker_found = false;
    _rotation_after_waypoint = false;
    _inside_picking = false;
}

void Turtlebot_controller::go_search()
{
    int room_index = 0; // 0,1 and 2 are the index for the rooom with the tool and 3 is for kuka room
    std_msgs::Int32 msg_for_kuka;
    move_base_msgs::MoveBaseGoal goal;
    
    _desired_marker = choose_item();

    ROS_INFO("Starting the search...");

    while(ros::ok() && !_marker_found && !_concluded)
    {
        //Definition of header goal for move base
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        //Setting goal
        goal.target_pose.pose.position.x =_w_p[room_index].x;
        goal.target_pose.pose.position.y =_w_p[room_index].y;
        goal.target_pose.pose.orientation.x = 0;
        goal.target_pose.pose.orientation.y = 0;
        goal.target_pose.pose.orientation.z = 0;
        goal.target_pose.pose.orientation.w = 1;

        //Sending goal
        _mv_base.sendGoal(goal);
        _mv_base.waitForResult();

        if(_mv_base.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            _waypoint_reached = true;

        //Rotating to look for the marker
        if(!_marker_identified && _waypoint_reached)
        {
            this->rotate();
        }

        if(_marker_detected == _desired_marker)
        {
            ROS_INFO("Tool found");
            this->pick_tool(room_index);
            _marker_found = true;
            room_index = 3;
        }

        if(room_index < 3)
            room_index = room_index +1;
        else
            _concluded = true;

        _marker_identified = false;
        _waypoint_reached = false;
        _rotation_after_waypoint = false;
    }

    if(_marker_found) //goes to kuka
    {
        ROS_INFO("Heading to the kuka room...");
        goal.target_pose.pose.position.x = _w_p[room_index].x;
        goal.target_pose.pose.position.y = _w_p[room_index].y;
        goal.target_pose.pose.orientation.x = 0;
        goal.target_pose.pose.orientation.y = 0;
        goal.target_pose.pose.orientation.z = 0;
        goal.target_pose.pose.orientation.w = 1;

        _mv_base.sendGoal(goal);
        _mv_base.waitForResult();
        ROS_INFO("Kuka station reached");

        //Signaling the manipulator that it can start the grasping
        msg_for_kuka.data = 1;
        _kuka_coord_pub.publish(msg_for_kuka);
    }
    else //goes back to room centre
    {
        ROS_INFO("Tool not found, returning to start position...");
        //Signals kuka the tool has not been found
        msg_for_kuka.data = 2;
        _kuka_coord_pub.publish(msg_for_kuka);

        goal.target_pose.pose.position.x = 0;
        goal.target_pose.pose.position.y = 0;
        goal.target_pose.pose.orientation.x = 0;
        goal.target_pose.pose.orientation.y = 0;
        goal.target_pose.pose.orientation.z = 0;
        goal.target_pose.pose.orientation.w = 1;

        _mv_base.sendGoal(goal);
        _mv_base.waitForResult();
    }
}

void Turtlebot_controller::look_for_marker(aruco_msgs::MarkerArrayConstPtr mark)
{
    if(_marker_detected != mark->markers[0].id && !_marker_found && this->is_in_marker_list(mark->markers[0].id))
    {
        if(!_inside_picking)
            ROS_INFO("A new marker has been detected");
            
        _marker_detected = mark->markers[0].id;
        _marker_identified = true;
        if(_desired_marker != _marker_detected) //cancel goal only if marker is not desired
        {
            ROS_INFO("Not the desired marker");
            _mv_base.cancelGoal(); 
        }   
    }

}

void Turtlebot_controller::pick_tool(const int index)
{
    ROS_INFO("Picking the tool up...");
    
    _inside_picking = true;

    if(_waypoint_reached && _rotation_after_waypoint)
    {
        sleep(1);
    }
    else if(_waypoint_reached && !_rotation_after_waypoint) //goes to the center of the room to pick the tool up
    {
        this->rotate();
        sleep(1);
    }
    ROS_INFO("Picking done");
}

void Turtlebot_controller::rotate()
{
    _marker_detected = 0;
    _marker_identified = false;

    geometry_msgs::Twist cmd_vel_msg;
    float f_rate = 10;
    ros::Rate r(f_rate);
    float ang_vel = 0.3;
    float t_rot = 6.28/ang_vel;
    float count = 0;

    while(ros::ok() && !_marker_identified && count < 1)
    {
        cmd_vel_msg.angular.z = ang_vel;
        _cmd_vel_pub.publish(cmd_vel_msg);
        count = count + 1/(f_rate * t_rot);
        r.sleep();
    }
    cmd_vel_msg.angular.z = 0;
    _cmd_vel_pub.publish(cmd_vel_msg);
    _rotation_after_waypoint = true;
}

int Turtlebot_controller::choose_item()
{
    bool ok = false;
    int your_choice = 0;

    while(!ok)
    {
        cout << "Choose the tool to pick up:" << endl;
        cout << "10 for screwdriver" << endl;
        cout << "20 for scissors" << endl;
        cout << "30 for wrench" << endl << endl;
        cin >> your_choice;
        cout << endl;

        if(your_choice == 10 || your_choice == 20 || your_choice == 30)
        {
            cout << "The choice is valid." << endl << "----------------------------" << endl << endl;
            ok = true;
        }

        else
        {
            cout << "Invalid choice, please retry..." << endl << endl;
        }
    }

    return your_choice;   
}

bool Turtlebot_controller::is_in_marker_list(const int mark)
{
    int N_mark = 3;
    int marker_list[N_mark];

    marker_list[0] = 10;
    marker_list[1] = 20;
    marker_list[2] = 30;

    for( int i = 0; i < N_mark; i++)
    {
        if(mark == marker_list[i])
            return true;
    }
    
    return false;
}

void Turtlebot_controller::run() {

	boost::thread go_search_t( &Turtlebot_controller::go_search, this);
	ros::spin();	
}

}

int main(int argc, char** argv){
    init(argc, argv, "turtlebot_controller_node");
    tbot_ctrl::Turtlebot_controller tbot_ctrl;
    tbot_ctrl.run();
    return 0;
}
