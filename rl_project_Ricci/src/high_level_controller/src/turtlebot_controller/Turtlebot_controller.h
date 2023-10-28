#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "aruco_msgs/MarkerArray.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "std_msgs/Int32.h"
#include <boost/thread.hpp>
#include <iostream>


#include <actionlib/client/simple_action_client.h>


using namespace ros;
using namespace std;


namespace tbot_ctrl
{

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client_mvBase;

//Number of waypoints
const int N = 4;

//Struct for the waypoints.
struct waypoint 
{
    float x;
    float y;
    waypoint(){x=0; y=0;} 
};

class Turtlebot_controller
{
    private:
        NodeHandle _nH;
        Subscriber _aruco_sub;
        Publisher _cmd_vel_pub; //publish command twist
        Publisher _kuka_coord_pub; //publish coordination messages
        Client_mvBase _mv_base; //send commands to turtlebot
        
        waypoint _w_p[N]; //waypoints

	int _desired_marker;
	int _marker_detected; //id number of the marker seen
	
        bool _marker_identified; //signals if a marker has been seen
        bool _marker_found; //signals if the desired marker has been found
        
        bool _waypoint_reached; //signals if the waypoint has been reached
        bool _rotation_after_waypoint; //signals if the robot as looked around after reaching the waypoint
        
        bool _inside_picking;
        bool _concluded;

        void go_search(); //moves the robot around and manage the search and deliver of the tool
        
        void look_for_marker(aruco_msgs::MarkerArrayConstPtr); //search for the desired marker
        
        void pick_tool(const int); //simulates the picking of the correct tool
        
        int choose_item(); //chooses item to pick up
        
        void rotate(); //rotate to search the marker
        
        bool is_in_marker_list(const int); //verify the marker seen belongs to the list of tools that can be picked up, 
        
        //prevents errors when seeing a partially occluded marker

        
    public:
        Turtlebot_controller();
        void run();    
};

}
