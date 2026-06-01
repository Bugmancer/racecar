#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <cmath>

#define forward_min  0.75
#define forward_max  1.25
#define forward_dis  1

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

move_base_msgs::MoveBaseGoal costmap_goal;
move_base_msgs::MoveBaseGoal last_costmap_goal;

float current_x = 0;
float current_y = 0;
tf::Quaternion current_q;
float yaw = 0;
float yaw_err = 0;
float resolution = 0;
int width = 0;
int height = 0;
float angle_bend = 0;
int path_cnt = 0;
int path_start = 0;
ros::Publisher bned_pub;
char car_costmap[60][20];
char l_path[20];
char r_path[20];

void rotate_coord(double yaw , float x , float y){

    costmap_goal.target_pose.pose.position.x = x*cos(-yaw)+y*sin(-yaw);
    costmap_goal.target_pose.pose.position.y = y*cos(-yaw)-x*sin(-yaw);
    costmap_goal.target_pose.pose.position.z = 0.0;

}

void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    float goal_x = 0;
    float goal_y = 0;
    float distance = 0;

    width = msg->info.width;
    height = msg->info.height;
    resolution = msg->info.resolution;
    costmap_goal.target_pose.header.frame_id = "map";
    costmap_goal.target_pose.header.stamp = ros::Time::now();

    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            std::cout << static_cast<int>(msg->data[i * width + j]) << " ";
        }
        std::cout << std::endl;
    }
    ROS_INFO("costmap");
    memset(car_costmap , 0 , sizeof(car_costmap));
    memset(r_path , 0 , sizeof(r_path));
    memset(l_path , 0 , sizeof(l_path));
    for(int i =0;i<60;i++){
        memcpy(&(car_costmap[i][0]) , &(msg->data[i*60+39]) , 20);
    }
    for (int i = 0; i < 60; ++i) {
        for (int j = 0; j < 30; ++j) {
            std::cout << static_cast<int>(car_costmap[i][j]) << " ";
        }
        std::cout << std::endl;
    }//车前代价地图
    for(int i = 0; i<20; i++){
        for(int j =30; j>0; j--){
            if(car_costmap[j][i]>10){
                r_path[i] = j;
                break;
            }
        }

        for(int j =30; j<60; j++){
            if(car_costmap[j][i]>10){
                l_path[i] = j;
                break;
            }
        }

    }
    
    goal_x = current_x + 0.5;
    goal_y = current_y + (((l_path[0]+r_path[0])/2)-30)*0.05;
    rotate_coord(yaw , goal_x , goal_y);
    
    // for(int  i = 40; i>20 ; i--){

    //     if(i==1){
    //         ROS_INFO("empty bend");//空弯道
    //         angle_bend = yaw;
    //         std_msgs::String nav_msg;
    //         nav_msg.data = "bend";
    //         bned_pub.publish(nav_msg);
    //         break;
    //     }

    //     if(msg->data[(i*width)+40]==0&&msg->data[((i-1)*width)+40]==0&&msg->data[((i+1)*width)+40]==0){
    //         ROS_INFO("goal located %d , %d" , i , msg->data[(i*width)+50]);
    //         path_start = i ;
    //         break;
    //     }

    // }


    // //设置目标点
    // goal_x = current_x + 0.5;
    // goal_y = current_y + (path_start-width/2.0)*resolution;
    // costmap_goal.target_pose.pose.position.x = goal_x;
    // costmap_goal.target_pose.pose.position.y = goal_y;
    // costmap_goal.target_pose.pose.position.z = 0.0;

    yaw_err = atan2((costmap_goal.target_pose.pose.position.y - current_y), (costmap_goal.target_pose.pose.position.x - current_x));
    costmap_goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_err);

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "send_goal");
    MoveBaseClient ac("move_base", true);
    tf::TransformListener listener;
    ros::NodeHandle goal_handler;
    ros::Subscriber goal_sub = goal_handler.subscribe("/move_base/local_costmap/costmap", 3, &costmapCallback);
    //ros::Subscriber pos_sub = goal_handler.subscribe("/odom", 10, &odomCallback);    
    bned_pub = goal_handler.advertise<std_msgs::String>("/empty_bend", 10);

    tf::StampedTransform transform;



    // 等待 move_base 启动
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    while(ros::ok()){

        try {
        listener.lookupTransform("map", "base_link", ros::Time(0), transform);
    } catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
        return -1;
    }
        current_x = transform.getOrigin().x();
        current_y = transform.getOrigin().y();
        yaw = tf::getYaw(transform.getRotation());
        ros::spinOnce();
        ROS_INFO("position: x=%.2f, y=%.2f , yaw = %.2f ", current_x,current_y,yaw);
       
        // ROS_INFO("info: w=%d, h=%d , y=%.2f", width,height,resolution);
        ROS_INFO("goal: x=%.2f, y=%.2f , yaw = %.2f", costmap_goal.target_pose.pose.position.x ,costmap_goal.target_pose.pose.position.y,yaw_err);
    
        ac.sendGoal(costmap_goal);
    
        //ac.waitForResult();

        // if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        //     //ROS_INFO("Reached goal!");
        // else
        //     //ROS_INFO("Failed to reach goal");    

    }

}