#include <ros/ros.h>
#include <math.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <std_msgs/String.h>
#include <std_msgs/UInt8MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <actionlib_msgs/GoalID.h>
#include <tf/transform_broadcaster.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>

class Navigation{
    private:
        ros::NodeHandle nh;
        ros::Publisher  goal_pub,
                        vel_pub,
                        empty_goal_pub;
        ros::Subscriber pose_sub,
                        list_sub;
        ros::ServiceServer start_srv;
        ros::ServiceClient clear_costmap_srv;
        struct Point {
            double x;
            double y;
            double z;
        };
        struct Spot {
            uint8_t num;
            std::string name;
            Point point;
        };
        std::vector<Spot> vec_spot;
        std::vector<int> vec_array_msg;
        int spot_num = 0;
        double position_x, position_y, target_yaw;
        geometry_msgs::Quaternion orientation;
        bool mode = false,
             first_action = true,
             rotate_flag = true;
        std::string yaml_path;

    public:
        Navigation();
        void loop();
        bool mode_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
        void pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg);
        void list_callback(const std_msgs::UInt8MultiArray& msg);
        void read_yaml();
        void send_goal(double*, double*, double*);
        double check_distance(double*, double*, double*, double*);
        void send_empty_goal();
        void clear_costmap();
        void rotate();
};

Navigation::Navigation(){
    ros::NodeHandle pnh("~");
    pnh.getParam("spot_yaml", yaml_path);
    ROS_INFO("start navigation node");
    read_yaml();
    start_srv = nh.advertiseService("/start_nav", &Navigation::mode_callback, this);
    list_sub = nh.subscribe("/list", 1, &Navigation::list_callback, this);
    pose_sub = nh.subscribe("/mcl_pose", 1, &Navigation::pose_callback, this);
}

void Navigation::loop(){
    if (mode){
        if (!vec_array_msg.empty()){
            double  *gx = &vec_spot[vec_array_msg[spot_num]].point.x,
                    *gy = &vec_spot[vec_array_msg[spot_num]].point.y,
                    *gz = &vec_spot[vec_array_msg[spot_num]].point.z,
                    *ppx = &position_x,
                    *ppy = &position_y, 
                    dist;
            if (!first_action && rotate_flag){
                rotate();
            }else{
                send_goal(gx, gy, gz);
                dist = check_distance(gx, gy, ppx, ppy);
                if (dist < 0.05){
                    spot_num++;
                    mode = false;
                    first_action = false;
                    rotate_flag = true;
                    send_empty_goal();
                    clear_costmap();
                    if (spot_num == vec_array_msg.size()){
                        spot_num = 0;
                    }
                }
            }
        }else{
            ROS_INFO("please publish std_msgs/UInt8MultiArray message");
            mode = false;
        }
    }else{
        ROS_INFO("waiting for service");
    }
}

bool Navigation::mode_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
    ROS_INFO("recived request");
    if (req.data == true){
        res.message = "start navigation mode";
        res.success = true;
        mode = true;
        ROS_INFO("start navigation");
    }else{
        res.message = "start camera mode";
        res.success = true;
        mode = false;
        ROS_INFO("start camera mode");
    }
    return res.success;
}

void Navigation::pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg){
    position_x = msg.pose.pose.position.x;
    position_y = msg.pose.pose.position.y;
    orientation = msg.pose.pose.orientation;
}

void Navigation::list_callback(const std_msgs::UInt8MultiArray& msg){
    vec_array_msg.clear();
    int sum = msg.data.size();
    ROS_INFO("I subscribed [%i]", sum);
    for (int i = 0; i < sum; i++){
        vec_array_msg.push_back(msg.data[i]);
        ROS_INFO("[%i]:%d", i, msg.data[i]);
    }
    vec_array_msg.push_back(0);
    spot_num = 0;
}

void Navigation::read_yaml(){
    // std::string yaml_path = ros::package::getPath("tokuron_nav") += "/spot/real_spot.yaml";
    YAML::Node config = YAML::LoadFile(yaml_path);
    ROS_INFO("%s", yaml_path.c_str());
    try {
        const YAML::Node& spots = config["spot"];
        for (const auto& spotNode : spots){
            Spot spot;
            spot.num = spotNode["num"].as<uint8_t>();
            spot.name = spotNode["name"].as<std::string>();

            const YAML::Node& pointNode = spotNode["point"];
            spot.point.x = pointNode["x"].as<double>();
            spot.point.y = pointNode["y"].as<double>();
            spot.point.z = pointNode["z"].as<double>();

            vec_spot.push_back(spot);

            std::cout << "Num: " << spot.num << std::endl;
            std::cout << "Spot: " << spot.name << std::endl;
            std::cout << "  - Point: (" << spot.point.x << ", " << spot.point.y << ", " << spot.point.z << ")\n";
        }
        ROS_INFO("read yaml");
    } catch (const std::exception& e) {
        std::cerr << "Error reading YAML file: " << e.what() << std::endl;
    }
}

void Navigation::send_goal(double *x, double *y, double *e){
    goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
 
    ros::Rate one_sec(1);
    one_sec.sleep();
     
    ros::Time time = ros::Time::now();
    geometry_msgs::PoseStamped goal_point;

    tf::Quaternion goal_orientation=tf::createQuaternionFromRPY(0, 0, *e);
 
    goal_point.pose.position.x = *x;
    goal_point.pose.position.y = *y;
    goal_point.pose.position.z =  0;
    goal_point.pose.orientation.x = goal_orientation[0];
    goal_point.pose.orientation.y = goal_orientation[1];
    goal_point.pose.orientation.z = goal_orientation[2];
    goal_point.pose.orientation.w = goal_orientation[3];
    goal_point.header.stamp = time;
    goal_point.header.frame_id = "map";
 
    goal_pub.publish(goal_point);
    ROS_INFO("send goal");
}

double Navigation::check_distance(double *gx, double *gy, double *position_x, double *position_y){
    double distance;
    distance = sqrt(std::pow(*position_x - *gx, 2) + std::pow(*position_y - *gy, 2));
    ROS_INFO("distance:%f", distance);
    return distance;
}

void Navigation::send_empty_goal(){
    sleep(1);
    empty_goal_pub = nh.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);
    actionlib_msgs::GoalID empty;
    empty.id = "";
    empty_goal_pub.publish(empty);
    ROS_INFO("publish empty goal");
}

void Navigation::clear_costmap(){
    clear_costmap_srv = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
    std_srvs::Empty srv;
    clear_costmap_srv.call(srv);
    ROS_INFO("service call -> /move_base/clear_costmaps");
}

void Navigation::rotate(){
    static bool get_target_yaw = true;
    double roll, pitch, yaw;
    tf::Quaternion quaternion;
    quaternionMsgToTF(orientation, quaternion);
    tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    geometry_msgs::Twist vel;
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    if (get_target_yaw){
        if (yaw > 0){
            target_yaw = -(3.14 - abs(yaw));
        }else{
            target_yaw = 3.14 - abs(yaw);
        }
        get_target_yaw = false;
    }
    vel.linear.x = 0.0;
    vel.angular.z = 0.6;
    vel_pub.publish(vel);
    if (target_yaw < yaw + 0.6 && target_yaw > yaw - 0.6){
        vel.angular.z = 0.0;
        vel_pub.publish(vel);
        get_target_yaw = true;
        rotate_flag = false;
    }
    ROS_INFO("rotating 180 degrees");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "navigation");
    Navigation navigation;
    ros::Rate rate(5);
    while (ros::ok()){
        navigation.loop();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}