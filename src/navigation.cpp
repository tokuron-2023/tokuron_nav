#include <ros/ros.h>
#include <math.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <unistd.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <actionlib_msgs/GoalID.h>
#include <tf/transform_broadcaster.h>
#include <std_srvs/SetBool.h>

class Navigation{
    private:
        ros::NodeHandle nh;
        ros::Publisher  goal_pub,
                        vel_pub,
                        empty_goal_pub;
        ros::Subscriber pose_sub,
                        list_sub;
        ros::ServiceServer srv;
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
        double px, py, pz;
        bool mode = false;

    public:
        Navigation();
        void loop();
        bool mode_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
        void pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg);
        void list_callback(const std_msgs::UInt8MultiArray& msg);
        void empty_goal_callback();
        void read_yaml();
        void send_goal(double*, double*, double*);
        double check_distance(double*, double*, double*, double*);
};

Navigation::Navigation(){
    ROS_INFO("start navigation node");
    read_yaml();
    srv = nh.advertiseService("/mode", &Navigation::mode_callback, this);
    list_sub = nh.subscribe("/list", 1, &Navigation::list_callback, this);
    pose_sub = nh.subscribe("/mcl_pose", 1, &Navigation::pose_callback, this);
}

void Navigation::loop(){
    if (mode){
        static int spot_num = 0;
        double  *gx = &vec_spot[vec_array_msg[spot_num]].point.x,
                *gy = &vec_spot[vec_array_msg[spot_num]].point.y,
                *gz = &vec_spot[vec_array_msg[spot_num]].point.z,
                *ppx = &px,
                *ppy = &py, 
                dist;
        send_goal(gx, gy, gz);
        dist = check_distance(gx, gy, ppx, ppy);
        if (dist < 0.05){
            spot_num++;
            mode = false;
            empty_goal_callback();
            if (spot_num == vec_array_msg.size()){
                spot_num = 0;
            }
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
    px = msg.pose.pose.position.x;
    py = msg.pose.pose.position.y;
    pz = msg.pose.pose.position.z;
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
}

void Navigation::empty_goal_callback(){
    sleep(1);
    empty_goal_pub = nh.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);
    actionlib_msgs::GoalID empty;
    empty.id = "";
    empty_goal_pub.publish(empty);
    ROS_INFO("publish empty goal");
}

void Navigation::read_yaml(){
    std::string pkg_path = ros::package::getPath("tokuron");
    std::string yaml_path = "/spot/spot.yaml";
    pkg_path += yaml_path;
    YAML::Node config = YAML::LoadFile(pkg_path);
    ROS_INFO("%s", pkg_path.c_str());
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

    tf::Quaternion orientation=tf::createQuaternionFromRPY(0, 0, *e);
 
    goal_point.pose.position.x = *x;
    goal_point.pose.position.y = *y;
    goal_point.pose.position.z =  0;
    goal_point.pose.orientation.x = orientation[0];
    goal_point.pose.orientation.y = orientation[1];
    goal_point.pose.orientation.z = orientation[2];
    goal_point.pose.orientation.w = orientation[3];
    goal_point.header.stamp = time;
    goal_point.header.frame_id = "map";
 
    goal_pub.publish(goal_point);
    ROS_INFO("send goal");
}

double Navigation::check_distance(double *gx, double *gy, double *px, double *py){
    double distance;
    distance = sqrt(std::pow(*px - *gx, 2) + std::pow(*py - *gy, 2));
    ROS_INFO("distance:%f", distance);
    return distance;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "navigation");
    Navigation navigation;
    ros::Rate rate(10);
    while (ros::ok()){
        navigation.loop();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}