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
#include <sensor_msgs/LaserScan.h>

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
        ros::Timer check_moving_timer;
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
        std::vector<Spot> spot_array;
        std::vector<int> gpt_array;
        int spot_num = 0;
        double position_x,
               position_y,
               old_pose_x,
               old_pose_y,
               target_yaw,
               dist_err,
               yaw_tolerance,
               pose_tolerance,
               goal_pub_rate;
        geometry_msgs::Quaternion orientation;
        bool start_nav = false,
             navigation = true,
             rotation = false,
             recovery = false,
             first = true;
        std::string yaml_path;

    public:
        Navigation();
        void loop();
        bool mode_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
        void pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg);
        void list_callback(const std_msgs::UInt8MultiArray& msg);
        void read_yaml();
        void send_goal(double*, double*, double*);
        double calc_distance(double*, double*, double*, double*);
        double calc_opposite_yaw(double);
        double calc_recovery_yaw(double*, double*, double*, double*);
        void check_moving(const ros::TimerEvent& e);
        void send_empty_goal();
        void clear_costmap();
        bool rotate(double);
        double get_goal_pub_rate(){return goal_pub_rate;};
};

Navigation::Navigation(){
    ros::NodeHandle pnh("~");
    pnh.param<std::string>("yaml_path", yaml_path, ros::package::getPath("tokuron_nav") += "/spot/real_spot.yaml");
    pnh.param<double>("dist_err", dist_err, 0.05);
    pnh.param<double>("yaw_tolerance", yaw_tolerance, 0.2);
    pnh.param<double>("pose_tolerance", pose_tolerance, 0.01);
    pnh.param<double>("goal_pub_rate", goal_pub_rate, 10);
    ROS_INFO("Start navigation node");
    read_yaml();
    start_srv = nh.advertiseService("/start_nav", &Navigation::mode_callback, this);
    list_sub = nh.subscribe("/list", 1, &Navigation::list_callback, this);
    pose_sub = nh.subscribe("/mcl_pose", 1, &Navigation::pose_callback, this);
    check_moving_timer = nh.createTimer(ros::Duration(3.0), &Navigation::check_moving, this);
}

void Navigation::loop(){
    if (start_nav){
        if (!gpt_array.empty()){
            static int count = 0;
            double  *gx = &spot_array[gpt_array[spot_num]].point.x,
                    *gy = &spot_array[gpt_array[spot_num]].point.y,
                    *gz = &spot_array[gpt_array[spot_num]].point.z,
                    *px = &position_x,
                    *py = &position_y;
            ROS_WARN("navigation:%d, rotaion:%d, recovery:%d", navigation, rotation, recovery);
            if (rotation){
                ROS_INFO("rotation");
                if (first){
                    target_yaw = calc_opposite_yaw(3.14);
                    first = false;
                }
                if (rotate(target_yaw)){
                    navigation = true;
                    rotation = false;
                    first = true;
                }
            }else if (recovery && count >= 30){
                ROS_INFO("recovery");
                if (first){
                    send_empty_goal();
                    target_yaw = calc_recovery_yaw(gx, gy, px, py);
                    first = false;
                    navigation = false;
                }
                if (rotate(target_yaw)){
                    navigation = true;
                    recovery = false;
                    first = true;
                }
            }else if (navigation){
                ROS_INFO("navigation");
                send_goal(gx, gy, gz);
                if (calc_distance(gx, gy, px, py) < dist_err){
                    spot_num++;
                    start_nav = false;
                    navigation = false;
                    rotation = true;
                    send_empty_goal();
                    clear_costmap();
                    if (spot_num == gpt_array.size()){
                        spot_num = 0;
                        gpt_array.clear();
                    }
                }
            }
            count++;
        }else{
            ROS_ERROR("Please publish std_msgs/UInt8MultiArray message");
            start_nav = false;
        }
    }else{
        ROS_INFO("Waiting for service");
    }
}

bool Navigation::mode_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
    ROS_INFO("Recived request");
    if (req.data == true){
        res.message = "Start navigation mode";
        res.success = true;
        start_nav = true;
        ROS_INFO("Start navigation");
    }else{
        res.message = "Start camera mode";
        res.success = true;
        start_nav = false;
        ROS_INFO("Start camera mode");
    }
    return res.success;
}

void Navigation::pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg){
    position_x = msg.pose.pose.position.x;
    position_y = msg.pose.pose.position.y;
    orientation = msg.pose.pose.orientation;
}

void Navigation::list_callback(const std_msgs::UInt8MultiArray& msg){
    int sum = msg.data.size();
    ROS_INFO("I subscribed [%i]", sum);
    if (sum > spot_array.size() - 1){
        ROS_ERROR("A maximum of %ld spots can be registered", spot_array.size() - 1);
    }else{
        gpt_array.clear();
        for (int i = 0; i < sum; i++){
            if (msg.data[i] > spot_array.size() - 1){
                ROS_ERROR("%d is unregistered number", msg.data[i]);
            }else{
                gpt_array.push_back(msg.data[i]);
                ROS_INFO("[%i]:%d", i, msg.data[i]);
            }
        }
        gpt_array.push_back(0);
        spot_num = 0;
    }
}

void Navigation::read_yaml(){
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

            spot_array.push_back(spot);

            std::cout << "Num: " << spot.num << std::endl;
            std::cout << "Spot: " << spot.name << std::endl;
            std::cout << "  - Point: (" << spot.point.x << ", " << spot.point.y << ", " << spot.point.z << ")\n";
        }
        ROS_INFO("Read yaml");
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
    ROS_INFO("Send goal");
}

double Navigation::calc_distance(double *gx, double *gy, double *px, double *py){
    double distance;
    distance = sqrt(std::pow(*px - *gx, 2) + std::pow(*py - *gy, 2));
    ROS_INFO("Goal distance:%f", distance);
    return distance;
}

double Navigation::calc_opposite_yaw(double rad){
    double roll, pitch, yaw;
    tf::Quaternion quaternion;
    quaternionMsgToTF(orientation, quaternion);
    tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    if (yaw > 0){
        target_yaw = -(rad - abs(yaw));
    }else{
        target_yaw = rad - abs(yaw);
    }
    return target_yaw;
}

double Navigation::calc_recovery_yaw(double *gx, double *gy, double *px, double *py){
    double target_yaw;
    target_yaw = atan2(*gy -*py, *gx -*px);
    return target_yaw;
}

void Navigation::check_moving(const ros::TimerEvent& e){
    // ROS_WARN("x:%f, y:%f", abs(old_pose_x - position_x), abs(old_pose_y - position_y));
    if (abs(old_pose_x - position_x) < pose_tolerance && abs(old_pose_y - position_y) < pose_tolerance){
        recovery = true;
        ROS_WARN("Stopping");
    }else{
        ROS_WARN("Moving");
    }
    old_pose_x = position_x;
    old_pose_y = position_y;
}

void Navigation::send_empty_goal(){
    empty_goal_pub = nh.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);
    actionlib_msgs::GoalID empty;
    empty.id = "";
    empty_goal_pub.publish(empty);
    ROS_INFO("Publish empty goal");
}

void Navigation::clear_costmap(){
    clear_costmap_srv = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
    std_srvs::Empty srv;
    clear_costmap_srv.call(srv);
    ROS_INFO("Service call -> /move_base/clear_costmaps");
}
bool Navigation::rotate(double target_yaw){
    double roll, pitch, yaw;
    tf::Quaternion quaternion;
    quaternionMsgToTF(orientation, quaternion);
    tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    geometry_msgs::Twist vel;
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    vel.linear.x = 0.0;
    if (target_yaw < yaw + yaw_tolerance && target_yaw > yaw - yaw_tolerance){
        vel.angular.z = 0.0;
        vel_pub.publish(vel);
        ROS_INFO("Finish rotate");
        return true;
    }else{
        if (target_yaw > 0){
            vel.angular.z = -0.6;
        }else{
            vel.angular.z = 0.6;
        }
        vel_pub.publish(vel);
        ROS_INFO("Start rotate");
        return false;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "navigation");
    Navigation navigation;
    ros::Rate rate(navigation.get_goal_pub_rate());
    while (ros::ok()){
        navigation.loop();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}