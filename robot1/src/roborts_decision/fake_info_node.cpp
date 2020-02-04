#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <thread>

double x_ = 0;
double y_ = 0;
double z_ = 0;
double roll_ = 0;
double pitch_ = 0;
double yaw_ = 0;

void teleop_thread();

int main(int argc, char **argv) {
    ros::init(argc, argv, "fake_info_node");
    ros::NodeHandle nh;

    ros::Publisher fake_info_pub = nh.advertise<geometry_msgs::PoseStamped>("fake_info", 10);
    geometry_msgs::PoseStamped fake_enemy_pose;
    std::thread teleop(teleop_thread);
    tf::TransformBroadcaster tf_pub;
    
    ros::Rate rate(10);

    while (ros::ok()) {
        fake_enemy_pose.header.frame_id = "map";
        fake_enemy_pose.header.stamp = ros::Time(0);

        fake_enemy_pose.pose.position.x = x_;
        fake_enemy_pose.pose.position.y = y_;
        fake_enemy_pose.pose.position.z = z_;
        tf::quaternionTFToMsg(tf::createQuaternionFromRPY(roll_, pitch_, yaw_), fake_enemy_pose.pose.orientation);
        fake_info_pub.publish(fake_enemy_pose);
        
        
        tf_pub.sendTransform(tf::StampedTransform(tf::Transform(
            tf::createQuaternionFromRPY(roll_, pitch_, yaw_), 
            tf::Vector3(x_, y_, z_)), 
            ros::Time(0), 
            "map", 
            "fake_enemy_pose"
        ));
        rate.sleep();
    }

    return 0;
}

void teleop_thread() {
    char key;
    while (true) {
        ROS_INFO("%f, %f, %f, %f, %f, %f", x_, y_, z_, roll_, pitch_, yaw_);
        std::cin >> key;
        switch (key) {
            case '6':
                x_ += 1;
                break;
            case '4':
                x_ -= 1;
                break;
            case '8':
                y_ += 1;
                break;
            case '2':
                y_ -= 1;
                break;
            case '9':
                yaw_ += 1;
                break;
            case '7':
                yaw_ -= 1;
                break;
        }
    }
}