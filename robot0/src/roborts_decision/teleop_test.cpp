#include <ros/ros.h>

#include "executor/chassis_executor.h"

#include "example_behavior/teleop_behavior.h"


int main(int argc, char **argv) {
    ros::init(argc, argv, "teleop_test_node");
    std::string full_path = ros::package::getPath("roborts_decision") + "/config/decision.prototxt";

    auto chassis_executor = new roborts_decision::ChassisExecutor;
    auto blackboard = new roborts_decision::Blackboard(full_path);

    roborts_decision::TeleOpBehavior teleop_behavior(chassis_executor, blackboard);

    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        teleop_behavior.Run();
        rate.sleep();
    }

    return 0;
}