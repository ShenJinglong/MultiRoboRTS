#ifndef ROBORTS_DECISION_TELEOP_BEHAVIOR_H
#define ROBORTS_DECISION_TELEOP_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"

namespace roborts_decision {

class TeleOpBehavior {
public:
    TeleOpBehavior(ChassisExecutor* &chassis_executor, Blackboard* &blackboard): chassis_executor_(chassis_executor), blackboard_(blackboard) {
        cmd_vel_acc_.twist.linear.x = 0;
        cmd_vel_acc_.twist.linear.y = 0;
        cmd_vel_acc_.twist.linear.z = 0;
        
        cmd_vel_acc_.twist.angular.x = 0;
        cmd_vel_acc_.twist.angular.y = 0;
        cmd_vel_acc_.twist.angular.z = 0;
    }

    void Run() {
        auto executor_state = Update();
        chassis_executor_->Execute(cmd_vel_acc_);
        ROS_INFO("current speed: x: %f, y: %f", cmd_vel_acc_.twist.linear.x, cmd_vel_acc_.twist.linear.y);
    }

    void Cancel() {
        chassis_executor_->Cancel();
    }

    BehaviorState Update() {
        return chassis_executor_->Update();
    }

private:

    std::thread command_thread_ = std::thread([&]() {
        while (command_ != 27) {
            std::cin >> command_;
            switch (command_) {
                case '8':
                    cmd_vel_acc_.twist.linear.x += 0.1;
                    break;
                case '2':
                    cmd_vel_acc_.twist.linear.x -= 0.1;
                    break;
                case '4':
                    cmd_vel_acc_.twist.linear.y += 0.1;
                    break;
                case '6':
                    cmd_vel_acc_.twist.linear.y -= 0.1;
                    break;
                case '7':
                    cmd_vel_acc_.twist.angular.z -= 0.1;
                    break;
                case '9':
                    cmd_vel_acc_.twist.angular.z += 0.1;
                    break;
                case '5':
                    cmd_vel_acc_.twist.linear.x = 0;
                    cmd_vel_acc_.twist.linear.y = 0;
                    cmd_vel_acc_.twist.angular.z = 0;
                    break;
                case 27:
                    return;
                default:
                    command_ = '5';
                    break;
            }
        }
    });

    ChassisExecutor *const chassis_executor_;
    Blackboard *const blackboard_;
    roborts_msgs::TwistAccel cmd_vel_acc_;
    char command_ = '5';
};

}

#endif