
#include <ros/ros.h>

#include "blackboard/blackboard.h"
#include "executor/chassis_executor.h"

#include "behavior_tree/behavior_node.h"
#include "behavior_tree/behavior_tree.h"

#include "example_behavior/patrol_behavior.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "behavior_tree_test_node");
    
    std::string full_path = ros::package::getPath("roborts_decision") + "/config/decision.prototxt";

    auto chassis_executor = new roborts_decision::ChassisExecutor;
    auto blackboard_ptr = std::make_shared<roborts_decision::Blackboard>(full_path);
    auto blackboard = blackboard_ptr.get();

    // roborts_decision::PatrolBehavior patrol_behavior(chassis_executor, blackboard, full_path);

    // roborts_decision::PatrolBehavior patrol_behavior(chassis_executor_ptr, blackboard_ptr, full_path);


    auto patrol_action = std::make_shared<roborts_decision::PatrolActionNode>(blackboard_ptr, chassis_executor, full_path);
    auto chase_action = std::make_shared<roborts_decision::ChaseActionNode>(blackboard_ptr, chassis_executor, full_path);

    auto patrol_condition = std::make_shared<roborts_decision::PreconditionNode>("patrol_condition", 
                                                                                blackboard_ptr,
                                                                                [&]() {
                                                                                    return true;
                                                                                },
                                                                                roborts_decision::AbortType::BOTH);
    patrol_condition->SetChild(patrol_action);
    auto chase_condition = std::make_shared<roborts_decision::PreconditionNode>("chase_condition", 
                                                                                blackboard_ptr,
                                                                                [&]() {
                                                                                    return true;
                                                                                },
                                                                                roborts_decision::AbortType::BOTH);
    chase_condition->SetChild(chase_action);
    

    auto mode_selector = std::make_shared<roborts_decision::SelectorNode>("root", blackboard_ptr);
    mode_selector->AddChildren(chase_condition);

    roborts_decision::BehaviorTree root(mode_selector, 200);
    root.Run();

    return 0;
}