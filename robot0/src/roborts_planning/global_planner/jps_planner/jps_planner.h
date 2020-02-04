#ifndef ROBORTS_PLANNING_GLOBAL_PLANNER_JPS_PLANNER_H
#define ROBORTS_PLANNING_GLOBAL_PLANNER_JPS_PLANNER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Eigen>

#include "alg_factory/algorithm_factory.h"
#include "state/error_code.h"
#include "costmap/costmap_interface.h"

#include "../global_planner_base.h"

namespace roborts_global_planner {

class JPSPlanner : public GlobalPlannerBase {

public:
    JPSPlanner(CostmapPtr costmap_ptr);
    virtual ~JPSPlanner();

    roborts_common::ErrorInfo Plan (
        const geometry_msgs::PoseStamped &start,
        const geometry_msgs::PoseStamped &goal,
        std::vector<geometry_msgs::PoseStamped> &pash
    );


private:
    enum SearchState {
        NOT_HANDLE,
        OPEN,
        CLOSED
    };

    roborts_common::ErrorInfo SearchPath (
        const int &start_index,
        const int &goal_index,
        std::vector<geometry_msgs::PoseStamped> &path
    );
    roborts_common::ErrorInfo GetMoveCost (
        const int &current_index,
        const int &neighbor_index,
        int &move_cost
    ) const;
    void GetDiagonalDistance (
        const int &index1, 
        const int &index2, 
        int &diagonal_distance
    ) const;
    void GetNeighbors (
        const int &current_index,
        const int &goal_index,
        std::vector<int> &neighbors_index
    );
    bool Jump (
        const int &current_index, 
        const int &goal_index, 
        const Eigen::Vector2i &expDir, 
        int &neighbor_index
    );
    inline bool HasForced(
        const int &cur_index,
        const Eigen::Vector2i &dir
    );

    struct Compare {
        bool operator()(const int &index1, const int &index2) {
            return JPSPlanner::f_score_.at(index1) > JPSPlanner::f_score_.at(index2);
        }
    };

    int neib_always_added_[9][8][3];
    int forced_neib_to_check_[9][2][3];
    int forced_neib_to_add_[9][2][3];
    int add_neib_map_[3][2] = {{8, 0}, {1, 2}, {3, 2}};    

    float heuristic_factor_;
    unsigned int inaccessible_cost_;
    unsigned int goal_search_tolerance_;
    unsigned int gridmap_height_;
    unsigned int gridmap_width_;
    unsigned char *cost_;
    static std::vector<int> f_score_;
    std::vector<int> g_score_;
    std::vector<int> parent_;
    std::vector<JPSPlanner::SearchState> state_;
    std::vector<Eigen::Vector2i> dir_;
};

std::vector<int> JPSPlanner::f_score_;
roborts_common::REGISTER_ALGORITHM(GlobalPlannerBase,
                                    "jps_planner",
                                    JPSPlanner,
                                    std::shared_ptr<roborts_costmap::CostmapInterface>);

}

#endif