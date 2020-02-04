#include "jps_planner.h"

namespace roborts_global_planner {

using roborts_common::ErrorCode;
using roborts_common::ErrorInfo;

/*
 * 地图坐标系：
 * * -------------------------------------------------------------------> x
 * |
 * |
 * |
 * |
 * |
 * |
 * |
 * |
 * |
 * |
 * |
 * |
 * |
 * y
 * 
**/


JPSPlanner::JPSPlanner(CostmapPtr costmap_ptr): 
        GlobalPlannerBase::GlobalPlannerBase(costmap_ptr),
        gridmap_width_(costmap_ptr->GetCostMap()->GetSizeXCell()),
        gridmap_height_(costmap_ptr->GetCostMap()->GetSizeYCell()),
        cost_(costmap_ptr->GetCostMap()->GetCharMap()) {

    heuristic_factor_ = 1.0;
    inaccessible_cost_ = 253;
    goal_search_tolerance_ = 0.45;

    int id = 0;
    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            int jump_kind = abs(dx) + abs(dy);

            for (int ii = 0; ii < add_neib_map_[jump_kind][0]; ++ii) {
                switch (jump_kind) {
                    case 0:
                        switch (ii) {
                            case 0:
                                neib_always_added_[id][ii][0] = -1;
                                neib_always_added_[id][ii][1] = -1;
                                break;
                            case 1:
                                neib_always_added_[id][ii][0] = 0;
                                neib_always_added_[id][ii][1] = -1;
                                break;
                            case 2:
                                neib_always_added_[id][ii][0] = 1;
                                neib_always_added_[id][ii][1] = -1;
                                break;
                            case 3:
                                neib_always_added_[id][ii][0] = -1;
                                neib_always_added_[id][ii][1] = 0;
                                break;
                            case 4:
                                neib_always_added_[id][ii][0] = 1;
                                neib_always_added_[id][ii][1] = 0;
                                break;
                            case 5:
                                neib_always_added_[id][ii][0] = -1;
                                neib_always_added_[id][ii][1] = 1;
                                break;
                            case 6:
                                neib_always_added_[id][ii][0] = 0;
                                neib_always_added_[id][ii][1] = 1;
                                break;
                            case 7:
                                neib_always_added_[id][ii][0] = 1;
                                neib_always_added_[id][ii][1] = 1;
                                break;
                        }
                        break;
                    case 1:
                        neib_always_added_[id][ii][0] = dx;
                        neib_always_added_[id][ii][1] = dy;
                        break;
                    case 2:
                        switch (ii) {
                            case 0:
                                neib_always_added_[id][ii][0] = dx;
                                neib_always_added_[id][ii][1] = 0;
                                break;
                            case 1:
                                neib_always_added_[id][ii][0] = 0;
                                neib_always_added_[id][ii][1] = dy;
                                break;
                            case 2:
                                neib_always_added_[id][ii][0] = dx;
                                neib_always_added_[id][ii][1] = dy;
                                break;
                        }
                        break;
                }
            }

            for (int ii = 0; ii < add_neib_map_[jump_kind][1]; ++ii) {
                switch (jump_kind) {
                    case 1:
                        switch (ii) {
                            case 0:
                                forced_neib_to_check_[id][ii][0] = 0;
                                forced_neib_to_check_[id][ii][1] = 1;
                                break;
                            case 1:
                                forced_neib_to_check_[id][ii][0] = 0;
                                forced_neib_to_check_[id][ii][1] = -1;
                                break;
                        }
                        forced_neib_to_add_[id][ii][0] = dx;
                        forced_neib_to_add_[id][ii][1] = forced_neib_to_check_[id][ii][1];
                        if (dy != 0) {
                            forced_neib_to_check_[id][ii][0] = forced_neib_to_check_[id][ii][1];
                            forced_neib_to_check_[id][ii][1] = 0;
                            forced_neib_to_add_[id][ii][0] = forced_neib_to_check_[id][ii][0];
                            forced_neib_to_add_[id][ii][1] = dy;
                        }
                        break;
                    case 2:
                        switch (ii) {
                            case 0:
                                forced_neib_to_check_[id][ii][0] = -dx;
                                forced_neib_to_check_[id][ii][1] = 0;
                                forced_neib_to_add_[id][ii][0] = -dx;
                                forced_neib_to_add_[id][ii][1] = dy;
                                break;
                            case 1:
                                forced_neib_to_check_[id][ii][0] = 0;
                                forced_neib_to_check_[id][ii][1] = -dy;
                                forced_neib_to_add_[id][ii][0] = dx;
                                forced_neib_to_add_[id][ii][1] = -dy;
                                break;
                        }
                        break;
                }
            }
            ++id;
        }
    }
}

JPSPlanner::~JPSPlanner() {
    cost_ = nullptr;
}

ErrorInfo JPSPlanner::Plan(const geometry_msgs::PoseStamped &start,
                            const geometry_msgs::PoseStamped &goal,
                            std::vector<geometry_msgs::PoseStamped> &path) {
    
    unsigned int start_x, start_y, goal_x, goal_y, tmp_goal_x, tmp_goal_y;
    unsigned int valid_goal[2];
    unsigned int shortest_dist = std::numeric_limits<unsigned int>::max();
    bool goal_valid = false;

    if (!costmap_ptr_->GetCostMap()->World2Map(start.pose.position.x,
                                                start.pose.position.y,
                                                start_x,
                                                start_y)) {
        ROS_WARN("Failed to transform start pose from map frame to costmap frame");
        return ErrorInfo(ErrorCode::GP_POSE_TRANSFORM_ERROR,
                            "Start pose can't be transformed to costmap frame.");
    }
    if (!costmap_ptr_->GetCostMap()->World2Map(goal.pose.position.x,
                                                goal.pose.position.y,
                                                goal_x,
                                                goal_y)) {
        ROS_WARN("Failed to transform goal pose from map frame to costmap frame");
        return ErrorInfo(ErrorCode::GP_POSE_TRANSFORM_ERROR,
                            "Goal pose can't be transformed to costmap frame.");
    }

    if (costmap_ptr_->GetCostMap()->GetCost(goal_x, goal_y) < inaccessible_cost_) {
        valid_goal[0] = goal_x;
        valid_goal[1] = goal_y;
        goal_valid = true;
    } else {
        tmp_goal_y = goal_y - goal_search_tolerance_;

        while (tmp_goal_y <= goal_y + goal_search_tolerance_) {
            tmp_goal_x = goal_x - goal_search_tolerance_;
            while (tmp_goal_x <= goal_x + goal_search_tolerance_) {
                unsigned char cost = costmap_ptr_->GetCostMap()->GetCost(tmp_goal_x, tmp_goal_y);
                unsigned int dist = abs(goal_x - tmp_goal_x) + abs(goal_y - tmp_goal_y);
                if (cost < inaccessible_cost_ && dist < shortest_dist) {
                    shortest_dist = dist;
                    valid_goal[0] = tmp_goal_x;
                    valid_goal[1] = tmp_goal_y;
                    goal_valid = true;
                }
                tmp_goal_x += 1;
            }
            tmp_goal_y += 1;
        }
    }
    ErrorInfo error_info;
    if (!goal_valid) {
        error_info = ErrorInfo(ErrorCode::GP_GOAL_INVALID_ERROR);
        path.clear();
    } else {
        unsigned int start_index, goal_index;
        start_index = costmap_ptr_->GetCostMap()->GetIndex(start_x, start_y);
        goal_index = costmap_ptr_->GetCostMap()->GetIndex(valid_goal[0], valid_goal[1]);

        costmap_ptr_->GetCostMap()->SetCost(start_x, start_y, roborts_costmap::FREE_SPACE);
        
        if (start_index == goal_index) {
            path.clear();
            path.push_back(start);
            path.push_back(goal);
        } else {
            error_info = SearchPath(start_index, goal_index, path);
            if (error_info.IsOK()) {
                path.back().pose.orientation = goal.pose.orientation;
                path.back().pose.position.z = goal.pose.position.z;
            }
        }
    }

    return error_info;
}

ErrorInfo JPSPlanner::SearchPath(const int &start_index, 
                                    const int &goal_index,
                                    std::vector<geometry_msgs::PoseStamped> &path) {

    g_score_.clear();
    f_score_.clear();
    parent_.clear();
    state_.clear();
    dir_.clear();
    gridmap_width_ = costmap_ptr_->GetCostMap()->GetSizeXCell();
    gridmap_height_ = costmap_ptr_->GetCostMap()->GetSizeYCell();
    ROS_INFO("Search in a map %d", gridmap_width_ *  gridmap_height_);
    cost_ = costmap_ptr_->GetCostMap()->GetCharMap();
    g_score_.resize(gridmap_height_ * gridmap_width_, std::numeric_limits<int>::max());
    f_score_.resize(gridmap_height_ * gridmap_width_, std::numeric_limits<int>::max());
    parent_.resize(gridmap_height_ * gridmap_width_, std::numeric_limits<int>::max());
    state_.resize(gridmap_height_ * gridmap_width_, SearchState::NOT_HANDLE);
    dir_.resize(gridmap_height_ * gridmap_width_, Eigen::Vector2i(0, 0));

    std::priority_queue<int, std::vector<int>, Compare> openlist;
    
    // std::vector<int> openlist_recorder;
    // unsigned char *cost_copy = new unsigned char[gridmap_width_ * gridmap_height_];
    // memcpy(cost_copy, cost_, gridmap_width_ * gridmap_height_);
    
    g_score_.at(start_index) = 0;
    openlist.push(start_index);
    
    // openlist_recorder.push_back(start_index);

    std::vector<int> neighbors_index;
    int current_index, move_cost, h_score, count = 0;


    // int dst_index;
    // if (Jump(costmap_ptr_->GetCostMap()->GetIndex(20, 20), costmap_ptr_->GetCostMap()->GetIndex(60, 55) , Eigen::Vector2i(1, 0), dst_index)) {
    //     unsigned int dst_x, dst_y;
    //     costmap_ptr_->GetCostMap()->Index2Cells(dst_index, dst_x, dst_y);
    //     ROS_INFO("dst: %d %d", dst_x, dst_y);
    // }


    while (!openlist.empty()) {
        // ROS_INFO("++++++++++++++++++++++++++++++++++++++++");
        // std::priority_queue<int, std::vector<int>, Compare> displaylist;
        // displaylist = openlist;

        // for (int ii = 0; ii < displaylist.size(); ++ii) {
        //     unsigned int mx;
        //     unsigned int my;
        //     int index = displaylist.top();
        //     displaylist.pop();
        //     costmap_ptr_->GetCostMap()->Index2Cells(index, mx, my);
        //     ROS_INFO("display: %d %d", mx, my);
        // }
        // ROS_INFO("----------------------------------------");

        current_index = openlist.top();
        openlist.pop();
        state_.at(current_index) = SearchState::CLOSED;

        if (current_index == goal_index) {
            ROS_INFO("Search takes %d cycle counts", count);
            break;
        }

        GetNeighbors(current_index, goal_index, neighbors_index);
        // ROS_INFO("++++++++++++++++++++++++++++++++++++++++");
        // for (int ii = 0; ii < neighbors_index.size(); ++ii) {
        //     unsigned int mx;
        //     unsigned int my;
        //     costmap_ptr_->GetCostMap()->Index2Cells(neighbors_index[ii], mx, my);
        //     ROS_INFO("neighbor: %d %d", mx, my);
        // }
        // ROS_INFO("----------------------------------------");
        

        for (auto neighbor_index : neighbors_index) {
            if (neighbor_index < 0 || neighbor_index >= gridmap_height_ * gridmap_width_) {
                continue;
            }
            if (cost_[neighbor_index] >= inaccessible_cost_ || state_.at(neighbor_index) == SearchState::CLOSED) {
                continue;
            }
            GetMoveCost(current_index, neighbor_index, move_cost);
            if (g_score_.at(neighbor_index) > g_score_.at(current_index) + move_cost + cost_[neighbor_index]) {
                g_score_.at(neighbor_index) = g_score_.at(current_index) + move_cost + cost_[neighbor_index];
                parent_.at(neighbor_index) = current_index;

                if (state_.at(neighbor_index) == SearchState::NOT_HANDLE) {
                    GetDiagonalDistance(neighbor_index, goal_index, h_score);
                    f_score_.at(neighbor_index) = g_score_.at(neighbor_index) + h_score;
                    openlist.push(neighbor_index);
                    // openlist_recorder.push_back(neighbor_index);
                    state_.at(neighbor_index) = SearchState::OPEN;
                }
            }
        }
        count++;
    }
    
    // for (int ii = 0; ii < gridmap_width_ * gridmap_height_; ++ii) {
    //     if (cost_copy[ii] >= inaccessible_cost_) {
    //         cost_copy[ii] = '@';
    //     } else {
    //         cost_copy[ii] = '.';
    //     }
    // }

    // for (auto open_index : openlist_recorder) {
    //     cost_copy[open_index] = '*';
    // }

    // for (int ii = 0; ii < gridmap_width_; ++ii) {
    //     std::cout << ii % 10;
    // }
    // std::cout << std::endl << std::endl;

    // for (int ii = 0; ii < gridmap_height_; ++ii) {
    //     for (int jj = 0; jj < gridmap_width_; ++jj) {
    //         std::cout << cost_copy[ii * gridmap_width_ + jj];
    //     }
    //     std::cout << " " << ii;
    //     std::cout << std::endl;
    // }

    // delete cost_copy;

    if (current_index != goal_index) {
        ROS_WARN("Gloabal planner can't search the valid path!");
        return ErrorInfo(ErrorCode::GP_PATH_SEARCH_ERROR, "Valid global path not found.");
    }
    // ROS_INFO("Success!!!:)");

    unsigned int iter_index = current_index, iter_x, iter_y;

    geometry_msgs::PoseStamped iter_pos;
    iter_pos.pose.orientation.w = 1;
    iter_pos.header.frame_id = "map";
    path.clear();

    costmap_ptr_->GetCostMap()->Index2Cells(iter_index, iter_x, iter_y);
    costmap_ptr_->GetCostMap()->Map2World(iter_x, iter_y, iter_pos.pose.position.x, iter_pos.pose.position.y);
    path.push_back(iter_pos);
    while (iter_index != start_index) {
        iter_index = parent_[iter_index];

        costmap_ptr_->GetCostMap()->Index2Cells(iter_index, iter_x, iter_y);
        costmap_ptr_->GetCostMap()->Map2World(iter_x, iter_y, iter_pos.pose.position.x, iter_pos.pose.position.y);
        path.push_back(iter_pos);
    }
    std::reverse(path.begin(), path.end());

    return ErrorInfo(ErrorCode::OK);
}

ErrorInfo JPSPlanner::GetMoveCost(const int &current_index,
                                    const int &neighbor_index,
                                    int &move_cost) const {
    double delta_y = abs(current_index / gridmap_width_ - neighbor_index / gridmap_width_);
    double delta_x = abs(current_index % gridmap_width_ - neighbor_index % gridmap_width_);
    move_cost = int(14 * std::min(delta_x, delta_y) + 10 * abs(delta_x - delta_y));
    return ErrorInfo(ErrorCode::OK);
}

void JPSPlanner::GetDiagonalDistance(const int &index1, 
                                        const int &index2, 
                                        int &diagonal_distance) const {
    double delta_y = abs(index1 / gridmap_width_ - index2 / gridmap_width_);
    double delta_x = abs(index1 % gridmap_width_ - index2 % gridmap_width_);

    diagonal_distance = heuristic_factor_ * (14 * std::min(delta_x, delta_y) + 10 * abs(delta_x - delta_y));
}

void JPSPlanner::GetNeighbors(const int &current_index, const int &goal_index, std::vector<int> &neighbors_index) {
    neighbors_index.clear();
    
    const int jump_kind = abs(dir_.at(current_index)(0)) + abs(dir_.at(current_index)(1));

    // ROS_INFO("111: %d", jump_kind);

    int num_neib = add_neib_map_[jump_kind][0];
    int num_fneib = add_neib_map_[jump_kind][1];
    int id = (dir_.at(current_index)(0) + 1) + (dir_.at(current_index)(1) + 1) * 3;

    // ROS_INFO("222: %d %d %d", num_neib, num_fneib, id);

    for (int ii = 0; ii < num_neib + num_fneib; ++ii) {
        Eigen::Vector2i expandDir;
        int neighbor_index;

        if (ii < num_neib) {
            expandDir(0) = neib_always_added_[id][ii][0];
            expandDir(1) = neib_always_added_[id][ii][1];

            if (!Jump(current_index, goal_index, expandDir, neighbor_index)) {
                continue;
            }
        } else {
            int n_index = current_index + forced_neib_to_check_[id][ii - num_neib][0] + forced_neib_to_check_[id][ii - num_neib][1] * gridmap_width_;
            if (cost_[n_index] >= inaccessible_cost_) {
                expandDir(0) = forced_neib_to_add_[id][ii - num_neib][0];
                expandDir(1) = forced_neib_to_add_[id][ii - num_neib][1];

                if (!Jump(current_index, goal_index, expandDir, neighbor_index)) {
                    continue;
                }
            } else {
                continue;
            }
        }

        dir_.at(neighbor_index) = expandDir;
        neighbors_index.push_back(neighbor_index);
    } 
}

bool JPSPlanner::Jump(const int &current_index, const int &goal_index, const Eigen::Vector2i &expDir, int &neighbor_index) {
    neighbor_index = current_index + expDir(0) + expDir(1) * gridmap_width_;

    if (cost_[neighbor_index] >= inaccessible_cost_) {
        return false;
    }
    if (neighbor_index == goal_index) {
        return true;
    }
    if (HasForced(neighbor_index, expDir)) {
        return true;
    }

    const int id = (expDir(0) + 1) + 3 * (expDir(1) + 1);
    const int jump_kind = abs(expDir(0)) + abs(expDir(1));
    int num_neib = add_neib_map_[jump_kind][0];
    for (int ii = 0; ii < num_neib - 1; ++ii) {
        int new_neib_index;
        Eigen::Vector2i new_dir(neib_always_added_[id][ii][0], neib_always_added_[id][ii][1]);
        if (Jump(neighbor_index, goal_index, new_dir, new_neib_index)) {
            return true;
        }
    }

    return Jump(neighbor_index, goal_index, expDir, neighbor_index);
}

inline bool JPSPlanner::HasForced(const int &cur_index, const Eigen::Vector2i &dir) {
    int jump_kind = abs(dir(0)) + abs(dir(1));
    int id = (dir(0) + 1) + 3 * (dir(1) + 1);

    switch (jump_kind) {
        case 1:
            for (int ii = 0; ii < 2; ++ii) {
                // ROS_INFO("check: %d %d", forced_neib_to_check_[id][ii][0], forced_neib_to_check_[id][ii][1]);
                int neib_index_to_check = cur_index + forced_neib_to_check_[id][ii][0] + forced_neib_to_check_[id][ii][1] * gridmap_width_;
                if (cost_[neib_index_to_check] >= inaccessible_cost_) {
                    return true;
                }
            }
            return false;
            break;
        case 2:
            for (int ii = 0; ii < 2; ++ii) {
                int neib_index_to_check = cur_index + forced_neib_to_check_[id][ii][0] + forced_neib_to_check_[id][ii][1] * gridmap_width_;
                if (cost_[neib_index_to_check] >= inaccessible_cost_) {
                    return true;
                }
            }
            return false;
            break;
        default:
            return false;
    }
}

}