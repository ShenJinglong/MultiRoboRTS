#ifndef ROBORTS_COSTMAP_BONUS_LAYER_H
#define ROBORTS_COSTMAP_BONUS_LAYER_H

#include <chrono>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>
#include <ros/subscriber.h>

#include "costmap_layer.h"

namespace roborts_costmap {

class BonusLayer : public CostmapLayer {
public:
    BonusLayer() {
        costmap_ = nullptr;
    }
    virtual ~BonusLayer() { }
    virtual void OnInitialize();
    virtual void Activate();
    virtual void Deactivate();
    virtual void Reset();
    virtual void UpdateCosts(Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j);
    virtual void UpdateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y,
                            double *max_x, double *max_y);
    void OnBonusInfo(const geometry_msgs::PointStamped::ConstPtr &bonus_info);
    void RemoveStalePoints();

private:

    bool rolling_window_;
    std::chrono::system_clock::time_point reset_time_;
    std::vector<geometry_msgs::PointStamped> bonus_points_;
    ros::Subscriber bonus_info_sub_;
};

} //namespace roborts_costmap

#endif //ROBORTS_COSTMAP_BONUS_LAYER_H