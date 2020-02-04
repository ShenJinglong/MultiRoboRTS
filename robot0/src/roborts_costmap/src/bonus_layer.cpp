#include "bonus_layer.h"
#include "observation.h"
#include <sensor_msgs/PointCloud.h>

namespace roborts_costmap {

void BonusLayer::OnInitialize() {
    ros::NodeHandle nh;
    is_enabled_ = true;
    rolling_window_ = layered_costmap_->IsRollingWindow();
    bool track_unknown_space = layered_costmap_->IsTrackingUnknown();
    if (track_unknown_space) {
        default_value_ = NO_INFORMATION;
    } else {
        default_value_ = FREE_SPACE;
    }
    is_current_ = true;
    BonusLayer::MatchSize();
    tf_->waitForTransform("robot_0/odom", "map", ros::Time(0), ros::Duration(3.0));
    bonus_info_sub_ = nh.subscribe<geometry_msgs::PointStamped>("bonus_info", 10, boost::bind(&BonusLayer::OnBonusInfo, this, _1));
}

void BonusLayer::Activate() {
    OnInitialize();
}

void BonusLayer::Deactivate() {

}

void BonusLayer::Reset() {
    ResetMaps();
}

void BonusLayer::OnBonusInfo(const geometry_msgs::PointStamped::ConstPtr &bonus_info) {
    bonus_points_.push_back(*bonus_info);
    RemoveStalePoints();
}

void BonusLayer::RemoveStalePoints() {
    std::vector<geometry_msgs::PointStamped> temp_vec;
    for (int i = 0; i < bonus_points_.size(); ++i) {
        ros::Duration delta_t = ros::Time::now() - bonus_points_[i].header.stamp;
        ROS_INFO("%f", delta_t.toSec());
        if (delta_t.toSec() < bonus_points_[i].point.z) {
            temp_vec.push_back(bonus_points_[i]);
        }
    }
    bonus_points_ = temp_vec;
}

void BonusLayer::UpdateBounds(double robot_x,
                                double robot_y,
                                double robot_yaw,
                                double *min_x,
                                double *min_y,
                                double *max_x,
                                double *max_y) {
    
    if (!is_enabled_) {
        ROS_ERROR("Bonus layer is not enabled.");
        return;
    }
    RemoveStalePoints();

    if (rolling_window_) {
        ResetMaps();
        UpdateOrigin(robot_x - GetSizeXWorld() / 2, robot_y - GetSizeYWorld() / 2);
    
        UseExtraBounds(min_x, min_y, max_x, max_y);

        sensor_msgs::PointCloud pcin;
        pcin.header.frame_id = "map";
        pcin.header.stamp = ros::Time();
        for (int i = 0; i < bonus_points_.size(); ++i) {
            geometry_msgs::Point32 temp_p;
            temp_p.x = bonus_points_[i].point.x;
            temp_p.y = bonus_points_[i].point.y;
            temp_p.z = 0;
            pcin.points.push_back(temp_p);
        }
        sensor_msgs::PointCloud pcout;
        tf_->transformPointCloud("robot_0/odom", pcin, pcout);

        Observation fake_bonus_area;
        for (int i = 0; i < pcout.points.size(); ++i) {
            pcl::PointXYZ pcl_pc;
            pcl_pc.x = pcout.points[i].x;
            pcl_pc.y = pcout.points[i].y;
            pcl_pc.z = pcout.points[i].z;
            fake_bonus_area.cloud_->points.push_back(pcl_pc);
        }
        fake_bonus_area.obstacle_range_ = 5;

        //////////////////////////////

        const pcl::PointCloud<pcl::PointXYZ> &cloud = *(fake_bonus_area.cloud_);
        double sq_bonus_area = fake_bonus_area.obstacle_range_ * fake_bonus_area.obstacle_range_;
        for (unsigned int i = 0; i < cloud.points.size(); ++i) {
            double px = cloud.points[i].x, py = cloud.points[i].y, pz = cloud.points[i].z;

            unsigned int mx, my;
            if (!World2Map(px, py, mx, my)) {
                // ROS_INFO("Transfer Error!");
                continue;
            }
            unsigned int index = GetIndex(mx, my);
            costmap_[index] = LETHAL_OBSTACLE;

            Touch(px, py, min_x, min_y, max_x, max_y);
        }
    }
}

void BonusLayer::UpdateCosts(Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j) {
    if (!is_enabled_) {
        ROS_WARN("Bonus layer is not enabled");
        return;
    }
    // ROS_INFO("Updating bonus layer!");

    unsigned char *master_array = master_grid.GetCharMap();
    unsigned int span = master_grid.GetSizeXCell();
    double resolution = master_grid.GetResolution();
    // ROS_INFO("%f", resolution);
    // ROS_INFO("origin_x: %f, origin_y: %f", origin_x_, origin_y_);
    
    if (rolling_window_) {
        UpdateOverwriteByMax(master_grid, min_i, min_j, max_i, max_j);
    } else {
        for (int i = 0; i < bonus_points_.size(); ++i) {
            unsigned int index = GetIndex((unsigned int)(bonus_points_[i].point.x / resolution), (unsigned int)(bonus_points_[i].point.y / resolution));
            master_array[index] = LETHAL_OBSTACLE;
        }
    }
}

} //namespace roborts_costmap