#ifndef COSTMAP_OBSTACLE_LAYER_H
#define COSTMAP_OBSTACLE_LAYER_H

// #include "CostmapBase.h"
#include "Point.h"
#include "static_layer.h"

class ObstacleLayer : public CostmapBase {
 public:
  ObstacleLayer(StaticLayer* static_layer, const Pose2dT& pose);

  void Initialize(const Pose2dT& pose);

  void Reset(const Pose2dT& pose);

  void SetStaticLayer(StaticLayer* static_layer);

  void LoadStaticInfo(const Pose2dT& pose);

  void LoadStaticInfo(double lower_left_x, double lower_left_y, double upper_right_x, double upper_right_y);

  void UpdateObstacle(const Pose2dT& cur_pose);

  void UpdateOrigin(double new_origin_x, double new_origin_y);

  void UpdateStaticLayer(double lower_left_x, double lower_left_y, double upper_right_x, double upper_right_y);

  void SetRollingDirection(uint8_t direction);

  uint8_t Getfold();

  inline void AddCost(int row, int col, int dlt_cost){
    uint8_t cost_tmp = GetCost(row, col) + dlt_cost;
    if(cost_tmp > 255)
      cost_tmp = 255;
    if(cost_tmp < 0)
      cost_tmp = 0;
    SetCost(row, col, cost_tmp);
  }

 private:
  uint8_t fold_;
  StaticLayer* static_layer_;
  CCloud cloud_;
  uint8_t rolling_direction_;
  Pose2dT last_pose_;
  Pose2dT cur_pose_;
  // double origin_x_;
  // double origin_y_;
};

#endif  // COSTMAP_OBSTACLE_LAYER_H