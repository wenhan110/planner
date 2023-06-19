#ifndef COSTMAP_PLANNERMAP_H
#define COSTMAP_PLANNERMAP_H

#include "Point.h"
#include "inflation_layer.h"
#include "obstacle_layer.h"
#include "static_layer.h"
// #include "pure_pursuit.h"
// #include "dwa_traj.h"

class PlannerMap {
 public:
  PlannerMap(bool first_mapping);

  ~PlannerMap();

  void Initialize();

StaticLayer* GetStaticLayer(){
    return static_layer_;
}

ObstacleLayer* GetObstacleLayer(){
    return obstacle_layer_;
}

InflationLayer* GetObsInflationLayer(){
    return obs_inflation_layer_;
}

InflationLayer* GetStaInflationLayer(){
    return sta_inflation_layer_;
}

  void ProjectGP2LocalMap(uint32_t static_row, uint32_t static_col);

  void UpdateObstacle(const Pose2dT& cur_pose);

  void UpdateObservation(const CCloud& cloud);

  void UpdateRobotPose(const Pose2dT& robot_pose);

  void ManageGlobalMap();

  // protected:
  // LayeredCostmap* layered_costmap_;

 private:
  Pose2dT cur_pose_;
  Pose2dT last_pose_;
  ObstacleLayer* obstacle_layer_;
  StaticLayer* static_layer_;
  InflationLayer* obs_inflation_layer_;
  InflationLayer* sta_inflation_layer_;
  CCloud cloud_;

//   PurePursuit* pure_pursuit_;
//   DWATraj* dwa_traj_;

  bool need_rolling_;  
  bool first_mapping_;
  FILE* global_map_file_;
};

#endif  // COSTMAP_PLANNERMAP_H