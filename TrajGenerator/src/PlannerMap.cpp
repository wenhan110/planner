#include "PlannerMap.h"
#include <iostream>

PlannerMap::PlannerMap(bool first_mapping) : first_mapping_(first_mapping) { Initialize(); }

PlannerMap::~PlannerMap() {
  delete static_layer_;
  delete obstacle_layer_;
  delete obs_inflation_layer_;
  delete sta_inflation_layer_;
  fclose(global_map_file_);
}

void PlannerMap::Initialize() {
  static_layer_ = new StaticLayer(first_mapping_);
  // std::cout<<"stATIC SUCCESS"<<std::endl;
  obstacle_layer_ = new ObstacleLayer(static_layer_, cur_pose_);
    // std::cout<<"obstacle SUCCESS"<<std::endl;
  obs_inflation_layer_ = new InflationLayer(obstacle_layer_);
    // std::cout<<"obs_inflation SUCCESS"<<std::endl;
  sta_inflation_layer_ = new InflationLayer(static_layer_);
    // std::cout<<"sta_inflation SUCCESS"<<std::endl;
  first_mapping_ = false;
}

void PlannerMap::ProjectGP2LocalMap(uint32_t static_row, uint32_t static_col) {
  if(static_layer_->GetCost(static_row, static_col) <= INSCRIBED_INFLATED_OBSTACLE){
      static_layer_->SetCost(static_row, static_col, EDGE_COST);
  }
  // obstacle_layer_->Initialize(cur_pose_); 
}

void PlannerMap::UpdateRobotPose(const Pose2dT &robot_pose) { cur_pose_ = robot_pose; }

// void PlannerMap::UpdateObstacle(const Pose2dT &cur_pose) {
//   double robot_x = cur_pose.x;
//   double robot_y = cur_pose.y;
//   double robot_yaw = cur_pose.th;
//   float dx = cur_pose.x - last_pose_.x;
//   float dy = cur_pose.y - last_pose_.y;
//   float dlt = sqrt(dx * dx + dy * dy);

//   if (dlt >= 1.0) {
//     if (dx >= 0 && dy >= 0) {
//       obstacle_layer_->SetRollingDirection(ROLLING_QUADRANT_1);
//     } else if (dx >= 0 && dy <= 0) {
//       obstacle_layer_->SetRollingDirection(ROLLING_QUADRANT_4);
//     } else if (dx <= 0 && dy >= 0) {
//       obstacle_layer_->SetRollingDirection(ROLLING_QUADRANT_2);
//     } else if (dx <= 0 && dy <= 0) {
//       obstacle_layer_->SetRollingDirection(ROLLING_QUADRANT_3);
//     }

//     obstacle_layer_->UpdateObstacle(cur_pose);
//     last_pose_ = cur_pose_;
//     cur_pose_ = cur_pose;
//   }

//   obs_inflation_layer_->UpdateMap(obstacle_layer_);
//   sta_inflation_layer_->UpdateMap(static_layer_);
// }


void PlannerMap::UpdateObstacle(const Pose2dT &cur_pose){
  cout<<"last pose is : "<<last_pose_.x<<" "<<last_pose_.y<<" "<<last_pose_.th<<endl;
    UpdateRobotPose(cur_pose);
    ///todo : update cloud
    // if(cloud_.size <= 0){
    //     printf("cloud size is 0\n");
    // }
    // else{
    //     for(int i = 0; i < cloud_.size; i++){
    //         Point2dT pt(cloud_.points[i].x, cloud_.points[i].y);
    //         pt = cur_pose * pt;
    //         int cell_col = static_cast <uint32_t>((pt.x - obstacle_layer_->GetOriginX()) / obstacle_layer_->GetResolution());
    //         int cell_row = static_cast <uint32_t>((-pt.y + obstacle_layer_ ->GetOriginY()) / obstacle_layer_->GetResolution());
    //         if(cell_row >= 0 && cell_row < obstacle_layer_->GetCellSizeRow() && cell_col >= 0 && cell_col < obstacle_layer_->GetCellSizeCol()){
    //             if(cloud_.points[i].z < 0.4){
    //                 obstacle_layer_->AddCost(cell_row, cell_col, -25);
    //             }
    //             else if(cloud_.points[i].z > 0.6){
    //                 obstacle_layer_->AddCost(cell_row, cell_col, 25);
    //             }
    //         }
    //     }
    // }

    double dx = cur_pose.x - last_pose_.x;
    double dy = cur_pose.y - last_pose_.y;
    double dlt = sqrt(dx * dx + dy * dy);
    cout<<"dlt is : "<<dlt<<"!!!!!!!!!!!!!!!!!"<<endl;
    clock_t start, end;

    start = clock();

    if(dlt >= 0.2){
      cout<<"need rolling!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
        need_rolling_ = true;
        if(dx >= 0 && dy >= 0){
            obstacle_layer_->SetRollingDirection(ROLLING_QUADRANT_1);
        }
        else if(dx >= 0 && dy <= 0){
            obstacle_layer_->SetRollingDirection(ROLLING_QUADRANT_4);
        }
        else if(dx <= 0 && dy >= 0){
            obstacle_layer_->SetRollingDirection(ROLLING_QUADRANT_2);
        }
        else if(dx <= 0 && dy <= 0){
            obstacle_layer_->SetRollingDirection(ROLLING_QUADRANT_3);
        }
        obstacle_layer_->UpdateObstacle(cur_pose);
        last_pose_ = cur_pose_;
        // cur_pose_ = cur_pose;
    }

    end = clock();
    // std::cout<<"update obstacle time: "<<1000*(double)(end - start) / CLOCKS_PER_SEC<<"ms"<<std::endl;
    
    start = clock();
    // cout<<"ok"<<endl;
    obs_inflation_layer_->UpdateMap(obstacle_layer_);
    end = clock();
    std::cout<<"update obs_inflation_layer_ time: "<<1000*(double)(end - start) / CLOCKS_PER_SEC<<"ms"<<std::endl;
    start = clock();
    sta_inflation_layer_->UpdateMap(static_layer_);
    end = clock();
    // std::cout<<"update sta_inflation_layer_ time: "<<1000*(double)(end - start) / CLOCKS_PER_SEC<<"ms"<<std::endl;
}

void PlannerMap::ManageGlobalMap() {
  global_map_file_ = fopen("global_map.bin", "w+");
  static_layer_->WriteGlobalMap(global_map_file_);
}

void PlannerMap::UpdateObservation(const CCloud &cloud) {
  cloud_ = cloud;
  // UpdateObstacle(cur_pose_);
}
