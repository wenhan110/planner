#include "planner.h"

Planner::Planner(){
    Cell global_start_(0, 0);
    Cell global_goal_(0, 0);
    planner_map_ = new PlannerMap(false);
    a_star_ = new AStar(planner_map_, 252, 1);
    dwa_traj_ = new DWATraj(planner_map_, 121, 10, 0.5, 0.05);
    pure_pursuit_ = new PurePursuit(planner_map_);
}

Planner::~Planner(){

}

void Planner::SetStartGoal(){
    double origin_x = planner_map_->GetStaticLayer()->GetOriginX();
    double origin_y = planner_map_->GetStaticLayer()->GetOriginY();
    // cout<<"origin_x: "<<origin_x<<endl;
    // cout<<"origin_y: "<<origin_y<<endl;
    double resolution = planner_map_->GetStaticLayer()->GetResolution();
    global_start_.col_ = (start_pose_.x - origin_x) / resolution;
    global_start_.row_ = (-start_pose_.y + origin_y) / resolution;
    global_goal_.col_ = (goal_pose_.x - origin_x) / resolution;
    global_goal_.row_ = (-goal_pose_.y + origin_y) / resolution;
    cout<<"global_start_: "<<global_start_.row_<<", "<<global_start_.col_<<endl;
    cout<<"global_goal_: "<<global_goal_.row_<<", "<<global_goal_.col_<<endl;
}


void Planner::UpdateCurPose(const Pose2dT& cur_pose){
    cur_pose_ = cur_pose;
}

void Planner::MotionModelUpdate(const double v_cmd, const double omega_cmd){
    cur_state_.UpdateWithCircleModel(v_cmd, omega_cmd);
}

void Planner::UpdateGoalPose(const Pose2dT& goal_pose){
    goal_pose_ = goal_pose;
}

void Planner::UpdateStartPose(const Pose2dT& start_pose){
    start_pose_ = start_pose;
}

//After global planning, update the global plan into static layer and load info to cur obstacle layer
void Planner::UpdateGlobalPath(){
    clock_t start, end = clock();
    uint32_t start_row = global_start_.row_, start_col = global_start_.col_;
    uint32_t goal_row = global_goal_.row_, goal_col = global_goal_.col_;
    start = clock();
    a_star_->Search(start_row, start_col, goal_row, goal_col);
    end = clock();  
    cout<<"A* search time: "<<1000 * (double)(end - start) / CLOCKS_PER_SEC<<" ms"<<endl;
    global_path_ = a_star_->GetGlobalPath();
    start = clock();
    for(uint32_t i = 0; i < global_path_.size(); i++){
        // printf("global_path_[%d]: %d, %d\n", i, global_path_[i].row_, global_path_[i].col_);
        uint32_t static_row = global_path_[i].row_, static_col = global_path_[i].col_;
        planner_map_->ProjectGP2LocalMap(static_row, static_col);
    }
    end = clock();
    cout<<"ProjectGP2LocalMap time: "<<1000 * (double)(end - start) / CLOCKS_PER_SEC<<" ms"<<endl;
    start = clock();
    planner_map_->GetObstacleLayer()->Reset(cur_pose_);
    end = clock();
    cout<<"Reset time: "<<1000 * (double)(end - start) / CLOCKS_PER_SEC<<" ms"<<endl;
}

//To transform the cells-path to the position-path
void Planner::TransformGlobalPath(){
    if(global_path_.size() == 0){
        cout<<"global_path_ is empty!"<<endl;
        return;
    }
    for(uint32_t i = 0; i < global_path_.size(); i++){
        double origin_x = planner_map_->GetStaticLayer()->GetOriginX();
        double origin_y = planner_map_->GetStaticLayer()->GetOriginY();
        double resolution = planner_map_->GetStaticLayer()->GetResolution();
        double path_x = (global_path_[i].col_ + 0.5) * resolution + origin_x;
        double path_y = -(global_path_[i].row_ + 0.5) * resolution + origin_y;
        Point2dT path_point(path_x, path_y);
        // cout<<"path_point: "<<path_point.x<<", "<<path_point.y<<endl;
        global_path_points_.push_back(path_point);
    }
    cout<<"global_path_points_ size: "<<global_path_points_.size()<<endl;
}

// Right now the purpose is to set the local goal point for trajectory planning
void Planner::PruneGlobalPath(){
    if(global_path_points_.size() == 0){
        cout<<"global_path_points_ is empty!"<<endl;
        return;
    }
    double origin_x = planner_map_->GetObstacleLayer()->GetOriginX();
    double origin_y = planner_map_->GetObstacleLayer()->GetOriginY();
    double resolution = planner_map_->GetObstacleLayer()->GetResolution();
    uint32_t cell_row_size = planner_map_->GetObstacleLayer()->GetCellSizeRow();
    uint32_t cell_col_size = planner_map_->GetObstacleLayer()->GetCellSizeCol();
    double left_bound = origin_x, right_bound = origin_x + cell_col_size * resolution;
    double upper_bound = origin_y, lower_bound = origin_y - cell_row_size * resolution;
    for(uint32_t i = 0; i < global_path_points_.size(); i++){
        if(global_path_points_[i].x >= left_bound && global_path_points_[i].x <= right_bound && global_path_points_[i].y >= lower_bound && global_path_points_[i].y <= upper_bound){
            local_path_points_.push_back(global_path_points_[i]);
        }
    }
    local_goal_ = local_path_points_[local_path_points_.size() - 1];
    cout<<"local goal is : "<<local_goal_.x<<", "<<local_goal_.y<<endl;
    dwa_traj_->SetGoalPoint(local_goal_);
}

bool Planner::CheckGlobalGoal(){
    double cur_x = cur_state_.GetX();
    double cur_y = cur_state_.GetY();
    double cur_yaw = cur_state_.GetYaw();
    if(fabs(cur_x - goal_pose_.x) < 0.09 && fabs(cur_y - goal_pose_.y) < 0.09) {
        return true;
    }
    else{
        return false;
    }
}

void Planner::VisualLocalRefPath(){
    double origin_x = planner_map_->GetObstacleLayer()->GetOriginX();
    double origin_y = planner_map_->GetObstacleLayer()->GetOriginY();
    double resolution = planner_map_->GetObstacleLayer()->GetResolution();
    for(uint32_t i = 0; i < local_path_points_.size(); i++){
        cout<<"local_path_points_["<<i<<"]: "<<local_path_points_[i].x<<", "<<local_path_points_[i].y<<endl;
        uint32_t col = (local_path_points_[i].x - origin_x) / resolution;
        uint32_t row = (-local_path_points_[i].y + origin_y) / resolution;
        planner_map_->GetObstacleLayer()->SetCost(row, col, 1);
    }
    Matrix local_ref_path = planner_map_->GetObstacleLayer()->GetCostmap();
    cout<<"local_ref_path: "<<endl<<local_ref_path<<endl;
}

void Planner::VisualizeGlobalPath(){
    double origin_x = planner_map_->GetStaticLayer()->GetOriginX();
    double origin_y = planner_map_->GetStaticLayer()->GetOriginY();
    double resolution = planner_map_->GetStaticLayer()->GetResolution();
    double cur_x = cur_state_.GetX();
    double cur_y = cur_state_.GetY();
    uint32_t col = (cur_x - origin_x) / resolution;
    uint32_t row = (-cur_y + origin_y) / resolution;
    planner_map_->GetStaticLayer()->SetCost(row, col, 1);
}

void Planner::VisualizeLocalTrajectory(){
    double origin_x = planner_map_->GetObstacleLayer()->GetOriginX();
    double origin_y = planner_map_->GetObstacleLayer()->GetOriginY();
    double resolution = planner_map_->GetObstacleLayer()->GetResolution();
    double cur_x = cur_state_.GetX();
    double cur_y = cur_state_.GetY();
    uint32_t col = (cur_x - origin_x) / resolution;
    uint32_t row = (-cur_y + origin_y) / resolution;
    planner_map_->GetObstacleLayer()->SetCost(row, col, 1);
}

