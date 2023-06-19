#include <iostream>

// #include "../costmap/include/PlannerMap.h"
// #include "dwa_traj.h"
// #include "PlannerMap.h"
// #include "pure_pursuit.h"
// #include "a_star.h"
#include "planner.h"
// #include <queue>
// #include "config.h"
// #include "MotionModel.sh"

int main(){


//Map Setting 
    // StaticLayer *static_layer = new StaticLayer(true);
    // static_layer->SetCost(29, 29, 255);
    // static_layer->SetCost(24, 33, 255);
    // static_layer->SetCost(36, 33, 255);
    // static_layer->SetCost(27, 25, 255);
    // static_layer->SetCost(7, 9, 255);
    // static_layer->SetCost(36, 25, 255);
    // static_layer->SetCost(25, 33, 255);
    // static_layer->SetCost(36, 33, 255);
    // FILE* global_map_file = fopen("global_map.bin", "wb");
    // static_layer->WriteGlobalMap(global_map_file);


//Map setting on arm
    StaticLayer *static_layer = new StaticLayer(true);
    static_layer->SetCost(299, 299, 255);
    static_layer->SetCost(240, 330, 255);
    static_layer->SetCost(360, 330, 255);
    static_layer->SetCost(270, 250, 255);
    static_layer->SetCost(70, 90, 255);
    static_layer->SetCost(360, 250, 255);
    static_layer->SetCost(250, 330, 255);
    static_layer->SetCost(360, 330, 255);
    FILE* global_map_file = fopen("global_map.bin", "wb");
    static_layer->WriteGlobalMap(global_map_file);
    
    // Matrix data = static_layer->GetCostmap();
    // cout << "data: " << endl<<data<<endl;


   //Test1: vertificate the DWA algorithm 

    // DWATraj dwa_traj(121, 10, 1.0, 0.1);  //num, step, horizon_time, pred_time
    // dwa_traj.local_map_->UpdateObstacle(Pose2dT(0, 0, 0));
    // dwa_traj.visual_map_->UpdateObstacle(Pose2dT(0, 0, 0));

    // Matrix data_infla = dwa_traj.GetInflationMap();
    // cout << "data_infla: " << endl<<data_infla<<endl;

    // cout<<"infla_map_cell_size: "<<dwa_traj.local_map_->GetObsInflationLayer()->GetCellSizeCol()<<endl;
    
    // clock_t start, end;
    // start = clock();
    // // dwa_traj.TrajGenerate(0, 0, 0.2, 0, 0);  //vCur, omegaCur, xCur, yCur, yawCur
    // dwa_traj.TrajGenerate(0, 0, -0.24, -0.08, 0);  //vCur, omegaCur, xCur, yCur, yawCur
    // end = clock();
    // cout<<"Generate_time is: "<<1000*(double)(end-start)/CLOCKS_PER_SEC<<"ms"<<endl;
    // start = clock();
    // dwa_traj.GetOptimalTraj();
    // end = clock();
    // cout<<"GetOptimalTraj_time is: "<<1000*(double)(end-start)/CLOCKS_PER_SEC<<"ms"<<endl;

    // dwa_traj.VisualizeOptimalTraj();

    // dwa_traj.VisualizeTraj();


//test 2: put the control cmd onto the robot, predict horizon: 1s
    // Pose2dT cur_pose(-0.45, -0.02, 0);
    // DWATraj dwa_traj(121, 10, 0.5, 0.05);  //num, step, horizon_time, pred_time
    // MotionModel start_state(-0.45, -0.02, 0, 0, 0, 0.05);
    // for(uint32_t j = 24; j < 27; j++){
    //     for(uint32_t i = 0; i < OBSTACLE_MAP_COLS; i++){
    //         dwa_traj.local_map_->GetObstacleLayer()->SetCost(j, i, 224);
    //         dwa_traj.visual_map_->GetObstacleLayer()->SetCost(j, i, 224);
    //     }
    // }
    // dwa_traj.visual_map_->UpdateObstacle(cur_pose);
    // for(uint32_t i = 0; i < dwa_traj.visual_map_list_.size(); i++){
    //     dwa_traj.visual_map_list_[i]->UpdateObstacle(cur_pose);
    // }
    // dwa_traj.SetCurState(start_state);
    // dwa_traj.cur_pose_ = cur_pose;
    // // dwa_traj.local_map_->UpdateObstacle(dwa_traj.cur_pose_);
    // for(int i = 0; i < 55; i++){
    //     cout<<"----------------START----------------------  "<<i<<endl;
    //     dwa_traj.local_map_->UpdateObstacle(dwa_traj.cur_pose_);
    //     // Matrix data_ver = dwa_traj.local_map_->GetObsInflationLayer()->GetCostmap();
    //     // cout<<"data_ver: "<<endl<<data_ver<<endl;
    //     // dwa_traj.visual_map_->UpdateObstacle(dwa_traj.cur_pose_);
    //     // dwa_traj.TrajGenerate(0, 0, -0.24, -0.08, 0);  //vCur, omegaCur, xCur, yCur, yawCur
    //     cout<<"cur_pose: "<<dwa_traj.cur_pose_.x<<" "<<dwa_traj.cur_pose_.y<<" "<<dwa_traj.cur_pose_.th<<endl;
    //     clock_t start, end;
    //     start = clock();
    //     dwa_traj.TrajGenerate();
    //     end = clock();
    //     cout<<"Generate_time is: "<<1000*(double)(end-start)/CLOCKS_PER_SEC<<"ms"<<endl;
    //     start = clock();
    //     dwa_traj.GetOptimalTraj();
    //     end = clock();
    //     cout<<"GetOptimalTraj_time is: "<<1000*(double)(end-start)/CLOCKS_PER_SEC<<"ms"<<endl;
    //     // dwa_traj.VisualizeTraj(i);
    //     // dwa_traj.DrawOptimalTraj();
    //     // dwa_traj.VisualizeOptimalTraj();
    //     // dwa_traj.VisualizeTraj();
    //     // dwa_traj.VisualizeOptimalTraj();
    //     // dwa_traj.DrawOptimalTraj();
    //     dwa_traj.ComputeTwistCommand();
    //     // dwa_traj.UpdateOptimalCmd();
    //     double v_cmd, omega_cmd;
    //     dwa_traj.GetTwistCmd(v_cmd, omega_cmd);
    //     cout<<"v: "<<v_cmd<<" omega: "<<omega_cmd<<endl;
    //     // MotionModel cur_state;
    //     // dwa_traj.GetCurState(cur_state);
    //     dwa_traj.cur_state_.UpdateWithCircleModel(v_cmd, omega_cmd);
    //     dwa_traj.cur_pose_.x = dwa_traj.cur_state_.GetX();
    //     dwa_traj.cur_pose_.y = dwa_traj.cur_state_.GetY();
    //     dwa_traj.cur_pose_.th = dwa_traj.cur_state_.GetYaw();
    //     double origin_x = dwa_traj.visual_map_->GetObsInflationLayer()->GetOriginX();
    //     double origin_y = dwa_traj.visual_map_->GetObsInflationLayer()->GetOriginY();
    //     double cell_size = dwa_traj.visual_map_->GetObsInflationLayer()->GetCellSizeCol();
    //     double resolution = dwa_traj.visual_map_->GetObsInflationLayer()->GetResolution();
    //     uint32_t cell_col = (int)((dwa_traj.cur_pose_.x - origin_x) / resolution);
    //     uint32_t cell_row = (int)((-dwa_traj.cur_pose_.y + origin_y) / resolution);
    //     dwa_traj.visual_map_->GetObsInflationLayer()->SetCost(cell_row, cell_col, 1);
    //     Matrix control_position = dwa_traj.visual_map_->GetObsInflationLayer()->GetCostmap();
    //     cout<<"control_position: "<<endl<<control_position<<endl;
    // }



//Test 3: Pure Pursuit 
    // PlannerMap *local_map = new PlannerMap(false);
    // PurePursuit pure_pursuit(local_map);
    // pure_pursuit.obs_inflation_layer_->UpdateMap(pure_pursuit.obs_layer_);
    // for(uint32_t i = 0; i < pure_pursuit.obs_inflation_layer_->GetCellSizeRow(); i++){
    //     pure_pursuit.obs_inflation_layer_->SetCost(i, 44, 224);
    // }
    // Matrix data_infla = pure_pursuit.obs_inflation_layer_->GetCostmap();
    // // cout<<"data_infla: "<<endl<<data_infla<<endl;

    // MotionModel cur_state(0.3, -0.4, 0*M_PI/4, 0.1, 0, 0.1);
    // pure_pursuit.SetCurState(cur_state);
    // pure_pursuit.cur_pose_.x = cur_state.GetX();
    // pure_pursuit.cur_pose_.y = cur_state.GetY();
    // pure_pursuit.cur_pose_.th = cur_state.GetYaw();
    // pure_pursuit.SetTargetYaw(+M_PI / 2);
    // pure_pursuit.SetTargetLaneX(0.4);
    // // pure_pursuit.ComputeTwistCmd();
    // for(uint32_t i = 0; i < 92; i++){
    //     cout<<"-------------------START-------------------"<<i<<endl;
    //     pure_pursuit.cur_pose_.x = pure_pursuit.cur_state_.GetX();
    //     pure_pursuit.cur_pose_.y = pure_pursuit.cur_state_.GetY();
    //     pure_pursuit.cur_pose_.th = pure_pursuit.cur_state_.GetYaw();
    //     cout<<"cur_pose: "<<pure_pursuit.cur_pose_.x<<" "<<pure_pursuit.cur_pose_.y<<" "<<pure_pursuit.cur_pose_.th<<endl;
    //     double origin_x = pure_pursuit.obs_inflation_layer_->GetOriginX();
    //     double origin_y = pure_pursuit.obs_inflation_layer_->GetOriginY();
    //     double resolution =   pure_pursuit.obs_inflation_layer_->GetResolution();
    //     uint32_t cell_col = (int)((pure_pursuit.cur_pose_.x - origin_x) / resolution);
    //     uint32_t cell_row = (int)((-pure_pursuit.cur_pose_.y + origin_y) / resolution);
    //     pure_pursuit.obs_inflation_layer_->SetCost(cell_row, cell_col, 1);
        
    //     pure_pursuit.ComputeTwistCmd();
    //     double v_cmd = pure_pursuit.getVcmd();
    //     double omega_cmd = pure_pursuit.getOmegacmd();
    //     cout<<"v_cmd: "<<v_cmd<<" omega_cmd: "<<omega_cmd<<endl;
    //     pure_pursuit.cur_state_.UpdateWithCircleModel(v_cmd, omega_cmd);
    //     Matrix data_trajectory = pure_pursuit.obs_inflation_layer_->GetCostmap();
    //     cout<<"data_trajectory: "<<endl<<data_trajectory<<endl;
    //     if(pure_pursuit.CheckGoal()){
    //         cout<<"Goal Reached!"<<endl;
    //         break;
    //     }
    // }

//Test 3.a: Pure Pursuit on arm 
    // PlannerMap *local_map = new PlannerMap(false);
    // PurePursuit pure_pursuit(local_map);
    // pure_pursuit.obs_inflation_layer_->UpdateMap(pure_pursuit.obs_layer_);
    // for(uint32_t i = 0; i < pure_pursuit.obs_inflation_layer_->GetCellSizeRow(); i++){
    //     pure_pursuit.obs_inflation_layer_->SetCost(i, 440, 224);
    // }
    // Matrix data_infla = pure_pursuit.obs_inflation_layer_->GetCostmap();
    // // cout<<"data_infla: "<<endl<<data_infla<<endl;

    // MotionModel cur_state(3.8, -2.4, 1.9*M_PI/4, 0.5, 0, 0.1);
    // pure_pursuit.SetCurState(cur_state);
    // pure_pursuit.cur_pose_.x = cur_state.GetX();
    // pure_pursuit.cur_pose_.y = cur_state.GetY();
    // pure_pursuit.cur_pose_.th = cur_state.GetYaw();
    // pure_pursuit.SetTargetYaw(+M_PI / 2);
    // pure_pursuit.SetTargetLaneX(3.8);
    // clock_t start, end;
    // // pure_pursuit.ComputeTwistCmd();
    // for(uint32_t i = 0; i < 92; i++){
    //     cout<<"-------------------START-------------------"<<i<<endl;
    //     pure_pursuit.cur_pose_.x = pure_pursuit.cur_state_.GetX();
    //     pure_pursuit.cur_pose_.y = pure_pursuit.cur_state_.GetY();
    //     pure_pursuit.cur_pose_.th = pure_pursuit.cur_state_.GetYaw();
    //     cout<<"cur_pose: "<<pure_pursuit.cur_pose_.x<<" "<<pure_pursuit.cur_pose_.y<<" "<<pure_pursuit.cur_pose_.th<<endl;
    //     double origin_x = pure_pursuit.obs_inflation_layer_->GetOriginX();
    //     double origin_y = pure_pursuit.obs_inflation_layer_->GetOriginY();
    //     double resolution =   pure_pursuit.obs_inflation_layer_->GetResolution();
    //     uint32_t cell_col = (int)((pure_pursuit.cur_pose_.x - origin_x) / resolution);
    //     uint32_t cell_row = (int)((-pure_pursuit.cur_pose_.y + origin_y) / resolution);
    //     pure_pursuit.obs_inflation_layer_->SetCost(cell_row, cell_col, 1);
        
    //     start = clock();
    //     pure_pursuit.ComputeTwistCmd();
    //     end = clock();
    //     cout<<"Time: "<<1000 * (double)(end - start) / CLOCKS_PER_SEC<<" ms"<<endl;
    //     double v_cmd = pure_pursuit.getVcmd();
    //     double omega_cmd = pure_pursuit.getOmegacmd();
    //     cout<<"v_cmd: "<<v_cmd<<" omega_cmd: "<<omega_cmd<<endl;
    //     pure_pursuit.cur_state_.UpdateWithCircleModel(v_cmd, omega_cmd);
    //     // Matrix data_trajectory = pure_pursuit.obs_inflation_layer_->GetCostmap();
    //     // cout<<"data_trajectory: "<<endl<<data_trajectory<<endl;
    //     if(pure_pursuit.CheckGoal()){
    //         cout<<"Goal Reached!"<<endl;
    //         break;
    //     }
    // }

//Test4. AStar simple test

    // static_layer->SetCost(27, 33, 255);
    // static_layer->SetCost(28, 33, 255);
    // static_layer->SetCost(29, 33, 255);
    // static_layer->SetCost(31, 33, 255);
    // static_layer->SetCost(33, 33, 255);
    // static_layer->SetCost(33, 31, 255);
    // static_layer->SetCost(33, 29, 255);
    // static_layer->SetCost(33, 27, 255);
    // static_layer->SetCost(33, 26, 255);
    // static_layer->SetCost(33, 23, 255);
    // FILE* global_map_file = fopen("global_map.bin", "wb");
    // static_layer->WriteGlobalMap(global_map_file);

    // PlannerMap* global_map = new PlannerMap(false);
    // AStar a_star(global_map, 252, 1);
    // a_star.sta_inflation_layer_->UpdateMap(a_star.static_layer_);
    // Matrix data = a_star.sta_inflation_layer_->GetCostmap();
    // cout<<"data: "<<endl<<data<<endl;
    // // start: 5, 14; goal: 45, 14
    // a_star.Search(5, 25, 45, 30);
    // a_star.VisualizePath();
    // Matrix path = a_star.sta_inflation_layer_->GetCostmap();
    // cout<<"path: "<<endl<<path<<endl;



//Test 4.b AStar Simple test
    // clock_t start, end;
    // static_layer->SetCost(270, 330, 255);
    // static_layer->SetCost(280, 330, 255);
    // static_layer->SetCost(290, 330, 255);
    // static_layer->SetCost(310, 330, 255);
    // static_layer->SetCost(330, 330, 255);
    // static_layer->SetCost(330, 310, 255);
    // static_layer->SetCost(330, 290, 255);
    // static_layer->SetCost(330, 270, 255);
    // static_layer->SetCost(330, 260, 255);
    // static_layer->SetCost(330, 230, 255);
    // FILE* global_map_file = fopen("global_map.bin", "wb");
    // static_layer->WriteGlobalMap(global_map_file);

    // PlannerMap* global_map = new PlannerMap(false);
    // AStar a_star(global_map, 252, 1);
    // a_star.sta_inflation_layer_->UpdateMap(a_star.static_layer_);
    // start = clock();
    // a_star.Search(350, 250, 450, 300);
    // end = clock();
    // cout<<"Time: "<<1000 * (double)(end - start) / CLOCKS_PER_SEC<<" ms"<<endl;


//Test5. Planner structure test

    
//     Planner* planner = new Planner();
//     Pose2dT cur_pose(-0.5, 1.0 , -1 * M_PI / 2);
//     Pose2dT goal_pose(2.5, -2.5, 0);
//     MotionModel start_state(-0.5, 1.0, -1 * M_PI / 2, 0, 0, 0.05);
//     planner->SetCurrentState(start_state);
//     planner->UpdateCurPose(cur_pose);
//     planner->UpdateGoalPose(goal_pose);
//     planner->UpdateStartPose(cur_pose);
//     planner->GetPlannerMap()->UpdateObstacle(cur_pose);
//     planner->SetStartGoal();
//     // planner->GetAStar()->Search(planner->GetStartRow(), planner->GetStartCol(), planner->GetGoalRow(), planner->GetGoalCol());
//     // Matrix data_sta = planner->GetPlannerMap()->GetStaticLayer()->GetCostmap();
//     // cout<<"data_sta: "<<endl<<data_sta<<endl;
//     // Matrix data_obs = planner->GetPlannerMap()->GetObstacleLayer()->GetCostmap();
//     // cout<<"data_obs: "<<endl<<data_obs<<endl;

//     planner->UpdateGlobalPath();
//     planner->GetAStar()->VisualizePath();
//     Matrix data_sta_infla = planner->GetPlannerMap()->GetStaInflationLayer()->GetCostmap();
//     cout<<"data_sta_infla: "<<endl<<data_sta_infla<<endl;
//     // Matrix data_sta_new = planner->GetPlannerMap()->GetStaticLayer()->GetCostmap();
//     // cout<<"data_sta_new: "<<endl<<data_sta_new<<endl;
//     // Matrix data_obs_new = planner->GetPlannerMap()->GetObstacleLayer()->GetCostmap();
//     // cout<<"data_obs_new: "<<endl<<data_obs_new<<endl;
//     PlannerMap* debug_map = planner->GetDWATraj()->visual_map_;
//     debug_map->UpdateObstacle(cur_pose);
//     planner->TransformGlobalPath();
//     // planner->PruneGlobalPath();  
//     // planner->VisualLocalRefPath();
//     DWATraj* dwa_traj = planner->GetDWATraj();
//     PlannerMap* plannermap = planner->GetPlannerMap();


//     for(uint32_t i = 0; i < 5002; i++){
//         cout<<"-------------------START-------------------"<<i<<endl;
//         //First step: update the plannermap every loop
//         cur_pose.x = planner->GetCurState().GetX();
//         cur_pose.y = planner->GetCurState().GetY();
//         cur_pose.th = planner->GetCurState().GetYaw();
//         double origin_x = debug_map->GetStaticLayer()->GetOriginX();
//         double origin_y = debug_map->GetStaticLayer()->GetOriginY();
//         double resolution = debug_map->GetStaticLayer()->GetResolution();
//         uint32_t row = (-cur_pose.y + origin_y) / resolution;
//         uint32_t col = (cur_pose.x - origin_x) / resolution;
//         debug_map->GetStaInflationLayer()->SetCost(row, col, 1);
//         plannermap->UpdateObstacle(cur_pose);
//         // debug_map->UpdateObstacle(cur_pose);
//         cout<<"cur_pose is : "<<cur_pose.x<<" "<<cur_pose.y<<" "<<cur_pose.th<<endl;
//         // Matrix data_sta = plannermap->GetStaticLayer()->GetCostmap();
//         // cout<<"data_sta: "<<endl<<data_sta<<endl;
//         // Matrix data_obs = plannermap->GetObstacleLayer()->GetCostmap();
//         // cout<<"data_obs: "<<endl<<data_obs<<endl;
//         // Matrix data_sta_infla = plannermap->GetStaInflationLayer()->GetCostmap();
//         // cout<<"data_sta_infla: "<<endl<<data_sta_infla<<endl;
//         // Matrix data_obs_infla = plannermap->GetObsInflationLayer()->GetCostmap();
//         // cout<<"data_obs_infla: "<<endl<<data_obs_infla<<endl;
//         //Second step: Prune the global path and generate trajectories
//         planner->PruneGlobalPath();
//         dwa_traj->TrajGenerate(planner->GetCurState());
//         dwa_traj->GetOptimalTraj();
//         dwa_traj->ComputeTwistCommand();

//         double v_cmd, omega_cmd;
//         dwa_traj->GetTwistCmd(v_cmd, omega_cmd);
//         //Last Step: Update the cur_pose
//         planner->MotionModelUpdate(v_cmd, omega_cmd);
//         if(planner->CheckGlobalGoal()){
//             cout<<"Goal Reached!"<<endl;
//             break;
//         }

//     // Matrix data_debug_sta = debug_map->GetStaInflationLayer()->GetCostmap();
//     // cout<<"data_debug_sta: "<<endl<<data_debug_sta<<endl;

// }



//Test 5.b: combine on arm
    clock_t start, end;
    Planner* planner = new Planner();
    Pose2dT cur_pose(-4.5, 4.0 , -1 * M_PI / 2);
    Pose2dT goal_pose(15.5, -17.5, 0);
    MotionModel start_state(-4.5, 4.0, -1 * M_PI / 2, 0, 0, 0.05);
    planner->SetCurrentState(start_state);
    planner->UpdateCurPose(cur_pose);
    planner->UpdateGoalPose(goal_pose);
    planner->UpdateStartPose(cur_pose);
    planner->GetPlannerMap()->UpdateObstacle(cur_pose);
    planner->SetStartGoal();

    start = clock();
    planner->UpdateGlobalPath();
    end = clock();
    cout<<"UpdateGlobalPath time: "<<1000 * (double)(end - start) / CLOCKS_PER_SEC << " ms"<<endl;

    planner->TransformGlobalPath();
    // planner->PruneGlobalPath();  
    // planner->VisualLocalRefPath();
    DWATraj* dwa_traj = planner->GetDWATraj();
    PlannerMap* plannermap = planner->GetPlannerMap();


    for(uint32_t i = 0; i < 5002; i++){
        cout<<"-------------------START-------------------"<<i<<endl;
        //First step: update the plannermap every loop
        cur_pose.x = planner->GetCurState().GetX();
        cur_pose.y = planner->GetCurState().GetY();
        cur_pose.th = planner->GetCurState().GetYaw();
        start = clock();
        plannermap->UpdateObstacle(cur_pose);
        end = clock();
        cout<<"UpdateObstacle time: "<<1000 * (double)(end - start) / CLOCKS_PER_SEC << " ms"<<endl;
        cout<<"cur_pose is : "<<cur_pose.x<<" "<<cur_pose.y<<" "<<cur_pose.th<<endl;

        //Second step: Prune the global path and generate trajectories
        planner->PruneGlobalPath();
        start = clock();
        dwa_traj->TrajGenerate(planner->GetCurState());
        dwa_traj->GetOptimalTraj();
        end = clock();
        cout<<"DWA time: "<<1000 * (double)(end - start) / CLOCKS_PER_SEC << " ms"<<endl;
        dwa_traj->ComputeTwistCommand();
        double v_cmd, omega_cmd;
        dwa_traj->GetTwistCmd(v_cmd, omega_cmd);
        //Last Step: Update the cur_pose
        planner->MotionModelUpdate(v_cmd, omega_cmd);
        if(planner->CheckGlobalGoal()){
            cout<<"Goal Reached!"<<endl;
            break;
        }


}

}


// return 0;

