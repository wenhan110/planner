#include "dwa_traj.h"
#include "float.h"
#include <queue>

DWATraj::DWATraj(PlannerMap* plannermap, uint32_t traj_num, uint32_t pred_steps, double pred_time, double model_update_dt):
                traj_num_(traj_num), pred_steps_(pred_steps), pred_time_(pred_time), model_update_dt_(model_update_dt){
    // local_map_ = new PlannerMap(false); 
    
    visual_map_ = new PlannerMap(false);
    SetMapPtr(plannermap);
    visual_map_list_.resize(500);
    for(uint32_t i = 0; i < 500; i ++){
        visual_map_list_[i] = new PlannerMap(false);
    }
    weight_follow_ = 1;
    weight_obs_ = 10;
    weight_goal_ = 1.0;
    weight_smooth_ = 1.0;
    weight_twirling_ = 1.0;
    isValidTraj_ = false;
    MotionModel cur_state_(0.0, 0.0, 0.0, 0.0, 0.0, 0.1);
}

DWATraj::~DWATraj(){
    // delete local_map_;
    // delete visual_map_;
    pred_traj_.clear();
    input_seq_list_.clear();
    traj_cost_list_.clear();
    optimal_traj_.clear();
}

void DWATraj::SetMapPtr(PlannerMap* plannermap){
    obs_layer_ = plannermap->GetObstacleLayer();
    obs_inflation_layer_ = plannermap->GetObsInflationLayer();
}


void DWATraj::TrajGenerate(double vCur, double omegaCur, double xCur, double yCur, double yawCur){
    // isValidTraj_ = false;
    vCur_ = vCur;
    omegaCur_ = omegaCur;
    cur_pose_.x = xCur;
    cur_pose_.y = yCur;
    cur_pose_.th = yawCur;
    // cout<<"ok"<<endl;

    UpdateInputRange();
    GenerateInputSequence(input_seq_list_);  //121 inputs
    for(uint32_t i = 0; i < input_seq_list_.size(); i++){
        // GenerateInputSequence(input_seq_list_);
        std::vector<MotionModel> traj_tmp;
        // cout<<"i: "<< i << "ok"<<endl;
        TrajPredict(input_seq_list_[i].lin_spd, input_seq_list_[i].ang_spd, traj_tmp);
        ComputeCost(traj_tmp, traj_cost_list_);
        pred_traj_.push_back(traj_tmp);
    }
    // cout<<"pred_traj_ size: "<<pred_traj_.size()<<endl;
    // cout<<"traj_cost_list_ size: "<<traj_cost_list_.size()<<endl;
}


void DWATraj::TrajGenerate(MotionModel& cur_state){
    clock_t start, end;
    pred_traj_.clear();
    input_seq_list_.clear();
    traj_cost_list_.clear();
    vCur_ = cur_state.GetVel();
    omegaCur_ = cur_state.GetOmega();
    cur_pose_.x = cur_state.GetX();
    cur_pose_.y = cur_state.GetY();
    cur_pose_.th = cur_state.GetYaw();
    // cout<<"cur_pose_ is: "<<cur_pose_.x<<", "<<cur_pose_.y<<", "<<cur_pose_.th<<endl;
    start = clock();
    UpdateInputRange();
    GenerateInputSequence(input_seq_list_);  //121 inputs
    for(uint32_t i = 0; i < input_seq_list_.size(); i++){
        // GenerateInputSequence(input_seq_list_);
        std::vector<MotionModel> traj_tmp;
        // cout<<"i: "<< i << "ok"<<endl;
        clock_t start_loop, end_loop;
        start_loop = clock();
        TrajPredict(input_seq_list_[i].lin_spd, input_seq_list_[i].ang_spd, traj_tmp);
        end_loop = clock();
        cout<<"TrajPredict time is: "<<1000 * (double)(end_loop - start_loop)/CLOCKS_PER_SEC<<" ms"<<endl;
        start_loop = clock();
        ComputeCost(traj_tmp, traj_cost_list_);
        end_loop = clock();
        cout<<"ComputeCost time is: "<<1000 * (double)(end_loop - start_loop)/CLOCKS_PER_SEC<<" ms"<<endl;
        pred_traj_.push_back(traj_tmp);
        // cout<<endl;
    }
    end = clock();
    cout<<"TrajPredict and compute cost time is: "<<1000 * (double)(end - start)/CLOCKS_PER_SEC<<" ms"<<endl;
    cout<<"pred_traj_ size: "<<pred_traj_.size()<<endl;
    // cout<<"traj_cost_list_ size: "<<traj_cost_list_.size()<<endl;
}

void DWATraj::GetOptimalTraj(){
    uint32_t minIndex = std::min_element(traj_cost_list_.begin(), traj_cost_list_.end()) - traj_cost_list_.begin();
    if(traj_cost_list_[minIndex] > DBL_MAX){
        isValidTraj_ = false;
        cout<<"No valid trajectory!"<<endl;
    }
    else{
        isValidTraj_ = true;
        optimal_traj_ = pred_traj_[minIndex];
        double v_opt = optimal_traj_[0].GetVel();
        double omega_opt = optimal_traj_[0].GetOmega();
        cout<<"optimal index is: "<<minIndex<<" ";
        cout<<"optimal_cost is : "<<traj_cost_list_[minIndex]<<" ";
        cout<<"optimal size is : "<<optimal_traj_.size()<<endl;
        cout<<"optimal v is: "<<v_opt<<" ";
        cout<<"optimal omega is: "<<omega_opt<<endl;
    }
}

void DWATraj::GetOptimalControl(std::queue<CTwist2D>& queue){
    if(optimal_traj_.empty()){
        cout<<"optimal_traj_ is empty!"<<endl;
        return;
    }
    for(uint32_t i = 0; i < optimal_traj_.size(); i++){
        CTwist2D twist;
        twist.lin_spd = optimal_traj_[i].GetVel();
        twist.ang_spd = 0.0;
        queue.push(twist);
    }
    cout<<"optimal control size is: "<<queue.size()<<endl;
}

void DWATraj::UpdateOptimalCmd(){
    if(optimal_traj_.empty()){
        cout<<"optimal_traj_ is empty!"<<endl;
        return;
    }
    double lamba = 10000;
    double sum_weight, sum_vel, sum_omega = 0;
    for(uint32_t i = 0; i < pred_traj_.size(); i++){
        double cost = traj_cost_list_[i];
        // cout<<"cost is: "<<cost<<" ";
        sum_weight += exp(-cost / lamba);
        sum_vel += exp(-cost / lamba) * pred_traj_[i][0].GetVel();
        sum_omega += exp(-cost / lamba) * pred_traj_[i][0].GetOmega();
    }
    cout<<"sum_weight is: "<<sum_weight<<" ";
    v_cmd_ = sum_vel / sum_weight;
    omega_cmd_ = sum_omega / sum_weight;
    cout<<endl;
}

void DWATraj::ComputeTwistCommand(){
    if(optimal_traj_.empty()){
        cout<<"optimal_traj_ is empty!"<<endl;
        return;
    }
    v_cmd_ = optimal_traj_[0].GetVel();
    omega_cmd_ = optimal_traj_[0].GetOmega();
}

void UpdateState(const double& v, const double& omega){
    
} ///todo

void DWATraj::UpdateInputRange(){
    float vCur_min = vCur_ - MAX_LINEAR_ACC * pred_time_;
    float vCur_max = vCur_ + MAX_LINEAR_ACC * pred_time_;
    float omegaCur_min = omegaCur_ - MAX_ANGULAR_ACC * pred_time_;
    float omegaCur_max = omegaCur_ + MAX_ANGULAR_ACC * pred_time_;   // todo : determine the range
    
    v_list_max_ = std::min(MAX_LINEAR, vCur_max);
    v_list_min_ = std::max(MIN_LINEAR, vCur_min);
    // cout<<"v_list_max_: "<<v_list_max_<<endl;
    // cout<<"v_list_min_: "<<v_list_min_<<endl;
    // cout<<"ok"<<endl;
    omega_list_max_ = std::min(MAX_ANGULAR, omegaCur_max);
    omega_list_min_ = std::max(MIN_ANGULAR, omegaCur_min);
    cout<<"omega_list_max_: "<<omega_list_max_<<endl;
    cout<<"omega_list_min_: "<<omega_list_min_<<endl;

    // cout<<"ok"<<endl;
}

void DWATraj::GenerateInputSequence(std::vector<CTwist2D>& list){
    if(!list.empty()){
        list.clear();
    }
    // cout<<"size of list: "<<list.size()<<endl;
    float v_fold_ = (v_list_max_ - v_list_min_) / (sqrt(traj_num_) - 1);
    float dYaw_fold_ = (omega_list_max_ - omega_list_min_) / (sqrt(traj_num_) - 1);
    // cout<<"v_fold_: "<<v_fold_<<endl;
    // cout<<"dYaw_fold_: "<<dYaw_fold_<<endl;
    float v_tmp = v_list_min_;
    // float omega_tmp = omega_list_min_;     //todo: how to determine the distribution of the input sequence
    while(v_tmp <= v_list_max_){
        float omega_tmp = omega_list_min_;
        // cout<<"yes"<<endl;
        while(omega_tmp <= omega_list_max_){
            // cout<<"omega_tmp: "<<omega_tmp<<endl;
            CTwist2D twist_tmp;
            twist_tmp.lin_spd = v_tmp;
            twist_tmp.ang_spd = omega_tmp;
            // cout<<"v_tmp: "<<v_tmp<<" omega_tmp: "<<omega_tmp<<endl;
            list.push_back(twist_tmp);
            omega_tmp += dYaw_fold_;
        }
        v_tmp += v_fold_;
        // cout<<"v_tmp_update: "<<v_tmp<<endl;
    }
    // cout<<"input sequence size: "<<list.size()<<endl;
}

void DWATraj::TrajPredict(const double& v, const double& omega, std::vector<MotionModel>& traj){
    // cout<<"v: "<<v<<" omega: "<<omega<<endl;
    traj.clear();
    MotionModel state_tmp(cur_pose_.x, cur_pose_.y, cur_pose_.th, v, omega, model_update_dt_);
    // cout<<"state_tmp: "<<state_tmp.GetX()<<" "<<state_tmp.GetY()<<" "<<state_tmp.GetYaw()<<endl;
    // cout<<"ok"<<endl;
    double resolution = obs_inflation_layer_->GetResolution();
    // cout<<"resolution: "<<resolution<<endl;
    double origin_x = obs_inflation_layer_->GetOriginX();
    double origin_y = obs_inflation_layer_->GetOriginY();
    double right_x = origin_x + obs_inflation_layer_->GetCellSizeCol() * resolution;
    // cout<<"cellsize: "<<local_map_->GetObsInflationLayer()->GetCellSizeCol()<<endl;
    double low_y = origin_y - obs_inflation_layer_->GetCellSizeRow() * resolution;
    // cout<<"origin_x: "<<origin_x<<endl;
    // cout<<"origin_y: "<<origin_y<<endl;
    // cout<<"right_x: "<<right_x<<endl;
    // cout<<"low_y: "<<low_y<<endl;
    for(int i = 0; i < pred_steps_; i++){
        traj.push_back(state_tmp);
        MotionModelUpdate(state_tmp, v, omega);
        // cout<<"state_tmp: "<<state_tmp.GetX()<<" "<<state_tmp.GetY()<<" "<<state_tmp.GetYaw()<<endl;
        if(state_tmp.GetX() <= origin_x || state_tmp.GetX() >= right_x || state_tmp.GetY() >= origin_y || state_tmp.GetY() <= low_y){
            // cout<<"out of map!"<<endl;
            break;
        }
        // traj.push_back(state_tmp);
    }
    // cout<<"traj size: "<<traj.size()<<endl;
}

void DWATraj::MotionModelUpdate(MotionModel &state, const double& v, const double& omega){
    state.UpdateWithCircleModel(v, omega);
    // state.UpdateWithSL(v, omega);
    // state.UpdateWithTL(v, omega);
    // cout<<"update success"<<endl;
}

void DWATraj::ComputeCost(std::vector<MotionModel>& traj, std::vector<double>& cost_list){
    double origin_x = obs_inflation_layer_->GetOriginX();
    double origin_y = obs_inflation_layer_->GetOriginY();
    double resolution = obs_inflation_layer_->GetResolution();
    double cost = 0;
    for(uint32_t i = 0; i < traj.size(); i++){
        double x_tmp = traj[i].GetX();
        double y_tmp = traj[i].GetY();
        // cout<<"x_tmp: "<<x_tmp<<" y_tmp: "<<y_tmp<<" ";
        uint32_t cell_col = (int)((x_tmp - origin_x) / resolution);
        uint32_t cell_row = (int)((-y_tmp + origin_y) / resolution);
        // cout<<"cell_col: "<<cell_col<<" cell_row: "<<cell_row<<endl;
        // Matrix data_obs_infla = obs_inflation_layer_->GetCostmap();
        // cout<<"data_obs_infla: "<<data_obs_infla<<endl;
        int data = obs_inflation_layer_->GetCost(cell_row, cell_col);
        // cout<<"data: "<<data<<"   ";
        double edge_cost = (data - EDGE_COST) * (data - EDGE_COST);
        // cout<<"edge_cost: "<<edge_cost<<" ";
        // cout<<"result is: "<<1.0 / (data - LETHAL_OBSTACLE)<<endl;
        // double obs_cost = 1.0 / abs(data - LETHAL_OBSTACLE ) + 1.0 / abs(data - INSCRIBED_INFLATED_OBSTACLE );
        double obs_cost = 1.0 / abs(data - LETHAL_OBSTACLE + 1e-5) + 1.0 / abs(data - INSCRIBED_INFLATED_OBSTACLE + 1e-5);
        // cout<<"obs_cost: "<<obs_cost<<" ";
        // cout <<"result is: "<<LETHAL_OBSTACLE<<endl;
        double twirling_cost = (traj[i].GetYaw() - traj[i].GetYawLast()) * (traj[i].GetYaw() - traj[i].GetYawLast());

        double tmp_cost = weight_follow_ * double(edge_cost) + weight_obs_ * obs_cost + weight_twirling_ * twirling_cost;
        // cout<<"weight_follow_: "<<weight_follow_<<" weight_obs_: "<<weight_obs_<<endl;
        // cout<<"tmp_cost: "<<tmp_cost<<endl;
        cost += tmp_cost;
    }
    uint32_t last_index = traj.size() - 1;
    double goal_cost = weight_goal_ * ((traj[last_index].GetX() - global_goal_point_.x) * (traj[last_index].GetX() - global_goal_point_.x) + (traj[last_index].GetY() - global_goal_point_.y) * (traj[last_index].GetY() - global_goal_point_.y));
    cost += goal_cost;
    cost = cost / traj.size();
    // cout<<"cost: "<<cost<<"  ";
    cost_list.push_back(cost);
}


void DWATraj::VisualizeOptimalTraj(){
    if(optimal_traj_.empty()){
        cout<<"optimal_traj_ is empty!"<<endl;
        return;
    }
    for(uint32_t i = 0; i < optimal_traj_.size(); i++){
        double x = optimal_traj_[i].GetX();
        double y = optimal_traj_[i].GetY();
        // cout<<"x: "<<x<<" y: "<<y<<endl;
        double origin_x = visual_map_->GetObsInflationLayer()->GetOriginX();
        double origin_y = visual_map_->GetObsInflationLayer()->GetOriginY();
        // cout<<"origin_x: "<<origin_x<<" origin_y: "<<origin_y<<endl;
        double resolution = visual_map_->GetObsInflationLayer()->GetResolution();
        uint32_t cell_col = (int)((x - origin_x) / resolution);
        uint32_t cell_row = (int)((-y + origin_y) / resolution);
        // cout<<"cell_col: "<<cell_col<<" cell_row: "<<cell_row<<endl;
        visual_map_->GetObsInflationLayer()->SetCost(cell_row, cell_col, 1);
    }
    Matrix visual_optim_data = visual_map_->GetObsInflationLayer()->GetCostmap();
    cout<<"visual_optim_traj_data: "<<endl<<visual_optim_data<<endl;
}

void DWATraj::VisualizeTraj(){
    if(pred_traj_.empty()){
        cout<<"pred_traj_ is empty!"<<endl;
        return;
    }
    for(uint32_t i = 0; i < visual_map_->GetObsInflationLayer()->GetCellSizeRow(); i++){
        for(uint32_t j = 0; j < visual_map_->GetObsInflationLayer()->GetCellSizeCol(); j++){
            if(visual_map_->GetObsInflationLayer()->GetCost(i, j) == 2)
                visual_map_->GetObsInflationLayer()->SetCost(i, j, 0);
            // visual_map_->GetObsInflationLayer()->SetCost(i, j, 0);
        }
    }
    for(uint32_t i = 0; i < pred_traj_.size(); i++){
        for(uint32_t j = 0; j < pred_traj_[i].size(); j++){
            double x = pred_traj_[i][j].GetX();
            double y = pred_traj_[i][j].GetY();
            // cout<<"x: "<<x<<" y: "<<y;;
            double origin_x = visual_map_->GetObsInflationLayer()->GetOriginX();
            double origin_y = visual_map_->GetObsInflationLayer()->GetOriginY();
            // cout<<"origin_x: "<<origin_x<<" origin_y: "<<origin_y<<endl;
            double resolution = visual_map_->GetObsInflationLayer()->GetResolution();
            uint32_t cell_col = (int)((x - origin_x) / resolution);
            uint32_t cell_row = (int)((-y + origin_y) / resolution);
            // if(visual_map_->GetObsInflationLayer()->GetData(cell_row, cell_col) != LETHAL_OBSTACLE && visual_map_->GetObsInflationLayer()->GetData(cell_row, cell_col) != INSCRIBED_INFLATED_OBSTACLE){
            //     visual_map_->GetObsInflationLayer()->SetCost(cell_row, cell_col, 2);
            // }
            visual_map_->GetObsInflationLayer()->SetCost(cell_row, cell_col, 2);
        }
        // cout<<endl;
    }
    Matrix visual_data = visual_map_->GetObsInflationLayer()->GetCostmap();
    cout<<"visual_data: "<<endl<<visual_data<<endl;
}

void DWATraj::VisualizeTraj(int index){
    if(pred_traj_.empty()){
        cout<<"pred_traj_ is empty!"<<endl;
        return;
    }
    for(uint32_t i = 0; i < pred_traj_.size(); i++){
        for(uint32_t j = 0; j < pred_traj_[i].size(); j++){
            cout<<"i, j is : "<<i<<" "<< j<<endl;
            double x = pred_traj_[i][j].GetX();
            double y = pred_traj_[i][j].GetY();
            cout<<"x: "<<x<<" y: "<<y<<endl;

            double origin_x = visual_map_list_[index]->GetObsInflationLayer()->GetOriginX();
            double origin_y = visual_map_list_[index]->GetObsInflationLayer()->GetOriginY();
            double resolution = visual_map_list_[index]->GetObsInflationLayer()->GetResolution();
            uint32_t cell_col = (int)((x - origin_x) / resolution);
            uint32_t cell_row = (int)((-y + origin_y) / resolution);
            cout<<"cell_col: "<<cell_col<<" cell_row: "<<cell_row<<endl;
            visual_map_list_[index]->GetObsInflationLayer()->SetCost(cell_row, cell_col, 2);
        }
    }
    Matrix visual_data = visual_map_list_[index]->GetObsInflationLayer()->GetCostmap();
    cout<<"visual_data of steps "<<index<<" is: "<<endl<<visual_data<<endl;
}

void DWATraj::DrawOptimalTraj(){
    if(optimal_traj_.empty()){
        cout<<"optimal_traj_ is empty!"<<endl;
        return;
    }
    for(uint32_t i = 0; i < 2; i ++){
        double x = optimal_traj_[i].GetX();
        double y = optimal_traj_[i].GetY();
        double origin_x = visual_map_->GetObsInflationLayer()->GetOriginX();
        double origin_y = visual_map_->GetObsInflationLayer()->GetOriginY();
        double resolution = visual_map_->GetObsInflationLayer()->GetResolution();
        uint32_t cell_col = (int)((x - origin_x) / resolution);
        uint32_t cell_row = (int)((-y + origin_y) / resolution);
        visual_map_->GetObsInflationLayer()->SetCost(cell_row, cell_col, 1);
    }
    Matrix visual_optim_data = visual_map_->GetObsInflationLayer()->GetCostmap();
    cout<<"visual_optim_data: "<<endl<<visual_optim_data<<endl;
}
