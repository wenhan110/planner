
#include "inflation_layer.h"
#include <algorithm>

InflationLayer::InflationLayer(CostmapBase *costmap)
    : inflation_radius_(INFLATION_RADIUS), weight_(INFLATION_WEIGHT), inscribed_radius_(INSCRIBED_RADIUS), seen_(NULL) {
  data_ = costmap->GetCostmap();
  label_ = costmap->GetLabelmap();
  resolution_ = costmap->GetResolution();
  origin_x_ = costmap->GetOriginX();
  origin_y_ = costmap->GetOriginY();
  size_col_ = costmap->GetCellSizeCol();
  size_row_ = costmap->GetCellSizeRow();
  Initialize();
}

void InflationLayer::Reset(CostmapBase *costmap){
  data_ = costmap->GetCostmap();
  label_ = costmap->GetLabelmap();
  resolution_ = costmap->GetResolution();
  origin_x_ = costmap->GetOriginX();
  origin_y_ = costmap->GetOriginY();
  size_col_ = costmap->GetCellSizeCol();
  size_row_ = costmap->GetCellSizeRow();
  // if(seen_) delete[] seen_;
  uint32_t size_x = data_.cols();
  uint32_t size_y = data_.rows();
  // seen_size_ = size_x * size_y;
  // seen_ = new bool[seen_size_];
  for(int i = 0; i < seen_size_; i++){
    seen_[i] = false;
  }
  // cout<<"reset ok"<<endl;
}

void InflationLayer::Initialize() {
  cell_inflation_radius_ = inflation_radius_ / resolution_;
  cached_cell_inflation_radius_ = cell_inflation_radius_ + 2;
  ComputeCaches();
  uint32_t size_x = data_.cols();
  uint32_t size_y = data_.rows();
  if (seen_) delete[] seen_;
  seen_size_ = size_x * size_y;
  seen_ = new bool[seen_size_];
}


void InflationLayer::UpdateMap(CostmapBase *costmap) {

  // cout<<"size of seen_ is: "<<seen_size_<<endl;
  Reset(costmap);
//   cout<<"ok"<<endl;
  unsigned int size_y = data_.rows();
  unsigned int size_x = data_.cols();
//   if(seen_ == NULL){
//     cout<<"seen_ is NULL"<<endl;
//   }
//   else{
//     cout<<"seen_ is not NULL"<<endl;
//   }
//   cout<<"seen_size is :" <<seen_size_<<endl;
//   cout<<"ok"<<endl;
  if (seen_ == NULL) {
    cout<<"seen_ is NULL"<<endl;
    seen_size_ = size_x * size_y;
    seen_ = new bool[seen_size_];
  } else if (seen_size_ != size_x * size_y) {
    // cout<<"ok"<<endl;
    delete[] seen_;
    seen_size_ = size_x * size_y;
    seen_ = new bool[seen_size_];
    // cout<<"ok"<<endl;
    // cout<<"size of seen_ is : "<<seen_size_<<endl;
  }
  // cout<<"ok"<<endl;
  std::vector<CellData> &obs_bin = inflation_cells_[0.0];
  for (int i = 0; i < data_.rows(); i++) {
    for (int j = 0; j < data_.cols(); j++) {
      if (data_(i, j) == LETHAL_OBSTACLE) {
        obs_bin.push_back(CellData(i * data_.cols() + j, j, i, j, i));
      }
    }
  }
  // cout<<"size of obs_bin is: "<<obs_bin.size()<<endl;
//   cout<<"ok"<<endl;

  std::map<double, std::vector<CellData> >::iterator bin;

//   cout<<"ok"<<endl;
  for (bin = inflation_cells_.begin(); bin != inflation_cells_.end(); bin++) {
    for (int i = 0; i < bin->second.size(); i++) {
      const CellData &cell = bin->second[i];
      unsigned int index = cell.index_;
    //   cout<<"index is : "<<i<<endl; 
      if (seen_[index]) {
        // cout<<"is seen!"<<endl;
        continue;
      }
      seen_[index] = true;
      unsigned int mx = cell.col_;
      unsigned int my = cell.row_;
      unsigned int src_x = cell.src_col_;
      unsigned int src_y = cell.src_row_;
      int cost = CostLookup(mx, my, src_x, src_y);
    //   cout<<"index is : "<<i<<endl;
      data_(my, mx) = std::max(data_(my, mx), cost);
    //   cout<<"index is : "<<i<<endl;
      if (my > 0) Enqueue(index - size_x, mx, my - 1, src_x, src_y);
    //   cout<<"index is : "<<i<<endl;

      if (my < size_y - 1) Enqueue(index + size_x, mx, my + 1, src_x, src_y);
      if (mx > 0) Enqueue(index - 1, mx - 1, my, src_x, src_y);
      if (mx < size_x - 1) Enqueue(index + 1, mx + 1, my, src_x, src_y);
    //   cout<<"index is : "<<i<<endl;
    }
  }
  // delete[] seen_;
  inflation_cells_.clear();
//   cout<<"ok"<<endl;
}

inline void InflationLayer::Enqueue(uint32_t index, uint32_t mx, uint32_t my, uint32_t src_x, uint32_t src_y) {
  if (!seen_[index]) {
    double distance = DistanceLookup(mx, my, src_x, src_y);
    // cout<<"ok"<<endl;
    if (distance > cell_inflation_radius_) return;
    inflation_cells_[distance].push_back(CellData(index, mx, my, src_x, src_y));
    // cout<<"ok"<<endl;
  }
}

void InflationLayer::ComputeCaches() {
  if (cell_inflation_radius_ == 0) return;
  cached_costs_.resize(cached_cell_inflation_radius_, cached_cell_inflation_radius_);
  cached_distances_.resize(cached_cell_inflation_radius_, cached_cell_inflation_radius_);
  for (int i = 0; i < cached_cell_inflation_radius_; i++) {
    for (int j = 0; j < cached_cell_inflation_radius_; j++) {
      cached_distances_(i, j) = hypot(i, j);
      cached_costs_(i, j) = ComputeCost(cached_distances_(i, j));
    }
  }
}
