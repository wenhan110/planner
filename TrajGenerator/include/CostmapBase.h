#ifndef COSTMAP_COSTMAPBASE_H
#define COSTMAP_CODTMAPBASE_H

// #include <Eigen/Core>
// #include <Eigen/Dense>
 
#include <iostream>
#include "Typedefs.h"

using namespace std;

struct MapStruct {
  Matrix data_;
  Matrix label_;
  float resolution_;
  Time last_update_utc_time_;
  double origin_x_;
  double origin_y_;
};

class CostmapBase {
 public:
  CostmapBase();
  virtual ~CostmapBase() {}
  virtual void Initialize() {}

  double GetResolution() { return resolution_; }

  double GetOriginX() { return origin_x_; }

  double GetOriginY() { return origin_y_; }

  Matrix GetCostmap() { return data_; }

  Matrix GetLabelmap() { return label_; }

  uint8_t GetData(unsigned int x, unsigned int y) { return data_(x, y); }

  uint8_t GetLabel(unsigned int x, unsigned int y) { return label_(x, y); }

  uint32_t GetCellSizeRow() { return size_row_; }

  uint32_t GetCellSizeCol() { return size_col_; }

  void SetCost(uint32_t row, uint32_t col, uint8_t cost) { data_(row, col) = cost; }

  void Write2Binary(FILE *file, const MapStruct &map);

  void ReadFromBinary(FILE *file, MapStruct &map);

  uint8_t GetCost(uint32_t row, uint32_t col) { return data_(row, col); }


 protected:

  MapStruct map_struct_;
  double size_row_, size_col_;

  double resolution_, origin_x_, origin_y_;
  bool first_mapping_flag_;
  Matrix data_;
  Matrix label_;
  unsigned char default_value_;
};

#endif  // COSTMAP_COSTMAPBASE_H