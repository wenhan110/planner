#ifndef COSTMAP_INFLATION_LAYER_H
#define COSTMAP_INFLATION_LAYER_H

#include <map>
#include <vector>

// #include "CostmapBase.h"

#include "obstacle_layer.h"

class CellData {
 public:
  CellData(double i, unsigned int col, unsigned int row, unsigned int s_col, unsigned int s_row)
      : index_(i), row_(row), col_(col), src_row_(s_row), src_col_(s_col) {}

  unsigned int index_;
  unsigned int row_, col_, src_row_, src_col_;
};

class InflationLayer : public CostmapBase {
 public:
  InflationLayer(CostmapBase *costmap);

  void Initialize();

  void Reset(CostmapBase *costmap);
  inline uint32_t ComputeCost(int distance_cell) const {
    uint8_t cost = 0;
    if (distance_cell == 0)
      cost = LETHAL_OBSTACLE;
    else if (distance_cell * resolution_ <= inscribed_radius_)
      cost = INSCRIBED_INFLATED_OBSTACLE;
    else {
      double euclidean_distance = distance_cell * resolution_;
      double factor = exp(-1.0 * weight_ * (euclidean_distance - inscribed_radius_));
      cost = (uint8_t)((INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
    }
    return cost;
  }

  void UpdateMap(CostmapBase *costmap);

 private:
  inline double DistanceLookup(int mx, int my, int src_x, int src_y) {
    uint32_t dcol = abs(mx - src_x);
    uint32_t drow = abs(my - src_y);
    return cached_distances_(drow, dcol);
  }

  inline uint8_t CostLookup(int mx, int my, int src_x, int src_y) {
    uint32_t dcol = abs(mx - src_x);
    uint32_t drow = abs(my - src_y);
    return cached_costs_(drow, dcol);
  }

  void ComputeCaches();
  // void DeleteKernels();

  // CostmapBase *costmap_;

  inline void Enqueue(uint32_t index, uint32_t mx, uint32_t my, uint32_t src_x, uint32_t src_y);

  double inflation_radius_, inscribed_radius_, circumscribed_radius_, weight_;
  unsigned int cell_inflation_radius_, cached_cell_inflation_radius_;

  std::map<double, std::vector<CellData> > inflation_cells_;
  bool *seen_;
  int seen_size_;
  // Vector cached_costs_;
  // Vectord cached_distances_;
  Matrix cached_costs_;
  Matrixd cached_distances_;
};

#endif  // COSTMAP_INFLATION_LAYER_H