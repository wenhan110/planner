#ifndef COSTMAP_TYPEDEFS_H
#define COSTMAP_TYPEDEFS_H

#include <eigen3/Eigen/Core>

using Matrix = Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>;
// using Matrix = Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic>;
using Matrixd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;
using Vector = Eigen::Matrix<uint8_t, Eigen::Dynamic, 1>;
using Vectord = Eigen::Matrix<double, Eigen::Dynamic, 1>;
using Time = uint64_t;


// typedef enum {
//   RESIZING_RIGHTUP = 1,
//   RESIZING_LEFTUP = 2,
//   RESIZING_LEFTDOWN = 3,
//   RESIZING_RIGHTDOWN = 4,
//   RESIZING_UP = 5,
//   RESIZING_LEFT = 6,
//   RESIZING_DOWN = 7,
//   RESIZING_RIGHT = 8
// } RESIZING_DIRECTION_ENUM;

// typedef enum {
//   ROLLING_QUADRANT_1 = 1,
//   ROLLING_QUADRANT_2 = 2,
//   ROLLING_QUADRANT_3 = 3,
//   ROLLING_QUADRANT_4 = 4,
// } ROLLING_DIRECTION_ENUM;

// static const uint8_t NO_INFORMATION = 255;
// static const uint8_t LETHAL_OBSTACLE = 255;
// static const uint8_t INSCRIBED_INFLATED_OBSTACLE = 254;
// static const uint8_t EDGE_COST = 253;
// static const uint8_t FREE_SPACE = 0;

static const int NO_INFORMATION = 255;
static const int LETHAL_OBSTACLE = 255;
static const int INSCRIBED_INFLATED_OBSTACLE = 254;
static const int EDGE_COST = 224;
static const int FREE_SPACE = 0;
// static const int WEIGHT_FOLLOW = 1;
// static const int WEIGHT_OBSTACLE = 1;
// static const int WEIGHT_TWRILING = 1;
// static const int WEIGHT_ACC = 1;

// static const double INFLATION_RADIUS = 0.6;

#define STATIC_MAP_ROWS 600
#define STATIC_MAP_COLS 600
#define STATIC_MAP_RESOLUTION 0.1
#define OBSTACLE_MAP_ROWS 500
#define OBSTACLE_MAP_COLS 500
#define OBSTACLE_MAP_RESOLUTION 0.02
#define NEED_ROLLING_DISTANCE 0.2

#define INFLATION_RADIUS 0.8
#define INSCRIBED_RADIUS 0.4
#define CIRCUMSCRIBED_RADIUS 0.5
#define INFLATION_WEIGHT 4.0

#define RESIZING_RIGHTUP 1
#define RESIZING_LEFTUP 2
#define RESIZING_LEFTDOWN 3
#define RESIZING_RIGHTDOWN 4
#define RESIZING_UP 5
#define RESIZING_LEFT 6
#define RESIZING_DOWN 7
#define RESIZING_RIGHT 8

#define ROLLING_QUADRANT_1 1
#define ROLLING_QUADRANT_2 2
#define ROLLING_QUADRANT_3 3
#define ROLLING_QUADRANT_4 4

// static const int STATIC_MAP_ROWS  = 60
// #define STATIC_MAP_COLS 60
// #define STATIC_MAP_RESOLUTION 0.1
// #define OBSTACLE_MAP_ROWS 50
// #define OBSTACLE_MAP_COLS 50
// #define OBSTACLE_MAP_RESOLUTION 0.02
// #define NEED_ROLLING_DISTANCE 2.0

// #define INFLATION_RADIUS 0.2
// #define INSCRIBED_RADIUS 0.1
// #define CIRCUMSCRIBED_RADIUS 0.5
// #define INFLATION_WEIGHT 4.0

// #define RESIZING_RIGHTUP 1
// #define RESIZING_LEFTUP 2
// #define RESIZING_LEFTDOWN 3
// #define RESIZING_RIGHTDOWN 4
// #define RESIZING_UP 5
// #define RESIZING_LEFT 6
// #define RESIZING_DOWN 7
// #define RESIZING_RIGHT 8

// #define ROLLING_QUADRANT_1 1
// #define ROLLING_QUADRANT_2 2
// #define ROLLING_QUADRANT_3 3
// #define ROLLING_QUADRANT_4 4


#endif  // COSTMAP_TYPEDEFS_H
