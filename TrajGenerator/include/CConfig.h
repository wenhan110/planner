//
// Created by bacon on 22-9-2.
//

#ifndef SRC_CCONFIG_H
#define SRC_CCONFIG_H
#ifdef __cplusplus
extern "C" {
#endif

#define MATLAB_FREQ       (10)
#define M_PI_F 3.141592653589793f
#define MAX_SCAN_POINTS   1024
#define BUMPER_ANG_RANGE  ( 20.0f * M_PI_F / 180.0f )
#define BUMPER_ANG_L      ( 45.0f * M_PI_F / 180.0f  - BUMPER_ANG_RANGE / 2)
#define BUMPER_ANG_R      (-45.0f * M_PI_F / 180.0f  - BUMPER_ANG_RANGE / 2)
#define BUMPER_ANG_M      (  0.0f * M_PI_F / 180.0f  - BUMPER_ANG_RANGE / 2)
#define BUMPER_TRIG_RANGE (0.07)
#define DIR_NORTH         (0)
#define DIR_EAST          (-M_PI_F/2)
#define DIR_SOUTH         (M_PI_F)
#define DIR_WEST          (M_PI_F/2)
#define LINE_SPACE        (0.40f)
#define ROBOT_WIDTH        0.36
#define ROBOT_LENGTH       0.45
#define LOCAL_MAP_MAX_SIZE  (512*512)
#define CLOUD_MAX_SIZE      (2560)
#define LOCAL_PLANNER_PATHS       17     // how manny paths to generate
#define LOCAL_PLANNER_PRED_TIME   1.0
#define LOCAL_PLANNER_SLICES      6
#define LOCAL_PLANNER_MAX_LINEAR  0.3
#define LOCAL_PLANNER_MAX_ANGULAR 1.5
#define ROBOT_SIDE_LEFT  ( 1)
#define ROBOT_SIDE_RIGHT (-1)
#define ANGLE_SPEED_P    (2)
#define SQUARE_WIDTH     (15)
#define TMP_TEST_PID_SPIN 0

#define STATIC_MAP_ROWS 600
#define STATIC_MAP_COLS 600
#define STATIC_MAP_RESOLUTION 0.1
#define OBSTACLE_MAP_ROWS 500
#define OBSTACLE_MAP_COLS 500
#define OBSTACLE_MAP_RESOLUTION 0.02

#ifdef __cplusplus
};
#endif

#endif //SRC_CCONFIG_H
