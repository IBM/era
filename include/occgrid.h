#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#ifndef OCCGRID_H_
#define OCCGRID_H_

//Define Costmap values
#define NO_INFORMATION 254
#define FREE_SPACE 0
#define LETHAL_OBSTACLE 255

//Define datatype values
#define INT8    1;
#define UINT8   2;
#define INT16   3;
#define UINT16  4;
#define INT32   5;
#define UINT32  6;
#define FLOAT32 7;
#define FLOAT64 8;

//Define public structures
typedef struct Header {
    unsigned int seq;
    time_t stamp;
    char* frame_id;
} Header;

typedef struct geometry_msgs_Point {
    double x;
    double y;
    double z;
} Point;

typedef struct geomety_msgs_Quaternion {
    double x;
    double y;
    double z;
    double w;
} Quaternion;

typedef struct Vector3 {
    double x;
    double y;
    double z;
} Vector3;

typedef struct geometry_msgs_Pose {
    Point position;
    Quaternion orientation;
} Pose;

typedef struct geometry_msgs_Twist {
    Vector3 linear;
    Vector3 angular;
} Twist;

typedef struct Pointfield {
    char name;
    unsigned int offset;
    unsigned short datatype;
    unsigned int count_; //NOTE: 'count' can not be used
} Pointfield;

typedef struct geometry_msgs_Point32 {
    float x;
    float y;
    float z;
} Point32;

typedef struct ChannelFloat32 {
    char name;
    float* values[];
} ChannelFloat32;

typedef struct MapLocation {
    unsigned int x;
    unsigned int y;
} MapLocation;

typedef struct sensor_msgs_PointCloud {
    Header header;
    ChannelFloat32* channels[3]; //TODO: Flexible array has to be at end, fix array size
    Point32* points[];
} PointCloud;

typedef struct sensor_msgs_PointCloud2 {
    Header header;
    unsigned int height;
    unsigned int width;
    bool is_bigendian;
    unsigned int point_step;
    unsigned int row_step;
    bool is_dense;
    Pointfield* fields[3]; //TODO: Flexible array has to be at end, fix array size
    float data[15000]; //NOTE: Supposed to be int8 but does not exist in C
} PointCloud2;

typedef struct MapMetaData {
    time_t ap_load_time;
    float resolution;
    unsigned int width;
    unsigned int height;
    Pose origin;
} MapMetaData;

typedef struct nav_msgs_OccupancyGrid {
    Header header;
    MapMetaData info;
    unsigned short* data[]; //TODO: Test
} OccGrid;

typedef struct geometry_msgs_PoseWithCovariance {
    Pose pose;
    double* covariance[36];
} PoseWithCovariance;

typedef struct geometry_msgs_TwistWithCovariance {
    Twist twist;
    double* covariace[36];
} TwistWithCovariance;

typedef struct nav_msgs_Odometry {
    Header header;
    PoseWithCovariance pose;
    TwistWithCovariance twist;
    char* child_frame_id[]; //TODO: Test
} Odometry;

typedef struct Costmap2D {
    unsigned int size_x;
    unsigned int size_y;
    unsigned char default_value;
    unsigned char costmap_[2500];
} Costmap2D;

typedef struct Observation {
    bool rolling_window_;
    Point master_origin;
    float master_resolution;
    double max_obstacle_height_;
    double min_obstacle_height_;
    double raytrace_range_;
    MapLocation map_coordinates;
    Costmap2D master_costmap;
} Observation;

//Define global variables
Observation master_observation;
char data[199992];
bool rotating_window;

//Define functions
unsigned char* combineGrids(unsigned char* grid1, unsigned char* grid2, double robot_x1, double robot_y1,
                            double robot_x2, double robot_y2, unsigned int cell_size_x, unsigned int cell_size_y, double resolution, char def_val);

unsigned char* cloudToOccgrid(float* data, unsigned int data_size, double robot_x, double robot_y, double robot_z, double robot_yaw, bool rolling_window,
                              double min_obstacle_height, double max_obstacle_height, double raytrace_range, unsigned int size_x,
                              unsigned int size_y, double resolution, unsigned char default_value);

void updateMap(float* data, unsigned int data_size, double robot_x, double robot_y, double robot_z, double robot_yaw);

void updateOrigin(double new_origin_x, double new_origin_y);

void copyMapRegion(unsigned char* source_map, unsigned int sm_lower_left_x, unsigned int sm_lower_left_y,
                       unsigned int sm_size_x, unsigned int dm_lower_left_x,
                       unsigned int dm_lower_left_y, unsigned int dm_size_x, unsigned int region_size_x,
                       unsigned int region_size_y);

void updateBounds(float* data, unsigned int data_size, double robot_x, double robot_y, double robot_z, double robot_yaw, double min_x, double min_y, double max_x, double max_y);

void raytraceFreespace(float* data, unsigned int data_size, double min_x, double min_y, double max_x, double max_y, double robot_x, double robot_y, double robot_z, double robot_yaw);

bool worldToMap(double wx, double wy, double robot_x, double robot_y);

unsigned int cellDistance(double world_dist);

void raytraceLine(unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1, unsigned int max_length); //TODO: Default max argument for max_length is UNIT_MAX

void bresenham2D(unsigned int abs_da, unsigned int abs_db, int error_b, int offset_a,
                        int offset_b, unsigned int offset, unsigned int max_length);

void markCell(unsigned char value, unsigned int offset);

void updateRaytraceBounds(double ox, double oy, double wx, double wy, double range,
                          double min_x, double min_y, double max_x, double max_y);

void touch(double x, double y, double min_x, double min_y, double max_x, double max_y);

unsigned int getIndex(unsigned int x, unsigned int y);

void printMap();

void addStaticObstacle(unsigned char* obstacle_type);

void initCostmap(bool rolling_window, double min_obstacle_height, double max_obstacle_height, double raytrace_range, unsigned int size_x,
                 unsigned int size_y, double resolution, unsigned char default_value, double robot_x, double robot_y, double robot_z);

#endif // OCCGRID_H_
