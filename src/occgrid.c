#include "occgrid.h"

/*************** HELPER FUNCTIONS ******************/

double hypot(double x, double y) {
    return sqrt(x * x + y * y);
}

int max(int num1, int num2) {
    return (num1 > num2) ? num1 : num2;
}

int min(int num1, int num2) {
    return (num1 < num2) ? num1 : num2;
}

int sign (int x) {
    return x > 0 ? 1.0 : -1.0;
}

int abs(int x) {
    if (x < 0.0) return x * -1.0;
    else return x;
}

double getSizeInMetersX() {
  return (master_observation.master_costmap.size_x - 1 + 0.5) * master_observation.master_resolution;
}

double getSizeInMetersY() {
  return (master_observation.master_costmap.size_y - 1 + 0.5) * master_observation.master_resolution;
}

unsigned int getIndex(unsigned int i, unsigned int j) {
    //printf("size_x * j + i = %d * %d + %d = %d", master_observation.master_costmap.size_x, j, i, master_observation.master_costmap.size_x * j + i);
    return (master_observation.master_costmap.size_x / master_observation.master_resolution) * j + i;
}

void printMap() {
    printf("map: \n");
    for (int i = 0; i < master_observation.master_costmap.size_y / master_observation.master_resolution; i++) {
        for (int j = 0; j < master_observation.master_costmap.size_x / master_observation.master_resolution; j++) {
            int index = i * master_observation.master_costmap.size_y / master_observation.master_resolution + j;
            printf("%4d", master_observation.master_costmap.costmap_[index]);
        }
        printf("\n\n");
    }
}

void addStaticObstacle(unsigned char* obstacle_type) {
    int cell_size_x = master_observation.master_costmap.size_x / master_observation.master_resolution;
    int cell_size_y = master_observation.master_costmap.size_y / master_observation.master_resolution;
    if (obstacle_type == "border") {
        printf("Add Static Obstacle: Wall Border \n");
        for (int i = 0; i < cell_size_x; i++) {
            for (int j = 0; j < cell_size_y; j++) {
                int index = cell_size_x * j + i;
                if (i == (int) master_observation.master_origin.x || i == cell_size_x - 1) master_observation.master_costmap.costmap_[index] = LETHAL_OBSTACLE;
                else if (j == (int) master_observation.master_origin.y || j == cell_size_y - 1 ) master_observation.master_costmap.costmap_[index] = LETHAL_OBSTACLE;
            }
        }
    }
}

void initCostmap(bool rolling_window, double min_obstacle_height, double max_obstacle_height, double raytrace_range, unsigned int size_x,
                 unsigned int size_y, double resolution, unsigned char default_value, double robot_x, double robot_y, double robot_z) {
    printf("Initialize Master Costmap\n");

    master_observation.rolling_window_ = rolling_window; //TODO:
    master_observation.min_obstacle_height_ = min_obstacle_height; //TODO:
    master_observation.max_obstacle_height_ = max_obstacle_height; //TODO:
    master_observation.raytrace_range_ = raytrace_range; //TODO:

    master_observation.master_costmap.size_x = size_x;
    master_observation.master_costmap.size_y = size_y;
    master_observation.master_costmap.default_value = default_value;

    master_observation.master_resolution = resolution;

    master_observation.master_origin.x = robot_x - (size_x - 1) / 2;
    master_observation.master_origin.y = robot_y - (size_y - 1) / 2;
    master_observation.master_origin.z = robot_z;
    printf("Master Origin -> <%f, %f, %f>\n", master_observation.master_origin.x, master_observation.master_origin.y, master_observation.master_origin.z);

    for (int i = 0; i < master_observation.master_costmap.size_x * master_observation.master_costmap.size_y / (master_observation.master_resolution * master_observation.master_resolution); i++) {
        master_observation.master_costmap.costmap_[i] = master_observation.master_costmap.default_value;
    }

    printf("Initialize Master Costmap ... DONE\n\n");
}

/******************* FUNCTIONS *********************/

unsigned char* combineGrids(unsigned char* grid1, unsigned char* grid2, double robot_x1, double robot_y1,
                            double robot_x2, double robot_y2, unsigned int cell_size_x, unsigned int cell_size_y, double resolution, char def_val){
    //TODO: Input parameters needed: size of map, resolution, default value
    //TODO: Change size_x to cell_size_x
    //grid1 is previous map, grid2 is current map

    //Calculate the new origin of the map
    double new_origin_x = robot_x2 - (cell_size_x - 1) / 2.0;
    double new_origin_y = robot_y2 - (cell_size_y - 1) / 2.0;

    //Calculate the old origin of the map
    double origin_x = robot_x1 - (cell_size_x - 1) / 2;
    double origin_y = robot_y1 - (cell_size_y - 1) / 2;

    //Calculate the number of cells between the old and new origin
    unsigned int cell_ox = ((new_origin_x - origin_x) / resolution);
    unsigned int cell_oy = ((new_origin_y - origin_y) / resolution);

    //Determine the lower left cells of the origin
    unsigned int g1_lower_left_x = min(max(cell_ox, 0), cell_size_x);
    unsigned int g1_lower_left_y = min(max(cell_oy, 0), cell_size_y);
    unsigned int g2_lower_left_x = g1_lower_left_x - cell_ox;
    unsigned int g2_lower_left_y = g1_lower_left_y - cell_oy;

    //Calculate the indexes of which to start 'copying' over
    unsigned int g1_index = (g1_lower_left_y * cell_size_x + g1_lower_left_x);
    unsigned int g2_index = (g2_lower_left_y * cell_size_x + g2_lower_left_x);

    //The size of the overlapping region
    unsigned int region_size_x = cell_size_x - cell_ox;
    unsigned int region_size_y = cell_size_y - cell_oy;

    printf("Lower Left of Old Map = (%d, %d) \n", g1_lower_left_x, g1_lower_left_y);
    printf("Lower Left of New Map = (%d, %d) \n", g2_lower_left_x, g2_lower_left_y);
    printf("Index of Old Map, Index of New Map = %d, %d \n", g1_index, g2_index);
    printf("Dimensions of Overlapping Region = (%d, %d) \n", region_size_x, region_size_y);

    printf("OLD map: \n");
    for (int i = 0; i < 10; i++) {
        for (int j = 0; j < 10; j++) {
            int index = i * 10 + j;
            printf("%4d", *(grid1 + index));
        }
        printf("\n\n");
    }

    printf("NEW map: \n");
    for (int i = 0; i < 10; i++) {
        for (int j = 0; j < 10; j++) {
            int index = i * 10 + j;
            printf("%4d", *(grid2 + index));
        }
        printf("\n\n");
    }

    //Iterate through grids and assign corresponding max value
    unsigned int total_count = 0;
    unsigned int count = 0;
    for (int i = 0; i < cell_size_x; i++) {
        for (int j = 0; j < cell_size_y; j++) {
            if (g1_index == cell_size_x * cell_size_y) return grid2;
            if (count == region_size_x) {
                g1_index = g1_index + cell_ox;
                g2_index = g2_index + cell_ox;
                count = 0;
            }
            grid2[g2_index] = max(grid2[g2_index], grid1[g1_index]);
            printf("%d, %d \n", g1_index, g2_index);
            g1_index++;
            g2_index++;
            count++;
            total_count++;
        }
    }
}

unsigned char* cloudToOccgrid(float* data, unsigned int data_size, double robot_x, double robot_y, double robot_z, double robot_yaw, bool rolling_window, double min_obstacle_height, double max_obstacle_height, double raytrace_range, unsigned int size_x,
                              unsigned int size_y, double resolution, unsigned char default_value) {

    initCostmap(rolling_window, min_obstacle_height, max_obstacle_height, raytrace_range, size_x, size_y, resolution, default_value, robot_x, robot_y, robot_z);

    //printf("(1) Number of elements : %d ... ", data_size);
    //printf("First Coordinate = <%f, %f>\n", *data, *(data+1));
    updateMap(data, data_size, robot_x, robot_y, robot_z, robot_yaw);

    //printMap();
    return master_observation.master_costmap.costmap_;
}

void updateMap(float* data, unsigned int data_size, double robot_x, double robot_y, double robot_z, double robot_yaw) {
    if (master_observation.rolling_window_) {
        //printf("\nUpdating Map .... \n");
        //printf("   robot_x = %f, robot_y = %f, robot_yaw = %f \n", robot_x, robot_y, robot_yaw);
        //printf("   Master Origin = (%f, %f)\n", master_observation.master_origin.x, master_observation.master_origin.y);
        double new_origin_x = robot_x - master_observation.master_costmap.size_x / 2;
        double new_origin_y = robot_y - master_observation.master_costmap.size_y / 2;
        updateOrigin(new_origin_x, new_origin_y);
    }

    double minx_ = 1e30;
    double miny_ = 1e30;
    double maxx_ = -1e30;
    double maxy_ = -1e30;

    //printf("(1) Number of elements : %d ... ", data_size);
    //printf("First Coordinate = <%f, %f>\n", *data, *(data+1));
    //rotating_window = true; //Comment out if not rolling window

    updateBounds(data, data_size, robot_x, robot_y, robot_z, robot_yaw, minx_, miny_, maxx_, maxy_);
}

void updateOrigin(double new_origin_x, double new_origin_y) {
    //printf("\nUpdating Map Origin\n");
    //printf("New Origin -> <%f, %f>\n ", new_origin_x, new_origin_y);

    //project the new origin into the grid
    int cell_ox, cell_oy;
    //printf("Old Origin = <%f, %f> ... ", master_observation.master_origin.x, master_observation.master_origin.y);
    //printf("New Origin = <%f, %f>\n", new_origin_x, new_origin_y);
    cell_ox = (int) ((new_origin_x - master_observation.master_origin.x) / master_observation.master_resolution);
    cell_oy = (int) ((new_origin_y - master_observation.master_origin.y) / master_observation.master_resolution);
    //printf("New Cell Origin = <%d, %d>\n", cell_ox, cell_oy);

    // compute the associated world coordinates for the origin cell
    // because we want to keep things grid-aligned
    double new_grid_ox, new_grid_oy;
    new_grid_ox = master_observation.master_origin.x + cell_ox * master_observation.master_resolution;
    new_grid_oy = master_observation.master_origin.y + cell_oy * master_observation.master_resolution;
    //printf("New Grid Origin = <%f, %f> (Should be same as new_origin_x and new_origin_y)\n", new_grid_ox, new_grid_oy);

    // To save casting from unsigned int to int a bunch of times
    int size_x = master_observation.master_costmap.size_x / master_observation.master_resolution;
    int size_y = master_observation.master_costmap.size_y / master_observation.master_resolution;

    // we need to compute the overlap of the new and existing windows
    int lower_left_x, lower_left_y, upper_right_x, upper_right_y;
    lower_left_x = min(max(cell_ox, 0), size_x);
    lower_left_y = min(max(cell_oy, 0), size_y);
    upper_right_x = min(max(cell_ox + size_x, 0), size_x);
    upper_right_y = min(max(cell_oy + size_y, 0), size_y);
    //printf("The Corner Coordinates for Window = {%d, %d} {%d, %d}\n", lower_left_x, lower_left_y, upper_right_x, upper_right_y);

    unsigned int cell_size_x = (upper_right_x - lower_left_x);
    unsigned int cell_size_y = (upper_right_y - lower_left_y);
    //printf("Cell Sizes from Corner Coordinates = %d, %d\n", cell_size_x, cell_size_y);

    // we need a map to store the obstacles in the window temporarily
    //unsigned char* local_map [cell_size_x * cell_size_y]; //Uncomment if in C++

    // now we'll set the costmap to be completely unknown if we track unknown space
    //resetMaps();

    // update the origin with the appropriate world coordinates
    master_observation.master_origin.x = new_grid_ox;
    master_observation.master_origin.y = new_grid_oy;

    // compute the starting cell location for copying data back in
    int start_x = lower_left_x - cell_ox;
    int start_y = lower_left_y - cell_oy;
    //printf("lower_left_x - cell_ox = start_x ... (%d) - (%d) = %d\n", lower_left_x, cell_ox, start_x);
    //printf("lower_left_y - cell_oy = start_y ... (%d) - (%d) = %d\n", lower_left_y, cell_oy, start_y);

    // copy the local window in the costmap to the local map
    copyMapRegion(master_observation.master_costmap.costmap_, lower_left_x, lower_left_y,
                  master_observation.master_costmap.size_x / master_observation.master_resolution, start_x, start_y,
                  master_observation.master_costmap.size_x / master_observation.master_resolution, cell_size_x, cell_size_y);


    // now we want to copy the overlapping information back into the map, but in its new location
    //copyMapRegion(local_map, 0, 0, cell_size_x, master_observation.master_costmap.costmap_, start_x, start_y, master_observation.master_costmap.size_x, cell_size_x, cell_size_y);
}

//TODO: Modify such that it explicitly copies the data to the destination map
void copyMapRegion(unsigned char* source_map, unsigned int sm_lower_left_x, unsigned int sm_lower_left_y,
                       unsigned int sm_size_x, unsigned int dm_lower_left_x,
                       unsigned int dm_lower_left_y, unsigned int dm_size_x, unsigned int region_size_x,
                       unsigned int region_size_y) {
// we'll first need to compute the starting points for each map
    //printf("CopyMapRegion() Input Parameters: \n ... sm_lower_left_x = %d,\n ... sm_lowerLeft_y = %d,\n ... sm_size_x = %d,\n ... dm_lower_left_x = %d,\n ... dm_lower_left_y = %d,\n ... dm_size_x = %d,\n ... regions_size_x = %d,\n ... region_size_y = %d\n",sm_lower_left_x, sm_lower_left_y, sm_size_x, dm_lower_left_x, dm_lower_left_y, dm_size_x, region_size_x, region_size_y);
    unsigned int sm_index = (sm_lower_left_y * sm_size_x + sm_lower_left_x);
    unsigned int dm_index = (dm_lower_left_y * dm_size_x + dm_lower_left_x);
    //printf("%c\n", master_observation.master_costmap.default_value);
    //printf("{sm_index = %d, dm_index = %d}\n", sm_index, dm_index);

    unsigned int cell_size_x = master_observation.master_costmap.size_x / master_observation.master_resolution;
    unsigned int cell_size_y = master_observation.master_costmap.size_y / master_observation.master_resolution;

    //printf("\n Copying Map... \nRegion Size of Map -> <%d, %d>\n", region_size_x, region_size_y);
    char* local_costmap [cell_size_x * cell_size_y];
    for (int i = 0; i < cell_size_x * cell_size_y; i++) {
        local_costmap[i] = master_observation.master_costmap.default_value;
        //printf("%d, ", local_costmap[i]);
    }

    // now, we'll copy the source map into the destination map
    for (unsigned int i = 0; i < region_size_y; ++i){
        for (unsigned int j = 0; j < region_size_x; j++) {
            //printf("Source Map Value at Index <%d> = %d\n", sm_index, master_observation.master_costmap.costmap_[sm_index]);
            local_costmap[dm_index] = master_observation.master_costmap.costmap_[sm_index];
            //printf("dm_index, sm_index = %d, %d\n", dm_index, sm_index);
            sm_index++;
            dm_index++;
        }
        if (master_observation.master_costmap.size_x != region_size_x) {
            sm_index = sm_index + (master_observation.master_costmap.size_x / master_observation.master_resolution - region_size_x);
            dm_index = dm_index + (master_observation.master_costmap.size_x / master_observation.master_resolution - region_size_x);
        }
        //memcpy(dm_index, sm_index, region_size_x * sizeof(unsigned char*));
    }

    //printf("We made it!\n");

    for (int i = 0; i < cell_size_x * cell_size_y; i++) {
        master_observation.master_costmap.costmap_[i] = local_costmap[i];
    }
}

void updateBounds(float* data, unsigned int data_size, double robot_x, double robot_y, double robot_z,
                  double robot_yaw, double min_x, double min_y, double max_x, double max_y) {
    //printf("(1) Number of elements : %d ... ", data_size);
    //printf("First Coordinate = <%f, %f>\n", *data, *(data+1));

    //raytrace free space
    raytraceFreespace(data, data_size, min_x, min_y, max_x, max_y, robot_x, robot_y, robot_z, robot_yaw); //TODO: Reconfigure for 'cloud' parameter

    //printf("Number of elements : %d\n", sizeof(cloud.data) / sizeof(cloud.data[0]));

    //Iterate through cloud to register obstacles within costmap
    for(unsigned int i = 0; i < data_size; i = i + 3) { //TODO: Test if sizeof(points) works correctly
        //Only consider points within height boundaries
        if (data[i + 2] <= master_observation.max_obstacle_height_ && data[i + 2] >= master_observation.min_obstacle_height_) {
            double px = (double) *(data + i);
            double py = (double) *(data + i + 1);
            double pz = (double) *(data + i + 2);

            //if window rotates, then inversely rotate points
            if (rotating_window) {
                px = px*cos(robot_yaw) - py*sin(robot_yaw);
                py = px*sin(robot_yaw) + py*cos(robot_yaw);
            }

            //printf("World Coordinates (wx, wy) = (%f, %f)\n", px, py);
            //printf("Master Origin Coordinate = < %f, %f > \n", master_observation.master_origin.x, master_observation.master_origin.y);

            //printf("Map Coordinates [BEFORE Conversion] -> (%d, %d)\n", master_observation.map_coordinates.x, master_observation.map_coordinates.y);
            worldToMap(px, py, robot_x, robot_y);
            //printf("Map Coordinates (mx, my) = (%d, %d)\n", master_observation.map_coordinates.x, master_observation.map_coordinates.y);

            unsigned int index = getIndex(master_observation.map_coordinates.x, master_observation.map_coordinates.y);
            //printf("Index of Obstacle -> %d\n", index);
            master_observation.master_costmap.costmap_[index] = LETHAL_OBSTACLE; //TODO: Test simple test case (char) 255 = '255' ?
            touch(px, py, min_x, min_y, max_x, max_y);
        }
    }
}

/* Issues:
   (1) Finding the size of the array for iteration
   (2) Difference between origin_x_ and observation.origin_x
*/
void raytraceFreespace(float* data, unsigned int data_size, double min_x, double min_y, double max_x, double max_y, double robot_x, double robot_y, double robot_z, double robot_yaw) {
    //printf("(1) Number of elements : %d ... ", data_size);
    //printf("First Coordinate = <%f, %f>\n", *data, *(data+1));

    //Retrieve observation origin (i.e. origin of the pointcloud)
    double ox = robot_x;
    double oy = robot_y;
    //printf(">>> Odometry -> <%f, %f>\n", ox, oy);

    // get the map coordinates of the origin of the sensor
    unsigned int x0, y0;
    if (!worldToMap(0, 0, robot_x, robot_y)) //TODO:
    {
        printf("The origin for the sensor at (%.2f, %.2f) is out of map bounds. So, the costmap cannot raytrace for it.\n", ox, oy);
        return;
    }
    x0 = master_observation.map_coordinates.x;
    y0 = master_observation.map_coordinates.y;
    //printf(">>> Map Coordinates of the Sensor Origin -> <%d, %d>\n", x0, y0);

    // we can pre-compute the endpoints of the map outside of the inner loop... we'll need these later
    double map_end_x = master_observation.master_origin.x + master_observation.master_costmap.size_x * master_observation.master_resolution;
    double map_end_y = master_observation.master_origin.y + master_observation.master_costmap.size_y * master_observation.master_resolution;
    //printf(">>> End of Map Coordinates -> <%f, %f>\n", map_end_x, map_end_y);

    touch(ox, oy, min_x, min_y, max_x, max_y);

    for (int i = 0; i < data_size; i = i + 3) {
        double wx = (double) *(data + i);
        double wy = (double) *(data + i + 1);
        //printf(">>> World Coordinates of Data Point -> <%f, %f>\n", wx, wy);

        //if window rotates, then inversely rotate points
        if (rotating_window) {
            wx = wx*cos(robot_yaw) - wy*sin(robot_yaw);
            wy = wx*sin(robot_yaw) + wy*cos(robot_yaw);
        }

        // now we also need to make sure that the enpoint we're raytracing
        // to isn't off the costmap and scale if necessary
        double a = wx - ox;
        double b = wy - oy;

        if (wx < master_observation.master_origin.x) {
            double t = (master_observation.master_origin.x - ox) / a;
            wx = master_observation.master_origin.x;
            wy = oy + b * t;
        }

        if (wy < master_observation.master_origin.y) {
            double t = (master_observation.master_origin.y - oy) / b;
            wx = ox + a * t;
            wy = master_observation.master_origin.y;
        }

        // the maximum value to raytrace to is the end of the map
        if (wx > map_end_x)
        {
            double t = (map_end_x - ox) / a;
            wx = map_end_x - .001;
            wy = oy + b * t;
        }
        if (wy > map_end_y)
        {
            double t = (map_end_y - oy) / b;
            wx = ox + a * t;
            wy = map_end_y - .001;
        }

        // now that the vector is scaled correctly... we'll get the map coordinates of its endpoint
        unsigned int x1, y1;
        if (!worldToMap(wx, wy, robot_x, robot_y)) continue;
        x1 = master_observation.map_coordinates.x;
        y1 = master_observation.map_coordinates.y;

        unsigned int cell_raytrace_range = cellDistance(master_observation.raytrace_range_);
        //printf(">>> Cell Raytrace Range -> %d\n", cell_raytrace_range);

        // and finally... we can execute our trace to clear obstacles along that line
        raytraceLine(x0, y0, x1, y1, cell_raytrace_range);

        updateRaytraceBounds(ox, oy, wx, wy, master_observation.raytrace_range_, min_x, min_y, max_x, max_y);
    }
}

bool worldToMap(double wx, double wy, double robot_x, double robot_y) {
    //Calculate world coordinates relative to origin (and not robot's odometry)
    double wx_rel_origin = wx + robot_x;
    double wy_rel_origin = wy + robot_y;
    //printf("World To Map (Relative to Origin) = (%d, %d)\n", (int)((wx_rel_origin - master_observation.master_origin.x) / master_observation.master_resolution), (int)((wy_rel_origin - master_observation.master_origin.y) / master_observation.master_resolution));

    if (wx_rel_origin < master_observation.master_origin.x || wy_rel_origin < master_observation.master_origin.y) {
        printf("Coordinates Out Of Bounds .... (wx, wy) = (%f, %f); (ox, oy) = (%f, %f)\n", wx, wy, master_observation.master_origin.x, master_observation.master_origin.y);
        return false;
    }

    master_observation.map_coordinates.x = (int)((wx_rel_origin - master_observation.master_origin.x) / master_observation.master_resolution);
    master_observation.map_coordinates.y = (int)((wy_rel_origin - master_observation.master_origin.y) / master_observation.master_resolution);

    //printf("World To Map (wx, wy) = (%f, %f) -> (mx, my) = (%d, %d)\n\n", wx, wy, master_observation.map_coordinates.x, master_observation.map_coordinates.y);

    if (master_observation.map_coordinates.x < master_observation.master_costmap.size_x && master_observation.map_coordinates.y < master_observation.master_costmap.size_y) return true;

    return false;
}

unsigned int cellDistance(double world_dist) {
    double cells_dist = max (0.0, ceil(world_dist/master_observation.master_resolution));
    return (unsigned int) cells_dist;
}

void raytraceLine(unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1, unsigned int max_length) { //TODO: default parameter for max_length is UNIT_MAX; define UNIT_MAX
    //printf(">>> Raytrace Line from <%d, %d> to <%d, %d> \n", x0, y0, x1, y1);
    int dx = x1 - x0;
    int dy = y1 - y0;
    //printf("dx, dy -> %d ,%d\n", dx, dy);

    unsigned int abs_dx = abs(dx);
    unsigned int abs_dy = abs(dy);

    int offset_dx = sign(dx);
    //printf("offset_dx -> %d, \n", offset_dx);
    //printf("cell_size_x -> %d \n", (int) (master_observation.master_costmap.size_x / master_observation.master_resolution));
    int offset_dy = sign(dy) * (int) (master_observation.master_costmap.size_x / master_observation.master_resolution);
    //printf("offset_dy -> %d \n", offset_dy);

    unsigned int offset = y0 * master_observation.master_costmap.size_x / master_observation.master_resolution + x0;
    //printf("offset -> %d \n", offset);

    // we need to chose how much to scale our dominant dimension, based on the maximum length of the line
    double dist = hypot(dx, dy);
    double scale = (dist == 0.0) ? 1.0 : min(1.0, max_length / dist);

    // if x is dominan

    if (abs_dx >= abs_dy)
    {
        int error_y = abs_dx / 2;
        bresenham2D(abs_dx, abs_dy, error_y, offset_dx, offset_dy, offset, (unsigned int)(scale * abs_dx));
        return;
    }

    // otherwise y is dominant
    int error_x = abs_dy / 2;
    bresenham2D(abs_dy, abs_dx, error_x, offset_dy, offset_dx, offset, (unsigned int)(scale * abs_dy));
}

void bresenham2D(unsigned int abs_da, unsigned int abs_db, int error_b, int offset_a,
                        int offset_b, unsigned int offset, unsigned int max_length) {
    unsigned int end = min(max_length, abs_da);
    //printf("\n\n abs_da, end -> %d, %d\n", abs_da, end);
    for (unsigned int i = 0; i < end; ++i)
    {
        markCell(FREE_SPACE, offset);
        offset += offset_a;
        error_b += abs_db;
        if ((unsigned int)error_b >= abs_da)
        {
            offset += offset_b;
            error_b -= abs_da;
        }
    }
    markCell(FREE_SPACE, offset);
}

void markCell(unsigned char value, unsigned int offset) {
    //printf("OFFSET -> %d\n", offset);
    master_observation.master_costmap.costmap_[offset] = value;
}

void updateRaytraceBounds(double ox, double oy, double wx, double wy, double range,
                          double min_x, double min_y, double max_x, double max_y) {
    double dx = wx - ox, dy = wy - oy;
    double full_distance = hypot(dx, dy);
    double scale = min(1.0, range / full_distance);
    double ex = ox + dx * scale, ey = oy + dy * scale;
    touch(ex, ey, min_x, min_y, max_x, max_y);
}

void touch(double x, double y, double min_x, double min_y, double max_x, double max_y) {
    min_x = min(x, min_x);
    min_y = min(y, min_y);
    max_x = max(x, max_x);
    max_y = max(y, max_y);
}
