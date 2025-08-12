#include <iostream>
#include "parse_dataset.h"
#include <Eigen/Dense>


void print_infos(Dataset& ds);

void get_sensor_reading(
    Dataset& ds, 
    int& index, 
    double& time, 
    int32_t& ticks_steer,
    int32_t& ticks_track, 
    Eigen::Vector2d& z);