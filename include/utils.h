#pragma once
#include <iostream>
#include "parse_dataset.h"
#include <Eigen/Dense>
#include <cmath>
#include <fstream>

constexpr double TWO_PI = 2.0 * M_PI;

double wrapToPi(double a);

double wrapTwoPi(double a);

void print_infos(Dataset& ds);

void get_sensor_reading(
    Dataset& ds, 
    int& index, 
    double& time, 
    int32_t& ticks_steer,
    int32_t& ticks_track, 
    Eigen::Vector2d& z);

void save_error(const std::string& filename, const std::vector<double>& error_log);
void save_params(const std::string& filename, const Eigen::VectorXd& x);