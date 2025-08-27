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

double quaternionToYaw(double qx, double qy, double qz, double qw);

Eigen::Matrix3d v2t(const Eigen::Vector3d params);
Eigen::Vector3d t2v(const Eigen::Matrix3d T);

Eigen::VectorXd box_minus(const Eigen::VectorXd& h, const Eigen::VectorXd delta_z);
Eigen::VectorXd box_plus(const Eigen::VectorXd x, const Eigen::VectorXd delta_x);

class Tricycle;

Eigen::MatrixXd numericalJacobian(
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& delta_z,
    Tricycle& tr,
    int32_t& ticks_steer,
    int32_t& delta_ticks_track,
    double epsilon = 1e-7);

void get_sensor_reading(
    Dataset& ds, 
    int& index, 
    double& time, 
    int32_t& ticks_steer,
    int32_t& ticks_track, 
    Eigen::VectorXd& z);

void save_error(const std::string& filename, const std::vector<double>& error_log, const std::vector<int>& outliers_log);
void save_params(const std::string& filename, const Eigen::VectorXd& x);