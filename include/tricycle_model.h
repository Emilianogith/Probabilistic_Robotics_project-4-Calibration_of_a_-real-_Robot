#pragma once
#include <Eigen/Dense>
#include <cmath>
#include "utils.h"

class Tricycle{
public:

    Eigen::VectorXd tricycle_step(Eigen::VectorXd x, int32_t& ticks_steer, int32_t& delta_ticks_track);

     // Accessors
    Eigen::Vector2d pos() const { return Eigen::Vector2d(delta_x_, delta_y_); };
    Eigen::VectorXd pose() const { 
        Eigen::VectorXd v(3);
        v << delta_x_, delta_y_, delta_theta_;
        return v; };
    double x() const { return delta_x_; };
    double y() const { return delta_y_; };
    double theta() const { return delta_theta_; };
    double phi() const { return phi_; };

    double   tick_steer_max_ = 8192;
    double   tick_track_max_ = 5000;
private:

    // initialization of the state displacements
    double delta_x_ = 0.0, delta_y_ = 0.0, delta_theta_ = 0.0; // base pose in world
    double phi_ = 0.0;                 
};