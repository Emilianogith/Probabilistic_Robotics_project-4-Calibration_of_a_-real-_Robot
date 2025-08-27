#include "tricycle_model.h"
#include <iostream>

Eigen::VectorXd Tricycle::tricycle_step(Eigen::VectorXd x, int32_t& ticks_steer, int32_t& delta_ticks_track)
{
    double Ksteer_    = x(0);
    double Ktraction_ = x(1);
    double baseline_  = x(2);
    double steer_off_ = x(3);

    // convert ticks and avoid overflow
    double delta_traction = Ktraction_ * static_cast<double>(delta_ticks_track) / tick_track_max_;
    
    // integrate the model
    phi_         = Ksteer_ * wrapToPi(static_cast<double>(ticks_steer) * (2.0 * M_PI / tick_steer_max_)) + steer_off_;
    delta_theta_ = delta_traction * std::sin(phi_)/baseline_;
    delta_x_     = delta_traction * std::cos(delta_theta_) * std::cos(phi_);
    delta_y_     = delta_traction * std::sin(delta_theta_) * std::cos(phi_);    
    
    Eigen::VectorXd v(3);
    v << delta_x_,delta_y_,delta_theta_;

    return v;
}