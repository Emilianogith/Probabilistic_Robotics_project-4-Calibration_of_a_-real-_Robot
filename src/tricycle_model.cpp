#include "tricycle_model.h"
#include <iostream>

constexpr double TWO_PI = 2.0 * M_PI;

inline double wrapTwoPi(double a) {
    a = std::remainder(a, TWO_PI);  // (-π, π]
    if (a < 0) a += TWO_PI;         // (0, 2π]
    if (a == TWO_PI) a = 0.0;       // force half-open [0, 2π)
    return a;
}

void Tricycle::update_tricycle_state(int32_t& ticks_steer, int32_t& ticks_track)
{
    // convert ticks and avoid overflow
    double delta_traction = Ktraction_ * static_cast<double>(ticks_track);
    
    // integrate the model
    phi_    = wrapTwoPi(Ksteer_ * static_cast<double>(ticks_steer) + steer_off_);
    theta_  = wrapTwoPi(theta_ + delta_traction * std::sin(phi_)/baseline_);
    x_     += delta_traction * std::cos(theta_) * std::cos(phi_);
    y_     += delta_traction * std::sin(theta_) * std::cos(phi_);    

    // kinematic model assuming steer_off_ in radians and robot starting in [0, 0, 0, ~]
    // otherwise if using delta values: 
    // phi_ = Ksteer_ * static_cast<double>(ticks_steer) + steer_off_;
    // delta_phi_    = wrapTwoPi(phi_ - phi_prev_);
    // delta_theta_  = wrapTwoPi(delta_traction * std::sin(phi_)/baseline_);
    // delta_x_      = delta_traction * std::cos(theta_) * std::cos(phi_);
    // delta_y_      = delta_traction * std::sin(theta_) * std::cos(phi_);  
    // phi_prev_ = phi_;

}