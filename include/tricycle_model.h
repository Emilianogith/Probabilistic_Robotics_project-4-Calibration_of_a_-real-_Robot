#pragma once
#include <Eigen/Dense>
#include <cmath>
#include "utils.h"

class Tricycle{
public:

    void updateParams(const Eigen::Ref<const Eigen::VectorXd>& params) {
        Ksteer_    = params(0); 
        Ktraction_ = params(1);
        baseline_  = params(2); 
        steer_off_ = params(3);
    }

    void resetState() {
        x_ = 0.0; 
        y_ = 0.0; 
        theta_ = 0.0; 
        phi_ = 0.0;                 
    }

    void update_tricycle_state(int32_t& ticks_steer, int32_t& delta_ticks_track);

     // Accessors
    Eigen::Vector2d pos() const { return Eigen::Vector2d(x_, y_); }
    double x() const { return x_; }
    double y() const { return y_; }
    double theta() const { return theta_; }
    double phi() const { return phi_; }  

private:
        // kinematic parameters
        double   Ksteer_;      
        double   Ktraction_;    
        double   baseline_;            
        double   steer_off_;         

    // initial state
    double x_ = 0.0, y_ = 0.0, theta_ = 0.0; // base pose in world
    double phi_ = 0.0;                 
};