#include "parse_dataset.h"
#include "utils.h"
#include "tricycle_model.h"
#include <iostream>
#include <Eigen/Dense>
#include <cmath>

Tricycle tr;



Eigen::Vector2d h(const Eigen::VectorXd& x, int32_t& ticks_steer, int32_t& delta_ticks_track) {
    double Ksteer_    = x(2);
    double Ktraction_ = x(3);
    double baseline_  = x(4);
    double steer_off_ = x(5);
    double   tick_steer_max_ = 8192;
    double   tick_track_max_ = 5000;
    
    double delta_traction = Ktraction_ * static_cast<double>(delta_ticks_track) / tick_track_max_;

    double phi_tr_h    = Ksteer_ * wrapToPi(static_cast<double>(ticks_steer) * (2.0 * M_PI / tick_steer_max_)) + steer_off_;
    double x_tr_h     = tr.x() + delta_traction * std::cos(tr.theta()) * std::cos(phi_tr_h);
    double y_tr_h     = tr.y() + delta_traction * std::sin(tr.theta()) * std::cos(phi_tr_h);    
    double theta_tr_h  = tr.theta() + delta_traction * std::sin(phi_tr_h)/baseline_;


    Eigen::Matrix2d R;
    const double c_theta = std::cos(theta_tr_h);
    const double s_theta = std::sin(theta_tr_h);
    R << c_theta, -s_theta,
    s_theta,  c_theta;
    return Eigen::Vector2d(x_tr_h, y_tr_h) + R * Eigen::Vector2d(x(0), x(1));
    
}

Eigen::MatrixXd numericalJacobian(
    const Eigen::VectorXd& x,
    int32_t& ticks_steer,
    int32_t& delta_ticks_track,
    double epsilon = 1e-4)
{
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(2,6);
    
    for (int i = 0; i < 6; ++i) {
        Eigen::VectorXd x_plus  = x;
        Eigen::VectorXd x_minus = x;
        x_plus(i)  += epsilon;
        x_minus(i) -= epsilon;

        Eigen::Vector2d h_plus  = h(x_plus, ticks_steer, delta_ticks_track);
        Eigen::Vector2d h_minus = h(x_minus, ticks_steer, delta_ticks_track);

        J.col(i) = (h_plus - h_minus) / (2.0 * epsilon);

    }
    return J;
}


void compute_error_and_Jacobian(
    const Eigen::VectorXd& x, 
    const Eigen::Vector2d& z, 
    int32_t& ticks_steer,
    int32_t& delta_ticks_track,
    Eigen::Vector2d& error, 
    Eigen::MatrixXd& J)
    {
    
    Eigen::Vector2d h;
    Eigen::Vector2d pos_sensor = x.head<2>(); // position of the sensor in robot frame
    Eigen::Vector2d pos_robot;
    double theta, phi;

      
    // integrate the model
    tr.update_tricycle_state(ticks_steer, delta_ticks_track);
    pos_robot = tr.pos();
    theta     = tr.theta();
    phi       = tr.phi();

        
    // std::cout << "\n" << "x: " << tr.x() << std::endl;
    // std::cout << "y: " << tr.y() << std::endl;
    // std::cout << "theta: " << tr.theta() << std::endl;
    // std::cout << "phi: " << tr.phi() << "\n" << std::endl;
    
    Eigen::Matrix2d R;
    const double c_theta = std::cos(theta);
    const double s_theta = std::sin(theta);
    R << c_theta, -s_theta,
    s_theta,  c_theta;
    
    h = pos_robot + R * pos_sensor;          // position of the sensor expressed in world reference frame   
    error = h - z;

    // compute the NUMERIC jacobian
    J = numericalJacobian(x, ticks_steer, delta_ticks_track);
    // std::cout << "J" << J << std::endl;

    
    // compute the ANALYTHIC jacobian
    // Eigen::Matrix2d R_prime;
    // R_prime << -s_theta, -c_theta,
    // c_theta,  -s_theta;
    
    // const double c_phi = std::cos(phi);
    // const double s_phi = std::sin(phi);
    
    // double Ksteer    = x(2);
    // double Ktraction = x(3);
    // double baseline  = x(4);
    // double steer_off = x(5);

    // double   tick_steer_max_ = 8192;
    // double   tick_track_max_ = 5000;
    
    
    // // compute differentials
    // double diff_theta_phi  = Ktraction * static_cast<double>(delta_ticks_track) / tick_track_max_  * c_phi / baseline;
    // Eigen::Vector2d diff_p_phi = 
    // Ktraction * static_cast<double>(delta_ticks_track) / tick_track_max_ * (
    //     Eigen::Vector2d(-s_theta, c_theta) * c_phi * diff_theta_phi +
    //     Eigen::Vector2d(c_theta, s_theta) * (-s_phi)
    // );
    
    // // diff theta
    // double diff_theta_Ksteer    = diff_theta_phi * wrapToPi(static_cast<double>(ticks_steer) * (2.0 * M_PI / tick_steer_max_));
    // double diff_theta_Ktraction = static_cast<double>(delta_ticks_track) / tick_track_max_ * s_phi / baseline;
    // double diff_theta_baseline  = - Ktraction * static_cast<double>(delta_ticks_track) / tick_track_max_ * s_phi / std::pow(baseline,2);
    // double diff_theta_steer_off = diff_theta_phi;
    
    // // diff p
    // Eigen::Vector2d diff_p_Ksteer    = diff_p_phi * wrapToPi(static_cast<double>(ticks_steer) * (2.0 * M_PI / tick_steer_max_));
    // Eigen::Vector2d diff_p_Ktraction = 
    // static_cast<double>(delta_ticks_track) / tick_track_max_ * Eigen::Vector2d(c_theta, s_theta) * c_phi +
    // Ktraction * static_cast<double>(delta_ticks_track) / tick_track_max_ * Eigen::Vector2d(-s_theta, c_theta) * c_phi * diff_theta_Ktraction;
    // Eigen::Vector2d diff_p_baseline  = Ktraction * static_cast<double>(delta_ticks_track) / tick_track_max_ * Eigen::Vector2d(-s_theta, c_theta) * c_phi * diff_theta_baseline;
    // Eigen::Vector2d diff_p_steer_off = diff_p_phi;
    
    // Eigen::Vector2d diff_Ksteer    = diff_p_Ksteer    + R_prime * pos_sensor * diff_theta_Ksteer;
    // Eigen::Vector2d diff_Ktraction = diff_p_Ktraction + R_prime * pos_sensor * diff_theta_Ktraction;
    // Eigen::Vector2d diff_baseline  = diff_p_baseline  + R_prime * pos_sensor * diff_theta_baseline;
    // Eigen::Vector2d diff_steer_off = diff_p_steer_off + R_prime * pos_sensor * diff_theta_steer_off;
    
    // J.block(0,0,2,2) = R; 
    // J.col(2) << diff_Ksteer;
    // J.col(3) << diff_Ktraction;
    // J.col(4) << diff_baseline;
    // J.col(5) << diff_steer_off;
    
    // std::cout << "\n" << "J: " << "\n" << J << "\n" << std::endl;
}



int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " dataset.txt\n";
        return 1;
    }
    Dataset ds;
    if (!loadDataset(argv[1], ds)) {
        std::cerr << "Failed to read dataset: " << argv[1] << "\n";
        return 1;
    }

    print_infos(ds);

    // set max encoder values for the kinematic model
    tr.tick_steer_max_ = ds.joints.values[0];
    tr.tick_track_max_ = ds.joints.values[1];

    // Initialization
    double time;
    int32_t ticks_steer;
    int32_t delta_ticks_track;
    int dim_measurement = 2;
    int num_variables = 6;
    int num_measurements = ds.records.size();
    int max_iter = 1;

    double error_norm = 0;
    std::vector<double> error_log;

    Eigen::Vector2d error = Eigen::Vector2d::Zero();
    Eigen::MatrixXd Jacobian = Eigen::MatrixXd::Zero(dim_measurement, num_variables);
    
    Eigen::MatrixXd H;
    Eigen::VectorXd b;

    Eigen::Vector2d z;
    Eigen::VectorXd x(num_variables);
    x << ds.laser.tx,
        ds.laser.ty,
        ds.params.Ksteer,
        ds.params.Ktraction,
        ds.params.axis_length,
        ds.params.steer_offset;


    Eigen::Matrix2d Omega = 1e-3 * Eigen::Matrix2d::Identity();
   
    // ICP loop
    for (int iter = 0; iter < max_iter; iter ++){
        // clear H and b
        H = Eigen::MatrixXd::Zero(num_variables, num_variables);
        b = Eigen::VectorXd::Zero(num_variables);

        // update kinematic parameters in the kinematic model
        tr.updateParams(x.tail(x.size() - 2));
        tr.resetState();

        // one iter loop            num_measurements
        for(int index = 0; index < num_measurements; index++){

            // get a measurement
            get_sensor_reading(ds, index, time, ticks_steer, delta_ticks_track, z);
            
            // compute error and jacobian
            compute_error_and_Jacobian(x, z, ticks_steer, delta_ticks_track, error, Jacobian);

            error_norm += error.norm();
            // std::cout << "error" << error << std::endl;
            // std::cout << "Jacobian" << Jacobian << std::endl;

            // accumulate H and b 
            H += Jacobian.transpose() * Omega * Jacobian;
            b += Jacobian.transpose() * Omega * error;
            
            // std::cout << "H: " << H << std::endl;
            // std::cout << "b: " << b << std::endl;
        }
        
        // calibrate_parameters
        Eigen::VectorXd dx = H.ldlt().solve(-b);
        x += dx;
        std::cout << "dx: " << dx << std::endl;
        
        error_log.push_back(error_norm / num_measurements);
        error_norm = 0; 
    }


    std::cout << "\n" << "ICP algorithm terminated " << std::endl;
    std::cout << "x_s: " << x(0)<< std::endl;
    std::cout << "y_s: " << x(1)<< std::endl;
    std::cout << "K_steer: " << x(2)<< std::endl;
    std::cout << "K_traction: " << x(3)<< std::endl;
    std::cout << "baseline: " << x(4)<< std::endl;
    std::cout << "steer_offset: " << x(5)<< std::endl;

    //save error for the plot 
    save_error("error_log.csv",error_log);
    //save params 
    save_params("calibrated_params.csv", x);

    return 0;
}