#include "parse_dataset.h"
#include "utils.h"
#include "tricycle_model.h"
#include <iostream>
#include <Eigen/Dense>
#include <cmath>

Tricycle tr;

void compute_error_and_Jacobian(
    const Eigen::VectorXd& x, 
    const Eigen::Vector2d& z, 
    int32_t& ticks_steer,
    int32_t& ticks_track,
    Eigen::Vector2d& error, 
    Eigen::MatrixXd& J)
    {
    
    Eigen::Vector2d h;
    Eigen::Vector2d pos_sensor = x.head<2>(); // position of the sensor in robot frame
    Eigen::Vector2d pos_robot;
    double theta, phi;

    pos_robot = tr.pos();
    theta     = tr.theta();
    phi       = tr.phi();
    std::cout << "\n" << "x: " << tr.x() << std::endl;
    std::cout << "y: " << tr.y() << std::endl;
    // std::cout << "theta: " << tr.theta() << std::endl;
    // std::cout << "phi: " << tr.phi() << "\n" << std::endl;
    
    Eigen::Matrix2d R;
    const double c_theta = std::cos(theta);
    const double s_theta = std::sin(theta);
    R << c_theta, -s_theta,
    s_theta,  c_theta;
    
    h = pos_robot + R * pos_sensor;          // position of the sensor expressed in world reference frame   
    error = h - z;
    
    // compute the jacobian
    Eigen::Matrix2d R_prime;
    R_prime << -s_theta, -c_theta,
    c_theta,  -s_theta;
    
    const double c_phi = std::cos(phi);
    const double s_phi = std::sin(phi);
    
    double Ksteer    = x(2);
    double Ktraction = x(3);
    double baseline  = x(4);
    double steer_off = x(5);
    
    // compute differentials
    double diff_theta_phi  = Ktraction * ticks_track * c_phi / baseline;
    Eigen::Vector2d diff_p_phi = 
    Ktraction * ticks_track * (
        Eigen::Vector2d(-s_theta, c_theta) * c_phi * diff_theta_phi +
        Eigen::Vector2d(c_theta, s_theta) * (-s_phi)
    );
    
    // diff theta
    double diff_theta_Ksteer    = diff_theta_phi * ticks_steer;
    double diff_theta_Ktraction = ticks_track * s_phi / baseline;
    double diff_theta_baseline  = - Ktraction * ticks_track * s_phi / std::pow(baseline,2);
    double diff_theta_steer_off = diff_theta_phi;
    
    // diff p
    Eigen::Vector2d diff_p_Ksteer    = diff_p_phi * ticks_steer;
    Eigen::Vector2d diff_p_Ktraction = 
    ticks_track * Eigen::Vector2d(c_theta, s_theta) * c_phi +
    Ktraction * ticks_track * Eigen::Vector2d(-s_theta, c_theta) * c_phi * diff_theta_Ktraction;
    Eigen::Vector2d diff_p_baseline  = Ktraction * ticks_track * Eigen::Vector2d(-s_theta, c_theta) * c_phi * diff_theta_baseline;
    Eigen::Vector2d diff_p_steer_off = diff_p_phi;
    
    Eigen::Vector2d diff_Ksteer    = diff_p_Ksteer    + R_prime * pos_sensor * diff_theta_Ksteer;
    Eigen::Vector2d diff_Ktraction = diff_p_Ktraction + R_prime * pos_sensor * diff_theta_Ktraction;
    Eigen::Vector2d diff_baseline  = diff_p_baseline  + R_prime * pos_sensor * diff_theta_baseline;
    Eigen::Vector2d diff_steer_off = diff_p_steer_off + R_prime * pos_sensor * diff_theta_steer_off;
    
    J.block(0,0,2,2) = R; 
    J.col(2) << diff_Ksteer;
    J.col(3) << diff_Ktraction;
    J.col(4) << diff_baseline;
    J.col(5) << diff_steer_off;
    
    // std::cout << "\n" << "J: " << "\n" << J << "\n" << std::endl;

    // integrate the model at the end
    tr.update_tricycle_state(ticks_steer, ticks_track);
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


    // Initialization
    double time;
    int32_t ticks_steer;
    int32_t ticks_track;
    int dim_measurement = 2;
    int num_variables = 6;
    int num_measurements = ds.records.size();
    int max_iter = 20;

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

    Eigen::Matrix2d Omega = 1 * Eigen::Matrix2d::Identity();
   
    // ICP loop
    for (int iter = 0; iter < max_iter; iter ++){
        // clear H and b
        H = Eigen::MatrixXd::Zero(num_variables, num_variables);
        b = Eigen::VectorXd::Zero(num_variables);

        // update kinematic parameters in the kinematic model
        tr.updateParams(x.tail(x.size() - 2));

        // one iter loop            num_measurements
        for(int index = 0; index < num_measurements; index ++){

            // get a measurement
            get_sensor_reading(ds, index, time, ticks_steer, ticks_track, z);
            
            // compute error and jacobian
            compute_error_and_Jacobian(x, z, ticks_steer, ticks_track, error, Jacobian);

            error_norm += error.norm();
            // std::cout << "error" << error << std::endl;
            // std::cout << "Jacobian" << Jacobian << std::endl;


            // calibrate_parameters
            H += Jacobian.transpose() * Omega * Jacobian;
            b += Jacobian.transpose() * Omega * error;
            
            // std::cout << "H: " << H << std::endl;
            // std::cout << "b: " << b << std::endl;
        }

        Eigen::VectorXd dx = H.ldlt().solve(-b);
        x += dx;            
        
        tr.resetState();
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