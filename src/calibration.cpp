#include "parse_dataset.h"
#include "utils.h"
#include "tricycle_model.h"
#include <iostream>
#include <Eigen/Dense>
#include <cmath>

Tricycle tr;

void compute_error_and_Jacobian(
    const Eigen::VectorXd& x, 
    const Eigen::VectorXd& delta_z, 
    int32_t& ticks_steer,
    int32_t& delta_ticks_track,
    Eigen::VectorXd& error, 
    Eigen::MatrixXd& J)
    {
    
    Eigen::VectorXd h;
    Eigen::VectorXd sensor_pose = x.head(3); // pose of the sensor in robot frame
    Eigen::VectorXd robot_displacement;

    // integrate the model
    robot_displacement = tr.tricycle_step(x.tail(4), ticks_steer, delta_ticks_track);
    
    // compute the observation function's result
    h = t2v(v2t(sensor_pose).inverse() * v2t(robot_displacement) * v2t(sensor_pose)); // displacement in the pose of the sensor expressed in the starting sensor pose RF 
    
    // compute the residual
    error = box_minus(h, delta_z);
  
    // compute the NUMERIC jacobian
    J = numericalJacobian(x, delta_z, tr, ticks_steer, delta_ticks_track);
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
    // print_infos(ds);

    // set max encoder values for the kinematic model
    tr.tick_steer_max_ = ds.joints.values[0];
    tr.tick_track_max_ = ds.joints.values[1];

    // Initialization
    double time;
    int32_t ticks_steer;
    int32_t delta_ticks_track;
    int dim_measurement = 3;
    int num_variables = 7;
    int num_measurements = ds.records.size();
    int max_iter = 19;
    int skip_N = 3;
    int num_outliers = 0;
    double kernel_threshold = 2e-4;
    double chi_sq = 0;
    double total_iter_chi_sq = 0;
    std::vector<double> error_log;
    std::vector<int> outliers_log;

    Eigen::VectorXd error = Eigen::VectorXd::Zero(dim_measurement);
    Eigen::MatrixXd Jacobian = Eigen::MatrixXd::Zero(dim_measurement, num_variables);
    
    Eigen::MatrixXd H;
    Eigen::VectorXd b;

    Eigen::VectorXd x(num_variables);           // Euclidean parametrization
    Eigen::VectorXd delta_z(dim_measurement);   // Euclidean parametrization


    // initialize the state at the nominal values
    x << ds.laser.tx,
         ds.laser.ty,
         quaternionToYaw(ds.laser.qx, ds.laser.qy, ds.laser.qz, ds.laser.qw),
         ds.params.Ksteer,
         ds.params.Ktraction,
         ds.params.axis_length,
         ds.params.steer_offset;

    Eigen::MatrixXd Omega = 1 * Eigen::MatrixXd::Identity(dim_measurement, dim_measurement);

    // LS loop
    for (int iter = 0; iter < max_iter; iter ++){
        // clear H and b
        H = Eigen::MatrixXd::Zero(num_variables, num_variables);
        b = Eigen::VectorXd::Zero(num_variables);
        chi_sq = 0; 
        total_iter_chi_sq = 0;
        num_outliers = 0;

        // one iter loop            
        for(int index = 1; index < num_measurements; index+=skip_N){

            // get a measurement (returns delta_z displacement from prev to curr)
            get_sensor_reading(ds, index, time, ticks_steer, delta_ticks_track, delta_z);
          
            // compute error and jacobian
            compute_error_and_Jacobian(x, delta_z, ticks_steer, delta_ticks_track, error, Jacobian);

            // lessen the contribution of measurements having higher error
            chi_sq = error.transpose() * Omega * error; 
            if (chi_sq > kernel_threshold && index > 0) {
                error *= std::sqrt(kernel_threshold/chi_sq);
                chi_sq = kernel_threshold;
                num_outliers += 1;
                total_iter_chi_sq += chi_sq;
                continue;
            }

            // accumulate H and b 
            H += Jacobian.transpose() * Omega * Jacobian;
            b += Jacobian.transpose() * Omega * error;
            
            // std::cout << "H: " << H << std::endl;
            // std::cout << "b: " << b << std::endl;
        }
        
        // calibrate_parameters
        H += 2.0 * Eigen::MatrixXd::Identity(num_variables, num_variables);     // required for numerical stability
        Eigen::VectorXd dx = H.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(-b);
        x = box_plus(x, dx);
        
        error_log.push_back(total_iter_chi_sq);
        outliers_log.push_back(num_outliers);
    }

    std::cout << "\n" << "ICP algorithm terminated " << std::endl;
    std::cout << "x_s: " << x(0)<< std::endl;
    std::cout << "y_s: " << x(1)<< std::endl;
    std::cout << "theta_s: " << x(2)<< std::endl;
    std::cout << "K_steer: " << x(3)<< std::endl;
    std::cout << "K_traction: " << x(4)<< std::endl;
    std::cout << "baseline: " << x(5)<< std::endl;
    std::cout << "steer_offset: " << x(6)<< std::endl;

    //save error for the plot 
    save_error("error_log.csv",error_log, outliers_log);
    //save params 
    save_params("calibrated_params.csv", x);

    return 0;
}