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
    double theta;

    // integrate the model
    tr.update_tricycle_state(ticks_steer, ticks_track);
    pos_robot = tr.pos();
    theta = tr.theta();
    // std::cout << "\n" << "x: " << tr.x() << std::endl;
    // std::cout << "y: " << tr.y() << std::endl;
    // std::cout << "theta: " << tr.theta() << std::endl;
    // std::cout << "phi: " << tr.phi() << "\n" << std::endl;

    Eigen::Matrix2d R;
    const double c = std::cos(theta);
    const double s = std::sin(theta);
    R << c, -s,
         s,  c;

    h = pos_robot + R * pos_sensor;          // position of the sensor expressed in world reference frame   
    error = h - z;

    // compute the jacobian
    J << 1.1, 1.2, 0.5, 0.7, 0.9, 1.0,
         1.3, 0.2, 0.5, 0.2, 0.3, 1.1;

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

    Eigen::Vector2d error = Eigen::Vector2d::Zero();
    Eigen::MatrixXd Jacobian = Eigen::MatrixXd::Zero(dim_measurement, num_variables);
    
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(num_variables, num_variables);
    Eigen::MatrixXd b = Eigen::VectorXd::Zero(num_variables);

    Eigen::Vector2d z;
    Eigen::VectorXd x(num_variables);
    x << ds.laser.tx,
        ds.laser.ty,
        ds.params.Ksteer,
        ds.params.Ktraction,
        ds.params.axis_length,
        ds.params.steer_offset;

    Eigen::Matrix2d Omega;
    Omega << 1.0, 0.0,
             0.0, 1.0;



    // ICP loop            num_measurements
    for(int index = 0; index < 10; index ++){

        // get a measurement
        get_sensor_reading(ds, index, time, ticks_steer, ticks_track, z);
        // std::cout << "index: " << index << std::endl;
        // std::cout << "time: " << time << std::endl;
        // std::cout << "ticks_steer: " << ticks_steer << std::endl;
        // std::cout << "ticks_track: " << ticks_track << std::endl;
        // std::cout << "z: " << z << std::endl;
        // std::cout << "\n " << std::endl;
        
        // update kinematic parameters in the kinematic model
        tr.updateParams(x.tail(x.size() - 2));
        
        // compute error and jacobian
        compute_error_and_Jacobian(x, z, ticks_steer, ticks_track, error, Jacobian);

        std::cout << "error" << error << std::endl;
        std::cout << "Jacobian" << Jacobian << std::endl;


        // calibrate_parameters
        H += Jacobian.transpose()* Omega * Jacobian;
        b += Jacobian.transpose()* Omega * error;
        std::cout << "H: " << H << std::endl;
        std::cout << "b: " << b << std::endl;

        Eigen::VectorXd dx = H.ldlt().solve(-b);
        std::cout << "dx: " << dx << std::endl;
        x += dx;
    }

    std::cout << "\n" << "ICP algorithm terminated " << std::endl;
    std::cout << "x_s: " << x(0)<< std::endl;
    std::cout << "y_s: " << x(1)<< std::endl;
    std::cout << "Ksteer: " << x(2)<< std::endl;
    std::cout << "K_traction: " << x(3)<< std::endl;
    std::cout << "baseline_: " << x(4)<< std::endl;
    std::cout << "steer_offset: " << x(5)<< std::endl;
    return 0;
}