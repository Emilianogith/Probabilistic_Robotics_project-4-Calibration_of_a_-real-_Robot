#include "utils.h"
#include "tricycle_model.h"

double wrapToPi(double a) {
    a = std::remainder(a, TWO_PI);  // (-π, π]
    return a;
}

double wrapTwoPi(double a) {
    a = std::remainder(a, TWO_PI);  // (-π, π]
    if (a < 0) a += TWO_PI;         // (0, 2π]
    if (a == TWO_PI) a = 0.0;       // [0, 2π)
    return a;
}

void print_infos(Dataset& ds){
    std::cout << "Parameters:\n"
              << "  Ksteer=" << ds.params.Ksteer
              << " Ktraction=" << ds.params.Ktraction
              << " axis_length=" << ds.params.axis_length
              << " steer_offset=" << ds.params.steer_offset << "\n";

    std::cout << "Joints: ";
    for (size_t i=0;i<ds.joints.names.size();++i) {
        std::cout << (i? ", ":"") << ds.joints.names[i];
        if (i < ds.joints.values.size()) std::cout << "(" << ds.joints.values[i] << ")";
    }
    std::cout << "\n";

    std::cout << "Laser extrinsics (base->laser): t=[" << ds.laser.tx << "," << ds.laser.ty << "," << ds.laser.tz
              << "], q=[" << ds.laser.qx << "," << ds.laser.qy << "," << ds.laser.qz << "," << ds.laser.qw << "]\n";

    std::cout << "Loaded " << ds.records.size() << " records.\n";
    if (!ds.records.empty()) {
        const auto& r = ds.records.front();
        std::cout << "First record: time=" << r.time
                  << " ticks=(" << r.ticks_steer << "," << r.ticks_trac << ") "
                  << "model_pose=(" << r.model_x << "," << r.model_y << "," << r.model_th << ") "
                  << "tracker_pose=(" << r.track_x << "," << r.track_y << "," << r.track_th << ")\n";
    }
}

double quaternionToYaw(double qx, double qy, double qz, double qw) {
    return std::atan2(2.0 * (qw * qz + qx * qy),
                      1.0 - 2.0 * (qy * qy + qz * qz));
};

Eigen::Matrix3d v2t(const Eigen::Vector3d params){
    Eigen::Matrix3d T = Eigen::Matrix3d::Identity();  // Homogeneus transformation matrix SE(2)
    Eigen::Rotation2D<double> Rot(params(2));
    Eigen::Matrix2d R = Rot.toRotationMatrix();
    Eigen::Vector2d p = Eigen::Vector2d(params(0), params(1));
    T.block(0,0,2,2) = R;
    T.block(0,2,2,1) = p;
    return T;
};
Eigen::Vector3d t2v(const Eigen::Matrix3d T){
    Eigen::Vector3d params;
    params.head(2) = T.block(0,2,2,1);
    const double c = T(0,0);
    const double s = T(1,0);
    params(2) = std::atan2(s,c);
    return params;
};

Eigen::VectorXd box_minus(const Eigen::VectorXd& h, const Eigen::VectorXd delta_z){
    /* delta_z: measurement displacement during the motion (from step i to step i+1) of the tricycle w.r.t. the sensor RF at step i
        h: observation function */
    return t2v(v2t(delta_z).inverse() * v2t(h));
}

Eigen::VectorXd box_plus(const Eigen::VectorXd x, const Eigen::VectorXd delta_x){
    assert(x.size() == delta_x.size() && x.size() == 7);
    Eigen::VectorXd  new_x(x.size());
    new_x.head(3) = t2v(v2t(delta_x.head(3)) * v2t(x.head(3)));
    new_x.tail(4) = x.tail(4) + delta_x.tail(4); 
    return new_x;
}

Eigen::MatrixXd numericalJacobian(
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& delta_z,
    Tricycle& tr,
    int32_t& ticks_steer,
    int32_t& delta_ticks_track,
    double epsilon)
{
    Eigen::VectorXd robot_displacement  = tr.tricycle_step(x.tail(4), ticks_steer, delta_ticks_track);
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(delta_z.size(),x.size());
    
    for (int i = 0; i < x.size(); ++i) {
        Eigen::VectorXd perturbation_vector = Eigen::VectorXd::Zero(x.size());
        perturbation_vector(i) = epsilon;
        Eigen::VectorXd x_plus  = box_plus(x, perturbation_vector);
        Eigen::VectorXd x_minus = box_plus(x, -perturbation_vector);

        Eigen::VectorXd h_plus;
        Eigen::VectorXd h_minus;

        // Pertrurb only the pose of the sensor
        if (i < 3){
            Eigen::VectorXd sensor_pose_plus = x_plus.head(3);
            Eigen::VectorXd sensor_pose_minus = x_minus.head(3);
            
            h_plus = t2v(v2t(sensor_pose_plus).inverse() * v2t(robot_displacement) * v2t(sensor_pose_plus));
            h_minus = t2v(v2t(sensor_pose_minus).inverse() * v2t(robot_displacement) * v2t(sensor_pose_minus));

        }
        // Pertrurb only the kinematic params
        else{ 
            Eigen::VectorXd sensor_pose = x.head(3);

            Eigen::VectorXd robot_displacement_plus  = tr.tricycle_step(x_plus.tail(4), ticks_steer, delta_ticks_track);
            Eigen::VectorXd robot_displacement_minus = tr.tricycle_step(x_minus.tail(4), ticks_steer, delta_ticks_track);
               
            h_plus = t2v(v2t(sensor_pose).inverse() * v2t(robot_displacement_plus) * v2t(sensor_pose));
            h_minus = t2v(v2t(sensor_pose).inverse() * v2t(robot_displacement_minus) * v2t(sensor_pose));
        };

        Eigen::VectorXd error_plus = box_minus(h_plus, delta_z);
        Eigen::VectorXd error_minus = box_minus(h_minus, delta_z);

        J.col(i) = (error_plus - error_minus) / (2.0 * epsilon);
    }
    return J;
}

void get_sensor_reading(
    Dataset& ds, 
    int& index, 
    double& time, 
    int32_t& ticks_steer,
    int32_t& ticks_track, 
    Eigen::VectorXd& z){
    
    if (index < 0 || index >= static_cast<int>(ds.records.size())) {
        throw std::out_of_range("Index out of range in get_sensor_readings");
    }

    const auto& r = ds.records[index];
    const auto& prev_r = (index == 0) ? r : ds.records[index - 1];

    const uint32_t max_steer    = static_cast<uint32_t>(ds.joints.values[0]);
    const uint32_t max_traction = static_cast<uint32_t>(ds.joints.values[1]);
    
    time = r.time;
    ticks_steer = r.ticks_steer;    
    ticks_track = static_cast<int32_t>(r.ticks_trac - prev_r.ticks_trac);

    Eigen::Matrix3d T_i = v2t(Eigen::Vector3d(r.track_x, r.track_y, r.track_th));
    Eigen::Matrix3d T_i_prev = v2t(Eigen::Vector3d(prev_r.track_x, prev_r.track_y, prev_r.track_th));

    z = t2v(T_i_prev.inverse() * T_i);
}

void save_error(const std::string& filename, const std::vector<double>& error_log, const std::vector<int>& outliers_log) {
    std::ofstream out(filename);
    out << "iter, error, outliers\n";
    out << std::fixed << std::setprecision(10);
    for (size_t i = 0; i < error_log.size(); ++i) {
        out << i << ", " << error_log[i] << ", " << outliers_log[i] << "\n";
    }
}

void save_params(const std::string& filename, const Eigen::VectorXd& x) {
    std::ofstream out(filename);
    out << "param, value\n";
    out << std::fixed << std::setprecision(10);
    out << "x_s: " << x(0) << "\n";
    out << "y_s: " << x(1) << "\n";
    out << "theta_s: " << x(2) << "\n";
    out << "K_steer: " << x(3) << "\n";
    out << "K_traction: " << x(4) << "\n";
    out << "baseline: " << x(5) << "\n";
    out << "steer_offset: " << x(6) << "\n";
    
}
