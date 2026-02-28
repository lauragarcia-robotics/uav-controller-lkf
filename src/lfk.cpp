#include <student_headers/controller.h>

namespace task_01_controller
{

using namespace Eigen;

/**
 * @brief the prediction step of the LKF
 *
 * @param x current state vector: x = [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, acc_x, acc_y, acc_z]^T
 * @param x_cov current state covariance: x_cov in R^{9x9}
 * @param input current control input: input = [tilt_xz, tilt_yz, acceleration_z]^T, tilt_xz = desired tilt in the world's XZ [rad], tilt_yz = desired
 * tilt in the world's YZ plane [rad], acceleration_z = desired acceleration along world's z-axis [m/s^2]
 * @param dt the time difference in seconds between now and the last iteration
 *
 * @return <new_state, new_covariance>
 */
std::tuple<Vector9d, Matrix9x9d> Controller::lkfPredict(const Vector9d &x, const Matrix9x9d &x_cov, const Vector3d &input, const double &dt) {

  // x[k+1] = A*x[k] + B*u[k]
    
  Vector9d   new_x;      // the updated state vector, x[k+1]
  Matrix9x9d new_x_cov;  // the updated covariance matrix
  Matrix9x9d A=Matrix9x9d::Identity();
  A.block<3,3>(0,3)=Matrix3d::Identity()*dt;
  A.block<3,3>(0,6)=Matrix3d::Identity()*0.5*dt*dt;
  A.block<3,3>(3,6)=Matrix3d::Identity()*dt;
  Matrix9x3d B=Matrix9x3d::Zero();
  B.block<3,3>(6,0)=Matrix3d::Zero();
  B(6,0)=0.05;
  B(7,1)=0.05;
  B(8,2)=0.01;

  // PUT YOUR CODE HERE
  new_x = A*x +B*input;
  new_x_cov = A*x_cov * A.transpose() + lkf_Q;

  // Print the following values into a log file.
  // * the file will be place in "simulation/student_log.txt"
  // * use this for ploting in custom scipts, e.g., using Matlab or Python.
  //
  // std::stringstream string_to_be_logged;
  // string_to_be_logged << std::fixed << dt << ", " << x[0] << ", " << x[1] << ", " << x[2];
  // action_handlers_.logLine(string_to_be_logged);

  return {new_x, new_x_cov};
}

/**
 * @brief LKF filter correction step
 *
 * @param x current state vector: x = [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, acc_x, acc_y, acc_z]^T
 * @param x_cov current state covariance: x_cov in R^{9x9}
 * @param measurement measurement vector: measurement = [pos_x, pos_y, pos_z, acc_x, acc_y, acc_z]^T
 * @param dt the time difference in seconds between now and the last iteration
 *
 * @return <new_state, new_covariance>
 */
std::tuple<Vector9d, Matrix9x9d> Controller::lkfCorrect(const Vector9d &x, const Matrix9x9d &x_cov, const Vector6d &measurement, const double &dt) {

  Vector9d   new_x;      // the updated state vector, x[k+1]
  Matrix9x9d new_x_cov;  // the updated covariance matrix
  Matrix6x9d H=Matrix6x9d::Zero();
  H.block<3,3>(0,0)=Matrix3d::Identity();
  H.block<3,3>(3,6)=Matrix3d::Identity();

  Matrix6x6d S=H*x_cov*H.transpose()+lkf_R;
  Matrix9x6d K=x_cov*H.transpose()*S.inverse();
  Vector6d y=measurement-H*x;

  // PUT YOUR CODE HERE
  new_x = x+K*y;
  new_x_cov = (Matrix9x9d::Identity()-K*H)*x_cov;

  return {new_x, new_x_cov};
}

}  // namespace task_01_controller
