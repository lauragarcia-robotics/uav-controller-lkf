#include <task_01_controller/utils.h>

#include <student_headers/controller.h>

#include <iostream>

namespace task_01_controller
{

using namespace Eigen;

/**
 * @brief The controller initialization method. It is called ONLY ONCE in the lifetime of the controller.
 * Use this method to do any heavy pre-computations.
 *
 * @param mass UAV mass [kg]
 * @param user_params user-controllable parameters
 * @param g gravitational acceleration [m/s^2]
 * @param action_handlers methods for the user
 */
void Controller::init(const double mass, const UserParams_t user_params, const double g, ActionHandlers_t &action_handlers) {

  // copy the mass and the gravity acceleration
  this->_mass_ = mass;
  this->_g_    = g;

  // the action handlers will allow you to plot data
  this->action_handlers_ = action_handlers;

  // INITIALIZE YOUR CONTROLLER HERE
  err_pos_previous=Vector3d::Zero();
  err_pos_integral=Vector3d::Zero();
  first_iteration=true;

  // INITIALIZE YOUR KALMAN FILTER HERE
  lkf_state=Vector9d::Zero();
  lkf_cov=Matrix9x9d::Identity();
  lkf_initialized=false;

  // SET THE STATE AND THE COVARIANCE MATRICES AS GLOBAL VARIABLES
  lkf_Q.setZero();
  lkf_Q.diagonal().segment<3>(0).setConstant(0.005);
  lkf_Q.diagonal().segment<3>(3).setConstant(0.0045);
  lkf_Q.diagonal().segment<3>(6).setConstant(0.02); 

  lkf_R.setZero();
  lkf_R.diagonal().segment<3>(0).setConstant(0.045);
  lkf_R.diagonal().segment<3>(3).setConstant(0.026);
}

/**
 * @brief This method is called to reset the internal state of the controller, e.g., just before
 * the controller's activation. Use it to, e.g., reset the controllers integrators and estimators.
 */
void Controller::reset() {

  // IT WOULD BE GOOD TO RESET THE PID'S INTEGRALS
  err_pos_previous=Vector3d::Zero();
  err_pos_integral=Vector3d::Zero();
  first_iteration=true;

  // IT WOULD BE NICE TO RESET THE KALMAN'S STATE AND COVARIANCE
  lkf_state=Vector9d::Zero();
  lkf_cov=Matrix9x9d::Identity();
  lkf_initialized=false;
  
  // ALSO, THE NEXT iteration calculateControlSignal() IS GOING TO BE "THE 1ST ITERATION"
}

/**
 * @brief the main routine, is called to obtain the control signals
 *
 * @param uav_state the measured UAV state, contains position and acceleration
 * @param user_params user-controllable parameters
 * @param control_reference the desired state of the UAV, position, velocity, acceleration, heading
 * @param dt the time difference in seconds between now and the last time calculateControlSignal() got called
 *
 * @return the desired control signal: the total thrust force and the desired orientation
 */
std::pair<double, Matrix3d> Controller::calculateControlSignal(const UAVState_t uav_state, const UserParams_t user_params,
                                                               const ControlReference_t control_reference, const double dt) {

  // 
                                                                // Publish the following values as "ROS topics" such that they can be plotted
  // * plotting can be achived using, e.g., the tool called PlotJuggler
  // * try the "plot.sh" script, which will run PlotJuggler
  //
  // action_handlers_.plotValue("pos_x", uav_state.position[0]);
  // action_handlers_.plotValue("pos_y", uav_state.position[1]);
  // action_handlers_.plotValue("pos_z", uav_state.position[2]);

  // publish the following pose as "ROS topic", such that it can be plotted by Rviz
  //
  // action_handlers_.visualizePose("uav_pose_offset", uav_state.position[0], uav_state.position[1], uav_state.position[2] + 1.0, uav_state.heading);

  // Print the following values into a log file.
  // * the file will be place in "simulation/student_log.txt"
  // * use this for ploting in custom scipts, e.g., using Matlab or Python.
  //
  // std::stringstream string_to_be_logged;
  // string_to_be_logged << std::fixed << dt << ", " << uav_state.position[0] << ", " << uav_state.position[1] << ", " << uav_state.position[2];
  // action_handlers_.logLine(string_to_be_logged);

  // | ---------- calculate the output control signals ---------- |

  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // THIS IS THE PLACE FOR YOUR CODE
  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

  // Read parameters defined 
  double P_xy=user_params.param1;
  double D_xy=user_params.param2;
  double I_xy=user_params.param3;
  double P_z=user_params.param4;
  double D_z=user_params.param5;
  double I_z=user_params.param6;

  //Calculate position error, derivate and integral
  Vector3d err_pos=control_reference.position-uav_state.position;
  Vector3d d_err_pos=(err_pos-err_pos_previous)/dt;
  err_pos_integral+=err_pos*dt;
  err_pos_previous=err_pos;

  //PID law
  double acc_x=P_xy*err_pos[0]+D_xy*d_err_pos[0]+I_xy*err_pos_integral[0]+
  control_reference.acceleration[0];
  double acc_y=P_xy*err_pos[1]+D_xy*d_err_pos[1]+I_xy*err_pos_integral[1]+
  control_reference.acceleration[1];
  double acc_z=P_z*err_pos[2]+D_z*d_err_pos[2]+I_z*err_pos_integral[2]+
  control_reference.acceleration[2];

  //Convert accelerations into inclinations
  double des_tilt_x=std::atan2(acc_x,_g_);
  double des_tilt_y=std::atan2(acc_y,_g_);

  //Show 
  action_handlers_.plotValue("err_pos_x",err_pos[0]);
  action_handlers_.plotValue("err_pos_y",err_pos[1]);
  action_handlers_.plotValue("err_pos_z",err_pos[2]);
  // LATER, CALL THE lkfPredict() AND lkfCorrect() FUNCTIONS HERE TO OBTAIN THE FILTERED POSITION STATE
  // DON'T FORGET TO INITIALZE THE STATE DURING THE FIRST ITERATION
  // | ----------- Kalman Filter Integration ----------- |

  if (!lkf_initialized) {
    lkf_state.head<3>() = uav_state.position;         // posición inicial
    lkf_state.segment<3>(6) = uav_state.acceleration; // aceleración inicial
    lkf_initialized = true;
  }

  Vector6d measurement;
  measurement.head<3>() = uav_state.position;
  measurement.tail<3>() = uav_state.acceleration;

  Vector3d input;
  input << des_tilt_x, des_tilt_y, acc_z;

  std::tie(lkf_state, lkf_cov) = lkfPredict(lkf_state, lkf_cov, input, dt);
  std::tie(lkf_state, lkf_cov) = lkfCorrect(lkf_state, lkf_cov, measurement, dt);

  // Opcional: usar posición filtrada en lugar de la medida directa
  // Vector3d filtered_position = lkf_state.head<3>();

  

  // | ---------------- add gravity compensation ---------------- |

  double des_accel_z =acc_z+_g_;

  // | --------------- return the control signals --------------- |

  double   body_thrust;
  Matrix3d desired_orientation;

  std::tie(body_thrust, desired_orientation) = augmentInputs(des_tilt_x, des_tilt_y, des_accel_z * _mass_, control_reference.heading);

  return {body_thrust, desired_orientation};
};

}  // namespace task_01_controller
