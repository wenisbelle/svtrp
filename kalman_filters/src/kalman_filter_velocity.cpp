#include "geometry_msgs/msg/twist.hpp"
#include "rcl/node_options.h"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <array>

#include <chrono>
#include <cstdlib>
#include <inttypes.h>
#include <iostream>
#include <memory>
#include <thread>

using namespace std::chrono_literals;


/************************************
************FIRST CLASS *************
*************************************/

class Kalman_Filter : public rclcpp::Node {
public:

  explicit Kalman_Filter(
      const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
      : Node("kalman_filter_velocity", node_options)
      {
    
    // Esse é o meu construtor posso inicializar as variaveis aqui

    // Initialize the MutuallyExclusive callback group object
    callback_publisher_odometry_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    callback_subscriber_mesurement_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
        
    callback_subscriber_model_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions options_model;
    options_model.callback_group = callback_subscriber_model_group_;

    rclcpp::SubscriptionOptions options_mesurement;
    options_mesurement.callback_group = callback_subscriber_mesurement_group_; 

    publisher_ =
        this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    timer_ = this->create_wall_timer(
        50ms, std::bind(&Kalman_Filter::timer_callback, this),
        callback_publisher_odometry_group_); // 500 antes

    subscription_mesurement_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu", 10,
        std::bind(&Kalman_Filter::mesurement_callback, this, std::placeholders::_1),
        options_mesurement);

    subscription_model_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "robot_base_controller/odom", 10,
        std::bind(&Kalman_Filter::model_callback, this, std::placeholders::_1),
        options_model);
  }


private:

  struct CorrectedValues{
    Eigen::Matrix<float, 2, 1> corrected_x;
    Eigen::Matrix<float, 2, 2> corrected_P;

  };

  void timer_callback() {
    //auto message = nav_msgs::msg::Odometry();
    //publisher_->publish(message);
    std::this_thread::sleep_for(1000ms);
    std::cout << "publisher callback" << std::endl;
    corrected_parameters = correction_step(x_model, P_model, z);
  }

 
  void mesurement_callback(const sensor_msgs::msg::Imu::SharedPtr imu) {
    std::this_thread::sleep_for(1000ms);
    std::cout << "mesurement callback" << std::endl;
    //ARRUMAR ISSO DE ACORDO COM O SENSOR
     z << imu->linear_acceleration.x, 
               imu->angular_velocity.z;

    std::cout << "Dados do sensor" << std::endl;
    std::cout << z << std::endl;
  }

  void model_callback(const nav_msgs::msg::Odometry::SharedPtr model) {
    std::this_thread::sleep_for(1000ms);
    
    x_model << model->twist.twist.linear.x,
               model->twist.twist.angular.z;
    std::cout << x_model << std::endl;
    std::cout << "model callback" << std::endl;
  }

  // Fazer as funções pra receber a velocidade das rodas e transformar em velocidade linear e angular - Tenho isso no Matlab
  // Fazer a funcção pra receber os dados do IMU e transformar nas velocidades lineares e angulares - Preciso só ver como fazer integações numéricas no ROS

  CorrectedValues correction_step(Eigen::Matrix<float, 2, 1>& predicted_x,
                                  Eigen::Matrix<float, 2, 2>& last_P,
                                  Eigen::Matrix<float, 2, 1>& mesured_z){

    CorrectedValues corrected_values;
       
    // residue
    auto y = mesured_z - H*predicted_x;

    // new covariance 
    auto Sk = H*last_P*H.transpose() + R;

    // Kalman Gain
    auto K = last_P*H.transpose()*Sk.inverse();

    // new state
    corrected_values.corrected_x = predicted_x + K*y;

    // new covariance
    corrected_values.corrected_P = (Eigen::Matrix2f::Identity() - K*H)*last_P;

    return corrected_values;
  }

  Eigen::Matrix<float, 2, 2> Transform_Float2Matrix2f(float* array){
    Eigen::Matrix<float, 2, 2> matrix;
    matrix << array[XX_TWIST_ODOMETRY_COEFICIENT], array[XZ_TWIST_ODOMETRY_COEFICIENT],
              array[ZX_TWIST_ODOMETRY_COEFICIENT], array[ZZ_TWIST_ODOMETRY_COEFICIENT];
    return matrix;
  }

    
    std::array<float, 36> Transform_Matrix2f2Float(Eigen::Matrix<float, 2, 2>& matrix)
    {    
        std::array<float, 36> array;
        
        for (int i = 0; i < 36; i++) {
            array[i] = 0.0f;
        }
        
        array[XX_TWIST_ODOMETRY_COEFICIENT] = static_cast<float>(matrix.coeff(0, 0));
        array[XZ_TWIST_ODOMETRY_COEFICIENT] = static_cast<float>(matrix.coeff(0, 1));
        array[ZX_TWIST_ODOMETRY_COEFICIENT] = static_cast<float>(matrix.coeff(1, 0));
        array[ZZ_TWIST_ODOMETRY_COEFICIENT] = static_cast<float>(matrix.coeff(1, 1));

        return array;
    }

  
  // Kalman Filter variables
  Eigen::Matrix<float, 2, 1> x_model;
  Eigen::Matrix<float, 2, 2> P_model = Eigen::Matrix2f::Zero();
  Eigen::Matrix<float, 2, 1> x;
  Eigen::Matrix<float, 2, 2> P;
  Eigen::Matrix<float, 2, 2> Q;
	Eigen::Matrix<float, 2, 2> F;
	Eigen::Matrix<float, 2, 2> B;
	Eigen::Matrix<float, 2, 1> z; // mesurement
	Eigen::Matrix<float, 2, 2> H;
	Eigen::Matrix<float, 2, 2> R;
  CorrectedValues corrected_parameters;
  


    // These values are because of the conversion between the matrices and the float arrays of the 
    // ros2 messages for covariance
    const int XX_TWIST_ODOMETRY_COEFICIENT = 0;   
    const int XZ_TWIST_ODOMETRY_COEFICIENT = 5;
    const int ZX_TWIST_ODOMETRY_COEFICIENT = 30;
    const int ZZ_TWIST_ODOMETRY_COEFICIENT = 35;

  // ROS variables
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_mesurement_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_model_;
  rclcpp::CallbackGroup::SharedPtr callback_publisher_odometry_group_;
  rclcpp::CallbackGroup::SharedPtr callback_subscriber_mesurement_group_;
  rclcpp::CallbackGroup::SharedPtr callback_subscriber_model_group_;
};



/************************************
************** MAIN *****************
*************************************/

int main(int argc, char *argv[]) {
  
  rclcpp::init(argc, argv);

  std::shared_ptr<Kalman_Filter> Kalman_Filter_Velocity_Node =
      std::make_shared<Kalman_Filter>();

  // Initialize one MultiThreadedExecutor object
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(Kalman_Filter_Velocity_Node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}