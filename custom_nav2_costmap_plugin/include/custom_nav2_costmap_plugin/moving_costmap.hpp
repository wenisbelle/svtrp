#ifndef MOVING_COSTMAP_HPP_
#define MOVING_COSTMAP_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include <math.h>

namespace custom_moving_costmap
{

class CustomMovingCostmap : public nav2_costmap_2d::Layer
{
public:
  CustomMovingCostmap();

  virtual void onInitialize();
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y);
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);

  virtual void reset()
  {
    return;
  }

  virtual void onFootprintChanged();

  virtual bool isClearable() {return false;}

  bool pointInsideCircle(int x_point, int y_point);

private:
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;

  // Indicates that the entire gradient should be recalculated next time.
  bool need_recalculation_;

  // Size of gradient in cells
  int GRADIENT_SIZE = 20;
  // Step of increasing cost per one cell in gradient
  int GRADIENT_FACTOR = 10;
  
  // Init Crazy Circle pose
  float circle_p_x;
  float circle_p_y;
  float circle_p_y_delta = 0;
  bool delta_flag = true;
  int obstacle_radius = 30;
  float x_oscilation_height;
  float y_oscilation_range;
};

}  // namespace 

#endif  // COSTMAP_HPP_