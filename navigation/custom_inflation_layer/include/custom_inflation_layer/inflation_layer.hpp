/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
  * Copyright (c) 2008, 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * (license text omitted for brevity)
 *
 * Author: Eitan Marder-Eppstein
 * David V. Lu!!
 *********************************************************************/
#ifndef CUSTOM_INFLATION_LAYER__INFLATION_LAYER_HPP_
#define CUSTOM_INFLATION_LAYER__INFLATION_LAYER_HPP_

#include <map>
#include <vector>
#include <mutex>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/cost_values.hpp" // For cost constants
#include "nav_msgs/msg/path.hpp" // Path 메시지 사용을 위해 추가
#include "nav2_costmap_2d/costmap_2d_ros.hpp"


// Changed namespace to custom_inflation_layer
namespace custom_inflation_layer
{
/**
 * @class CellData
 * @brief Storage for cell information used during obstacle inflation
 */
class CellData
{
public:
  CellData(unsigned int x, unsigned int y, unsigned int sx, unsigned int sy)
  : x_(x), y_(y), src_x_(sx), src_y_(sy)
  {
  }
  unsigned int x_, y_;
  unsigned int src_x_, src_y_;
};

/**
 * @class InflationLayer
 * @brief Layer to convolve costmap by robot's radius or footprint to prevent
 * collisions and largely simply collision checking
 */
class InflationLayer : public nav2_costmap_2d::Layer
{
public:
  InflationLayer();
  virtual ~InflationLayer();

  virtual void onInitialize();
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y);
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);
  virtual void matchSize();
  virtual bool isClearable() {return false;}
  virtual void reset()
  {
    matchSize();
    current_ = false;
  }

  inline unsigned char computeCost(double distance) const
  {
    unsigned char cost = 0;
    if (distance == 0) {
      // FIX: Explicitly specify the namespace for the cost constants
      cost = nav2_costmap_2d::LETHAL_OBSTACLE;
    } else if (distance * resolution_ <= inscribed_radius_) {
      // FIX: Explicitly specify the namespace for the cost constants
      cost = nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
    } else {
      // make sure cost falls off by Euclidean distance
      double factor =
        exp(-1.0 * cost_scaling_factor_ * (distance * resolution_ - inscribed_radius_));
      // FIX: Explicitly specify the namespace for the cost constants
      cost = static_cast<unsigned char>((nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
    }
    return cost;
  }

  // need to test or edit this function below..
  static std::shared_ptr<custom_inflation_layer::InflationLayer> getInflationLayer(
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros,
    const std::string layer_name = "")
  {
    const auto layered_costmap = costmap_ros->getLayeredCostmap();
    for (auto layer = layered_costmap->getPlugins()->begin();
      layer != layered_costmap->getPlugins()->end();
      ++layer)
    {
      auto inflation_layer = std::dynamic_pointer_cast<custom_inflation_layer::InflationLayer>(*layer);
      if (inflation_layer) {
        if (layer_name.empty() || inflation_layer->getName() == layer_name) {
          return inflation_layer;
        }
      }
    }
    return nullptr;
  }
  
  
  typedef std::recursive_mutex mutex_t;
  mutex_t * getMutex() { return access_; }
  double getCostScalingFactor() { return cost_scaling_factor_; }
  double getInflationRadius() { return inflation_radius_; }

protected:
  virtual void onFootprintChanged();

  inline double distanceLookup(
    unsigned int mx, unsigned int my, unsigned int src_x,
    unsigned int src_y)
  {
    unsigned int dx = (mx > src_x) ? mx - src_x : src_x - mx;
    unsigned int dy = (my > src_y) ? my - src_y : src_y - my;
    return cached_distances_[dx * cache_length_ + dy];
  }

  inline unsigned char costLookup(
    unsigned int mx, unsigned int my, unsigned int src_x,
    unsigned int src_y)
  {
    unsigned int dx = (mx > src_x) ? mx - src_x : src_x - mx;
    unsigned int dy = (my > src_y) ? my - src_y : src_y - my;
    return cached_costs_[dx * cache_length_ + dy];
  }

  void computeCaches();
  int generateIntegerDistances();
  unsigned int cellDistance(double world_dist)
  {
    return layered_costmap_->getCostmap()->cellDistance(world_dist);
  }

  inline void enqueue(
    unsigned int index, unsigned int mx, unsigned int my,
    unsigned int src_x, unsigned int src_y, bool is_special);


private: // private으로 변경하여 멤버 변수들 캡슐화

  // 경로 구독 콜백 함수
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);

  // 장애물이 경로와 가까운지 확인하는 헬퍼 함수
  bool isNearPath(unsigned int mx, unsigned int my);

  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  double inflation_radius_, inscribed_radius_, cost_scaling_factor_;
  bool inflate_unknown_, inflate_around_unknown_;
  unsigned int cell_inflation_radius_;
  unsigned int cached_cell_inflation_radius_;
  std::vector<std::vector<CellData>> inflation_cells_;
  double resolution_;
  std::vector<bool> seen_;
  std::vector<unsigned char> cached_costs_;
  std::vector<double> cached_distances_;
  std::vector<std::vector<int>> distance_matrix_;
  unsigned int cache_length_;
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
  bool need_reinflation_;
  mutex_t * access_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  // --- 새로 추가된 멤버 변수들 ---
  bool use_path_aware_inflation_;
  double path_proximity_threshold_;
  double special_inflation_radius_;
  unsigned int special_cell_inflation_radius_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  nav_msgs::msg::Path::SharedPtr current_global_path_;
  std::mutex path_mutex_;
  // ** 수정된 부분 끝 **
};

}  // namespace custom_inflation_layer

#endif  // CUSTOM_INFLATION_LAYER__INFLATION_LAYER_HPP_