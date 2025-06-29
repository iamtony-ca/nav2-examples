/*********************************************************************
 * (라이선스 헤더는 이전과 동일)
 *********************************************************************/
#include "custom_inflation_layer/inflation_layer.hpp"

#include <limits>
#include <map>
#include <vector>
#include <algorithm>
#include <utility>

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include "nav_msgs/msg/path.hpp"
#include <tf2/utils.h>

PLUGINLIB_EXPORT_CLASS(custom_inflation_layer::InflationLayer, nav2_costmap_2d::Layer)

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::FREE_SPACE;
using rcl_interfaces::msg::ParameterType;

namespace custom_inflation_layer
{

// 생성자, 소멸자는 이전과 동일
InflationLayer::InflationLayer()
: inflation_radius_(0),
  inscribed_radius_(0),
  cost_scaling_factor_(0),
  inflate_unknown_(false),
  inflate_around_unknown_(false),
  cell_inflation_radius_(0),
  cached_cell_inflation_radius_(0),
  resolution_(0),
  cache_length_(0),
  last_min_x_(std::numeric_limits<double>::lowest()),
  last_min_y_(std::numeric_limits<double>::lowest()),
  last_max_x_(std::numeric_limits<double>::max()),
  last_max_y_(std::numeric_limits<double>::max())
{
  access_ = new mutex_t();
}

InflationLayer::~InflationLayer()
{
  auto node = node_.lock();
  if (dyn_params_handler_ && node) {
    node->remove_on_set_parameters_callback(dyn_params_handler_.get());
  }
  dyn_params_handler_.reset();
  delete access_;
}


void InflationLayer::onInitialize()
{
  // 파라미터 선언은 이전과 동일
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("inflation_radius", rclcpp::ParameterValue(0.25));
  declareParameter("cost_scaling_factor", rclcpp::ParameterValue(5.0));
  declareParameter("inflate_unknown", rclcpp::ParameterValue(false));
  declareParameter("inflate_around_unknown", rclcpp::ParameterValue(false));
  declareParameter("use_path_aware_inflation", rclcpp::ParameterValue(true));
  declareParameter("path_proximity_threshold", rclcpp::ParameterValue(0.5));
  declareParameter("special_inflation_radius", rclcpp::ParameterValue(0.6));
  declareParameter("global_path_topic", rclcpp::ParameterValue("/plan"));

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // 파라미터 읽기는 이전과 동일
  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "inflation_radius", inflation_radius_);
  node->get_parameter(name_ + "." + "cost_scaling_factor", cost_scaling_factor_);
  node->get_parameter(name_ + "." + "inflate_unknown", inflate_unknown_);
  node->get_parameter(name_ + "." + "inflate_around_unknown", inflate_around_unknown_);
  node->get_parameter(name_ + "." + "use_path_aware_inflation", use_path_aware_inflation_);
  node->get_parameter(name_ + "." + "path_proximity_threshold", path_proximity_threshold_);
  node->get_parameter(name_ + "." + "special_inflation_radius", special_inflation_radius_);
  std::string path_topic;
  node->get_parameter(name_ + "." + "global_path_topic", path_topic);

  if (use_path_aware_inflation_) {
    RCLCPP_INFO(logger_, "Path aware inflation is enabled.");
    path_sub_ = node->create_subscription<nav_msgs::msg::Path>(
      path_topic,
      rclcpp::SystemDefaultsQoS(),
      std::bind(&InflationLayer::pathCallback, this, std::placeholders::_1));
  }

  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&InflationLayer::dynamicParametersCallback, this, std::placeholders::_1));

  current_ = true;
  seen_.clear();
  cached_distances_.clear();
  cached_costs_.clear();
  need_reinflation_ = false;

  matchSize();
}

void InflationLayer::matchSize()
{
  std::lock_guard<mutex_t> guard(*getMutex());
  nav2_costmap_2d::Costmap2D * costmap = layered_costmap_->getCostmap();
  resolution_ = costmap->getResolution();
  
  // --- 수정된 부분 ---
  // 두 반경 중 더 큰 값을 기준으로 캐시 크기 계산을 위한 cell_inflation_radius_를 설정합니다.
  double max_inflation_radius = std::max(inflation_radius_, special_inflation_radius_);
  cell_inflation_radius_ = cellDistance(max_inflation_radius);
  // special_cell_inflation_radius_는 이제 enqueue의 조건 검사용으로만 사용됩니다.
  special_cell_inflation_radius_ = cellDistance(special_inflation_radius_);
  
  computeCaches();
  seen_ = std::vector<bool>(costmap->getSizeInCellsX() * costmap->getSizeInCellsY(), false);
}

// updateBounds, onFootprintChanged, pathCallback, isNearPath 함수는 이전과 동일
void InflationLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
  std::lock_guard<mutex_t> guard(*getMutex());
  if (need_reinflation_) {
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;

    *min_x = std::numeric_limits<double>::lowest();
    *min_y = std::numeric_limits<double>::lowest();
    *max_x = std::numeric_limits<double>::max();
    *max_y = std::numeric_limits<double>::max();
    need_reinflation_ = false;
  } else {
    double tmp_min_x = last_min_x_;
    double tmp_min_y = last_min_y_;
    double tmp_max_x = last_max_x_;
    double tmp_max_y = last_max_y_;
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;

    // 업데이트 범위를 정할 때도 가장 큰 반경을 고려해야 합니다.
    double max_radius = std::max(inflation_radius_, special_inflation_radius_);
    *min_x = std::min(tmp_min_x, *min_x) - max_radius;
    *min_y = std::min(tmp_min_y, *min_y) - max_radius;
    *max_x = std::max(tmp_max_x, *max_x) + max_radius;
    *max_y = std::max(tmp_max_y, *max_y) + max_radius;
  }
}

void
InflationLayer::onFootprintChanged()
{
  std::lock_guard<mutex_t> guard(*getMutex());
  inscribed_radius_ = layered_costmap_->getInscribedRadius();
  // matchSize()가 max_radius 기준으로 cell_inflation_radius_를 다시 계산할 것이므로, 여기서도 호출
  matchSize(); 
  need_reinflation_ = true;

  RCLCPP_DEBUG(
    logger_, "InflationLayer::onFootprintChanged(): num footprint points: %zu,"
    " inscribed_radius_ = %.3f, inflation_radius_ = %.3f",
    layered_costmap_->getFootprint().size(), inscribed_radius_, inflation_radius_);
}

void InflationLayer::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(path_mutex_);
  current_global_path_ = msg;
}

bool InflationLayer::isNearPath(unsigned int mx, unsigned int my)
{
  std::lock_guard<std::mutex> lock(path_mutex_);
  if (!current_global_path_ || current_global_path_->poses.empty()) {
    return false;
  }
  double wx, wy;
  layered_costmap_->getCostmap()->mapToWorld(mx, my, wx, wy);
  double min_dist_sq = std::numeric_limits<double>::max();
  for (const auto & pose : current_global_path_->poses) {
    double dx = pose.pose.position.x - wx;
    double dy = pose.pose.position.y - wy;
    min_dist_sq = std::min(min_dist_sq, dx * dx + dy * dy);
  }
  return min_dist_sq < (path_proximity_threshold_ * path_proximity_threshold_);
}


void InflationLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
  int max_i,
  int max_j)
{
  std::lock_guard<mutex_t> guard(*getMutex());
  // --- 수정된 부분 ---
  // cell_inflation_radius_가 max_radius 기준으로 계산되므로, 0일때만 체크하면 됨.
  if (!enabled_ || cell_inflation_radius_ == 0) {
    return;
  }

  // updateBounds를 수정했으므로, 이 로직은 더이상 필요하지 않거나 단순화될 수 있습니다.
  // 하지만 안전을 위해 유지합니다.
  double max_radius = std::max(inflation_radius_, special_inflation_radius_);
  unsigned int cell_max_radius = cellDistance(max_radius);

  unsigned char * master_array = master_grid.getCharMap();
  unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

  if (seen_.size() != size_x * size_y) {
    RCLCPP_WARN(
      logger_, "InflationLayer::updateCosts(): seen_ vector size is wrong");
    seen_ = std::vector<bool>(size_x * size_y, false);
  }
  std::fill(seen_.begin(), seen_.end(), false);

  const int base_min_i = min_i;
  const int base_min_j = min_j;
  const int base_max_i = max_i;
  const int base_max_j = max_j;
  
  min_i -= static_cast<int>(cell_max_radius);
  min_j -= static_cast<int>(cell_max_radius);
  max_i += static_cast<int>(cell_max_radius);
  max_j += static_cast<int>(cell_max_radius);

  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(static_cast<int>(size_x), max_i);
  max_j = std::min(static_cast<int>(size_y), max_j);

  auto & obs_bin = inflation_cells_[0];
  for (int j = min_j; j < max_j; j++) {
    for (int i = min_i; i < max_i; i++) {
      int index = static_cast<int>(master_grid.getIndex(i, j));
      unsigned char cost = master_array[index];
      if (cost == LETHAL_OBSTACLE || (inflate_around_unknown_ && cost == NO_INFORMATION)) {
        obs_bin.emplace_back(i, j, i, j);
      }
    }
  }
  
  // 이 부분부터의 로직은 이전과 동일하게 유지됩니다.
  // is_special 플래그를 결정하고 enqueue에 전달하는 방식은 그대로입니다.
  for (auto & dist_bin : inflation_cells_) {
      for (std::size_t i = 0; i < dist_bin.size(); ++i) {
          const CellData & cell = dist_bin[i];
          unsigned int index = master_grid.getIndex(cell.x_, cell.y_);

          if (seen_[index]) {
              continue;
          }
          seen_[index] = true;
          
          bool is_special = use_path_aware_inflation_ && isNearPath(cell.src_x_, cell.src_y_);
          
          unsigned char cost = costLookup(cell.x_, cell.y_, cell.src_x_, cell.src_y_);
          unsigned char old_cost = master_array[index];
          
          if (static_cast<int>(cell.x_) >= base_min_i &&
            static_cast<int>(cell.y_) >= base_min_j &&
            static_cast<int>(cell.x_) < base_max_i &&
            static_cast<int>(cell.y_) < base_max_j)
          {
            if (old_cost == NO_INFORMATION &&
              (inflate_unknown_ ? (cost > FREE_SPACE) : (cost >= INSCRIBED_INFLATED_OBSTACLE))) {
                  master_array[index] = cost;
              } else {
                  master_array[index] = std::max(old_cost, cost);
              }
          }
          
          if (cell.x_ > 0) {
              enqueue(index - 1, cell.x_ - 1, cell.y_, cell.src_x_, cell.src_y_, is_special);
          }
          if (cell.y_ > 0) {
              enqueue(index - size_x, cell.x_, cell.y_ - 1, cell.src_x_, cell.src_y_, is_special);
          }
          if (cell.x_ < size_x - 1) {
              enqueue(index + 1, cell.x_ + 1, cell.y_, cell.src_x_, cell.src_y_, is_special);
          }
          if (cell.y_ < size_y - 1) {
              enqueue(index + size_x, cell.x_, cell.y_ + 1, cell.src_x_, cell.src_y_, is_special);
          }
    }
    dist_bin.clear();
    dist_bin.shrink_to_fit();
  }

  current_ = true;
}

void InflationLayer::enqueue(
  unsigned int index, unsigned int mx, unsigned int my,
  unsigned int src_x, unsigned int src_y, bool is_special)
{
  if (!seen_[index]) {
    double distance = distanceLookup(mx, my, src_x, src_y);
    
    // --- 수정된 부분 ---
    // is_special 플래그에 따라 사용할 inflation 반경(cell 단위) 결정
    unsigned int normal_cell_radius = cellDistance(inflation_radius_);
    unsigned int radius_to_use = is_special ? special_cell_inflation_radius_ : normal_cell_radius;
    
    if (distance > radius_to_use) {
      return;
    }
    
    // --- 수정된 부분 (핵심 버그 수정) ---
    // distance_matrix_의 인덱스 계산에 사용되는 'r'은 이제 항상 캐시 생성 시 사용된
    // 최대 반경 기준의 cell_inflation_radius_를 사용하므로 안전합니다.
    const unsigned int r = cell_inflation_radius_ + 2;
    const auto dist = distance_matrix_[mx - src_x + r][my - src_y + r];
    inflation_cells_[dist].emplace_back(mx, my, src_x, src_y);
  }
}

// computeCaches, generateIntegerDistances, dynamicParametersCallback 함수는 이전과 거의 동일
void InflationLayer::computeCaches()
{
  std::lock_guard<mutex_t> guard(*getMutex());
  // cell_inflation_radius_는 이미 matchSize에서 max값으로 계산됨
  if (cell_inflation_radius_ == 0) {
    return;
  }
  
  // cache_length_는 cell_inflation_radius_ (max 기준)에 따라 계산됨
  cache_length_ = cell_inflation_radius_ + 2;

  if (cell_inflation_radius_ != cached_cell_inflation_radius_) {
    cached_costs_.resize(cache_length_ * cache_length_);
    cached_distances_.resize(cache_length_ * cache_length_);
    for (unsigned int i = 0; i < cache_length_; ++i) {
      for (unsigned int j = 0; j < cache_length_; ++j) {
        cached_distances_[i * cache_length_ + j] = hypot(i, j);
      }
    }
    cached_cell_inflation_radius_ = cell_inflation_radius_;
  }

  for (unsigned int i = 0; i < cache_length_; ++i) {
    for (unsigned int j = 0; j < cache_length_; ++j) {
      cached_costs_[i * cache_length_ + j] = computeCost(cached_distances_[i * cache_length_ + j]);
    }
  }

  int max_dist = generateIntegerDistances();
  inflation_cells_.clear();
  inflation_cells_.resize(max_dist + 1);
}

int InflationLayer::generateIntegerDistances()
{
  // 이 함수는 cell_inflation_radius_를 사용하며, 이 값은 이미 max_radius 기준으로 설정되어 있으므로 안전합니다.
  const int r = cell_inflation_radius_ + 2;
  const int size = r * 2 + 1;
  // ... 이하 로직 동일 ...
  std::vector<std::pair<int, int>> points;
  for (int y = -r; y <= r; y++) {
    for (int x = -r; x <= r; x++) {
      if (x * x + y * y <= r * r) {
        points.emplace_back(x, y);
      }
    }
  }
  std::sort(
    points.begin(), points.end(),
    [](const std::pair<int, int> & a, const std::pair<int, int> & b) -> bool {
      return a.first * a.first + a.second * a.second < b.first * b.first + b.second * b.second;
    }
  );
  std::vector<std::vector<int>> distance_matrix(size, std::vector<int>(size, 0));
  std::pair<int, int> last = {0, 0};
  int level = 0;
  for (auto const & p : points) {
    if (p.first * p.first + p.second * p.second !=
      last.first * last.first + last.second * last.second)
    {
      level++;
    }
    distance_matrix[p.first + r][p.second + r] = level;
    last = p;
  }
  distance_matrix_ = distance_matrix;
  return level;
}

rcl_interfaces::msg::SetParametersResult
InflationLayer::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<mutex_t> guard(*getMutex());
  rcl_interfaces::msg::SetParametersResult result;
  bool need_cache_recompute = false;

  for (auto parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();

    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      // --- 수정된 부분: special_inflation_radius 변경 감지 ---
      if (param_name == name_ + "." + "inflation_radius") {
        inflation_radius_ = parameter.as_double();
        need_reinflation_ = true;
        need_cache_recompute = true;
      } else if (param_name == name_ + "." + "special_inflation_radius") {
        special_inflation_radius_ = parameter.as_double();
        need_reinflation_ = true;
        need_cache_recompute = true;
      } else if (param_name == name_ + "." + "cost_scaling_factor") {
        cost_scaling_factor_ = parameter.as_double();
        need_reinflation_ = true;
        need_cache_recompute = true;
      }
    } else if (param_type == ParameterType::PARAMETER_BOOL) {
      if (param_name == name_ + "." + "enabled") {
        enabled_ = parameter.as_bool();
        need_reinflation_ = true;
        current_ = false;
      } else if (param_name == name_ + "." + "inflate_unknown") {
        inflate_unknown_ = parameter.as_bool();
        need_reinflation_ = true;
      } else if (param_name == name_ + "." + "inflate_around_unknown") {
        inflate_around_unknown_ = parameter.as_bool();
        need_reinflation_ = true;
      }
    }
  }

  if (need_cache_recompute) {
    matchSize(); // matchSize가 max_radius 기준으로 캐시를 다시 계산해 줄 것임
  }
  result.successful = true;
  return result;
}


}  // namespace custom_inflation_layer