void InflationLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
  int max_i,
  int max_j)
{
  std::lock_guard<mutex_t> guard(*getMutex());
  if (!enabled_) {
    return;
  }

  // cell_inflation_radius_는 항상 max_radius를 기준으로 계산되므로 0일때만 체크하면 됨
  double max_radius = std::max(inflation_radius_, special_inflation_radius_);
  unsigned int cell_max_radius = cellDistance(max_radius);
  if (cell_max_radius == 0) {
    return;
  }

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

  // --- 🐞 새로운 핵심 수정 부분 시작 🐞 ---
  // inflation 비용을 덧칠하기 전에, 업데이트 할 영역(bounds) 내의
  // 오래된 inflation 비용을 먼저 깨끗하게 지웁니다.
  // 이렇게 하면 이전 사이클의 "잔상"이 남는 것을 방지할 수 있습니다.
  // 단, 실제 장애물(LETHAL_OBSTACLE)과 미확인 지역(NO_INFORMATION)은 보존해야 합니다.
  for (int j = base_min_j; j < base_max_j; j++) {
    for (int i = base_min_i; i < base_max_i; i++) {
      unsigned char cost = master_array[master_grid.getIndex(i, j)];
      if (cost != LETHAL_OBSTACLE && cost != NO_INFORMATION) {
        master_array[master_grid.getIndex(i, j)] = FREE_SPACE;
      }
    }
  }
  // --- 새로운 핵심 수정 부분 끝 ---

  // 1단계: 장애물 씨앗(seed) 찾기
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
  
  // 2단계: inflation 전파 (이후 로직은 이전과 동일)
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
            // 이제 old_cost는 FREE_SPACE, LETHAL_OBSTACLE, NO_INFORMATION 중 하나이므로
            // std::max가 안전하게 동작합니다.
            master_array[index] = std::max(old_cost, cost);
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
