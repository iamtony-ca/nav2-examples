

catch (nav2_core::InvalidPlanner & ex) {
    exceptionWarning(curr_start, curr_goal, goal->planner_id, ex);
    result->error_code = ActionThroughPosesResult::INVALID_PLANNER;
    action_server_poses_->terminate_current(result);
// 

  } catch (nav2_core::StartOccupied & ex) {
    exceptionWarning(curr_start, curr_goal, goal->planner_id, ex);
    result->error_code = ActionThroughPosesResult::START_OCCUPIED;
    action_server_poses_->terminate_current(result);
// recovery planner

  } catch (nav2_core::GoalOccupied & ex) {
    exceptionWarning(curr_start, curr_goal, goal->planner_id, ex);
    result->error_code = ActionThroughPosesResult::GOAL_OCCUPIED;
    action_server_poses_->terminate_current(result);
// clear costmap, wait, skip and replanning 

  } catch (nav2_core::NoValidPathCouldBeFound & ex) {
    exceptionWarning(curr_start, curr_goal, goal->planner_id, ex);
    result->error_code = ActionThroughPosesResult::NO_VALID_PATH;
    action_server_poses_->terminate_current(result);
// recovery planner, skip and replanning

  } catch (nav2_core::PlannerTimedOut & ex) {
    exceptionWarning(curr_start, curr_goal, goal->planner_id, ex);
    result->error_code = ActionThroughPosesResult::TIMEOUT;
    action_server_poses_->terminate_current(result);
// recovery planner, skip and replanning

  } catch (nav2_core::StartOutsideMapBounds & ex) {
    exceptionWarning(curr_start, curr_goal, goal->planner_id, ex);
    result->error_code = ActionThroughPosesResult::START_OUTSIDE_MAP;
    action_server_poses_->terminate_current(result);
// 

  } catch (nav2_core::GoalOutsideMapBounds & ex) {
    exceptionWarning(curr_start, curr_goal, goal->planner_id, ex);
    result->error_code = ActionThroughPosesResult::GOAL_OUTSIDE_MAP;
    action_server_poses_->terminate_current(result);


  } catch (nav2_core::PlannerTFError & ex) {
    exceptionWarning(curr_start, curr_goal, goal->planner_id, ex);
    result->error_code = ActionThroughPosesResult::TF_ERROR;
    action_server_poses_->terminate_current(result);


  } catch (nav2_core::NoViapointsGiven & ex) {
    exceptionWarning(curr_start, curr_goal, goal->planner_id, ex);
    result->error_code = ActionThroughPosesResult::NO_VIAPOINTS_GIVEN;
    action_server_poses_->terminate_current(result);

    
  } catch (nav2_core::PlannerCancelled &) {
    RCLCPP_INFO(get_logger(), "Goal was canceled. Canceling planning action.");
    action_server_poses_->terminate_all();
  } catch (std::exception & ex) {
    exceptionWarning(curr_start, curr_goal, goal->planner_id, ex);
    result->error_code = ActionThroughPosesResult::UNKNOWN;
    action_server_poses_->terminate_current(result);
  }