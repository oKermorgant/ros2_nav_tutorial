#include <ros2_nav_tutorial/robot.h>


namespace ros2_nav_tutorial
{
void Robot::initActionServer()
{
  move_goal_action = rclcpp_action::create_server<action::MoveGoal>(
        get_node_base_interface(),
        get_node_clock_interface(),
        get_node_logging_interface(),
        get_node_waitables_interface(),
        "move_goal",
        // what happens when a new goal is received
        [&](auto uuid, auto goal){return this->handle_goal(uuid, goal);},
  // what happens when a goal is canceled
  [&](const MoveGoalHandlePtr goal_handle){return handle_cancel(goal_handle);},
  // what happens when a goal is accepted: execute in a separate thread
  [&](const MoveGoalHandlePtr goal_handle){
    std::thread([&](){exec_accepted(goal_handle);}).detach();
  });
}


// what happens when a new goal is received
rclcpp_action::GoalResponse Robot::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveGoal::Goal> goal)
{
  (void)(uuid);
  (void)(goal);
  // reject if not in MOVE_GOAL mode
  if(current_mode != ModeRequest::GOAL_ACTION)
    return rclcpp_action::GoalResponse::REJECT;

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// what happens when a goal is canceled
rclcpp_action::CancelResponse Robot::handle_cancel(
    const MoveGoalHandlePtr goal_handle)
{
  // stop the robot
  v = 0;
  w = 0;
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

// what happens when the action is running
void Robot::exec_accepted(const MoveGoalHandlePtr goal_handle)
{    
  const double d(0.01);
  const double Kp(2.5);
  const double vmax(1);
  const double wmax(1);

  const auto goal_pos = goal_handle->get_goal()->goal;

  auto feedback = std::make_shared<MoveGoal::Feedback>();
  bool canceled = false;
  rclcpp::Rate rate(10);
  while(rclcpp::ok())
  {
    // Check if there is a cancel request
    if (goal_handle->is_canceling() || current_mode != ModeRequest::GOAL_ACTION)
    {
      canceled = true;
      break;
    }

    // move towards goal
    const auto c(cos(pose.theta));
    const auto s(sin(pose.theta));

    const auto dx(goal_pos.x - (pose.x + d*c));
    const auto dy(goal_pos.y - (pose.y + d*s));

    v = Kp*(c*dx + s*dy);
    w = Kp*(-s*dx/d + c*dy/d);

    const auto ratio = std::max(std::abs(v)/vmax, std::abs(w)/wmax);
    if(ratio > 1)
    {
      v /= ratio;
      w /= ratio;
    }
    // publish / check error
    feedback->distance = sqrt(dx*dx + dy*dy);
    if(feedback->distance < 0.01)
      break;
    rate.sleep();
  }

  // stop robot...
  v = 0;
  w = 0;

  // write angle error
  auto result = std::make_shared<MoveGoal::Result>();
  result->delta = pose.theta - goal_pos.theta;
  result->delta = std::fmod(result->delta+M_PI, 2*M_PI) - M_PI;

  if(canceled)
  {
    RCLCPP_INFO(this->get_logger(), "Goal canceled");
    goal_handle->canceled(result);
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    goal_handle->succeed(result);
  }
}
}
