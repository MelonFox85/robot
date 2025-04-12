// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interfaces:action/SetMotorSpeed.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__ACTION__DETAIL__SET_MOTOR_SPEED__BUILDER_HPP_
#define CUSTOM_INTERFACES__ACTION__DETAIL__SET_MOTOR_SPEED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interfaces/action/detail/set_motor_speed__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interfaces
{

namespace action
{

namespace builder
{

class Init_SetMotorSpeed_Goal_delta_plane_angle
{
public:
  explicit Init_SetMotorSpeed_Goal_delta_plane_angle(::custom_interfaces::action::SetMotorSpeed_Goal & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::action::SetMotorSpeed_Goal delta_plane_angle(::custom_interfaces::action::SetMotorSpeed_Goal::_delta_plane_angle_type arg)
  {
    msg_.delta_plane_angle = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::action::SetMotorSpeed_Goal msg_;
};

class Init_SetMotorSpeed_Goal_delta_body_angle
{
public:
  explicit Init_SetMotorSpeed_Goal_delta_body_angle(::custom_interfaces::action::SetMotorSpeed_Goal & msg)
  : msg_(msg)
  {}
  Init_SetMotorSpeed_Goal_delta_plane_angle delta_body_angle(::custom_interfaces::action::SetMotorSpeed_Goal::_delta_body_angle_type arg)
  {
    msg_.delta_body_angle = std::move(arg);
    return Init_SetMotorSpeed_Goal_delta_plane_angle(msg_);
  }

private:
  ::custom_interfaces::action::SetMotorSpeed_Goal msg_;
};

class Init_SetMotorSpeed_Goal_delta_linear_velocity
{
public:
  explicit Init_SetMotorSpeed_Goal_delta_linear_velocity(::custom_interfaces::action::SetMotorSpeed_Goal & msg)
  : msg_(msg)
  {}
  Init_SetMotorSpeed_Goal_delta_body_angle delta_linear_velocity(::custom_interfaces::action::SetMotorSpeed_Goal::_delta_linear_velocity_type arg)
  {
    msg_.delta_linear_velocity = std::move(arg);
    return Init_SetMotorSpeed_Goal_delta_body_angle(msg_);
  }

private:
  ::custom_interfaces::action::SetMotorSpeed_Goal msg_;
};

class Init_SetMotorSpeed_Goal_delta_plane_angular_velocity
{
public:
  explicit Init_SetMotorSpeed_Goal_delta_plane_angular_velocity(::custom_interfaces::action::SetMotorSpeed_Goal & msg)
  : msg_(msg)
  {}
  Init_SetMotorSpeed_Goal_delta_linear_velocity delta_plane_angular_velocity(::custom_interfaces::action::SetMotorSpeed_Goal::_delta_plane_angular_velocity_type arg)
  {
    msg_.delta_plane_angular_velocity = std::move(arg);
    return Init_SetMotorSpeed_Goal_delta_linear_velocity(msg_);
  }

private:
  ::custom_interfaces::action::SetMotorSpeed_Goal msg_;
};

class Init_SetMotorSpeed_Goal_delta_body_angular_velocity
{
public:
  explicit Init_SetMotorSpeed_Goal_delta_body_angular_velocity(::custom_interfaces::action::SetMotorSpeed_Goal & msg)
  : msg_(msg)
  {}
  Init_SetMotorSpeed_Goal_delta_plane_angular_velocity delta_body_angular_velocity(::custom_interfaces::action::SetMotorSpeed_Goal::_delta_body_angular_velocity_type arg)
  {
    msg_.delta_body_angular_velocity = std::move(arg);
    return Init_SetMotorSpeed_Goal_delta_plane_angular_velocity(msg_);
  }

private:
  ::custom_interfaces::action::SetMotorSpeed_Goal msg_;
};

class Init_SetMotorSpeed_Goal_delta_distance
{
public:
  Init_SetMotorSpeed_Goal_delta_distance()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetMotorSpeed_Goal_delta_body_angular_velocity delta_distance(::custom_interfaces::action::SetMotorSpeed_Goal::_delta_distance_type arg)
  {
    msg_.delta_distance = std::move(arg);
    return Init_SetMotorSpeed_Goal_delta_body_angular_velocity(msg_);
  }

private:
  ::custom_interfaces::action::SetMotorSpeed_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::action::SetMotorSpeed_Goal>()
{
  return custom_interfaces::action::builder::Init_SetMotorSpeed_Goal_delta_distance();
}

}  // namespace custom_interfaces


namespace custom_interfaces
{

namespace action
{


}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::action::SetMotorSpeed_Result>()
{
  return ::custom_interfaces::action::SetMotorSpeed_Result(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace custom_interfaces


namespace custom_interfaces
{

namespace action
{

namespace builder
{

class Init_SetMotorSpeed_Feedback_success
{
public:
  Init_SetMotorSpeed_Feedback_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::custom_interfaces::action::SetMotorSpeed_Feedback success(::custom_interfaces::action::SetMotorSpeed_Feedback::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::action::SetMotorSpeed_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::action::SetMotorSpeed_Feedback>()
{
  return custom_interfaces::action::builder::Init_SetMotorSpeed_Feedback_success();
}

}  // namespace custom_interfaces


namespace custom_interfaces
{

namespace action
{

namespace builder
{

class Init_SetMotorSpeed_SendGoal_Request_goal
{
public:
  explicit Init_SetMotorSpeed_SendGoal_Request_goal(::custom_interfaces::action::SetMotorSpeed_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::action::SetMotorSpeed_SendGoal_Request goal(::custom_interfaces::action::SetMotorSpeed_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::action::SetMotorSpeed_SendGoal_Request msg_;
};

class Init_SetMotorSpeed_SendGoal_Request_goal_id
{
public:
  Init_SetMotorSpeed_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetMotorSpeed_SendGoal_Request_goal goal_id(::custom_interfaces::action::SetMotorSpeed_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_SetMotorSpeed_SendGoal_Request_goal(msg_);
  }

private:
  ::custom_interfaces::action::SetMotorSpeed_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::action::SetMotorSpeed_SendGoal_Request>()
{
  return custom_interfaces::action::builder::Init_SetMotorSpeed_SendGoal_Request_goal_id();
}

}  // namespace custom_interfaces


namespace custom_interfaces
{

namespace action
{

namespace builder
{

class Init_SetMotorSpeed_SendGoal_Response_stamp
{
public:
  explicit Init_SetMotorSpeed_SendGoal_Response_stamp(::custom_interfaces::action::SetMotorSpeed_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::action::SetMotorSpeed_SendGoal_Response stamp(::custom_interfaces::action::SetMotorSpeed_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::action::SetMotorSpeed_SendGoal_Response msg_;
};

class Init_SetMotorSpeed_SendGoal_Response_accepted
{
public:
  Init_SetMotorSpeed_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetMotorSpeed_SendGoal_Response_stamp accepted(::custom_interfaces::action::SetMotorSpeed_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_SetMotorSpeed_SendGoal_Response_stamp(msg_);
  }

private:
  ::custom_interfaces::action::SetMotorSpeed_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::action::SetMotorSpeed_SendGoal_Response>()
{
  return custom_interfaces::action::builder::Init_SetMotorSpeed_SendGoal_Response_accepted();
}

}  // namespace custom_interfaces


namespace custom_interfaces
{

namespace action
{

namespace builder
{

class Init_SetMotorSpeed_GetResult_Request_goal_id
{
public:
  Init_SetMotorSpeed_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::custom_interfaces::action::SetMotorSpeed_GetResult_Request goal_id(::custom_interfaces::action::SetMotorSpeed_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::action::SetMotorSpeed_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::action::SetMotorSpeed_GetResult_Request>()
{
  return custom_interfaces::action::builder::Init_SetMotorSpeed_GetResult_Request_goal_id();
}

}  // namespace custom_interfaces


namespace custom_interfaces
{

namespace action
{

namespace builder
{

class Init_SetMotorSpeed_GetResult_Response_result
{
public:
  explicit Init_SetMotorSpeed_GetResult_Response_result(::custom_interfaces::action::SetMotorSpeed_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::action::SetMotorSpeed_GetResult_Response result(::custom_interfaces::action::SetMotorSpeed_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::action::SetMotorSpeed_GetResult_Response msg_;
};

class Init_SetMotorSpeed_GetResult_Response_status
{
public:
  Init_SetMotorSpeed_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetMotorSpeed_GetResult_Response_result status(::custom_interfaces::action::SetMotorSpeed_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_SetMotorSpeed_GetResult_Response_result(msg_);
  }

private:
  ::custom_interfaces::action::SetMotorSpeed_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::action::SetMotorSpeed_GetResult_Response>()
{
  return custom_interfaces::action::builder::Init_SetMotorSpeed_GetResult_Response_status();
}

}  // namespace custom_interfaces


namespace custom_interfaces
{

namespace action
{

namespace builder
{

class Init_SetMotorSpeed_FeedbackMessage_feedback
{
public:
  explicit Init_SetMotorSpeed_FeedbackMessage_feedback(::custom_interfaces::action::SetMotorSpeed_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::action::SetMotorSpeed_FeedbackMessage feedback(::custom_interfaces::action::SetMotorSpeed_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::action::SetMotorSpeed_FeedbackMessage msg_;
};

class Init_SetMotorSpeed_FeedbackMessage_goal_id
{
public:
  Init_SetMotorSpeed_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetMotorSpeed_FeedbackMessage_feedback goal_id(::custom_interfaces::action::SetMotorSpeed_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_SetMotorSpeed_FeedbackMessage_feedback(msg_);
  }

private:
  ::custom_interfaces::action::SetMotorSpeed_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::action::SetMotorSpeed_FeedbackMessage>()
{
  return custom_interfaces::action::builder::Init_SetMotorSpeed_FeedbackMessage_goal_id();
}

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__ACTION__DETAIL__SET_MOTOR_SPEED__BUILDER_HPP_
