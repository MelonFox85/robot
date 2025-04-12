// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_interfaces:action/SetMotorSpeed.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__ACTION__DETAIL__SET_MOTOR_SPEED__STRUCT_H_
#define CUSTOM_INTERFACES__ACTION__DETAIL__SET_MOTOR_SPEED__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in action/SetMotorSpeed in the package custom_interfaces.
typedef struct custom_interfaces__action__SetMotorSpeed_Goal
{
  float delta_distance;
  float delta_body_angular_velocity;
  float delta_plane_angular_velocity;
  float delta_linear_velocity;
  float delta_body_angle;
  float delta_plane_angle;
} custom_interfaces__action__SetMotorSpeed_Goal;

// Struct for a sequence of custom_interfaces__action__SetMotorSpeed_Goal.
typedef struct custom_interfaces__action__SetMotorSpeed_Goal__Sequence
{
  custom_interfaces__action__SetMotorSpeed_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interfaces__action__SetMotorSpeed_Goal__Sequence;


// Constants defined in the message

/// Struct defined in action/SetMotorSpeed in the package custom_interfaces.
typedef struct custom_interfaces__action__SetMotorSpeed_Result
{
  uint8_t structure_needs_at_least_one_member;
} custom_interfaces__action__SetMotorSpeed_Result;

// Struct for a sequence of custom_interfaces__action__SetMotorSpeed_Result.
typedef struct custom_interfaces__action__SetMotorSpeed_Result__Sequence
{
  custom_interfaces__action__SetMotorSpeed_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interfaces__action__SetMotorSpeed_Result__Sequence;


// Constants defined in the message

/// Struct defined in action/SetMotorSpeed in the package custom_interfaces.
typedef struct custom_interfaces__action__SetMotorSpeed_Feedback
{
  /// Result: Подтверждение выполнения
  bool success;
} custom_interfaces__action__SetMotorSpeed_Feedback;

// Struct for a sequence of custom_interfaces__action__SetMotorSpeed_Feedback.
typedef struct custom_interfaces__action__SetMotorSpeed_Feedback__Sequence
{
  custom_interfaces__action__SetMotorSpeed_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interfaces__action__SetMotorSpeed_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "custom_interfaces/action/detail/set_motor_speed__struct.h"

/// Struct defined in action/SetMotorSpeed in the package custom_interfaces.
typedef struct custom_interfaces__action__SetMotorSpeed_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  custom_interfaces__action__SetMotorSpeed_Goal goal;
} custom_interfaces__action__SetMotorSpeed_SendGoal_Request;

// Struct for a sequence of custom_interfaces__action__SetMotorSpeed_SendGoal_Request.
typedef struct custom_interfaces__action__SetMotorSpeed_SendGoal_Request__Sequence
{
  custom_interfaces__action__SetMotorSpeed_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interfaces__action__SetMotorSpeed_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/SetMotorSpeed in the package custom_interfaces.
typedef struct custom_interfaces__action__SetMotorSpeed_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} custom_interfaces__action__SetMotorSpeed_SendGoal_Response;

// Struct for a sequence of custom_interfaces__action__SetMotorSpeed_SendGoal_Response.
typedef struct custom_interfaces__action__SetMotorSpeed_SendGoal_Response__Sequence
{
  custom_interfaces__action__SetMotorSpeed_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interfaces__action__SetMotorSpeed_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/SetMotorSpeed in the package custom_interfaces.
typedef struct custom_interfaces__action__SetMotorSpeed_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} custom_interfaces__action__SetMotorSpeed_GetResult_Request;

// Struct for a sequence of custom_interfaces__action__SetMotorSpeed_GetResult_Request.
typedef struct custom_interfaces__action__SetMotorSpeed_GetResult_Request__Sequence
{
  custom_interfaces__action__SetMotorSpeed_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interfaces__action__SetMotorSpeed_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "custom_interfaces/action/detail/set_motor_speed__struct.h"

/// Struct defined in action/SetMotorSpeed in the package custom_interfaces.
typedef struct custom_interfaces__action__SetMotorSpeed_GetResult_Response
{
  int8_t status;
  custom_interfaces__action__SetMotorSpeed_Result result;
} custom_interfaces__action__SetMotorSpeed_GetResult_Response;

// Struct for a sequence of custom_interfaces__action__SetMotorSpeed_GetResult_Response.
typedef struct custom_interfaces__action__SetMotorSpeed_GetResult_Response__Sequence
{
  custom_interfaces__action__SetMotorSpeed_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interfaces__action__SetMotorSpeed_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "custom_interfaces/action/detail/set_motor_speed__struct.h"

/// Struct defined in action/SetMotorSpeed in the package custom_interfaces.
typedef struct custom_interfaces__action__SetMotorSpeed_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  custom_interfaces__action__SetMotorSpeed_Feedback feedback;
} custom_interfaces__action__SetMotorSpeed_FeedbackMessage;

// Struct for a sequence of custom_interfaces__action__SetMotorSpeed_FeedbackMessage.
typedef struct custom_interfaces__action__SetMotorSpeed_FeedbackMessage__Sequence
{
  custom_interfaces__action__SetMotorSpeed_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interfaces__action__SetMotorSpeed_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_INTERFACES__ACTION__DETAIL__SET_MOTOR_SPEED__STRUCT_H_
