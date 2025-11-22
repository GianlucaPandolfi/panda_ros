# Robot controller

The [*impedance_controller* node](../overview/panda_utils.md#impedance_controller) can be easily reused to implement other control law, acting in joint or operational space. The focus will be on the 2 main functions of the node 
```cpp
on_configure(const rclcpp_lifecycle::State &)
...
on_activate(const rclcpp_lifecycle::State &)
```
allowing to characterize almost entirely the behaviour of the controller.  

## *on_configure*  

This function initialize almost all the parameters of the node, and depending on the mode of operation chosen for the controller, initialize the communication with the **Franka Emika panda** robot using the FCI.  
The first part of the function is used to initialize the common resources to both the modes: 
- the publishers used for debug, with a helper `DebugPublisher` class,
- the controller gains and control update frequency. 

After this section the simulation doesn't need additional setup.  

If we are communicating with the FCI we need to: 

- instantiate the communication, using the **robot ip**,
- set the [collision behaviour](https://frankarobotics.github.io/libfranka/0.18.0/classfranka_1_1Robot.html#a168e1214ac36d74ac64f894332b84534) according to the control law chosen,
- set the load on the robot.  

Eventually, we can print some useful debug information.  
In the same function i've defined the **robot_control_callback**. This is the callback we pass to the _libfranka_ control function that spawns a **real-time thread** and controls the robot at a fixed frequency of 1 kHz.  

### _robot_control_callback_  

The most important function to modify for the implementation of a different control strategy. Within the function:

- the robot status is **read**,
- the quantities useful for control are **computed**,
- the control input vector is **computed**,
- the portions of the status useful for **debugging** and **monitoring** purpose on the ROS2 network are copied into **dedicated structures**,
- the **internal status** of the robot and the **forward kinematics** are published.

The publication of the status and fkine of the robot in the same thread is done to always keep the updated state on the network. The _dedicated structures_ are simply **mutex guarded** structure used to copy the robot state and publish it from another thread, lightening the **main control thread**.  
The only thread that **needs** to be real-time is the control thread spawned by _libfranka_, so the other threads can be spawned with simple threads or can be used a much lower priority if spawned in real-time.  

An example of a robot_control_callback function can be found in the _impedance_controller_ node of the project of course. Furthermore, the function used in the impedance controller node implements a **torque level** control: the _libfranka_ library accepts also function that allows to control the robot at higher level, with joint or cartesian velocites signals for example.

## *on_activate*  

The second function that should be modified is the _on_activate_ one. This function, like _on_configure_, is divided in 2 sections: one is common to both the modes of operation and one is specialized depending on the mode chosen.  
If the robot is controlled in the simulated environment, the function simply spawns the control thread that communicates with the gazebo env through the publisher and the subscribers of the netowrk. The control thread function is named ```control()``` and is almost identical to the _robot_control_callback_, implementing the same control law.  
If the robot is controlled through the FCI the function spawns several threads, each with its own purpose: 

- a thread that publishes only the `Pose`s of the robot's frames.
- A thread that updates internal compliance mode variables, allowing the robot to compute jacobian, pose, and current external wrench, depending on the operator's point of contact with the robot, and modifying the controller gains on the fly. 
- A thread publishing debug infos, reading the **mutex guarded** structure shared with the _robot_control_callback_.
- The control thread spawned with the [**control**](https://frankarobotics.github.io/libfranka/0.18.0/classfranka_1_1Robot.html#a0d5effba5daff2fee123802bbd5f95d1) function of the _libfranka_ lib, after configuring the SCHED_FIFO priority to 99.  

The function can be easily expanded adding other threads one would like to spawn.

## Publishers, subscribers, services and parameters  

Of course, if one wants to implement different control laws, one would need additional publisher/subscriber objects to communicate with the ROS2 network, other services to implement different behaviors, or new parameters to be specified. All of this can be implemented using the controller class as a starting template, communicating with the FCI using the object's internal structures and implementing the necessary logic within callbacks, threads, and the _robot_control_callback_ function.

## Implementing a control law  

As said, to implement a control law with this setup we need to modify primarly the _robot_control_callback_. Following the instructions in the [dedicated section](#robot_control_callback) we'll now implement an impedance control law in the cartesian space.  

First thing first, this is the signature of the function 

```cpp
auto get_jacob = [this](
                     const Eigen::Vector<double, 7> &current_joint_pos) {
  auto state = franka::RobotState{};
  for (size_t i = 0; i < state.q.size(); i++) {
    state.q[i] = current_joint_pos[i];
  }

  if (compliance_mode.load() && last_joint_contact_frame.has_value()) {

    jacobian = geom_utils::get_jacobian(panda_franka_model->zeroJacobian(
        last_joint_contact_frame.value(), state));

  } else {
    jacobian = geom_utils::get_jacobian(
        panda_franka_model->zeroJacobian(franka::Frame::kFlange, state));
  }
  return jacobian;
};

robot_control_callback =
    [this, get_jacob](const franka::RobotState &state,franka::Duration dt) 
      -> franka::Torques 
    {
        if (!(start_flag.load() && rclcpp::ok())) {
          // Send last commanded joint effort command
          return franka::MotionFinished(franka::Torques(state.tau_J_d));
        }
        ...
    }
```

The `get_jacob` function is used to calculate the jacobian, based on the `last_joint_contact_frame` the operator touches, if there's any.  
The first if statement inside the callback is used to stop the control thread, and requires the last commanded torque command to be sent.  

```cpp
{
  std::lock_guard<std::mutex> mut(desired_cartesian_mutex);
  update_cartesian_cmd();
}

// Get q and q_dot
//
Eigen::Map<const Eigen::Vector<double, 7>> current_joints_config_vec(
    state.q.data());

if (dt.toSec() != 0.0) {
  for (int i = 0; i < 7; i++) {
    current_joints_speed[i] = franka::lowpassFilter(
        dt.toSec(), state.dq[i], current_joints_speed[i],
        joints_speed_cutoff_freq);
  }

} else {
  for (int i = 0; i < 7; i++) {
    current_joints_speed[i] = state.dq[i];
  }
}
```

Then, we need to access the desired cartesian commands, relative to the pose, velocity and acceleration of the end-effector. We acquire the current joints' position and speed, filtering the latter.  

```cpp
Pose current_pose;
Eigen::Quaterniond error_quat{};
Eigen::Vector<double, 6> error_pose_vec{};
std::array<double, 49> mass_matrix_raw;

if (compliance_mode.load() && last_joint_contact_frame.has_value()) {

  current_pose = geom_utils::get_pose(panda_franka_model->pose(
      last_joint_contact_frame.value(), state));

  jacobian = geom_utils::get_jacobian(panda_franka_model->zeroJacobian(
      last_joint_contact_frame.value(), state));

} else {
    
  current_pose = geom_utils::get_pose(
      panda_franka_model->pose(franka::Frame::kFlange, state));

  jacobian = geom_utils::get_jacobian(
      panda_franka_model->zeroJacobian(franka::Frame::kFlange, state));

}

Eigen::Quaterniond current_quat{};
current_quat.w() = current_pose.orientation.w;
current_quat.x() = current_pose.orientation.x;
current_quat.y() = current_pose.orientation.y;
current_quat.z() = current_pose.orientation.z;
current_quat.normalize();
current_quat = quaternionContinuity(current_quat, old_quaternion);
old_quaternion = current_quat;

// Calculate pose error
Eigen::AngleAxisd error_angle_axis;
{
  Eigen::Quaterniond desired_quat{};
  desired_quat.w() = desired_pose.orientation.w;
  desired_quat.x() = desired_pose.orientation.x;
  desired_quat.y() = desired_pose.orientation.y;
  desired_quat.z() = desired_pose.orientation.z;
  desired_quat.normalize();

  error_quat = desired_quat * current_quat.inverse();
  error_quat.normalize();
  error_angle_axis = Eigen::AngleAxisd{error_quat};

  error_pose_vec(0) = desired_pose.position.x - current_pose.position.x;
  error_pose_vec(1) = desired_pose.position.y - current_pose.position.y;
  error_pose_vec(2) = desired_pose.position.z - current_pose.position.z;
  error_pose_vec(3) = error_quat.x();
  error_pose_vec(4) = error_quat.y();
  error_pose_vec(5) = error_quat.z();
}
```

In this snippet of code we compute the pose error as 

$$
\tilde{\bar{x}} = \begin{bmatrix} x_{d} - x_{e} \cr y_{d} - y_{e} \cr z_{d} - z_{e} \cr vec(Q_d * Q_e^{-1}) \end{bmatrix}
$$
and the current angle error for debug purposes. The `quaternionContinuity` function helps avoding big jumps between the previous and the current quaternion.  

```cpp
mass_matrix_raw = this->panda_franka_model.value().mass(state);
Eigen::Map<const Eigen::Matrix<double, 7, 7>> mass_matrix(
    mass_matrix_raw.data());

Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(
    this->panda_franka_model.value().coriolis(state).data());

// Calculate jacobian SVD
Eigen::JacobiSVD<Eigen::MatrixXd> svd(
    jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
double sigma_min = svd.singularValues().tail(1)(0);

Eigen::Matrix<double, 7, 6> jacobian_pinv =
    compute_jacob_pseudoinv(jacobian);
```

Then, we calculate the \\(B(q)\\) matrix, the coriolis terms and the pseudo-inverse of the geometrical jacobian.  

```cpp
Eigen::Map<const Eigen::Vector<double, 7>> tau_ext_measured(
    state.tau_ext_hat_filtered.data());

Eigen::Matrix<double, 7, 6> jacobian_transposed = jacobian.transpose();

if (dt.toSec() == 0.0) {
  extern_tau = extern_tau.Zero();
} else {
  for (int i = 0; i < 6; i++) {
    extern_tau[i] =
        franka::lowpassFilter(dt.toSec(), tau_ext_measured[i],
                              extern_tau[i], external_tau_cutoff_freq);
  }
}

Eigen::Vector<double, 7> tau_ext = extern_tau;
if (dt.toSec() == 0.0) {
  h_e = h_e.Zero();
} else {
  if (compliance_mode.load() && last_joint_contact_frame.has_value()) {
    int index = 0;
    switch (last_joint_contact_frame.value()) {
    case franka::Frame::kJoint1: {
      index = 0;
      break;
    }
    case franka::Frame::kJoint2: {
      index = 1;
      break;
    }
    case franka::Frame::kJoint3: {
      index = 2;
      break;
    }
    case franka::Frame::kJoint4: {
      index = 3;
      break;
    }
    case franka::Frame::kJoint5: {
      index = 4;
      break;
    }
    case franka::Frame::kJoint6: {
      index = 5;
      break;
    }
    case franka::Frame::kJoint7: {
      index = 6;
      break;
    }
    case franka::Frame::kFlange: {
      index = 6;
      break;
    }
    case franka::Frame::kEndEffector:
    case franka::Frame::kStiffness:
      break;
    }
    for (int i = index + 1; i < tau_ext.size(); i++) {
      tau_ext[i] = 0.0;
    }
  }
  h_e_measured =
      compute_jacob_pseudoinv_h_e(jacobian_transposed) * tau_ext;

  h_e = h_e_measured;
}
```

Then we have to acquire the external forces sensed by the arm along its joints. We filter these values another time, after the _libfranka_ lib. To compute the external wrench applied to the end-effector considered we need to zero out the torques contributes relative to the subsequent frames to the one touched by the operator, e.g. if the operator toches the 4th frame, we consider only the torques acting on joints 1-4. This is done by the `switch` statement in the snippet. This last procedure is only necessary in case of _compliance mode_ behaviour.  

```cpp
Eigen::Vector<double, 7> y;
Eigen::Vector<double, 6> y_cartesian;
Eigen::Vector<double, 6> current_twist;
Eigen::Vector<double, 6> error_twist;
{
  current_twist = jacobian * current_joints_speed;
  error_twist = desired_twist_vec - current_twist;
}

{
  if (compliance_mode.load()) {

    y = jacobian_pinv * MD_1 *
        (
          -KD * current_twist 
          - MD *
           get_j_dot(get_jacob, current_joints_config_vec,
                     current_joints_speed) *
           current_joints_speed
          - h_e
        );
  } else {
    y_cartesian =
        (
          MD * desired_accel_vec +
          KD * error_twist + KP * error_pose_vec 
          -MD *
              get_j_dot(get_jacob, current_joints_config_vec,
                        current_joints_speed) *
              current_joints_speed
          - h_e
        );
    y = jacobian_pinv * MD_1 * y_cartesian;
  }
}

Eigen::Vector<double, 7> control_input_vec =
    mass_matrix * y + coriolis + extern_tau;

// Clamp tau
clamp_control(control_input_vec);
```

Finally, we can compute the control input torques according to the law. After computing the velocity error (`error_twist`), depending on whether or not we are in _compliance mode_ the control law includes the acceleration and pose error terms. In the last line of code the control input is then clamped based on torque limits.  

The torque commanded to the low-level interface is just a part of the total torque commanded by the _libfranka_ library. In fact, the real torque commanded to the arm is the sum of 3 terms: 

- \\(tau_d\\) = user commanded torque
- \\(tau_g\\) = torque required for gravity compensation
- \\(tau_f\\) = torque required to compensate motor friction

Because of this, our computed control torque doesn't include neither the gravity term nor the friction one.  


```cpp
// Safety checks
if (dt.toSec() == 0.0) {
  for (int i = 0; i < control_input_vec.size(); i++) {
    control_input_vec[i] = 0.0;
  }
} else {

  RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Running safety checks");
  try {
    for (int i = 0; i < control_input_vec.size(); i++) {
      if (abs(control_input_vec[i]) >=
          percentage_effort_safe_limit * effort_limits[i]) {
        RCLCPP_INFO_STREAM_ONCE(this->get_logger(),
                                "Running safety check: effort limit");
        RCLCPP_ERROR_STREAM(this->get_logger(),
                            "Torque abs value over limit ("
                                << percentage_effort_safe_limit * 100.0
                                << "%)");
        panda_franka->stop();
        start_flag.store(false);
        return franka::MotionFinished(franka::Torques(state.tau_J_d));
      } else if (abs(state.dq[i]) >= joint_speed_safe_limit) {
        RCLCPP_INFO_STREAM_ONCE(
            this->get_logger(),
            "Running safety check: joint limit speed");
        RCLCPP_ERROR_STREAM(this->get_logger(),
                            "Joint velocity over the safety value "
                                << joint_speed_safe_limit);
        panda_franka->stop();
        start_flag.store(false);
        return franka::MotionFinished(franka::Torques(state.tau_J_d));
      }
    }
    if (current_pose.position.z <= 0.15) {
      RCLCPP_ERROR_STREAM(this->get_logger(),
                          "Height of the end affector wrt base under "
                          "allowed value 0.15m");
      panda_franka->stop();
      start_flag.store(false);
      return franka::MotionFinished(franka::Torques(state.tau_J_d));
    }

  } catch (std::exception &ex) {
    RCLCPP_ERROR_STREAM(this->get_logger(),
                        "Error in safety checks: " << ex.what());
    panda_franka->stop();
    start_flag.store(false);
    return franka::MotionFinished(franka::Torques(state.tau_J_d));
  }
  RCLCPP_INFO_STREAM_ONCE(this->get_logger(),
                          "Finished safety checks first time");
}
last_control_input = control_input_vec;

if (control_input_vec.array().isNaN().any() ||
    control_input_vec.array().isInf().any()) {
  RCLCPP_ERROR_STREAM(
      this->get_logger(),
      "Control input vec Nan or Inf: "
          << control_input_vec
          << ", Desired pose: " << desired_pose.position.x << ", "
          << desired_pose.position.y << ", " << desired_pose.position.z
          << ", Desired twist: " << desired_twist_vec
          << ", Desired accel: " << desired_accel_vec
          << ", Jacobian pinv: " << jacobian_pinv << ", error twist: "
          << error_twist << ", error pose: " << error_pose_vec
          << ", extern_tau: " << extern_tau << ", coriolis: "
          << coriolis << ", joint vel: " << current_joints_speed
          << ", jacobian: " << jacobian
          << ", current joint pos: " << current_joints_config_vec);

  panda_franka->stop();
  start_flag.store(false);
  return franka::MotionFinished(franka::Torques(state.tau_J_d));
}
```

Before sending the torques to the _libfranka_ low-level interface, we run a few safety checks on: 

- torque values 
- joints's speed 
- end-effector height (to avoid table contact)

An additional check is run to see if any of the torques, for whatever reason, is equal to NaN or Inf, printing all the control related variables for debugging.  

All the checks, if failed, immediately stop the control thread.  

```cpp
std::array<double, 7> tau;
for (size_t i = 0; i < 7; ++i) {
  tau[i] = control_input_vec[i];
}

RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Filling debug data");

// Fill struct for TFs prints
if (panda_franka_state.mut.try_lock()) {
  panda_franka_state.state = state;
  panda_franka_state.mut.unlock();
}

// Fill struct for debug data publishing
if (debug_pub.data().mut.try_lock()) {

  try {
    debug_pub.data().h_e = h_e_measured;
    debug_pub.data().h_e_calculated = h_e;
    debug_pub.data().tau_ext = tau_ext_measured;
    debug_pub.data().tau_ext_calculated = extern_tau;
    debug_pub.data().error_theta =
        error_angle_axis.angle() * 180.0 / M_PI;
    debug_pub.data().sigma_min = sigma_min;
    debug_pub.data().current_twist = current_twist;
    debug_pub.data().des_twist = desired_twist;
    debug_pub.data().des_accel = desired_accel;
    debug_pub.data().current_j_dot_q_dot =
        get_j_dot(get_jacob, current_joints_config_vec,
                  current_joints_speed) *
        current_joints_speed;
    debug_pub.data().gravity = panda_franka_model->gravity(state);
    debug_pub.data().coriolis = coriolis;
    debug_pub.data().filtered_joints_vec = current_joints_speed;
    debug_pub.data().error_pose_vec.head(3) = error_pose_vec.head(3);
    debug_pub.data().error_pose_vec.tail(3) = error_pose_vec.tail(3);
    // w value of the pose message in a vector<7>
    debug_pub.data().error_pose_vec(3) = 1.0;
    debug_pub.data().tau_d_calculated = tau;
    debug_pub.data().tau_d_last = state.tau_J_d;
    debug_pub.data().tau_read = state.tau_J;
    debug_pub.data().y = y;
    debug_pub.data().y_cartesian = y_cartesian;

    // Robot state
    debug_pub.data().robot_state->q = state.q;
    debug_pub.data().robot_state->dq = state.dq;
    debug_pub.data().robot_state->O_T_EE = state.O_T_EE;
    debug_pub.data().current_pose = current_pose;
    debug_pub.data().has_data = true;
  } catch (std::exception &ex) {
    RCLCPP_ERROR_STREAM(
        this->get_logger(),
        "Error copying data in controller: " << ex.what());
  }
  debug_pub.data().mut.unlock();
}

// The pose and the joints' values are published directly by the main
// control thread
PoseStamped pose_stamp;
pose_stamp.header.stamp = this->now();
pose_stamp.pose = current_pose;
robot_pose_pub->try_publish(pose_stamp);

joint_state_to_pub.header.stamp = this->now();
for (size_t i = 0; i < joint_state_to_pub.position.size(); i++) {
  joint_state_to_pub.position[i] = state.q[i];
  joint_state_to_pub.velocity[i] = state.dq[i];
  joint_state_to_pub.effort[i] = state.tau_J[i];
}

joint_states_pub->try_publish(joint_state_to_pub);

RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Sent command first time");
return franka::Torques(tau);
```

In the end, we fill the _libfranka_ struct for torque command, update the debug structs use for the publication of robot related variables, publish directly in the callback the current pose and joint configuration of the robot, and then as final line, we return the input torques.  

The whole callback composed by all these pieces is reported in the next snippet of code

```cpp
robot_control_callback =
    [this, get_jacob](const franka::RobotState &state,
                      franka::Duration dt) -> franka::Torques {
  if (!(start_flag.load() && rclcpp::ok())) {
    // Send last commanded joint effort command
    return franka::MotionFinished(franka::Torques(state.tau_J_d));
  }

  {
    std::lock_guard<std::mutex> mut(desired_cartesian_mutex);
    update_cartesian_cmd();
  }

  // Get q and q_dot
  //
  Eigen::Map<const Eigen::Vector<double, 7>> current_joints_config_vec(
      state.q.data());

  if (dt.toSec() != 0.0) {
    for (int i = 0; i < 7; i++) {
      current_joints_speed[i] = franka::lowpassFilter(
          dt.toSec(), state.dq[i], current_joints_speed[i],
          joints_speed_cutoff_freq);
    }

  } else {
    for (int i = 0; i < 7; i++) {
      current_joints_speed[i] = state.dq[i];
    }
  }

  Pose current_pose;
  Eigen::Quaterniond error_quat{};
  Eigen::Vector<double, 6> error_pose_vec{};
  std::array<double, 49> mass_matrix_raw;

  if (compliance_mode.load() && last_joint_contact_frame.has_value()) {

    current_pose = geom_utils::get_pose(panda_franka_model->pose(
        last_joint_contact_frame.value(), state));

    jacobian = geom_utils::get_jacobian(panda_franka_model->zeroJacobian(
        last_joint_contact_frame.value(), state));
  } else {
    current_pose = geom_utils::get_pose(
        panda_franka_model->pose(franka::Frame::kFlange, state));
    jacobian = geom_utils::get_jacobian(
        panda_franka_model->zeroJacobian(franka::Frame::kFlange, state));
  }

  Eigen::Quaterniond current_quat{};
  current_quat.w() = current_pose.orientation.w;
  current_quat.x() = current_pose.orientation.x;
  current_quat.y() = current_pose.orientation.y;
  current_quat.z() = current_pose.orientation.z;
  current_quat.normalize();
  current_quat = quaternionContinuity(current_quat, old_quaternion);
  old_quaternion = current_quat;

  // Calculate pose error
  Eigen::AngleAxisd error_angle_axis;
  {
    Eigen::Quaterniond desired_quat{};
    desired_quat.w() = desired_pose.orientation.w;
    desired_quat.x() = desired_pose.orientation.x;
    desired_quat.y() = desired_pose.orientation.y;
    desired_quat.z() = desired_pose.orientation.z;
    desired_quat.normalize();

    error_quat = desired_quat * current_quat.inverse();
    error_quat.normalize();
    error_angle_axis = Eigen::AngleAxisd{error_quat};

    error_pose_vec(0) = desired_pose.position.x - current_pose.position.x;
    error_pose_vec(1) = desired_pose.position.y - current_pose.position.y;
    error_pose_vec(2) = desired_pose.position.z - current_pose.position.z;
    error_pose_vec(3) = error_quat.x();
    error_pose_vec(4) = error_quat.y();
    error_pose_vec(5) = error_quat.z();
  }

  mass_matrix_raw = this->panda_franka_model.value().mass(state);
  Eigen::Map<const Eigen::Matrix<double, 7, 7>> mass_matrix(
      mass_matrix_raw.data());

  Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(
      this->panda_franka_model.value().coriolis(state).data());

  // Calculate jacobian SVD
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
  double sigma_min = svd.singularValues().tail(1)(0);

  Eigen::Matrix<double, 7, 6> jacobian_pinv =
      compute_jacob_pseudoinv(jacobian);

  Eigen::Map<const Eigen::Vector<double, 7>> tau_ext_measured(
      state.tau_ext_hat_filtered.data());

  Eigen::Matrix<double, 7, 6> jacobian_transposed = jacobian.transpose();

  if (dt.toSec() == 0.0) {
    extern_tau = extern_tau.Zero();
  } else {
    for (int i = 0; i < 6; i++) {
      extern_tau[i] =
          franka::lowpassFilter(dt.toSec(), tau_ext_measured[i],
                                extern_tau[i], external_tau_cutoff_freq);
    }
  }

  Eigen::Vector<double, 7> tau_ext = extern_tau;
  if (dt.toSec() == 0.0) {
    h_e = h_e.Zero();
  } else {
    if (compliance_mode.load() && last_joint_contact_frame.has_value()) {
      int index = 0;
      switch (last_joint_contact_frame.value()) {
      case franka::Frame::kJoint1: {
        index = 0;
        break;
      }
      case franka::Frame::kJoint2: {
        index = 1;
        break;
      }
      case franka::Frame::kJoint3: {
        index = 2;
        break;
      }
      case franka::Frame::kJoint4: {
        index = 3;
        break;
      }
      case franka::Frame::kJoint5: {
        index = 4;
        break;
      }
      case franka::Frame::kJoint6: {
        index = 5;
        break;
      }
      case franka::Frame::kJoint7: {
        index = 6;
        break;
      }
      case franka::Frame::kFlange: {
        index = 6;
        break;
      }
      case franka::Frame::kEndEffector:
      case franka::Frame::kStiffness:
        break;
      }
      for (int i = index + 1; i < tau_ext.size(); i++) {
        tau_ext[i] = 0.0;
      }
    }
    h_e_measured =
        compute_jacob_pseudoinv_h_e(jacobian_transposed) * tau_ext;

    h_e = h_e_measured;
  }

  Eigen::Vector<double, 7> y;
  Eigen::Vector<double, 6> y_cartesian;
  Eigen::Vector<double, 6> current_twist;
  Eigen::Vector<double, 6> error_twist;
  {
    current_twist = jacobian * current_joints_speed;
    error_twist = desired_twist_vec - current_twist;
  }

  {
    if (compliance_mode.load()) {

      y = jacobian_pinv * MD_1 *
          (
            -KD * current_twist 
            - MD *
             get_j_dot(get_jacob, current_joints_config_vec,
                       current_joints_speed) *
             current_joints_speed
            - h_e
          );
    } else {
      y_cartesian =
          (
            MD * desired_accel_vec +
            KD * error_twist + KP * error_pose_vec 
            -MD *
                get_j_dot(get_jacob, current_joints_config_vec,
                          current_joints_speed) *
                current_joints_speed
            - h_e
          );
      y = jacobian_pinv * MD_1 * y_cartesian;
    }
  }

  Eigen::Vector<double, 7> control_input_vec =
      mass_matrix * y + coriolis + extern_tau -
      KD_J * current_joints_speed;

  clamp_control(control_input_vec);

  // Safety checks
  if (dt.toSec() == 0.0) {
    for (int i = 0; i < control_input_vec.size(); i++) {
      control_input_vec[i] = 0.0;
    }
  } else {

    RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Running safety checks");
    try {
      for (int i = 0; i < control_input_vec.size(); i++) {
        if (abs(control_input_vec[i]) >=
            percentage_effort_safe_limit * effort_limits[i]) {
          RCLCPP_INFO_STREAM_ONCE(this->get_logger(),
                                  "Running safety check: effort limit");
          RCLCPP_ERROR_STREAM(this->get_logger(),
                              "Torque abs value over limit ("
                                  << percentage_effort_safe_limit * 100.0
                                  << "%)");
          panda_franka->stop();
          start_flag.store(false);
          return franka::MotionFinished(franka::Torques(state.tau_J_d));
        } else if (abs(state.dq[i]) >= joint_speed_safe_limit) {
          RCLCPP_INFO_STREAM_ONCE(
              this->get_logger(),
              "Running safety check: joint limit speed");
          RCLCPP_ERROR_STREAM(this->get_logger(),
                              "Joint velocity over the safety value "
                                  << joint_speed_safe_limit);
          panda_franka->stop();
          start_flag.store(false);
          return franka::MotionFinished(franka::Torques(state.tau_J_d));
        }
      }
      if (current_pose.position.z <= 0.15) {
        RCLCPP_ERROR_STREAM(this->get_logger(),
                            "Height of the end affector wrt base under "
                            "allowed value 0.15m");
        panda_franka->stop();
        start_flag.store(false);
        return franka::MotionFinished(franka::Torques(state.tau_J_d));
      }

    } catch (std::exception &ex) {
      RCLCPP_ERROR_STREAM(this->get_logger(),
                          "Error in safety checks: " << ex.what());
      panda_franka->stop();
      start_flag.store(false);
      return franka::MotionFinished(franka::Torques(state.tau_J_d));
    }
    RCLCPP_INFO_STREAM_ONCE(this->get_logger(),
                            "Finished safety checks first time");
  }

  last_control_input = control_input_vec;

  if (control_input_vec.array().isNaN().any() ||
      control_input_vec.array().isInf().any()) {
    RCLCPP_ERROR_STREAM(
        this->get_logger(),
        "Control input vec Nan or Inf: "
            << control_input_vec
            << ", Desired pose: " << desired_pose.position.x << ", "
            << desired_pose.position.y << ", " << desired_pose.position.z
            << ", Desired twist: " << desired_twist_vec
            << ", Desired accel: " << desired_accel_vec
            << ", Jacobian pinv: " << jacobian_pinv << ", error twist: "
            << error_twist << ", error pose: " << error_pose_vec
            << ", extern_tau: " << extern_tau << ", coriolis: "
            << coriolis << ", joint vel: " << current_joints_speed
            << ", jacobian: " << jacobian
            << ", current joint pos: " << current_joints_config_vec);

    panda_franka->stop();
    start_flag.store(false);
    return franka::MotionFinished(franka::Torques(state.tau_J_d));
  }

  std::array<double, 7> tau;
  for (size_t i = 0; i < 7; ++i) {
    tau[i] = control_input_vec[i];
  }

  RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Filling debug data");

  // Fill struct for TFs prints
  if (panda_franka_state.mut.try_lock()) {
    panda_franka_state.state = state;
    panda_franka_state.mut.unlock();
  }

  // Fill struct for debug data publishing
  if (debug_pub.data().mut.try_lock()) {

    try {
      debug_pub.data().h_e = h_e_measured;
      debug_pub.data().h_e_calculated = h_e;
      debug_pub.data().tau_ext = tau_ext_measured;
      debug_pub.data().tau_ext_calculated = extern_tau;
      debug_pub.data().error_theta =
          error_angle_axis.angle() * 180.0 / M_PI;
      debug_pub.data().sigma_min = sigma_min;
      debug_pub.data().current_twist = current_twist;
      debug_pub.data().des_twist = desired_twist;
      debug_pub.data().des_accel = desired_accel;
      debug_pub.data().current_j_dot_q_dot =
          get_j_dot(get_jacob, current_joints_config_vec,
                    current_joints_speed) *
          current_joints_speed;
      debug_pub.data().gravity = panda_franka_model->gravity(state);
      debug_pub.data().coriolis = coriolis;
      debug_pub.data().filtered_joints_vec = current_joints_speed;
      debug_pub.data().error_pose_vec.head(3) = error_pose_vec.head(3);
      debug_pub.data().error_pose_vec.tail(3) = error_pose_vec.tail(3);
      // w value of the pose message in a vector<7>
      debug_pub.data().error_pose_vec(3) = 1.0;
      debug_pub.data().tau_d_calculated = tau;
      debug_pub.data().tau_d_last = state.tau_J_d;
      debug_pub.data().tau_read = state.tau_J;
      debug_pub.data().y = y;
      debug_pub.data().y_cartesian = y_cartesian;

      // Robot state
      debug_pub.data().robot_state->q = state.q;
      debug_pub.data().robot_state->dq = state.dq;
      debug_pub.data().robot_state->O_T_EE = state.O_T_EE;
      debug_pub.data().current_pose = current_pose;
      debug_pub.data().has_data = true;
    } catch (std::exception &ex) {
      RCLCPP_ERROR_STREAM(
          this->get_logger(),
          "Error copying data in controller: " << ex.what());
    }
    debug_pub.data().mut.unlock();
  }

  // The pose and the joints' values are published directly by the main
  // control thread
  PoseStamped pose_stamp;
  pose_stamp.header.stamp = this->now();
  pose_stamp.pose = current_pose;
  robot_pose_pub->try_publish(pose_stamp);

  joint_state_to_pub.header.stamp = this->now();
  for (size_t i = 0; i < joint_state_to_pub.position.size(); i++) {
    joint_state_to_pub.position[i] = state.q[i];
    joint_state_to_pub.velocity[i] = state.dq[i];
    joint_state_to_pub.effort[i] = state.tau_J[i];
  }

  joint_states_pub->try_publish(joint_state_to_pub);

  RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Sent command first time");
  return franka::Torques(tau);
};
```

As said in previous sections, the callback needs to be passed to the `control` function of the _libfranka_ `Robot` object, after having configured the SCHED_FIFO priority to 99.  

The procedure is very general, in the _impedance_controller_ node the lifecycle functions have been used, but nothing forbids you to use another organization for the node. Just keep in mind that only the lifecycle version have been tested.  
