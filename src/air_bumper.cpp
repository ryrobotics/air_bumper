#include <air_bumper.h>

namespace air_bumper{

AirBumper::AirBumper(ros::NodeHandle &nh)
{
  //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Read Param<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  nh.param<double>("air_bumper_node/flight_alt", flight_alt, 1.0);
  nh.param<double>("air_bumper_node/react_d", react_d, 0);
  nh.param<bool>("air_bumper_node/sim_flg", sim_flg, false);
  nh.param<bool>("air_bumper_node/grad_flg", grad_flg, false);

  nh.param("air_bumper_node/waypoint_num", waypoint_num, -1);
  for (int i = 0; i < waypoint_num; i++)
  {
    nh.param("air_bumper_node/waypoint" + to_string(i) + "_x", waypoints[i][0], -1.0);
    nh.param("air_bumper_node/waypoint" + to_string(i) + "_y", waypoints[i][1], -1.0);
    nh.param("air_bumper_node/waypoint" + to_string(i) + "_z", waypoints[i][2], -1.0);
  }

  cout << "Flight_alt: " << flight_alt << "[m]" << endl;
  cout << "React Dist: " << react_d << "[m]" << endl;
  cout << "Simulation flag (0/1): " << sim_flg << endl;
  cout << "Grad-based flag (0/1): " << grad_flg << endl;
  cout << "Please check the parameter and setting, enter 1/2 to continue, else for quit: " << endl;
  cout << "Enter 1 -> Normal flight, Enter 2-> Collision reaction" << endl;

  cin >> flight_mode;
  if (flight_mode != 1 && flight_mode != 2)
    return;

  // Subscribe the current state
  state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 1, &AirBumper::state_cb, this);
  // Subscribe the current pos（feedback signal）
  local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1, &AirBumper::local_pos_cb, this);
  // Subscribe the crash signal
  sub_crash = nh.subscribe("imu_detection/crash_point", 1, &AirBumper::crash_callback, this);
  // Subscrive waypoint confirm from controller
  confirm_sub = nh.subscribe("action_confrim", 1, &AirBumper::confirm_cb, this);

  // Publish the local velocity（control signal）
  motion_pub = nh.advertise<air_bumper::activity_pose>("move_act", 2);
  vec_pub = nh.advertise<geometry_msgs::Twist>("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
  react_pub = nh.advertise<geometry_msgs::PoseStamped>("airbumper/react", 1);
  // Service Client（set the mode and state）
  arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  // Set the UAV mode -- offboard
  offb_set_mode.request.custom_mode = "OFFBOARD";

  /************************************FOR SIMULATION ONLY************************************/
  // For arming the UAV
  arm_cmd.request.value = true;
  /*******************************************************************************************/

  engage_client = nh.serviceClient<std_srvs::Empty>("engage");

  if(grad_flg)
  {
    grad_map_.reset(new GradMap);
    grad_map_->initMap(nh);
  }

  react_w = 0;
  step = 0;
  waypoint_index = 0;

  last_request = ros::Time::now();
}

AirBumper::~AirBumper() {}

void AirBumper::state_cb(const mavros_msgs::State::ConstPtr &msg)
{
  current_state = *msg;
}

// Subscribe the current pose of UAV
void AirBumper::local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  local_pos = *msg;
  pos_buffer.push_back(local_pos);
}

// Subscrive waypoint confirm from controller
void AirBumper::confirm_cb(const air_bumper::confirmation::ConstPtr &msg)
{
  if (msg->confheader != response_header)
    return;
  cout << "Response!" << response_header << endl;
  cout << "Action!" << msg->confheader << endl;
  // response_header += 1; // > action header
  confirm_flag = true;
  response_header += 1;

  if (msg->name == "crash")
    ROS_WARN("REACTION CONFIRM");
  else
    ROS_WARN("CONFIRM");
}

void AirBumper::engageServiceCall()
{
  // Create a service message object
  std_srvs::Empty srv;

  // Call the service
  if (engage_client.call(srv))
    cout << "Taking Off!" << endl;
  else
    ROS_ERROR("Failed to call service engage");
}

// send control msg to "got_task_cb" in px4_controller
void AirBumper::set_pos_control(double pos_x, double pos_y, double pos_z, string type = "pass_by")
{
  motion_vector.type = type;
  motion_vector.header = action_header++;
  motion_vector.tgt_pose.pose.position.x = pos_x;
  motion_vector.tgt_pose.pose.position.y = pos_y;
  motion_vector.tgt_pose.pose.position.z = pos_z;
}

void AirBumper::crash_callback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
  // crash point is the negative value of the original collision accel
  crash_point = *msg; // accc in imu_detection.cpp

  crash_flag = true;

  Eigen::Vector3d crash(crash_point.point.x,
                        crash_point.point.y,
                        crash_point.point.z);

  rad_crash(0) = crash.norm();                  // r = ||x^2+y^2+z^2||
  rad_crash(1) = atan2(crash(1), crash(0));     // phi = atan2(y,x)
  rad_crash(2) = acos(crash(2) / rad_crash(0)); // theta = acos(z/r)
}

Eigen::Vector3d AirBumper::grad_recover_control(const Eigen::Vector3d &pos, double &dist)
{
  Eigen::Vector3d grad, grad_dir;

  dist = grad_map_->getDistWithGrad(grad_map_->_map_center, grad);

  grad_dir = grad.normalized();

  return grad_dir;
}

bool AirBumper::auto_take_off()
{
  if (current_state.mode != "OFFBOARD")
  {
    if (sim_flg)
    {
      if (set_mode_client.call(offb_set_mode) &&
          offb_set_mode.response.mode_sent)
          ROS_INFO_ONCE("Offboard enabled");
    }
    else
    {
      ROS_INFO_ONCE("Offboard not enabled");
    }
  }
  else
  {
    if (!current_state.armed)
    {
      if (sim_flg)
      {
        if (arming_client.call(arm_cmd) &&
            arm_cmd.response.success)
            ROS_INFO_ONCE("Vehicle armed");
      }
      else
      {
        ROS_INFO_ONCE("Offboard enabled && NOT Armed");
      }
    }
  }

  // take off and start task
  if (current_state.mode == "OFFBOARD" && current_state.armed)
  {
    engageServiceCall();
    return true;
  }

  return false;
}

void AirBumper::crash_recover_FSM()
{
  // collision detected && enable collision reaction && not in reaction
  if (flight_mode == 2 && crash_flag)
  {
    // Do not detect crash until reaction done
    // crash_flag = false;
    exec_flag = true;
    confirm_flag = false;
    // clear task counter
    action_header = 0;
    response_header = 0;
    // reaction_pos = pos_buffer[0];

    /***************************Calculate the grad-guided react pose***************************/
    Eigen::Vector3d pos_crash(local_pos.pose.position.x,
                              local_pos.pose.position.y,
                              local_pos.pose.position.z);
    Eigen::Vector3d grad_dir(0, 0, 0); // Note the direction is away from obstacles!
    double grad_dist = 0;
                            
    if(grad_flg)
    {
      grad_dir = grad_recover_control(pos_crash, grad_dist);

      double Qstar = 5 * react_d; // max range
      double crash_ptd = 0.5 * react_d; // the crash point lies on the frame (d/2)

      if (grad_dist > crash_ptd && grad_dist < Qstar)
      {
        double w1 = (1 / Qstar - 1 / grad_dist) * 1 / (grad_dist * grad_dist); // for grad_dir
        double w2 = (1 / Qstar - 1 / crash_ptd) * 1 / (crash_ptd * crash_ptd); // for crash_point_d (const)
        react_w = w1 / w2;

        // double max_react_w = ((1 / Qstar - 1 / crash_ptd) * 1 / (crash_ptd * crash_ptd)) / w1; // = 1
        // double min_react_w = ((1 / Qstar - 1 / Qstar) * 1 / (Qstar * Qstar)) / w1; // = 0
        // react_w = (react_w - min_react_w) / (max_react_w - min_react_w);

        cout << "grad_dist: " << grad_dist << endl;
        cout << "grad_dir:" << " x: " << grad_dir(0) << " y: " << grad_dir(1) << " z: " << grad_dir(2) << endl;
        cout << "react_w: " << react_w << " w1: " << w1 << " w2: " << w2 << endl;
      }
      else
      {
        react_w = 0; // not conisder grad
      }
    }
    else
    {
      react_w = 0; // not conisder grad
    }

    // double local_react_x = - cos(rad_crash(1)) * react_d * (1 - react_w) + react_w * grad_dir(0);
    // double local_react_y = - sin(rad_crash(1)) * react_d * (1 - react_w) + react_w * grad_dir(1);
    
    /***************************Generate global reaction pose***************************/
    if (abs(crash_point.point.z) > 10)
    {
      react_dz = 0.3; // [m]

      if (crash_point.point.z > 0) // UP collision
      {
        reaction_pos.pose.position.x = local_pos.pose.position.x - cos(rad_crash(1)) * react_d;
        reaction_pos.pose.position.y = local_pos.pose.position.y - sin(rad_crash(1)) * react_d;
        reaction_pos.pose.position.z = local_pos.pose.position.z - react_dz;
      }
      else if (crash_point.point.z < 0) // DOWN collision
      {
        reaction_pos.pose.position.x = local_pos.pose.position.x - cos(rad_crash(1)) * react_d;
        reaction_pos.pose.position.y = local_pos.pose.position.y - sin(rad_crash(1)) * react_d;
        reaction_pos.pose.position.z = local_pos.pose.position.z + react_dz;
      }

      if(grad_flg)
      {
        reaction_pos.pose.position.x = local_pos.pose.position.x - cos(rad_crash(1)) * react_d * (1 - react_w) + react_w * grad_dir(0);
        reaction_pos.pose.position.y = local_pos.pose.position.y - sin(rad_crash(1)) * react_d * (1 - react_w) + react_w * grad_dir(1);
      }

      cout << "Z Axis Reaction!" << endl;
    }
    else // XY collision
    {
      reaction_pos.pose.position.x = local_pos.pose.position.x - cos(rad_crash(1)) * react_d;
      reaction_pos.pose.position.y = local_pos.pose.position.y - sin(rad_crash(1)) * react_d;
      reaction_pos.pose.position.z = flight_alt;

      if(grad_flg)
      {
        reaction_pos.pose.position.x = local_pos.pose.position.x - cos(rad_crash(1)) * react_d * (1 - react_w) + react_w * grad_dir(0);
        reaction_pos.pose.position.y = local_pos.pose.position.y - sin(rad_crash(1)) * react_d * (1 - react_w) + react_w * grad_dir(1);
        reaction_pos.pose.position.z = flight_alt;
      }

      cout << "X-Y Axis Reaction!" << endl;
    }

    cout << "Local Pose x:" << local_pos.pose.position.x
         << ", y:" << local_pos.pose.position.y
         << ", z:" << local_pos.pose.position.z << endl;

    cout << "React Pose x:" << reaction_pos.pose.position.x - local_pos.pose.position.x
         << ", y:" << reaction_pos.pose.position.y - local_pos.pose.position.y
         << ", z:" << reaction_pos.pose.position.z - local_pos.pose.position.z<< endl;

    cout << endl;

    step = 99; // collision reaction
  }

  /***************************FSM for flight & crash reaction***************************/
  switch (step)
  {
  case 0:
    waypoint_lst = 0;
    cout << "Start Task" << endl;
    exec_flag = true;
    waypoint_index = 1;
    step = 1;
    break;

  // WP0 -> WP1 -> WP2 -> WP3 -> WP0
  case 1:
    if (exec_flag && !crash_flag) {
        waypoint_lst = waypoint_index;
        set_pos_control(waypoints[waypoint_index][0], waypoints[waypoint_index][1], waypoints[waypoint_index][2]);
        exec_flag = false;
    }

    if (confirm_flag) {
        exec_flag = true;
        confirm_flag = false;
        waypoint_index = (waypoint_index + 1) % waypoint_num;
        step = 1;
    }
    break;

  case 99: // reaction after crashing
    if (exec_flag) // don't update waypoint_lst
    {
      set_pos_control(reaction_pos.pose.position.x,
                      reaction_pos.pose.position.y,
                      reaction_pos.pose.position.z, "crash");

      cout << "RAPID REACT!" << endl;

      react_pub.publish(reaction_pos);

      crash_flag = false;
      exec_flag = false;
    }

    if (confirm_flag)
    {
      // use motion planning module to wait and replan
      exec_flag = true;
      confirm_flag = false;
      waypoint_index = waypoint_lst;
      step = 1;
    }

    break;

  default:
    break;
  }

  // cout << "REACTION POS_x:%f, POS_y:%f", reaction_pos.pose.position.x, reaction_pos.pose.position.y);

  // cout << "Crash Pose x: " << motion_vector.tgt_pose.pose.position.x
  //                << " y: " << motion_vector.tgt_pose.pose.position.y
  //                << " z: " << motion_vector.tgt_pose.pose.position.z;

  // Publish Pos
  motion_pub.publish(motion_vector);
}

} // namespace air_bumper
