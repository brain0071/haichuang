#include "Sub.h"


/**
 * @file control_guided.cpp
 * @author naodai (18341314091brain@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2022-05-31  modify: 2023.03.04
 * @copyright Copyright (c) 2022
 * 
 */
#include "Sub.h"


// max time target position no update, loss target
#define AUTO_GRASP_TIMEOUT_MS 3000
#define POS_UPDATE_TIMEOUT_MS 5000
#define VEL_UPDATE_TIMEOUT_MS 5000

// error distance less than 2,indicate grasping,position hold  
#define MAX_GRASP_DISTANCE 2  

// PID parameters
#define POSCONTROL_ACCEL_Z                    250.0f  // default vertical acceleration in cm/s/s.
#define POSCONTROL_POS_Z_P                    3.0f    // vertical position controller P gain default
#define POSCONTROL_VEL_Z_P                    8.0f    // vertical velocity controller P gain default
#define POSCONTROL_ACC_Z_P                    0.5f    // vertical acceleration controller P gain default
#define POSCONTROL_ACC_Z_I                    0.1f    // vertical acceleration controller I gain default
#define POSCONTROL_ACC_Z_D                    0.0f    // vertical acceleration controller D gain default
#define POSCONTROL_ACC_Z_IMAX                 100     // vertical acceleration controller IMAX gain default
#define POSCONTROL_ACC_Z_FILT_HZ              20.0f   // vertical acceleration controller input filter default
#define POSCONTROL_ACC_Z_DT                   0.0025f // vertical acceleration controller dt default

#define POSCONTROL_ACCEL_XY                   100.0f  // default horizontal acceleration in cm/s/s.  This is overwritten by waypoint and loiter controllers
#define POSCONTROL_POS_XY_P                   1.0f    // horizontal position controller P gain default   
#define POSCONTROL_VEL_XY_P                   1.0f    // horizontal velocity controller P gain default
#define POSCONTROL_VEL_XY_I                   0.5f    // horizontal velocity controller I gain default
#define POSCONTROL_VEL_XY_D                   0.0f    // horizontal velocity controller D gain default
#define POSCONTROL_VEL_XY_IMAX                1000.0f // horizontal velocity controller IMAX gain default
#define POSCONTROL_VEL_XY_FILT_HZ             5.0f    // horizontal velocity controller input filter
#define POSCONTROL_VEL_XY_FILT_D_HZ           5.0f    // horizontal velocity controller input filter for D
#define GRAVITY_MSS                           9.80665f



//naodai:2022.05.11 april tag information *********************************
static Vector3f pos_target_cm;
static Vector3f current_vel_cm;
static uint32_t pos_update_time_ms;
static uint32_t vel_update_time_ms;



struct Guided_Limit {
    uint32_t timeout_ms;  // timeout (in seconds) from the time that guided is invoked
    float alt_min_cm;   // lower altitude limit in cm above home (0 = no limit)
    float alt_max_cm;   // upper altitude limit in cm above home (0 = no limit)
    float horiz_max_cm; // horizontal position limit in cm from where guided mode was initiated (0 = no limit)
    uint32_t start_time;// system time in milliseconds that control was handed to the external computer
    Vector3f start_pos; // start position as a distance from home in cm.  used for checking horiz_max limit
} guided_limit;


// guided_init - initialise guided controller
bool Sub::guided_init(bool ignore_checks)
{
    if (!position_ok() && !ignore_checks) {
        return false;
    }
    
    return true;
}

 
// for grapsing 
// use apriltag position and velocity, ignore attitude and angle velocity. 
void Sub::guided_simPos_control_start()
{    
    guided_mode = Guided_WP;
    // gcs().send_text(MAV_SEVERITY_INFO, "guided mode:Guided_WP.\n");
    
}

// set position
bool Sub::guided_set_destination_pos(const Vector3f& pos){
    if (guided_mode != Guided_WP) {
        guided_simPos_control_start();
    }
    pos_target_cm.x = pos.x;
    pos_target_cm.y = pos.y;
    pos_target_cm.z = pos.z;
    pos_update_time_ms = AP_HAL::millis();
    printf("position target x:%f position target y:%f position targer z:%f\n", pos_target_cm.x, pos_target_cm.y,pos_target_cm.z);
    // gcs().send_text(MAV_SEVERITY_INFO, "position target x:%f position target y:%f current velocity x:%f current velocity y:%f\n", pos_target_cm.x, pos_target_cm.y,current_vel_cm.x, current_vel_cm.y);
    return true;
}

// set current velocity
bool Sub::guided_set_vel(const Vector3f& vel){
    if (guided_mode != Guided_WP) {
        guided_simPos_control_start();
    }

    current_vel_cm.x = vel.x;
    current_vel_cm.y = vel.y;
    current_vel_cm.z = vel.z;
    vel_update_time_ms = AP_HAL::millis();
    printf("current velocity x:%f current velocity y:%f current velocity z:%f\n", current_vel_cm.x, current_vel_cm.y,current_vel_cm.z);
    return true;
}

// guided_run - runs the guided controller
// should be called at 100hz or more
void Sub::guided_run()
{
    // call the correct auto controller
    switch (guided_mode) {
    case Guided_WP:
        // run position controller
        guided_simPos_control_run();
        break;
    case Guided_Velocity:
    //     // run velocity controller
    //     guided_vel_control_run();
        break;
    case Guided_PosVel:
        // run position-velocity controller
        // guided_posvel_control_run();
        break;
    case Guided_Angle:
    //     // run angle controller
    //     guided_angle_control_run();
        break;
    }
}

void Sub::guided_simPos_control_run()
{
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        gcs().send_text(MAV_SEVERITY_INFO, "motor disarm\n");
        sub.motors.armed(true);
        // printf("naodai: motor disarm.\n");
        // return;
    }
    // set motors to full range
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    
    // time out
    uint32_t tnow_pos = AP_HAL::millis();

    if (tnow_pos - pos_update_time_ms > POS_UPDATE_TIMEOUT_MS) {
        pos_target_cm.x = 0;
        pos_target_cm.y = 0;
        pos_target_cm.z = 0;
        gcs().send_text(MAV_SEVERITY_INFO, "position time out and set manual.\n");
        //set mamual mode
        sub.motors.armed(false);
        set_mode(MANUAL, MODE_REASON_TX_COMMAND);
        
        return;
        
    } 
    uint32_t tnow_vel = AP_HAL::millis();

    if (tnow_vel - vel_update_time_ms > VEL_UPDATE_TIMEOUT_MS) {
        current_vel_cm.x = 0;
        current_vel_cm.y = 0;
        current_vel_cm.z = 0;
        //set manual mode
        sub.motors.armed(false);
        set_mode(MANUAL, MODE_REASON_TX_COMMAND);
        gcs().send_text(MAV_SEVERITY_INFO, "velocity time out and set manual.\n");
        return;
    }

    float auto_dt = scheduler.get_loop_period_s();
    
    // 3-D P controller 
    Vector3f vel_target;
    vel_target = AC_PosControl::sqrt_controller(pos_target_cm, POSCONTROL_POS_XY_P, POSCONTROL_ACCEL_XY);

    // 2-D PID    
    Vector2f vel_target_xy;
    vel_target_xy.x = vel_target.x;
    vel_target_xy.y = vel_target.y;

    Vector2f cur_xy_vel;
    cur_xy_vel.x = current_vel_cm.x;
    cur_xy_vel.y = current_vel_cm.y;

    Vector2f vel_xy_error;
    vel_xy_error.x = vel_target_xy.x - cur_xy_vel.x;
    vel_xy_error.y = vel_target_xy.y - cur_xy_vel.y;

    AC_PID_2D _pid_vel_xy(POSCONTROL_VEL_XY_P, POSCONTROL_VEL_XY_I, POSCONTROL_VEL_XY_D, POSCONTROL_VEL_XY_IMAX, POSCONTROL_VEL_XY_FILT_HZ, POSCONTROL_VEL_XY_FILT_D_HZ, auto_dt);

    // update to Sub-4.0 pid controller
    Vector2f accel_target, vel_xy_p, vel_xy_i, vel_xy_d;
    
    // call pi controller
    _pid_vel_xy.set_input(vel_xy_error);
    // get p
    vel_xy_p = _pid_vel_xy.get_p();
    // update i term if we have not hit the accel or throttle limits OR the i term will reduce
    // TODO: move limit handling into the PI and PID controller
    vel_xy_i = _pid_vel_xy.get_i();
    // get d
    vel_xy_d = _pid_vel_xy.get_d();

    accel_target.x = vel_xy_p.x + vel_xy_i.x + vel_xy_d.x;
    accel_target.y = vel_xy_p.y + vel_xy_i.y + vel_xy_d.y;
    
    float lateral_out,forward_out;
    lateral_out = accel_target.x;
    forward_out = accel_target.x;
    
    // z-axis PID
    // velocity from pos controller
    float  vel_error_z, accel_target_z;
    // vel_target_z = AC_AttitudeControl::sqrt_controller(pos_target_cm.z, POSCONTROL_POS_Z_P, POSCONTROL_ACCEL_Z, auto_dt);
    vel_error_z = vel_target.z - current_vel_cm.z;
    AC_P _p_vel_z(POSCONTROL_POS_Z_P);
    accel_target_z = _p_vel_z.get_p(vel_error_z);
    float z_accel_meas = -GRAVITY_MSS * 100.0f;
    AC_PID _pid_accel_z(POSCONTROL_ACC_Z_P, POSCONTROL_ACC_Z_I, POSCONTROL_ACC_Z_D, 0.0f, POSCONTROL_ACC_Z_IMAX, 0.0f, POSCONTROL_ACC_Z_FILT_HZ, 0.0f, auto_dt);
    float thr_out = _pid_accel_z.update_all(accel_target_z, z_accel_meas, (motors.limit.throttle_lower || motors.limit.throttle_upper)) * 0.001f +motors.get_throttle_hover();

    lateral_out = lateral_out/500;
    forward_out = forward_out/500;
    // motor input
    motors.set_lateral(lateral_out);
    motors.set_forward(forward_out);
    motors.set_throttle(thr_out);

    printf("naodai: forward: %f, lateral: %f, throttle: %f\n", forward_out, lateral_out, thr_out);
    gcs().send_text(MAV_SEVERITY_INFO, "forward: %f, lateral: %f, throttle: %f\n", forward_out, lateral_out, thr_out);
}




// Guided Limit code
// guided_limit_clear - clear/turn off guided limits
void Sub::guided_limit_clear()
{
    guided_limit.timeout_ms = 0;
    guided_limit.alt_min_cm = 0.0f;
    guided_limit.alt_max_cm = 0.0f;
    guided_limit.horiz_max_cm = 0.0f;
}

// guided_limit_set - set guided timeout and movement limits
void Sub::guided_limit_set(uint32_t timeout_ms, float alt_min_cm, float alt_max_cm, float horiz_max_cm)
{
    guided_limit.timeout_ms = timeout_ms;
    guided_limit.alt_min_cm = alt_min_cm;
    guided_limit.alt_max_cm = alt_max_cm;
    guided_limit.horiz_max_cm = horiz_max_cm;
}
// guided_limit_init_time_and_pos - initialise guided start time and position as reference for limit checking
//  only called from AUTO mode's auto_nav_guided_start function
void Sub::guided_limit_init_time_and_pos()
{
    // initialise start time
    guided_limit.start_time = AP_HAL::millis();
    // initialise start position from current position
    guided_limit.start_pos = inertial_nav.get_position();
}
// guided_limit_check - returns true if guided mode has breached a limit
//  used when guided is invoked from the NAV_GUIDED_ENABLE mission command
bool Sub::guided_limit_check()
{
    // check if we have passed the timeout
    if ((guided_limit.timeout_ms > 0) && (AP_HAL::millis() - guided_limit.start_time >= guided_limit.timeout_ms)) {
        return true;
    }
    // get current location
    const Vector3f& curr_pos = inertial_nav.get_position();
    // check if we have gone below min alt
    if (!is_zero(guided_limit.alt_min_cm) && (curr_pos.z < guided_limit.alt_min_cm)) {
        return true;
    }
    // check if we have gone above max alt
    if (!is_zero(guided_limit.alt_max_cm) && (curr_pos.z > guided_limit.alt_max_cm)) {
        return true;
    }
    // check if we have gone beyond horizontal limit
    if (guided_limit.horiz_max_cm > 0.0f) {
        float horiz_move = get_horizontal_distance_cm(guided_limit.start_pos, curr_pos);
        if (horiz_move > guided_limit.horiz_max_cm) {
            return true;
        }
    }
    // if we got this far we must be within limits
    return false;
}