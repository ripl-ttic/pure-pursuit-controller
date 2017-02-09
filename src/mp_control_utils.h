#ifndef __mp_control_utils_h__
#define __mp_control_utils_h__

#include <glib.h>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

#include <lcm/lcm.h>
#include <bot_core/bot_core.h>
#include <lcmtypes/hr_lcmtypes.h>

typedef struct _mp_vehicle_controller_states_t {
    double integral;
} mp_vehicle_controller_states_t;

typedef struct _mp_states_t {
    double x;
    double y;
    double yaw;
} mp_states_t;

typedef struct _mp_control_aux_t {
    double integral;
    double proportional;
    double control_action;
    double distance_to_target;
    double target_in_front;
    
    double cross_track_error;
    double relative_angle;
    double cross_track_error_correction;
    double relative_angle_correction;

    double steer;
    
    // actuation
    double translational_velocity;
    double rotational_velocity;

    
    // Following values are after appropriate transformations
    double bot_x;    // x coordinate of the position   
    double bot_y;    // y coordinate of the position 
    double bot_yaw;    // yaw angle  
    double bot_v;    // velocity of the bot

    double target_x;   // x coordinate of the bot
    double target_y;   // y coordinate of the bot
    double target_yaw;   // yaw angle of the bot
    // Note that target is stored before M_PI rotataion around the yaw axis
} mp_control_aux_t;

typedef struct _mp_control_utils_t {
    mp_vehicle_controller_states_t veh_cont_states;
    mp_states_t *bot_states;
    mp_states_t *target_states;

    double bot_velocity;

    int64_t last_bot_state_update;

    mp_control_aux_t mp_control_aux;

    gboolean verbose;

} mp_control_utils_t;

mp_control_utils_t *
mp_control_new (void);

void
mp_control_veh_cont_init (mp_control_utils_t *);

int 
mp_control_update_bot_states (mp_control_utils_t *self, bot_core_pose_t *msg);

void
mp_control_update_bot_states2 (mp_control_utils_t *self, double x, double y, double yaw, double vel);

int
mp_control_update_target_states (mp_control_utils_t *self, double x, double y, double yaw);

int 
mp_control_compute_rel_quad (double x, double y, double x_0, double y_0, double theta );

int 
mp_control_compute_distance_to_target (mp_control_utils_t *self, double *distance,
                                       gboolean *target_in_front);

int
mp_control_compute_relangle_crosstrack_via_target_pose (mp_control_utils_t *self, double *relangle, double *crosstrack);

int 
mp_control_compute_control_steer_via_target_pose (mp_control_utils_t *self, double *steer_angle);

int 
mp_control_compute_lon_control_via_distance (mp_control_utils_t *self, double *control_action, double distance, double v_cmd);

int 
mp_control_compute_lon_control_via_target_pose (mp_control_utils_t *self, double *control_action, double v_cmd);

void
mp_control_update_aux_actuation (mp_control_utils_t *self, double trans_vel, double rot_vel);

int
mp_control_publish_aux_message (lcm_t *lcm, mp_control_utils_t *self);


// Trajectory following options

/*

int 
mp_control_set_trajectory (mp_control_utils_t *self, erlcm_engagement_plan_t *ep);

int 
mp_control_update_traj (mp_control_utils_t *self);

int
mp_control_find_first_point_in_front (mp_control_utils_t *self, erlcm_engagement_plan_t *ep);

int 
mp_control_compute_projection_onto_traj (mp_control_utils_t *self, erlcm_engagement_plan_t *ep, int point_in_front,
                                         double *x_proj, double *y_proj, double *theta_proj);

int 
mp_control_compute_control_steer_via_traj (mp_control_utils_t *self, erlcm_engagement_plan_t *ep, 
                                           double *steer_angle);

*/

int 
mp_control_compute_lon_control_backup (mp_control_utils_t *self, double *control_action, double backup_distance, double v_cmd);



#endif
