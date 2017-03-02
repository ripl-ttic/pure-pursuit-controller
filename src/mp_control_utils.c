#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <bot_core/bot_core.h>

/*
#include <common/geometry.h>
#include <common/math_util.h>
#include <common/globals.h>
#include <common/arconf.h>
*/

#include <lcmtypes/rrtstar.h>

#include "mp_control_utils.h"

#define PI M_PI
//#define DT 1/50.0     // Controller execution loop time
                      // This must be updated if the execution loop time is modified

// Lateral controller coefficients 
//#define K_CT 1.5 //0.9//0.7//0.3//0.7
//#define K_CT_TRAJ 2.0 //1.8


// Longitudinal controller coefficients
//#define KP 1.5 //0.75
//#define KI 1.0

//#define MAXINTEGRAL 0.5    // Saturation value of the integral term

#define V_CMD_SAFETY_DEFAULT 0.15 // when v_cmd > (V_CMD_CUTOFF+0.01), this value is applied
                                  // but it would not happen
#define V_CMD_MIN 0.05
#define V_CMD_CUTOFF 2.0//0.5

//#define K_STR 3.0//2.0//1.5//1.0//2.0
//#define K_STR_TRAJ 2.0 //1.0//2.0 //1.0 was used on the chair 

#define K_REL_SPC 0.005


//#define DEFAULT_INTEGRAL_STATE  -0.4


mp_control_utils_t *
mp_control_new (void)
{
    mp_control_utils_t *mp_cont;
    mp_cont = (mp_control_utils_t *) malloc (sizeof(mp_control_utils_t));
    
    mp_cont->veh_cont_states.integral = mp_cont->default_integral_state;

    mp_cont->bot_states = NULL;
    mp_cont->target_states = NULL;
    mp_cont->bot_velocity = 0.0;

    mp_cont->last_bot_state_update = 0;

    mp_cont->mp_control_aux.integral = 0.0;
    mp_cont->mp_control_aux.proportional = 0.0;
    mp_cont->mp_control_aux.control_action = 0.0;
    mp_cont->mp_control_aux.distance_to_target = 0.0;
    mp_cont->mp_control_aux.target_in_front = FALSE;
    
    mp_cont->mp_control_aux.cross_track_error = 0.0;
    mp_cont->mp_control_aux.relative_angle = 0.0;
    mp_cont->mp_control_aux.cross_track_error_correction = 0.0;
    mp_cont->mp_control_aux.relative_angle_correction = 0.0;

    mp_cont->mp_control_aux.steer = 0.0;

    mp_cont->verbose = FALSE;
    
    return mp_cont;
}


void 
mp_control_veh_cont_init (mp_control_utils_t *self) {
    if (!self)
        return;
    self->veh_cont_states.integral = self->default_integral_state;
}


// This function must be run before computing the controls. It sets the parameters 
// related to the bot's position within mp_control_utils_t.
int 
mp_control_update_bot_states (mp_control_utils_t *self, bot_core_pose_t *msg)
{
    double rpy[3];
    if (!msg)
        return -1;
    if (!self->bot_states)
        self->bot_states = (mp_states_t *)malloc (sizeof (mp_states_t));

    bot_quat_to_roll_pitch_yaw (msg->orientation, rpy);

    self->bot_states->x = msg->pos[0];
    self->bot_states->y = msg->pos[1];
    self->bot_states->yaw = rpy[2];
    self->bot_velocity = sqrt(msg->vel[0]*msg->vel[0] + msg->vel[1]*msg->vel[1]);
    self->last_bot_state_update = msg->utime;

    self->mp_control_aux.bot_x = self->bot_states->x;
    self->mp_control_aux.bot_y = self->bot_states->y;
    self->mp_control_aux.bot_yaw = self->bot_states->yaw;
    self->mp_control_aux.bot_v = self->bot_velocity;

    return 1;
}

void
mp_control_update_bot_states2 (mp_control_utils_t *self, double x, double y, double yaw, double vel) {
    if (!self->bot_states)
        self->bot_states = (mp_states_t *) malloc (sizeof (mp_states_t));

    self->bot_states->x = x;
    self->bot_states->y = y;
    self->bot_states->yaw = yaw;
    self->bot_velocity = vel;
    self->last_bot_state_update = bot_timestamp_now ();

    self->mp_control_aux.bot_x = self->bot_states->x;
    self->mp_control_aux.bot_y = self->bot_states->y;
    self->mp_control_aux.bot_yaw = self->bot_states->yaw;
    self->mp_control_aux.bot_v = self->bot_velocity;

    return;
}

void
mp_control_update_aux_actuation (mp_control_utils_t *self, double trans_vel, double rot_vel) {

    self->mp_control_aux.translational_velocity = trans_vel;
    self->mp_control_aux.rotational_velocity = rot_vel;

    return;
}

int
mp_control_update_target_states (mp_control_utils_t *self, double x, double y, double yaw)
{
    if (!self->target_states)
        self->target_states = (mp_states_t *) malloc (sizeof (mp_states_t));
    
    self->target_states->x = x;
    self->target_states->y = y;
    self->target_states->yaw = yaw;

    self->mp_control_aux.target_x = self->target_states->x;
    self->mp_control_aux.target_y = self->target_states->y;
    self->mp_control_aux.target_yaw = self->target_states->yaw;

    // obtain the target surface normal
    self->target_states->yaw += M_PI;
    if (self->target_states->yaw > 2*M_PI)
        self->target_states->yaw -= 2*M_PI;

    return 1;

}

int 
mp_control_compute_rel_quad (double x, double y, double x_0, double y_0, double yaw )
{
    double theta_k = atan2(y - y_0, x - x_0);
    theta_k -= yaw;
    while (theta_k <= 0)
        theta_k += 2*PI;
    if ( (0 <= theta_k) && (theta_k <= PI/2.0) )
        return 1;
    if ( (PI/2.0 <= theta_k) && (theta_k <= PI) )
        return 2;
    if ( (PI <= theta_k) && (theta_k <= 3*PI/2.0) )
        return 3;
    if ( (3.0*PI/2.0 <= theta_k) && (theta_k <= 2.0*PI) )
        return 4;
    return -1;
}

int 
mp_control_compute_distance_to_target (mp_control_utils_t *self, double *distance, 
                                       gboolean *target_in_front)
{
    if (!self->bot_states) {
        printf ("ERROR: No bot states\n");
        return -1;
    }
    if (!self->target_states) {
        printf ("ERROR: No target states\n");
        return -2;
    }

    // compute the distance of the bot to the line that is parallel to the target surface
    *distance = (fabs(-tan(self->target_states->yaw+M_PI_2)*self->bot_states->x + self->bot_states->y 
                 + tan(self->target_states->yaw+M_PI_2)*self->target_states->x - self->target_states->y))/ 
                    sqrt(tan(self->target_states->yaw+M_PI_2)*tan(self->target_states->yaw+M_PI_2)+1.0);

    int rel_quad_target = mp_control_compute_rel_quad (self->target_states->x, self->target_states->y,
                                                       self->bot_states->x, self->bot_states->y,
                                                       self->bot_states->yaw); 

    if (rel_quad_target == -1)
        return -1; // target position relative to target can not be determined.
                   // Then again, this should never happen
    
    if (rel_quad_target == 1 || rel_quad_target == 4)
        *target_in_front = TRUE;
    else 
        *target_in_front = FALSE;
    
    return 1;
}

int
mp_control_compute_relangle_crosstrack_via_target_pose (mp_control_utils_t *self, double *relangle, double *crosstrack) {
    if (!self->bot_states)
        return -1; // Error: No bot pose 
    if (!self->target_states)
        return -2; // Error: No target pose

    int rel_quad_target = mp_control_compute_rel_quad (self->target_states->x, self->target_states->y,
                                                       self->bot_states->x, self->bot_states->y,
                                                       self->bot_states->yaw); 
    
    if (rel_quad_target == -1)
        return -3; // Error: Target orientation relative bot not determined 

    //if (rel_quad_target == 2 || rel_quad_target == 3)
      //  return -4; // Error: Target behind the bot.

    *relangle = PI - (self->bot_states->yaw - self->target_states->yaw);
    while (*relangle < -PI)
        *relangle += 2*PI;
    while (*relangle > PI)
        *relangle -= 2*PI;
   
    // compute the distance of the bot to the line that is normal to the target surface
    *crosstrack = (fabs(-tan(self->target_states->yaw)*self->bot_states->x + self->bot_states->y 
                        + tan(self->target_states->yaw)*self->target_states->x - self->target_states->y))/ 
                              sqrt(tan(self->target_states->yaw)*tan(self->target_states->yaw)+1.0);

    return 1;
}

int
mp_control_compute_control_steer_via_target_pose (mp_control_utils_t *self, double *steer_angle)
{
    *steer_angle = 0.0;
    if (!self->bot_states)
        return -1; // Error: No bot pose 
    if (!self->target_states)
        return -2; // Error: No target pose
#if 0
    fprintf(stderr, "--Delta x : %.2f , Delta y : %.2f, Delta Yaw : %.2f\n", 
            self->target_states->x - self->bot_states->x, 
            self->target_states->y - self->bot_states->y, 
            PI - (self->bot_states->yaw - self->target_states->yaw));
#endif

    int rel_quad_target = mp_control_compute_rel_quad (self->target_states->x, self->target_states->y,
                                                       self->bot_states->x, self->bot_states->y,
                                                       self->bot_states->yaw); 
    
    if (rel_quad_target == -1)
        return -3; // Error: Target orientation relative bot not determined 
                   //        This should never happen

    if (self->verbose) {
        printf ("Bot states: %5.2lf, %5.2lf, %5.2lf; Target states: %5.2lf, %5.2lf, %5.2lf\n", 
                self->bot_states->x, self->bot_states->y, self->bot_states->yaw,
                self->target_states->x, self->target_states->y, self->target_states->yaw);
    }

    double relative_angle = PI - (self->bot_states->yaw - self->target_states->yaw);
    while (relative_angle < -PI)
        relative_angle += 2*PI;
    while (relative_angle > PI)
        relative_angle -= 2*PI;
    
    // compute the distance of the bot to the line that is normal to the target surface
    double cross_track_error = (fabs(-tan(self->target_states->yaw)*self->bot_states->x + self->bot_states->y 
                                     + tan(self->target_states->yaw)*self->target_states->x - self->target_states->y))/ 
        sqrt(tan(self->target_states->yaw)*tan(self->target_states->yaw)+1.0);

    //this part looks wierd - both cross track and relative angle corrrection is 
    //being amplified by the same gain 

    
    
    double cross_track_error_correction = self->K_str*atan (self->K_ct * cross_track_error);

    double relative_angle_correction = self->K_str*relative_angle;

    


    int rel_quad_bot = mp_control_compute_rel_quad (self->bot_states->x, self->bot_states->y,
                                                    self->target_states->x, self->target_states->y,
                                                    self->target_states->yaw); 
    if (rel_quad_bot == 1)
        *steer_angle = relative_angle_correction + cross_track_error_correction;
    else if (rel_quad_bot == 4)
        *steer_angle = relative_angle_correction - cross_track_error_correction;

    //this wont give any steering angle if its in another quadrant 

#if 0
    fprintf(stderr, " -- Crosstrack Error : %.2f Angle Correction : %.2f Quad : %d, Steer : %.2f\n", 
            cross_track_error_correction, relative_angle_correction, rel_quad_bot, 
            *steer_angle); 
#endif
    // Saturate steering if necessary TODO: this saturation can be made a bit smaller
    //max steer needs to be clamped at +/- PI/2
    //otherwise this will cause the tan value to flip sign and make the bot turn the other way 

    double max_steer = bot_to_radians(80);
    if (*steer_angle  >= max_steer)
        *steer_angle = max_steer;
    if (*steer_angle <= -max_steer)
        *steer_angle = -max_steer;
    
    // Update Controller Aux
    self->mp_control_aux.cross_track_error = cross_track_error;
    self->mp_control_aux.relative_angle = relative_angle;
    self->mp_control_aux.cross_track_error_correction = cross_track_error_correction;
    self->mp_control_aux.relative_angle_correction = relative_angle_correction;
    self->mp_control_aux.steer = *steer_angle;

    if (self->verbose) {
        printf("Rel_angle: %5.5lf; Cross_track_err: %5.10lf; Corrections: %5.5lf - %5.5lf steering: %5.5lf - rel_quad: %d\n", 
               relative_angle * 180 /PI, cross_track_error, 
               relative_angle_correction*180/M_PI, cross_track_error_correction*180/M_PI, *steer_angle * 180/M_PI, rel_quad_bot);
    }
    return 1;
}


int 
mp_control_compute_lon_control_via_distance (mp_control_utils_t *self, double *control_action, double distance, double v_cmd) {

    gboolean overshoot = FALSE;

    if (distance < 0.0) {
        // This means we overshooted and need to stop!
        // TODO: handle this case better
        distance = 0.0;
        overshoot = TRUE;
    }

    if (v_cmd < 0 || v_cmd > V_CMD_CUTOFF + 0.01) // would not happen, but just in case
        v_cmd = V_CMD_SAFETY_DEFAULT;

    double v_command = distance * v_cmd/V_CMD_CUTOFF;
    if (v_command >= v_cmd)
        v_command = v_cmd;
    if (v_command <= V_CMD_MIN)
        v_command = V_CMD_MIN;

    // calculate the control action
    double proportional_control_action =  self->Kp*(v_command - self->bot_velocity);
    double integral_control_action = self->Ki*(self->veh_cont_states.integral);
    *control_action = proportional_control_action + integral_control_action;

    // TODO: Some more heuristics here for better target engagement
    // Add extra control if the palet is behind
    if (overshoot)
        *control_action -=0.2;
    
    // Saturate the control action.
    if (*control_action > 1.0)
        *control_action = 1.0;
    if (*control_action < -1.0)
        *control_action = -1.0;

    // update integral state
    self->veh_cont_states.integral += (v_command - self->bot_velocity)*self->dt;
    if (self->veh_cont_states.integral >= self->max_integral)
        self->veh_cont_states.integral = self->max_integral;
    
    if (self->verbose) {
        printf ("Control action: %5.5lf; Distance : %5.5lf \n", 
                *control_action, distance);
    }

    // Update Controller Aux
    self->mp_control_aux.integral = integral_control_action;
    self->mp_control_aux.proportional = proportional_control_action;
    self->mp_control_aux.control_action = *control_action;
    self->mp_control_aux.distance_to_target = distance;
    self->mp_control_aux.target_in_front = !overshoot;

    return 1;

}

int 
mp_control_compute_lon_control_via_target_pose (mp_control_utils_t *self, double *control_action, double v_cmd)
{    

    // Nonlinear corrections to the control action:
    double distance_to_target;
    gboolean target_in_front;
    if ( mp_control_compute_distance_to_target (self, &distance_to_target, &target_in_front) == -1)
        return -1; // distance to target can not be computed...

    /*if (v_cmd < 0 || v_cmd > V_CMD_CUTOFF + 0.01) // would not happen, but just in case
        v_cmd = V_CMD_SAFETY_DEFAULT;*/

    if ( v_cmd < 0 )
	v_cmd = V_CMD_SAFETY_DEFAULT;
    if ( v_cmd > V_CMD_CUTOFF + 0.01 )
        v_cmd = V_CMD_CUTOFF;

    double v_command = v_cmd;///V_CMD_CUTOFF;//distance_to_target * v_cmd/V_CMD_CUTOFF;
    if (v_command >= v_cmd)
        v_command = v_cmd;
    if (v_command <= V_CMD_MIN)
        v_command = V_CMD_MIN;

    // calculate the control action
    double proportional_control_action =  self->Kp*(v_command - self->bot_velocity);
    double integral_control_action = self->Ki*(self->veh_cont_states.integral);
    *control_action = proportional_control_action + integral_control_action;

    // TODO: Some more heuristics here for better target engagement
    // Add extra control if the palet is behind
    if (!target_in_front)
        *control_action -=0.2;
    
    // Saturate the control action.
    if (*control_action > 1.0)
        *control_action = 1.0;
    if (*control_action < -1.0)
        *control_action = -1.0;

    // update integral state
    self->veh_cont_states.integral += (v_command - self->bot_velocity)*self->dt;
    if (self->veh_cont_states.integral >= self->max_integral)
        self->veh_cont_states.integral = self->max_integral;
    
    if (self->verbose) {
        printf ("P:%2.5lf I:%2.5lf - Control action: %5.5lf; Distance to target: %5.5lf, Target in front: %d \n", 
                proportional_control_action, integral_control_action, *control_action,
                distance_to_target, target_in_front?1:0);
    }

    // Update Controller Aux
    self->mp_control_aux.integral = integral_control_action;
    self->mp_control_aux.proportional = proportional_control_action;
    self->mp_control_aux.control_action = *control_action;
    self->mp_control_aux.distance_to_target = distance_to_target;
    self->mp_control_aux.target_in_front = target_in_front;
    return 1;
}

int
mp_control_publish_aux_message (lcm_t *lcm, mp_control_utils_t *self) {
  
    erlcm_trajectory_controller_aux_t mc_aux = {
        .utime = bot_timestamp_now (),
        .integral = self->mp_control_aux.integral,
        .proportional = self->mp_control_aux.proportional,
        .control_action = self->mp_control_aux.control_action,
        .distance_to_target = self->mp_control_aux.distance_to_target,
        .target_in_front = self->mp_control_aux.target_in_front,
        .cross_track_error = self->mp_control_aux.cross_track_error,
        .relative_angle = self->mp_control_aux.relative_angle,
        .cross_track_error_correction = self->mp_control_aux.cross_track_error,
        .relative_angle_correction = self->mp_control_aux.relative_angle_correction,
        .translational_velocity = self->mp_control_aux.translational_velocity,
        .rotational_velocity = self->mp_control_aux.rotational_velocity,
        .steer = self->mp_control_aux.steer,
        .bot_x = self->mp_control_aux.bot_x,
        .bot_y = self->mp_control_aux.bot_y,
        .bot_yaw = self->mp_control_aux.bot_yaw,
        .bot_v = self->mp_control_aux.bot_v,
        .target_x = self->mp_control_aux.target_x,
        .target_y = self->mp_control_aux.target_y,
        .target_yaw = self->mp_control_aux.target_yaw
    };
    erlcm_trajectory_controller_aux_t_publish (lcm, "TRAJECTORY_CONTROLLER_AUX", &mc_aux);
 
    return 1;
}


// This is used for backing up
int 
mp_control_compute_lon_control_backup (mp_control_utils_t *self, double *control_action, double backup_distance, double v_cmd) {

    // Nonlinear corrections to the control action:
    double distance_to_target;
    gboolean target_in_front;
    if ( mp_control_compute_distance_to_target (self, &distance_to_target, &target_in_front) == -1)
        return -1; // distance to target can not be computed...

    if (v_cmd < 0 || v_cmd > V_CMD_CUTOFF + 0.01) // would not happen, but just in case
        v_cmd = V_CMD_SAFETY_DEFAULT;

    // TODO: 2.5 should be made a parameter somewhere
    double v_command = (backup_distance - distance_to_target) * v_cmd/V_CMD_CUTOFF;
    if (v_command >= v_cmd)
        v_command = v_cmd;
    if (v_command <= V_CMD_MIN)
        v_command = V_CMD_MIN;

    // calculate the control action
    double proportional_control_action =  self->Kp*(v_command - self->bot_velocity);
    double integral_control_action = self->Ki*(self->veh_cont_states.integral);
    *control_action = proportional_control_action + integral_control_action;
    
    // Saturate the control action.
    if (*control_action > 1.0)
        *control_action = 1.0;
    if (*control_action < -1.0)
        *control_action = -1.0;

    // update integral state
    self->veh_cont_states.integral += (v_command - self->bot_velocity)*self->dt;
    if (self->veh_cont_states.integral >= self->max_integral)
        self->veh_cont_states.integral = self->max_integral;
    
    if (self->verbose) {
        printf ("Control action: %5.5lf; Distance to target: %5.5lf, Target in front: %d \n", 
                *control_action, distance_to_target, target_in_front?1:0);
    }

    // Update Controller Aux
    self->mp_control_aux.integral = integral_control_action;
    self->mp_control_aux.proportional = proportional_control_action;
    self->mp_control_aux.control_action = *control_action;
    self->mp_control_aux.distance_to_target = distance_to_target;
    self->mp_control_aux.target_in_front = target_in_front;
    return 1;
}

// This function is used to project the vehicle onto the reference trajectory.
// More precisely, it returns the index to the first point that is in front 
// of the vehicle.

/*
int
mp_control_find_first_point_in_front (mp_control_utils_t *self, erlcm_engagement_plan_t *ep)
{

    if ( (!ep) ||  (!self->bot_states) ) {
        printf ("mp_control_find_first_point_in_front: failed\n");
        return -1;
    }
    if (ep->num_steps == 0) {
        printf ("mp_control_find_first_point_in_front: Engagement plan is empty\n");
        return -2;
    }

    for (int i = 0; i < ep->num_steps; i++) {
        int rel_quad = mp_control_compute_rel_quad (ep->steps[i].x, ep->steps[i].y,
                                                    self->bot_states->x, self->bot_states->y, 
                                                    self->bot_states->yaw);
        if ( (rel_quad == 1) || (rel_quad == 4) )
            return i;
    }
    //printf("ERROR ALL OVER\n");
    return ep->num_steps;
}


int 
mp_control_compute_projection_onto_traj (mp_control_utils_t *self, erlcm_engagement_plan_t *ep, int point_in_front, 
                                         double *x_proj, double *y_proj, double *theta_proj)
{
    // For now assume that the points in the trajectory are very dense and that the 
    // trajectory extends to infinity
    if (point_in_front == 0);
    if (point_in_front < 0) {
        return -1; //error
    }
    if (point_in_front > ep->num_steps) {
        return -1; //end of the trajectory.
    }
    if (point_in_front == ep->num_steps) {
        point_in_front = ep->num_steps - 1;
    }

    *x_proj = ep->steps[point_in_front].x;
    *y_proj = ep->steps[point_in_front].y;
    *theta_proj = ep->steps[point_in_front].theta;
    return 1;
}


// This is a temporary function just for testing the trajectory tracker. 
int 
mp_control_update_traj (mp_control_utils_t *self) {
    return 1;
}


int 
mp_control_set_trajectory (mp_control_utils_t *self, erlcm_engagement_plan_t *ep) {
    return 1;
}


int
mp_control_compute_control_steer_via_traj (mp_control_utils_t *self, erlcm_engagement_plan_t *ep, 
                                           double *steer_angle)
{
    // TODO: update the trajectory points according to the moving pallet
    double x_proj=0.0, y_proj=0.0, theta_proj=0.0;
    int point_in_front;
    
    point_in_front = mp_control_find_first_point_in_front (self, ep);

    mp_control_compute_projection_onto_traj(self, ep, point_in_front, &x_proj, &y_proj, &theta_proj);
       
    double cross_track_error = sqrt ( (x_proj - self->bot_states->x)*(x_proj - self->bot_states->x) + 
                                    (y_proj - self->bot_states->y)*(y_proj - self->bot_states->y) );
    
    double relative_angle = theta_proj - self->bot_states->yaw;
   
    double cross_track_error_correction = self->K_str_traj*atan (self->K_ct_traj * cross_track_error);

    double relative_angle_correction = self->K_str_traj*K_REL_SPC*relative_angle;

    int rel_quad_bot = mp_control_compute_rel_quad (self->bot_states->x, self->bot_states->y,
                                                    x_proj, y_proj, theta_proj); 

    if (self->verbose) {
        printf ("rel_quad_bot: %d\n", rel_quad_bot);
    }

    if ( (rel_quad_bot == 1) || (rel_quad_bot == 2) )
        *steer_angle = relative_angle_correction + cross_track_error_correction;
    else if ( (rel_quad_bot == 3) || (rel_quad_bot == 4) )
        *steer_angle = relative_angle_correction - cross_track_error_correction;

    
    // Saturate steering if necessary
    if (*steer_angle  >= 1.0)
        *steer_angle = 1.0;
    if (*steer_angle <= -1.0)
        *steer_angle = -1.0;
    
    // Update Controller Aux
    self->mp_control_aux.cross_track_error = cross_track_error;
    self->mp_control_aux.relative_angle = relative_angle;
    self->mp_control_aux.cross_track_error_correction = cross_track_error_correction;
    self->mp_control_aux.relative_angle_correction = relative_angle_correction;

    if (self->verbose) {
        printf("Rel_angle: %5.5lf; Cross_track_err: %5.10lf; Corrections: %5.5lf - %5.5lf\n", 
               relative_angle*180 /PI, cross_track_error, 
               relative_angle_correction*180/M_PI, cross_track_error_correction*180/M_PI);
    }

    return 1;
}
*/

