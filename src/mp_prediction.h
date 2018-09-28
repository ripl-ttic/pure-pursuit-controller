#ifndef __mp_prediction_h__
#define __mp_prediction_h__

#include <unistd.h>
#include <glib.h>
#include <getopt.h>
#include <string.h>


#include <GL/gl.h>
#include <lcm/lcm.h>

#include <bot_core/bot_core.h>
#include <bot_lcmgl_client/lcmgl.h>

#include <lcmtypes/hr_lcmtypes.h>
#include <lcmtypes/obstacle_detector_lcmtypes.h>


typedef struct _mp_state_t {
    double x;       // first cartesian coordinate
    double y;       // second cartesian coordinate
    double t;       // orientation -- theta
} mp_state_t;


typedef struct _mp_input_t {
    double v;       // speed
    double d;       // steering -- delta
} mp_input_t;


typedef struct _mp_prediction_t {

    // Parameters of the vehicle model
    double wheelbase; // wheel base

    // Tunable controller parameters
    double K_str;
    double K_ct;

    double max_tv;
    double max_steer;

    bot_lcmgl_t *lcmgl; // To draw the path.
} mp_prediction_t;


mp_state_t* mp_prediction_new_state (mp_prediction_t *self);
mp_input_t* mp_prediction_new_input (mp_prediction_t *self);
int mp_prediction_delete_state (mp_prediction_t *self, mp_state_t *state);
int mp_prediction_delete_input (mp_prediction_t *self, mp_input_t *input);


mp_prediction_t* mp_prediction_create (lcm_t *lcm);


int mp_prediction_propagate_trajectory (mp_prediction_t *self, mp_state_t *state_ini, mp_state_t *state_fin,
                                        GSList **states_out, GSList **inputs_out);

int mp_prediction_destroy_trajectory (mp_prediction_t *self, GSList *states, GSList *inputs);


int mp_prediction_draw_trajectory (mp_prediction_t *self, GSList *states, GSList *inputs, double height);


obs_rect_t* mp_prediction_check_collision_with_rect_list (mp_prediction_t *self, GSList *states, GSList *inputs,
                                                                 obs_rect_list_t *rect_list);

int mp_prediction_draw_colliding_rect (mp_prediction_t *self, obs_rect_t *rect);

int mp_prediction_check_proximity_map (mp_prediction_t *self, bot_core_planar_lidar_t *proximity_map);


#endif
