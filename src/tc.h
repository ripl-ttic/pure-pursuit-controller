#ifndef __trajectory_controller_h__
#define __trajectory_controller_h__

#include <glib.h>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

#include <lcm/lcm.h>
#include <bot_core/bot_core.h>

#include <lcmtypes/er_lcmtypes.h>
  
typedef enum _manipulation_job_t{
    IDLE = 0,
    PICKUP_PALLET,
    DROPOFF_PALLET
} manipulation_job_t;

typedef struct _trajectory_controller_params_t trajectory_controller_params_t;
struct _trajectory_controller_params_t {

    double mast_max_carriage_height;
    double mast_max_sideshift;
    double mast_min_sideshift;
    double mast_max_separation;
    double mast_min_separation;
    double mast_max_tilt;

    double default_carriage_height_no_load;
    double default_mast_tilt_no_load;

    double default_carriage_height_with_load;
    double default_mast_tilt_with_load;
};


typedef struct _TrajectoryController TrajectoryController;
struct _TrajectoryController {

    lcm_t *lcm;
    GMutex *mutex;
    CTrans *ctrans;
    MTrans *mtrans;
    trajectory_controller_params_t params;

    erlcm_actuation_command_mast_t *mast_command_last;
    erlcm_trajectory_controller_job_enum_t current_manipulation_job;
    erlcm_trajectory_controller_action_enum_t current_manipulation_action;
    gboolean manipulation_standby;
};


TrajectoryController * tc_new ();

void tc_destroy (TrajectoryController *self);

erlcm_trajectory_controller_action_enum_t tc_next_action ( erlcm_trajectory_controller_job_enum_t current_job,
                                                          erlcm_trajectory_controller_action_enum_t current_action);

#endif
