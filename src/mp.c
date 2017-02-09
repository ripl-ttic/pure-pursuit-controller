#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <bot_core/bot_core.h>

/*#include <common/geometry.h>
#include <common/math_util.h>
#include <common/globals.h>
#include <common/arconf.h>

#include <lcmtypes/lcmtypes.h>
*/
#include <lcmtypes/er_lcmtypes.h>

#include "mp.h"

#define APPROACH_ORIENTATION_TOL 10*M_PI/180

void
mp_get_params (ManipulationPlanner *self, BotConf *config)
{
    
    self->params.default_carriage_height_no_load = bot_conf_get_double_or_fail (config, "manipulation_planner.mast.default_carriage_height_no_load");
    self->params.default_mast_tilt_no_load = bot_conf_get_double_or_fail (config, "manipulation_planner.mast.default_mast_tilt_no_load");

    self->params.default_carriage_height_with_load = bot_conf_get_double_or_fail (config, "manipulation_planner.mast.default_carriage_height_with_load");
    self->params.default_mast_tilt_with_load = bot_conf_get_double_or_fail (config, "manipulation_planner.mast.default_mast_tilt_with_load");

    self->params.mast_max_carriage_height = bot_conf_get_double_or_fail (config, "mast.max_fork_height") / 100;
    self->params.mast_max_sideshift = bot_conf_get_double_or_fail (config, "mast.max_sideshift_left") / 100;
    self->params.mast_max_separation = bot_conf_get_double_or_fail (config, "mast.carriage_width") / 100;
    self->params.mast_min_separation = 0.0; //bot_conf_get_double_or_fail (config, "mast.carriage_width") / 100;
    self->params.mast_max_tilt = 6.0 * M_PI / 180;
  
    return;
}

ManipulationPlanner *
mp_new ()
{
    ManipulationPlanner * self = calloc (1, sizeof (ManipulationPlanner));

    self->lcm = globals_get_lcm();

    self->ctrans = globals_get_ctrans();
    self->mtrans = globals_get_mtrans();

    // parse configuration parameters
    BotConf *config = globals_get_config ();
    mp_get_params (self, config);
    globals_release_config (config);
    config = NULL;

    self->mast_command_last = NULL;

    self->current_manipulation_job = ERLCM_MANIPULATION_PLANNER_JOB_ENUM_T_IDLE;
    self->current_manipulation_action = ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_IDLE;
    self->manipulation_standby = TRUE;

    return self;
}


void 
mp_destroy (ManipulationPlanner *self)
{

    if (!self)
        return;

    if (self->ctrans)
        globals_release_ctrans (self->ctrans);

    if (self->mtrans)
        globals_release_ctrans (self->ctrans);

    if (self->mast_command_last)
        erlcm_actuation_command_mast_t_destroy (self->mast_command_last);

    if (self->lcm)
        globals_release_lcm (self->lcm);
  
     free (self);
}


erlcm_manipulation_planner_action_enum_t mp_next_action ( erlcm_manipulation_planner_job_enum_t current_job,
							  erlcm_manipulation_planner_action_enum_t current_action)
{
    // A deterministic state machine for each job type
    if ( current_job == ERLCM_MANIPULATION_PLANNER_JOB_ENUM_T_PICKUP_PALLET) {
        switch ( current_action ) {

        case ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_COLLECT_SCANS:
        case ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_COLLECT_SCANS_TILT:
        case ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_COLLECT_SCANS_DRIVE:
            return ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_IDLE;
//            return ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_DETECT_PALLET_VIA_TILT;
//            return ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_DETECT_PALLET_VIA_HEIGHT;

        case ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_DETECT_PALLET_VIA_TILT:
            return ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_APPROACH_PALLET;
//            return ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_IDLE;

        case ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_APPROACH_PALLET:
            return ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_REACQUIRE_PALLET_VIA_TILT;
//          return ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_IDLE;

        case ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_REACQUIRE_PALLET_VIA_TILT:
            return  ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_INSERT_TINES;

        case ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_INSERT_TINES:
            return ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_RAISE_CARRIAGE;

        case ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_RAISE_CARRIAGE:
            return ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_GROUND_BACKUP;

        case ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_GROUND_BACKUP:
            return ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_IDLE;


        case ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_DETECT_PALLET_VIA_HEIGHT:
            return ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_TRUCK_APPROACH_FOR_PICKUP;

        case  ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_TRUCK_APPROACH_FOR_PICKUP:
            return ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_TRUCK_REACQUIRE_PALLET_VIA_HEIGHT;

        case ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_TRUCK_REACQUIRE_PALLET_VIA_HEIGHT:
            //            return ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_IDLE;
            return ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_TRUCK_INSERT_TINES;

        case ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_TRUCK_INSERT_TINES:
            //            return ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_IDLE;
            return ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_TRUCK_RAISE_CARRIAGE;

        case ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_TRUCK_RAISE_CARRIAGE:
            //            return ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_IDLE;
            return ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_TRUCK_BACKUP;

        case  ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_TRUCK_BACKUP:
            return ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_IDLE;


/*             // ------------------------------------------------------------- */
/*             // These are the drop off actions - they are put here temporarily */
/*         case ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_DRIVE_TO_GESTURE: */
/*             return ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_LOWER_CARRIAGE; */

/*         case ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_LOWER_CARRIAGE: */
/*             return ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_REMOVE_TINES; */

/*         case ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_REMOVE_TINES: */
/*             return ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_ADJUST_MAST; */

/*         case ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_ADJUST_MAST: */
/*             return ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_IDLE; */

/*         default : */
/*             return ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_IDLE; */


/*             // ------------------------------------------------------------- */
/*             // These are the truck actions - they are put here temporarily */
/*             //   They are all in piece by piece testing stage */
/*         case ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_TRUCK_APPROACH_FOR_DROPOFF: */
/*             return ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_TRUCK_RAISE_LOAD; */

/*         case ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_TRUCK_RAISE_LOAD: */
/*             return ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_TRUCK_PLACE_PALLET; */

/*         case ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_TRUCK_PLACE_PALLET: */
/*             return ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_TRUCK_LOWER_CARRIAGE; */

/*         case ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_TRUCK_LOWER_CARRIAGE: */
/*             return ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_REMOVE_TINES; */

/*         case ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_TRUCK_REMOVE_TINES: */
/*             return ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_IDLE; */


        default :
            return ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_IDLE;


        }
    }
    
    if ( current_job == ERLCM_MANIPULATION_PLANNER_JOB_ENUM_T_DROPOFF_PALLET) {
        switch ( current_action ) {


            // Ground actions
        case ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_DRIVE_TO_GESTURE:
            return ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_LOWER_CARRIAGE;

        case ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_LOWER_CARRIAGE:
            return ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_REMOVE_TINES;

        case ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_REMOVE_TINES:
            return ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_IDLE;

        case ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_ADJUST_MAST:
            return ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_IDLE;



            // Truck actions
        case ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_TRUCK_APPROACH_FOR_DROPOFF:
            return ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_TRUCK_RAISE_LOAD;

        case ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_TRUCK_RAISE_LOAD:
            return ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_TRUCK_PLACE_PALLET;

        case ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_TRUCK_PLACE_PALLET:
            return ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_TRUCK_LOWER_CARRIAGE;

        case ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_TRUCK_LOWER_CARRIAGE:
            return ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_TRUCK_REMOVE_TINES;

        case ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_TRUCK_REMOVE_TINES:
            return ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_IDLE;




        default:
            return ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_IDLE;
        }
    }

    return ERLCM_MANIPULATION_PLANNER_ACTION_ENUM_T_IDLE;
}
