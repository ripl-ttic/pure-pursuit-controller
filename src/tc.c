#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <bot_core/bot_core.h>

#include <lcmtypes/er_lcmtypes.h>

#include "tc.h"

#define APPROACH_ORIENTATION_TOL 10*M_PI/180

void
tc_get_params (TrajectoryController *self, BotConf *config)
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

TrajectoryController *
tc_new ()
{
    TrajectoryController * self = calloc (1, sizeof (TrajectoryController));

    self->lcm = globals_get_lcm();

    self->ctrans = globals_get_ctrans();
    self->mtrans = globals_get_mtrans();

    // parse configuration parameters
    BotConf *config = globals_get_config ();
    tc_get_params (self, config);
    globals_release_config (config);
    config = NULL;

    self->mast_command_last = NULL;

    self->current_manipulation_job = RIPL_MANIPULATION_PLANNER_JOB_ENUM_T_IDLE;
    self->current_manipulation_action = RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_IDLE;
    self->manipulation_standby = TRUE;

    return self;
}


void 
tc_destroy (TrajectoryController *self)
{

    if (!self)
        return;

    if (self->ctrans)
        globals_release_ctrans (self->ctrans);

    if (self->mtrans)
        globals_release_ctrans (self->ctrans);

    if (self->mast_command_last)
        ripl_actuation_command_mast_t_destroy (self->mast_command_last);

    if (self->lcm)
        globals_release_lcm (self->lcm);
  
     free (self);
}


ripl_manipulation_planner_action_enum_t tc_next_action ( ripl_manipulation_planner_job_enum_t current_job,
							  ripl_manipulation_planner_action_enum_t current_action)
{
    // A deterministic state machine for each job type
    if ( current_job == RIPL_MANIPULATION_PLANNER_JOB_ENUM_T_PICKUP_PALLET) {
        switch ( current_action ) {

        case RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_COLLECT_SCANS:
        case RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_COLLECT_SCANS_TILT:
        case RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_COLLECT_SCANS_DRIVE:
            return RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_IDLE;
//            return RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_DETECT_PALLET_VIA_TILT;
//            return RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_DETECT_PALLET_VIA_HEIGHT;

        case RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_DETECT_PALLET_VIA_TILT:
            return RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_APPROACH_PALLET;
//            return RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_IDLE;

        case RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_APPROACH_PALLET:
            return RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_REACQUIRE_PALLET_VIA_TILT;
//          return RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_IDLE;

        case RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_REACQUIRE_PALLET_VIA_TILT:
            return  RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_INSERT_TINES;

        case RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_INSERT_TINES:
            return RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_RAISE_CARRIAGE;

        case RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_RAISE_CARRIAGE:
            return RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_GROUND_BACKUP;

        case RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_GROUND_BACKUP:
            return RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_IDLE;


        case RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_DETECT_PALLET_VIA_HEIGHT:
            return RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_TRUCK_APPROACH_FOR_PICKUP;

        case  RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_TRUCK_APPROACH_FOR_PICKUP:
            return RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_TRUCK_REACQUIRE_PALLET_VIA_HEIGHT;

        case RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_TRUCK_REACQUIRE_PALLET_VIA_HEIGHT:
            //            return RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_IDLE;
            return RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_TRUCK_INSERT_TINES;

        case RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_TRUCK_INSERT_TINES:
            //            return RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_IDLE;
            return RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_TRUCK_RAISE_CARRIAGE;

        case RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_TRUCK_RAISE_CARRIAGE:
            //            return RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_IDLE;
            return RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_TRUCK_BACKUP;

        case  RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_TRUCK_BACKUP:
            return RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_IDLE;


/*             // ------------------------------------------------------------- */
/*             // These are the drop off actions - they are put here temporarily */
/*         case RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_DRIVE_TO_GESTURE: */
/*             return RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_LOWER_CARRIAGE; */

/*         case RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_LOWER_CARRIAGE: */
/*             return RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_REMOVE_TINES; */

/*         case RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_REMOVE_TINES: */
/*             return RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_ADJUST_MAST; */

/*         case RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_ADJUST_MAST: */
/*             return RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_IDLE; */

/*         default : */
/*             return RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_IDLE; */


/*             // ------------------------------------------------------------- */
/*             // These are the truck actions - they are put here temporarily */
/*             //   They are all in piece by piece testing stage */
/*         case RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_TRUCK_APPROACH_FOR_DROPOFF: */
/*             return RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_TRUCK_RAISE_LOAD; */

/*         case RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_TRUCK_RAISE_LOAD: */
/*             return RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_TRUCK_PLACE_PALLET; */

/*         case RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_TRUCK_PLACE_PALLET: */
/*             return RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_TRUCK_LOWER_CARRIAGE; */

/*         case RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_TRUCK_LOWER_CARRIAGE: */
/*             return RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_REMOVE_TINES; */

/*         case RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_TRUCK_REMOVE_TINES: */
/*             return RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_IDLE; */


        default :
            return RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_IDLE;


        }
    }
    
    if ( current_job == RIPL_MANIPULATION_PLANNER_JOB_ENUM_T_DROPOFF_PALLET) {
        switch ( current_action ) {


            // Ground actions
        case RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_DRIVE_TO_GESTURE:
            return RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_LOWER_CARRIAGE;

        case RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_LOWER_CARRIAGE:
            return RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_REMOVE_TINES;

        case RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_REMOVE_TINES:
            return RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_IDLE;

        case RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_ADJUST_MAST:
            return RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_IDLE;



            // Truck actions
        case RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_TRUCK_APPROACH_FOR_DROPOFF:
            return RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_TRUCK_RAISE_LOAD;

        case RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_TRUCK_RAISE_LOAD:
            return RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_TRUCK_PLACE_PALLET;

        case RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_TRUCK_PLACE_PALLET:
            return RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_TRUCK_LOWER_CARRIAGE;

        case RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_TRUCK_LOWER_CARRIAGE:
            return RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_TRUCK_REMOVE_TINES;

        case RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_TRUCK_REMOVE_TINES:
            return RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_IDLE;




        default:
            return RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_IDLE;
        }
    }

    return RIPL_MANIPULATION_PLANNER_ACTION_ENUM_T_IDLE;
}
