#include <unistd.h>
#include <glib.h>
#include <getopt.h>
#include <string.h>

#include <bot_core/bot_core.h>
#include <lcmtypes/hr_lcmtypes.h>
#include <lcmtypes/rrtstar.h>
#include <lcmtypes/obs_lcmtypes.h>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#include "mp_control_utils.h"
#include "mp_prediction.h"
#include <bot_param/param_client.h>


#define ENABLE_TELEPORT 0

//we need to use the small stopping distance only when the robot is at the last goal waypoint
// MOVED TO CONFIG
//#define STOPPING_DIST 0.5//0.5//0.5
//#define STOPPING_DIST_SMALL 0.2 //Sachi - reduce this to get the robot close - esp to elevators

// Determined from robot width & length in config
//#define VEHICLE_TURNING_FOOTPRINT 0.6 //0.3 //0.6 //actually 0.6

#define COLLISION_CHECK_PATH_DISTANCE 2.0 // Distance ahead of robot along path to check for collisions
//#define LINE_COLLISION_CHECK_DELTA 0.3 // Spacing for collision checking to next reference point

#define DISTANCE_CURVATURE_CHECK 1.0 // How far forward/backward relative to current reference
                                     // to check to estimate curvature. Used to reduce lookahead
#define THRESHOLD_CURVATURE_CHECK 80 // When the change in orientation exceeds this threshold, reduce lookahead

/*****************************************/
#define RENDER_REF_POINTS 1
#define RENDER_PREDICTED_TRAJ 0
#define RENDER_OPTIMAL_PATH 0
#define FIRST_TO_RENDER 0
#define DOUBLE_REF_POINTS 0
/*****************************************/


typedef struct _state_t state_t;
struct _state_t {
    lcm_t *lcm;
    bot_lcmgl_t *lcmgl;
    bot_lcmgl_t *lcmgl_text;
    bot_lcmgl_t *lcmgl_collision;

    GMainLoop *mainloop;
    guint status_update_timer_id;
    guint trajectory_controller_timer_id;

    GMutex *mutex;

    int verbose;

  //int subservient_to_joystick;
    int perform_collision_check;

    double collision_check_path_distance;

    ripl_guide_info_t *guide_pos;

    bot_core_pose_t *bot_pose_last;
    ripl_robot_status_t *robot_status_last;

    int stop_called;
    int trash_wp;

    mp_control_utils_t *mp_cont;

    /****************************************/
    int new_ref_point;
    ripl_ref_point_list_t *ref_point_list;
    int current_ref_point;
    int num_ref_points;
    int turn_in_place;
    /****************************************/
    int envoy;

    // Default translational and maximum rotational velocities defined in param config
    double default_tv;
    double thresh_tv;
    double max_rv;
    double max_rv_in_place;

    double current_tv;

    double translational_vel_last;
    double rotational_vel_last;

    double stopping_distance;
    double stopping_distance_small;

    double vehicle_turning_footprint;
    double vehicle_width;

    double line_collision_check_delta;

    int error_mode;

    // State constants are defined in ripl_trajectory_controller_status_t
    int32_t tc_state;

    int8_t goal_state;

    int goal_type;

    // Error modes constants are defined in ripl_trajectory_controller_status_t
    int32_t tc_error_mode;

    mp_prediction_t *mp_prediction;

    obs_obstacle_list_t *obstacles_last;
    obs_rect_list_t *sim_rect_last;

  //ripl_joystick_state_t *joystick_state_msg;

    gboolean path_obstructed;
};

int
mp_reset_variables (state_t *self) {
    self->tc_state = RIPL_TRAJECTORY_CONTROLLER_STATUS_T_STATE_IDLE;
    self->tc_error_mode = RIPL_TRAJECTORY_CONTROLLER_STATUS_T_ERROR_NONE;

}

int
mp_can_cancel_state (state_t *self) {
    return 0;
}


/* static void */
/* on_joystick_state (const lcm_recv_buf_t *rbuf, const char *channel, */
/*                    const ripl_joystick_state_t *msg, void *user) { */

/*     state_t *self = (state_t *) user; */

/*     if (self->joystick_state_msg) */
/*         ripl_joystick_state_t_destroy (self->joystick_state_msg); */

/*     self->joystick_state_msg = ripl_joystick_state_t_copy(msg); */
/* } */


static void
on_obstacles (const lcm_recv_buf_t *rbuf, const char *channel,
                  const obs_obstacle_list_t *msg, void *user) {

    state_t *self = (state_t *) user;

    g_mutex_lock (self->mutex);

    if (self->obstacles_last)
        obs_obstacle_list_t_destroy (self->obstacles_last);
    self->obstacles_last = obs_obstacle_list_t_copy (msg);

    g_mutex_unlock (self->mutex);

}


static void
on_sim_rects(const lcm_recv_buf_t * rbuf, const char *channel,
             const obs_rect_list_t *msg, void *user)
{
    state_t *self = (state_t *) user;

    g_mutex_lock (self->mutex);
    if (self->sim_rect_last){
        obs_rect_list_t_destroy(self->sim_rect_last);
    }
    self->sim_rect_last = obs_rect_list_t_copy(msg);

    g_mutex_unlock (self->mutex);
}

/**********************************************************************************************/

static void
on_ref_point_list (const lcm_recv_buf_t *rbuf, const char *channel,
                   const ripl_ref_point_list_t *msg, void *user)
{
    state_t *self = user;

    if (self->verbose)
        fprintf(stderr,"New Ref Point List\n");

    g_mutex_lock (self->mutex);

    if (!strcmp(channel, "ELEVATOR_GOAL_REF_LIST")) {
        fprintf(stderr, "Elevator Goal received \n");
        self->goal_type = 1;
    }
    else if(!strcmp(channel, "GOAL_REF_LIST")) {
        fprintf(stderr, "Normal Goal received \n");
        self->goal_type = 0;
    }

    //we need to create an amalgamted waypoint list - with the remaining points

    if(!self->ref_point_list || self->trash_wp ||
       self->ref_point_list->id != msg->id ||
       self->ref_point_list->num_ref_points == 0 ||
       msg->num_ref_points == 0 ||
       self->current_ref_point > self->ref_point_list->commited_point_id ||
       self->ref_point_list->mode == RIPL_REF_POINT_LIST_T_TURN_IN_PLACE ||
       msg->mode == RIPL_REF_POINT_LIST_T_TURN_IN_PLACE ||
       self->ref_point_list->commited_point_id == -1){
        //(msg->num_ref_points > 1 && msg->commited_point_id == msg->num_ref_points -1)){//if there was no commited point in the last list - do not merge

        fprintf(stderr," ++++ New goal waypoints received - resetting the traj list\n");
        if (0 && self->ref_point_list)
            fprintf (stdout, "trash_wp = %d,\n"
                     "ref_point_list->id = %d, msg->id = %d\n"
                     "ref_point_list->num_ref_points = %d\n"
                     "msg->num_ref_points = %d\n"
                     "current_ref_point = %d, committed_point_id = %d\n"
                     "committed_point_id = %d\n\n\n",
                     self->trash_wp, self->ref_point_list->id, msg->id,
                     self->ref_point_list->num_ref_points,
                     msg->num_ref_points,
                     self->current_ref_point, self->ref_point_list->commited_point_id,
                     self->ref_point_list->commited_point_id);



        if(self->ref_point_list && self->ref_point_list->commited_point_id == -1){
            fprintf(stderr, "+++++++++++++++ Committed Point error : +++\n");
        }

        fprintf(stderr, "New List size : %d Committed ID : %d\n",
                msg->num_ref_points, msg->commited_point_id);

        if (self->ref_point_list)
            ripl_ref_point_list_t_destroy (self->ref_point_list);
        self->ref_point_list = ripl_ref_point_list_t_copy(msg);

        if(self->ref_point_list->mode == RIPL_REF_POINT_LIST_T_TURN_IN_PLACE){
            self->turn_in_place = 1;
        }
        else if(self->ref_point_list->mode == RIPL_REF_POINT_LIST_T_NORMAL_MOTION){
            self->turn_in_place = 0;
        }

        self->goal_state = RIPL_RRT_GOAL_STATUS_T_ACTIVE;

        self->num_ref_points = self->ref_point_list->num_ref_points - 1;

        // Determine the current reference point based on lookahead distance
        self->current_ref_point = 0;
        fprintf (stdout, "Before: current_ref_point = %d, num_ref_points = %d\n", self->current_ref_point, self->num_ref_points);
        double distance_to_target = 0;
        gboolean target_in_front;
        // Update the state of the bot in control utils
        double rpy_bot[3];
        bot_quat_to_roll_pitch_yaw (self->bot_pose_last->orientation, rpy_bot);
        mp_control_update_bot_states2 (self->mp_cont,
                                       self->bot_pose_last->pos[0],
                                       self->bot_pose_last->pos[1],
                                       rpy_bot[2],
                                       sqrt(self->bot_pose_last->vel[0]*self->bot_pose_last->vel[0]
                                            +self->bot_pose_last->vel[1]*self->bot_pose_last->vel[1]) );

        // // Create an array of distances which contains a rolling aggregate of distances between point 0 and point i
        // // When calculating distance from current loc to carrot, just subtract total distance by the distance stored in index
        // double dist_btw_ref_points[self->num_ref_points];
        // for(int i = 0 ; i < self->num_ref_points; i ++){
        //     // [0] = dist between ref_points[0] and ref_points[1]
        //     // [1] = dist between ref_points[0] and ref_points[2]
        //     // TODO : Make this into a distance function
        //     double dx = (self->ref_point_list->ref_points[i + 1].x - self->ref_point_list->ref_points[i].x);
        //     double dy = (self->ref_point_list->ref_points[(i + 1)].y - self->ref_point_list->ref_points[i].y);
        //
        //     // Corner case since num distances is 1 less than total points
        //     if(i == 0){
        //         dist_btw_ref_points[i] =  sqrt(dx*dx + dy*dy);
        //     }else{
        //         dist_btw_ref_points[i] =  sqrt(dx*dx + dy*dy) + dist_btw_ref_points[i-1];
        //     }
        // }
        // distance_to_target = dist_btw_ref_points[self->num_ref_points - 1];
        // -Kevin
        while (distance_to_target < self->stopping_distance && self->current_ref_point < self->num_ref_points) {
            mp_control_update_target_states (self->mp_cont, self->ref_point_list->ref_points[self->current_ref_point].x,
                                             self->ref_point_list->ref_points[self->current_ref_point].y,
                                             self->ref_point_list->ref_points[self->current_ref_point].t);


            mp_control_compute_distance_to_target (self->mp_cont, &distance_to_target, &target_in_front);





            // compute the distance of the bot to the ref points
            //double dx = (self->bot_pose_last->pos[0] - self->ref_point_list->ref_points[self->current_ref_point].x);
            //double dy = (self->bot_pose_last->pos[1] - self->ref_point_list->ref_points[self->current_ref_point].y);

            //distance_to_target = sqrt(dx*dx + dy*dy);

            // Update the distance between each point by setting points we have already traveled to 0
            // if(self->current_ref_point > 0){
            //     distance_to_target -= dist_btw_ref_points[self->current_ref_point];
            // }
            // Kevin

            //fprintf (stdout, "\tdistance_to_target[%d] = %.2f\n", self->current_ref_point, distance_to_target);
            self->current_ref_point++;
        }

        fprintf (stdout, "After: current_ref_point = %d, num_ref_points = %d\n", self->current_ref_point, self->num_ref_points);


        self->stop_called = 0;
    }
    else{
        fprintf(stderr," ++++ Updated Waypoint list received\n");

        fprintf(stderr, "Current List size : %d Committed ID : %d New List size : %d New Committed ID : %d\n", self->ref_point_list->num_ref_points, self->ref_point_list->commited_point_id,
                msg->num_ref_points, msg->commited_point_id);

        ripl_ref_point_list_t *new_list = (ripl_ref_point_list_t *) calloc(1, sizeof(ripl_ref_point_list_t));

        new_list->id = self->ref_point_list->id;
        new_list->num_ref_points = self->ref_point_list->commited_point_id + 1 + msg->num_ref_points - self->current_ref_point;

        if(msg->commited_point_id != -1){
            new_list->commited_point_id = self->ref_point_list->commited_point_id + msg->commited_point_id + 1 - self->current_ref_point;
        }
        else{
            //new_list->commited_point_id = -1;
            new_list->commited_point_id = new_list->num_ref_points -1;
        }

        ripl_ref_point_t *list = calloc(new_list->num_ref_points, sizeof(ripl_ref_point_t));
        memcpy(list, &self->ref_point_list->ref_points[self->current_ref_point],
               (self->ref_point_list->commited_point_id + 1 - self->current_ref_point) * sizeof(ripl_ref_point_t));

        //reset the velocity values on these - otherwise robot will slow down at the old points
        fprintf(stderr, "Up to commit point : %d\n" , self->ref_point_list->commited_point_id + 1 - self->current_ref_point);

        for(int i=0; i < (self->ref_point_list->commited_point_id + 1 - self->current_ref_point) ; i++){

            //*********Sachi - fix this
            list[i].s = self->default_tv;
        }

        memcpy(&list[self->ref_point_list->commited_point_id + 1 - self->current_ref_point], msg->ref_points, (msg->num_ref_points) * sizeof(ripl_ref_point_t));

        new_list->ref_points = list;

        new_list->mode = msg->mode;

        fprintf(stderr, " +++ Ref point lists merged => Current Ref Point : %d\n",
                self->current_ref_point);

        if (self->ref_point_list)
            ripl_ref_point_list_t_destroy (self->ref_point_list);

        self->ref_point_list = new_list;

        if(self->ref_point_list->mode == RIPL_REF_POINT_LIST_T_TURN_IN_PLACE){
            self->turn_in_place = 1;
        }
        else if(self->ref_point_list->mode == RIPL_REF_POINT_LIST_T_NORMAL_MOTION){
            self->turn_in_place = 0;
        }

        self->goal_state = RIPL_RRT_GOAL_STATUS_T_ACTIVE;

        self->num_ref_points = self->ref_point_list->num_ref_points - 1;
        self->current_ref_point = 0; //reset the current point

        self->stop_called = 0;
    }

    g_mutex_unlock (self->mutex);

    return;
}

static void
on_ref_point_list_old (const lcm_recv_buf_t *rbuf, const char *channel,
                   const ripl_ref_point_list_t *msg, void *user)
{
    state_t *self = user;

    if (self->verbose)
        fprintf(stderr,"New Ref Point List\n");

    g_mutex_lock (self->mutex);

    //we need to create an amalgamted waypoint list - with the remaining points

    if (self->ref_point_list)
        ripl_ref_point_list_t_destroy (self->ref_point_list);
    self->ref_point_list = ripl_ref_point_list_t_copy(msg);

    if(self->ref_point_list->mode == RIPL_REF_POINT_LIST_T_TURN_IN_PLACE){
        self->turn_in_place = 1;
    }
    else if(self->ref_point_list->mode == RIPL_REF_POINT_LIST_T_NORMAL_MOTION){
        self->turn_in_place = 0;
    }


    self->goal_state = RIPL_RRT_GOAL_STATUS_T_ACTIVE;

    self->num_ref_points = self->ref_point_list->num_ref_points - 1;
    self->current_ref_point = 0;

    self->stop_called = 0;

    g_mutex_unlock (self->mutex);

    return;
}


static void
on_bot_pose (const lcm_recv_buf_t *rbuf, const char *channel,
             const bot_core_pose_t *msg, void *user)
{
    state_t *self = (state_t*) user;

    g_mutex_lock (self->mutex);

    if (self->bot_pose_last)
        bot_core_pose_t_destroy (self->bot_pose_last);

    self->bot_pose_last = bot_core_pose_t_copy (msg);

    g_mutex_unlock (self->mutex);

    return;
}

static void
on_guide_pose (const lcm_recv_buf_t *rbuf, const char *channel,
               const ripl_guide_info_t *msg, void *user)
{
    state_t *self = (state_t*) user;

    g_mutex_lock (self->mutex);

    if (self->guide_pos)
        ripl_guide_info_t_destroy (self->guide_pos);

    self->guide_pos = ripl_guide_info_t_copy (msg);

    g_mutex_unlock (self->mutex);

    return;
}

static void
on_robot_status (const lcm_recv_buf_t *rbuf, const char *channel,
                 const ripl_robot_status_t *msg, void *user)
{
    state_t *self = (state_t *) user;

    g_mutex_lock (self->mutex);

    if (self->robot_status_last)
        ripl_robot_status_t_destroy (self->robot_status_last);

    self->robot_status_last = ripl_robot_status_t_copy (msg);

    g_mutex_unlock (self->mutex);

    return;
}

static void
on_motion_command (const lcm_recv_buf_t *rbuf, const char *channel,
                 const ripl_speech_cmd_t *msg, void *user)
{
    state_t *self = (state_t *) user;

    g_mutex_lock (self->mutex);

    self->goal_state = RIPL_RRT_GOAL_STATUS_T_IDLE;

    if(!strcmp(msg->cmd_type,"FOLLOWER") && !strcmp(msg->cmd_property, "STOP")){
        if (self->ref_point_list)
            ripl_ref_point_list_t_destroy (self->ref_point_list);
        self->ref_point_list = NULL;
        self->current_ref_point = 0;
        self->num_ref_points = -1;
        fprintf(stderr,"Commanded to stop - stoping robot\n");
        self->stop_called = 1;

        self->goal_state = RIPL_RRT_GOAL_STATUS_T_FAILED;
        //we should declare failed/stopped
        // - maybe we should not flus this"
    }

    g_mutex_unlock (self->mutex);

    return;
}

static gboolean
on_status_timer (gpointer data)
{
    state_t * self = (state_t *) data;

    ripl_trajectory_controller_status_t msg = {
        .utime = bot_timestamp_now(),
        .num_cur_ref_points = self->num_ref_points,
        .trans_vel = self->translational_vel_last,
        .rot_vel = self->rotational_vel_last,
        .state = self->tc_state,
        .error_mode = self->tc_error_mode
    };

    ripl_trajectory_controller_status_t_publish (self->lcm, "TRAJECTORY_CONTROLLER_STATUS", &msg);

    int32_t id = -1;

    if(self->ref_point_list){
        id = self->ref_point_list->id;
    }

    ripl_rrt_goal_status_t s_msg = {
        .utime = bot_timestamp_now(),
        .id = id,
        .status = self->goal_state
    };

    ripl_rrt_goal_status_t_publish(self->lcm,
                                    "TRAJECTORY_CONTROLLER_GOAL_STATUS",
                                    &s_msg);

    return TRUE;
}



// Publish velocity message
// May need to be updated depending on how we send velocity
// commands to Husky via ROS2LCM bridge and manage robot status
void
robot_velocity_command(double tv, double rv, lcm_t* lcm)
{

  ripl_velocity_msg_t v;

  v.tv = tv;
  v.rv = rv;
  v.utime = bot_timestamp_now();
  ripl_velocity_msg_t_publish(lcm,"ROBOT_VELOCITY_CMD",&v);
}


//returns 1 if free to rotate return 0 if not
int free_for_rotation(state_t *self){
    if(!self->obstacles_last && !self->sim_rect_last ||  !self->bot_pose_last){
        return 1;
    }
    else{
        if(self->obstacles_last){
            for(int i=0; i < self->obstacles_last->rects.num_rects; i++){

                obs_rect_t *rect_curr =  &(self->obstacles_last->rects.rects[i]);
                double absolute_distance = hypot(self->obstacles_last->rects.xy[0] + rect_curr->dxy[0] -  self->bot_pose_last->pos[0],
                                                 self->obstacles_last->rects.xy[1] + rect_curr->dxy[1] -  self->bot_pose_last->pos[1]);

                double max_rect_size
                    = (rect_curr->size[0] > rect_curr->size[1]) ? rect_curr->size[0] : rect_curr->size[1];

                if(absolute_distance < max_rect_size + self->vehicle_turning_footprint){
                    return 0;
                }
            }
        }

        if(self->sim_rect_last){
            for(int i=0; i < self->sim_rect_last->num_rects; i++){
                obs_rect_t *rect_curr =  &(self->sim_rect_last->rects[i]);
                double absolute_distance = hypot(self->sim_rect_last->xy[0] + rect_curr->dxy[0] -  self->bot_pose_last->pos[0],
                                                 self->sim_rect_last->xy[1] + rect_curr->dxy[1] -  self->bot_pose_last->pos[1]);

                double max_rect_size
                = (rect_curr->size[0] > rect_curr->size[1]) ? rect_curr->size[0] : rect_curr->size[1];

                if(absolute_distance < max_rect_size + self->vehicle_turning_footprint){
                    return 0;
                }
            }
        }

        return 1;
    }
}


// Returns 1 if the straight-line path between the start and goal
// is free from any detected obstacles
int
free_for_drive_to_target (state_t *self, double start_x, double start_y,
                          double goal_x, double goal_y, double goal_t)
{

    if(!self->obstacles_last && !self->sim_rect_last ||  !self->bot_pose_last){
        fprintf (stderr, "No obstacle list or no robot pose\n");
        return 1;
    }
    else{
        double theta = atan2 (goal_y - start_y, goal_x - start_x);
        double distance = hypot (goal_y - start_y, goal_x - start_x);
        double d = 0;
        while (d < distance) {

            double line_x = start_x + d * cos(theta);
            double line_y = start_y + d * sin(theta);

            // Check actual obstacles
            if(self->obstacles_last){
                for(int i=0; i < self->obstacles_last->rects.num_rects; i++){

                    obs_rect_t *rect_curr =  &(self->obstacles_last->rects.rects[i]);
                    double absolute_distance = hypot(self->obstacles_last->rects.xy[0] + rect_curr->dxy[0] -  line_x,
                                                     self->obstacles_last->rects.xy[1] + rect_curr->dxy[1] -  line_y);

                    double max_rect_size
                        = (rect_curr->size[0] > rect_curr->size[1]) ? rect_curr->size[0] : rect_curr->size[1];

                    if(absolute_distance < max_rect_size + self->vehicle_width/2){
                        /* fprintf (stdout, "absolute_distance = %0.2f, max_rect_size = %.2f, width/2 = %.2f\n", */
                        /*          absolute_distance, max_rect_size, self->vehicle_width/2); */

                        if (self->verbose)
                            fprintf (stdout, "Obstacle detected along straight line segment between robot and next reference point\n");
                        return 0;
                    }
                }
            }

            // Check simulated obstacles
            if(self->sim_rect_last){
                for(int i=0; i < self->sim_rect_last->num_rects; i++){
                    obs_rect_t *rect_curr =  &(self->sim_rect_last->rects[i]);
                    double absolute_distance = hypot(self->sim_rect_last->xy[0] + rect_curr->dxy[0] -  line_x,
                                                     self->sim_rect_last->xy[1] + rect_curr->dxy[1] -  line_y);

                    double max_rect_size
                        = (rect_curr->size[0] > rect_curr->size[1]) ? rect_curr->size[0] : rect_curr->size[1];

                    if(absolute_distance < max_rect_size + self->vehicle_width/2){
                        if (self->verbose)
                            fprintf (stdout, "Simulated obstacle detected along straight line segment between robot and next reference point\n");
                        return 0;
                    }
                }
            }

            d += self->line_collision_check_delta;
        }

        return 1;
    }
}


int
manipulation_controller_turn_towards_target_local (state_t *self,
                                             double target_x,        // Target states
                                             double target_y,
                                             double target_t) {
    //for nwo something basic

    //these are relative to the robot

    double heading_to_person = atan2(target_y, target_x);
    double distance_to_person = hypot(target_x, target_y);
    double heading_max = bot_to_radians(120);
    double threshold = bot_to_radians(40);

    double rv = 0;

    //we should check if safe to turn
    if(distance_to_person < 5.0 && distance_to_person > 1.0){

        if(fabs(heading_to_person) < bot_to_radians(10)){
            fprintf(stderr, "Person within heading - Doing nothing\n");
        }
        else{
            //not sure if we should lock the mutex???
	    int is_free = 1;
	    if (self->perform_collision_check)
	        is_free = free_for_rotation(self);

            if(is_free){

                rv = bot_clamp(heading_to_person, -threshold, threshold) / heading_max * M_PI;
                if (self->verbose)
                    fprintf(stderr, "Turning Towards person %f \n", rv);
            }
            else{
                rv = 0;
                fprintf(stderr, "can't Turn - obtacle \n");
            }
        }
    }
    else{
        fprintf(stderr, "Person Too far/too close - not turning\n");
    }

    robot_velocity_command(0, rv, self->lcm);

}


int
manipulation_controller_turn_towards_target (state_t *self,
                                             double target_x,        // Target states
                                             double target_y,
                                             double target_t) {
    //for nwo something basic

    //these are relative to the robot

    if(self->bot_pose_last == NULL){
        fprintf(stderr, "No bot pose - returning\n");
        return -1;
    }

    double bot_rpy[3];
    bot_quat_to_roll_pitch_yaw (self->bot_pose_last->orientation, bot_rpy);

    double heading_difference = bot_mod2pi(target_t - bot_rpy[2]);

    if(heading_difference > bot_to_radians(180)){
        if (self->verbose)
            fprintf(stderr,"Turning Right\n");
        heading_difference = -(2* M_PI - heading_difference);
    }
    else if(heading_difference < bot_to_radians(-180)){
        if (self->verbose)
            fprintf(stderr,"Turning Right\n");
        heading_difference = (2* M_PI + heading_difference);
    }
    else if (self->verbose)
        fprintf(stderr,"Turning Left\n");

    if(fabs(heading_difference) < bot_to_radians(5)){ // mwalter: changed from 10 to 5
        fprintf(stderr, "Within Threshold - Stopping Rotation\n");
        robot_velocity_command(0, 0, self->lcm);
        return 1;
    }

    double heading_max = bot_to_radians(120);
    double threshold = bot_to_radians(40);
    //double max_rv = 0.75;//0.5;

    // In order to avoid jerky motion when turning on carpet, use different
    // maximum rotational velocities when moving forward at a sufficiently
    // high velocity. Otherwise, use a lower max_rv
    double max_rv = self->max_rv;
    if (fabs(self->bot_pose_last->vel[0]) < self->thresh_tv)
        max_rv = self->max_rv_in_place;


    double rv = 0;

    int is_free = 1;
    if (self->perform_collision_check)
        is_free = free_for_rotation(self);

    if(is_free){
        rv = bot_clamp(heading_difference/ threshold, -1.0, 1.0) * max_rv;// / heading_max * M_PI;
        //rv = bot_clamp(heading_difference, -threshold, threshold) * self->max_rv;// / heading_max * M_PI;
        if (self->verbose)
            fprintf(stderr, "Turning Towards Goal %f \n", rv);
    }
    else{
        rv = 0;
        robot_velocity_command(0, 0, self->lcm);
        if (self->verbose)
            fprintf(stderr, "can't Turn - obtacle \n");
        return -2;
    }

    robot_velocity_command(0, rv, self->lcm);
    return 0;
}


int
manipulation_controller_drive_to_target (state_t *self,
                                         double target_x,        // Target states
                                         double target_y,
                                         double target_t,
                                         double stopping_distance, // Stopping distance is measured
                                                                   // from the target to the bot
                                         double v_cmd) {

    static int64_t last_time = 0;


    // Update the state of the bot in control utils
    double rpy_bot[3];
    bot_quat_to_roll_pitch_yaw (self->bot_pose_last->orientation, rpy_bot);

    mp_control_update_bot_states2 (self->mp_cont,
                                   self->bot_pose_last->pos[0],
                                   self->bot_pose_last->pos[1],
                                   rpy_bot[2],
                                   sqrt(self->bot_pose_last->vel[0]*self->bot_pose_last->vel[0]
                                        +self->bot_pose_last->vel[1]*self->bot_pose_last->vel[1]) );


    // update the state of the target in control utils
    if (self->verbose) {
        fprintf (stdout, "target pose: %5.5lf - %5.5lf - %5.5lf - \n",
                 target_x, target_y, target_t);
    }

    mp_control_update_target_states (self->mp_cont,
                                     target_x,
                                     target_y,
                                     target_t);


    double distance_to_target;
    gboolean target_in_front;


    if (mp_control_compute_distance_to_target (self->mp_cont,
                                               &distance_to_target,
                                               &target_in_front) < 0 ) {
        // NOTE: this error should never occur - check for bugs in the code if it does
        printf("Error computing distance to the target");
        return -6;
    }

    if(self->verbose)
        fprintf(stderr,"Dist to target : %f Target in front : %d\n", distance_to_target, target_in_front);

    // Check whether the bot got to the goal
    if ( (distance_to_target <= stopping_distance)){
        // Terminate the control indicating with a return value 1
        if (self->verbose)
            fprintf (stdout, " --- Reached current reference point --- \n");
        return 1;
    }


    // Compute longitudinal control action
    double control_action;
    if ( mp_control_compute_lon_control_via_target_pose (self->mp_cont,
                                                         &control_action,
                                                         v_cmd) < 0) {
        printf ("Error computing the longitudinal control\n");
        return -4; // Longitudinal control can not be computed
    }

    // Compute steering control action
    double steer_cmd;
    int ret_compute_control_steer =
        mp_control_compute_control_steer_via_target_pose (self->mp_cont, &steer_cmd);

    //we need to clamp this steering command - and also reduce the TV in cases of clamping
    //double max_rv = 1.0;//0.5;//1.5;

    //we only care about the person when not doing elevator/portal related stuff
    if(self->envoy && (self->goal_type == 0)){//basic setting
        if(self->guide_pos && self->guide_pos->tracking_state){
            //have guide msg and have tracking
            if(self->guide_pos->pos[0] < - 2.5){
                //ramp speed down - dont try to match it

                if((bot_timestamp_now() - last_time)/ 1.0e6  > 0.2){
                    self->current_tv -= 0.05;
                    self->current_tv = bot_max(0, self->current_tv);
                    last_time = bot_timestamp_now ();
                }
            }

            if(self->guide_pos->pos[0] > - 2.0){ //bad idea - this will reduce the velocity too quickly
                //ramp speed down - dont try to match it
                if((bot_timestamp_now() - last_time)/ 1.0e6  > 0.2){
                    self->current_tv += 0.05;
                    //***** This is the change point
                    self->current_tv = bot_min(v_cmd, self->current_tv);
                    //self->current_tv = bot_min(0.5, self->current_tv);
                    last_time = bot_timestamp_now ();
                }
            }
        }
        else{
            fprintf(stderr, " In Envoy mode and can't spot the person - not moving\n");
            self->current_tv = 0;
        }
        //***** This is the change point
        self->current_tv = bot_min(v_cmd, self->current_tv);
        //Done change point
        v_cmd = self->current_tv;
    }



    // Check to see that the path to the goal is collision-free by approximating it
    // as a straight-line path
    //if (!free_for_drive_to_target (self, target_x, target_y, target_t)) {
    if (0) {
        fprintf (stdout, "Obstacle detected on path to next reference point. Stopping robot!\n");

        double tv = 0;
        double rv = 0;

        robot_velocity_command(tv, rv, self->lcm);
        mp_control_update_aux_actuation (self->mp_cont, tv, rv);
        mp_control_publish_aux_message (self->lcm, self->mp_cont);

        return -5;
    }




    //assuming that this is the curvature - prob wrong :)
    //  double
    // Model for steered car which seems to assume 1m wheelbase

    //v_cmd is 0 for the last point - this causes the rv to be zero as well

    // In order to avoid jerky motion when turning on carpet, use different
    // maximum rotational velocities when moving forward at a sufficiently
    // high velocity. Otherwise, use a lower max_rv
    double max_rv = self->max_rv;
    if (fabs(self->bot_pose_last->vel[0]) < self->thresh_tv)
        max_rv = self->max_rv_in_place;

    double rv = bot_clamp(v_cmd * tan(steer_cmd), -max_rv, max_rv);

    //note - if rv gets clamped - this causes us to overshoot the arc

    if(self->verbose){
        if(fabs(rv - v_cmd * tan(steer_cmd)) > 0.01){
            fprintf(stderr,"Warn : RV Clamped\n");
            //if RV clampped - adjust the tv
        }


        fprintf(stderr, " ======== Current TV : %f V_CMD : %f\n",
                self->current_tv, v_cmd);
    }
    //calculate tv from the curvature
    double tv_clamped = 0;

    //max tv is wrong - this just cause the robot to jump faster

    if(!target_in_front){
        if (self->verbose)
            fprintf(stderr, "Target is behind us - setting tv to 0\n");
        v_cmd = 0;
    }


    if(rv == 0 || steer_cmd == 0){
        tv_clamped = v_cmd;//self->current_tv;
    }
    else{
        tv_clamped = rv / tan(steer_cmd);
    }
    double tv = bot_clamp(tv_clamped, 0, v_cmd);//self->current_tv);

    if(self->verbose){
        fprintf(stderr, "Tan Steering : %.2f RV Orig : %.2f, RV Act : %.2f, tv : %.2f, tv clamped : %.2f\n",
            tan(steer_cmd), v_cmd * tan(steer_cmd), rv,
            tv_clamped, tv);
    }

    if(self->verbose){
        if(fabs(tv - v_cmd) > 0.01){
            fprintf(stderr,"Warn : TV Clamped\n");
        }

        fprintf(stderr,"Control Action : %f\n", control_action);
        fprintf(stderr, "TV : %f RV : %f\n", tv, rv);
        fprintf(stderr,"Steering Action : %f\n", steer_cmd);
    }

    if((tv < 0.05 && rv < 0.05) || distance_to_target <= stopping_distance ) {
        if ((distance_to_target <= stopping_distance + 0.1)){
            if (self->verbose)
                fprintf (stdout, " --- Control Terminated --- \n");

            if (self->verbose && distance_to_target <= stopping_distance)
                fprintf(stderr, "Almost at the goal \n");
            return 1;
        }
        else if (self->verbose)
            fprintf(stderr, "Very small control being applied : Dist to target : %f\n", distance_to_target);
    }

    robot_velocity_command(tv, rv, self->lcm);
    mp_control_update_aux_actuation (self->mp_cont, tv, rv);
    mp_control_publish_aux_message (self->lcm, self->mp_cont);

    //lets just convert them to the tv and rv and publish to the robot - for now

    if (ret_compute_control_steer < 0) {
        printf ("Error computing the steering angle\n");

        switch (ret_compute_control_steer) {
        case -1 :
            printf ("No bot states initialized in mp_control_utils \n");
            return -1;

        case  -2:
            printf("No target states initialized in mp_control_utils \n");
            return -2;

        case -3:
            printf("target-bot orienatation can not be determined\n");
            printf("THIS SHOULD NEVER HAPPEN - CHECK FOR BUGS\n");
            return -3;

        case -4:
            printf ("Overshot the target\n");
            goto continue_control;

        }
    }

 continue_control:
    return 0;

}


static gboolean
on_trajectory_controller_timer (gpointer data)
{
    state_t * self = (state_t *) data;

    self->path_obstructed = FALSE;

    if (self->robot_status_last) {
        if (!(self->robot_status_last->state == RIPL_ROBOT_STATUS_T_STATE_RUN)) {
            if (self->verbose)
                fprintf (stdout, "Robot must be in RUN state\n");

            return TRUE;
        }
    }
    else {
        if (self->verbose)
            fprintf (stdout, "No robot status message\n");
        return TRUE;
    }



    if(self->num_ref_points == -1){
        //now we should check if we have a valid person and turn towards him/her
        if(self->guide_pos && !self->stop_called){
            //check if we have a valid guide pos
            if(self->guide_pos->tracking_state ==1){
                if(self->verbose)
                    fprintf(stderr, "Turning towards the person - not implemented\n");

                //turn towards the person
                //manipulation_controller_turn_towards_target(self, self->guide_pos->pos[0],
                //                                            self->guide_pos->pos[1], self->guide_pos->person_heading);
            }
        }
        else{
            if(self->verbose)
                printf("Waiting for ref_points...\n");

            self->translational_vel_last = 0.0;
            self->rotational_vel_last = 0.0;
            robot_velocity_command(0, 0, self->lcm);
        }
    }
    else if(self->bot_pose_last == NULL){
        self->bot_pose_last = calloc(1, sizeof(bot_core_pose_t));

        self->tc_error_mode = RIPL_TRAJECTORY_CONTROLLER_STATUS_T_ERROR_NO_POSE;
        fprintf(stderr,"Exiting - No Pose\n");
        return TRUE;
    }
    else{
        if(self->verbose)
            fprintf(stderr,"Current Ref Point is %d (out of %d Ref points)\n", self->current_ref_point, self->num_ref_points);

        if( self->current_ref_point >= 0 ){

            //Render Reference Points and Optimal/Predicted trajectories
            bot_lcmgl_t *lcmgl = self->lcmgl;
            bot_lcmgl_t *lcmgl_text = self->lcmgl_text;

            //Render Reference Points...
            if (RENDER_REF_POINTS){
                for(int i = FIRST_TO_RENDER; i <= self->num_ref_points; i++){

                    double pos[3] = {self->ref_point_list->ref_points[i].x + 0.2,
                                     self->ref_point_list->ref_points[i].y + 0.2,
                                     self->bot_pose_last->pos[2]};
                    char p_info[100];

                    sprintf(p_info, "%d:%.2f\n", i, self->ref_point_list->ref_points[i].s);
                    bot_lcmgl_text(lcmgl_text, pos, p_info);

                    lcmglColor3f (0.0, 0.0, 5.0);
                    if(i == self->current_ref_point){
                        lcmglColor3f (1.0, 0.0, 0.0);
                        lcmglPointSize (25.0);
                    }
                    else{
                        lcmglColor3f (0.0, 0.0, 1.0);
                        lcmglPointSize (15.0);
                    }
                    lcmglBegin(GL_POINTS);
                    lcmglVertex3d ( self->ref_point_list->ref_points[i].x,
                                   self->ref_point_list->ref_points[i].y,
                                   self->bot_pose_last->pos[2]);
                    lcmglEnd();

                    lcmglPushMatrix();
                    lcmglTranslated(self->ref_point_list->ref_points[i].x, self->ref_point_list->ref_points[i].y, 0);
                    lcmglRotated((self->ref_point_list->ref_points[i].t * (180.0/M_PI) ), 0, 0, 1);

                    lcmglBegin(GL_LINES);
                    lcmglColor3f(0, 5, 0);
                    lcmglVertex2d( - .15, 0);
                    lcmglVertex2d( + .15, 0);
                    lcmglVertex2d( 0, .15);
                    lcmglVertex2d( + .15, 0);
                    lcmglVertex2d( 0, -.15);
                    lcmglVertex2d( + .15, 0);

                    lcmglEnd();

                    lcmglPopMatrix();
                }
            }

            //Render 'optimal trajectory' (simple line connecting reference points)
            if (RENDER_OPTIMAL_PATH){
                lcmglColor3f (3.0, 0.0, 0.0);
                lcmglLineWidth (1.2);
                lcmglBegin (GL_LINE_STRIP);

                for(int i = FIRST_TO_RENDER; i <= self->num_ref_points; i++){
                    lcmglVertex3d (self->ref_point_list->ref_points[i].x,
                                   self->ref_point_list->ref_points[i].y,
                                   self->bot_pose_last->pos[2]);
                }
                lcmglEnd();
            }

            //draw entire predicted trajectory
            if (RENDER_PREDICTED_TRAJ) {
                double rpy_bot[3];
                bot_quat_to_roll_pitch_yaw (self->bot_pose_last->orientation, rpy_bot);

                mp_state_t state_ini, state_fin;
                for(int i = FIRST_TO_RENDER; i <= self->num_ref_points; i++){

                    if ( i == FIRST_TO_RENDER){
                        state_ini.x = self->bot_pose_last->pos[0] ;
                        state_ini.y = self->bot_pose_last->pos[1] ;
                        state_ini.t = rpy_bot[2];
                    }

                    state_fin.x = self->ref_point_list->ref_points[i].x;
                    state_fin.y = self->ref_point_list->ref_points[i].y;
                    state_fin.t = self->ref_point_list->ref_points[i].t;

                    GSList *states = NULL;
                    GSList *inputs = NULL;

                    mp_prediction_propagate_trajectory (self->mp_prediction, &state_ini, &state_fin, &states, &inputs );

                    // mp_prediction_draw_trajectory (self->mp_prediction, states, inputs, self->bot_pose_last->pos[2]);

                    lcmglColor3f (0.0, 5.0, 0.0);
                    lcmglLineWidth (2.0);
                    lcmglBegin (GL_LINE_STRIP);

                    GSList *states_ptr = states;

                    while (states_ptr) {
                        mp_state_t *state_curr = (mp_state_t *)(states_ptr->data);
                        lcmglVertex3d (state_curr->x, state_curr->y, self->bot_pose_last->pos[2]);
                        state_ini.x = state_curr->x;
                        state_ini.y = state_curr->y;
                        state_ini.t = state_curr->t;

                        states_ptr = g_slist_next (states_ptr);
                    }
                    lcmglEnd ();
                }
            }

            bot_lcmgl_switch_buffer (self->lcmgl);
            bot_lcmgl_switch_buffer (self->lcmgl_text);
        }

        //controller won't drive to the initial position; 0s could be changed to the current pose

        if(self->ref_point_list->num_ref_points <= 0){
            fprintf(stderr,"Empty ref point list - stopping robot");
            self->translational_vel_last = 0.0;
            self->rotational_vel_last = 0.0;
            robot_velocity_command (0, 0, self->lcm);
            return TRUE;
        }

        if (self->current_ref_point > self->num_ref_points)
            fprintf (stderr, "Current ref point (%d) exceeds the number of reference points in the list (%d)\n",
                     self->current_ref_point, self->num_ref_points);

        if(self->ref_point_list->ref_points[self->current_ref_point].x == 0 && self->ref_point_list->ref_points[self->current_ref_point].y == 0  )
            self->current_ref_point++;

        //this is where the magic happens
        /*int ret_val = 0;
        ret_val = manipulation_controller_drive_to_target (self,
                                                           self->ref_point_list->ref_points[self->current_ref_point].x,
                                                           self->ref_point_list->ref_points[self->current_ref_point].y,
                                                           self->ref_point_list->ref_points[self->current_ref_point].t,
                                                           STOPPING_DIST,
                                                           self->ref_point_list->ref_points[self->current_ref_point].s);

        */

        int ret_val = 0;

        if(!self->turn_in_place){
            //this is where the magic happens

            // The stopping distance affects whether the controller
            // advaces to the next reference point For a non-dubins
            // vehicle, this should change according to the local
            // curvature of the path
            double stop_dist = self->stopping_distance;
            if(self->current_ref_point == self->num_ref_points){
                if(self->verbose){
                    fprintf(stderr, "+++++ Driving towards the last waypoint - using the small stopping distance\n");
                }
                stop_dist = self->stopping_distance_small;
            }
            // A simple hack is to look at the difference in heading
            // associated with nearby points (looking both backwards
            // and forwards along the path
            else if (1) {
                int ref_point_index = self->current_ref_point;
                double x0, x1, y0, y1, t0, t1;
                x0 = self->ref_point_list->ref_points[ref_point_index].x;
                y0 = self->ref_point_list->ref_points[ref_point_index].y;
                t0 = self->ref_point_list->ref_points[ref_point_index].t * 180/M_PI;
                double deltat_forward = 0;
                double dist = 0;
                ref_point_index++;
                // Check forward up till DISTANCE_CURVATURE_CHECK or end of path
                while ((ref_point_index < self->num_ref_points) && (dist < DISTANCE_CURVATURE_CHECK)) {
                    x1 = self->ref_point_list->ref_points[ref_point_index].x;
                    y1 = self->ref_point_list->ref_points[ref_point_index].y;
                    t1 = self->ref_point_list->ref_points[ref_point_index].t * 180/M_PI;

                    dist += sqrt ((x1-x0)*(x1-x0) + (y1-y0)*(y1-y0));

                    double delta = fabs(t1-t0);
                    deltat_forward += bot_min (delta, fabs(360-delta));//fabs (t1-t0);
                    //fprintf (stdout, "i = %d, t0 = %.2f, t1 = %.2f, delta = %.2f, deltat_forward = %.2f, dist = %.2f\n", ref_point_index-self->current_ref_point, t0, t1, delta, deltat_forward, dist);
                    x0 = x1;
                    y0 = y1;
                    t0 = t1;
                    ref_point_index++;
                }

                // Remove wrap-around
                //while (deltat_forward > 180)
                //    deltat_forward -= 360;
                //deltat_forward = fabs(deltat_forward);

                // Now, check backwards
                ref_point_index = self->current_ref_point;
                x0 = self->ref_point_list->ref_points[ref_point_index].x;
                y0 = self->ref_point_list->ref_points[ref_point_index].y;
                t0 = self->ref_point_list->ref_points[ref_point_index].t * 180/M_PI;
                double deltat_backward = 0;
                dist = 0;
                ref_point_index--;
                // Check forward up till DISTANCE_CURVATURE_CHECK or end of path
                while ((ref_point_index > 0) && (dist < DISTANCE_CURVATURE_CHECK)) {
                    x1 = self->ref_point_list->ref_points[ref_point_index].x;
                    y1 = self->ref_point_list->ref_points[ref_point_index].y;
                    t1 = self->ref_point_list->ref_points[ref_point_index].t * 180/M_PI;

                    dist += sqrt ((x1-x0)*(x1-x0) + (y1-y0)*(y1-y0));

                    double delta = fabs(t1-t0);
                    deltat_backward += bot_min (delta, fabs(360-delta));//fabs (t1-t0);
                    x0 = x1;
                    y0 = y1;
                    t0 = t1;
                    ref_point_index--;
                }

                // Remove wrap-around
                while (deltat_backward > 180)
                    deltat_backward -= 360;
                deltat_backward = fabs(deltat_backward);

                if ((deltat_backward > THRESHOLD_CURVATURE_CHECK) || (deltat_forward > THRESHOLD_CURVATURE_CHECK)) {
                    //if (self->verbose)
                        fprintf (stdout, "Reducing lookahead due to increased path curveature (deltat_forward = %.2f, deltat_backward = %.2f)\n", deltat_forward, deltat_backward);
                    stop_dist = self->stopping_distance_small;
                }
            }

            // Look ahead along a desired distance along the path for obstacles
            int ref_point_index = self->current_ref_point;
            //double start_x = self->bot_pose_last->pos[0];
            double start_x = self->ref_point_list->ref_points[ref_point_index].x;
            //double start_y = self->bot_pose_last->pos[1];
            double start_y = self->ref_point_list->ref_points[ref_point_index].y;
            ref_point_index++;
            int is_collision_free = 1;
            double collision_check_distance = 0;

            //fprintf (stdout, "perform_collision_check = %d\n", self->perform_collision_check);
            while (self->perform_collision_check & collision_check_distance < self->collision_check_path_distance &
                   ref_point_index < self->num_ref_points) {


                is_collision_free = free_for_drive_to_target (self, start_x, start_y,
                                                              self->ref_point_list->ref_points[ref_point_index].x,
                                                              self->ref_point_list->ref_points[ref_point_index].y,
                                                              self->ref_point_list->ref_points[ref_point_index].t);


                collision_check_distance += hypot (self->ref_point_list->ref_points[ref_point_index].x - start_x,
                                                   self->ref_point_list->ref_points[ref_point_index].y - start_y);

                /* fprintf (stdout, "is_collision_free = %d, start_x = %.2f, start_y = %.2f, x = %.2f, y = %.2f\n", */
                /*          is_collision_free, start_x, start_y, self->ref_point_list->ref_points[ref_point_index].x, self->ref_point_list->ref_points[ref_point_index].y); */


                // Render collision detection
                if (!is_collision_free) {
                    bot_lcmgl_t *lcmgl = self->lcmgl_collision;

                    lcmglColor3f (3.0, 0.0, 0.0);
                    lcmglLineWidth (1.2);
                    lcmglBegin (GL_LINE_STRIP);

                    lcmglVertex3d (start_x, start_y, 0);
                    lcmglVertex3d (self->ref_point_list->ref_points[ref_point_index].x, self->ref_point_list->ref_points[ref_point_index].y, 0);
                    lcmglEnd();

                    bot_lcmgl_switch_buffer (lcmgl);
                }

                if (is_collision_free == 0)
                    break;


                start_x = self->ref_point_list->ref_points[ref_point_index].x;
                start_y = self->ref_point_list->ref_points[ref_point_index].y;
                ref_point_index++;
            }

            if (is_collision_free == 0) {

                fprintf (stdout, "Obstacle detected %.2f meters ahead along path. Stopping robot!\n", collision_check_distance);

                double tv = 0;
                double rv = 0;

                robot_velocity_command(tv, rv, self->lcm);
                mp_control_update_aux_actuation (self->mp_cont, tv, rv);
                mp_control_publish_aux_message (self->lcm, self->mp_cont);

                // Set the error
                ret_val = -5;
            }
            else
                ret_val = manipulation_controller_drive_to_target (self,
                                                                   self->ref_point_list->ref_points[self->current_ref_point].x,
                                                                   self->ref_point_list->ref_points[self->current_ref_point].y,
                                                                   self->ref_point_list->ref_points[self->current_ref_point].t,
                                                                   stop_dist,
                                                                   self->ref_point_list->ref_points[self->current_ref_point].s);
        }
        else{
            if (self->verbose)
                fprintf(stderr, " +++ Turning in place ++++\n");

            ret_val = manipulation_controller_turn_towards_target(self,
                                                                  self->ref_point_list->ref_points[self->current_ref_point].x,
                                                                  self->ref_point_list->ref_points[self->current_ref_point].y,
                                                                  self->ref_point_list->ref_points[self->current_ref_point].t);
            if(ret_val==1){
                self->goal_state = RIPL_RRT_GOAL_STATUS_T_REACHED;
            }
            else if(ret_val ==0){
                self->goal_state = RIPL_RRT_GOAL_STATUS_T_ACTIVE;
            }
            else if(ret_val < 0){
                self->goal_state = RIPL_RRT_GOAL_STATUS_T_FAILED;
            }


        }

        if(self->new_ref_point == 1){

            if (self->verbose)
                fprintf(stdout, "Driving to ref_point %d at %f, %f...\n", self->current_ref_point,
                        self->ref_point_list->ref_points[self->current_ref_point].x,
                        self->ref_point_list->ref_points[self->current_ref_point].y);

            self->new_ref_point = 0;
        }

        if((self->current_ref_point == self->num_ref_points) && !self->turn_in_place) {
            if(ret_val == 1){
                self->goal_state = RIPL_RRT_GOAL_STATUS_T_REACHED;
            }
            else if(ret_val < 0){
                fprintf(stderr, "At last goal and error condition : %d\n", ret_val);
                self->goal_state = RIPL_RRT_GOAL_STATUS_T_FAILED;
            }
        }
        if(!self->turn_in_place && ret_val == 0){
            self->goal_state = RIPL_RRT_GOAL_STATUS_T_ACTIVE;
        }

        if (ret_val == 1){
            //stop the bot when it gets to the last reference point
            if (self->current_ref_point == self->num_ref_points ) {

                robot_velocity_command(0, 0, self->lcm);


#if ENABLE_TELEPORT
                //Teleport the robot
                botlcm_pose_t p;
                p.pos[0] = 0.0;
                p.pos[1] = 0.0;
                p.pos[2] = 0.0;

                p.orientation[0] = 1.0;
                p.orientation[1] = 0.0;
                p.orientation[2] = 0.0;
                p.orientation[3] = 0.0;

                p.accel[0] = 0.0;
                p.accel[1] = 0.0;
                p.accel[2] = 0.0;
                botlcm_pose_t_publish(self->lcm, "POSE", &p);

                botlcm_pose_t_publish(self->lcm, "SIM_TELEPORT", &p);

#endif
                //Reset reference point list
                self->num_ref_points = -1;


            }

            else {

                if (self->verbose)
                    printf("Done with ref_point %d at %f, %f...\n", self->current_ref_point,
                           self->ref_point_list->ref_points[self->current_ref_point].x,
                           self->ref_point_list->ref_points[self->current_ref_point].y);

                self->new_ref_point = 1;
                self->current_ref_point++;

            }
        }

    }

    return 1;

}


static void
usage (int argc, char ** argv)
{
    fprintf (stdout, "Usage: %s [options]\n"
             "\n"
             "    -e             Run in envoy mode\n"
             "    -a             Replace current waypoints with new waypoints\n"
             //"    -j             Subservient to joystick (L1 button must be depressed)\n"
             "    -D             Disable collision check\n"
             "    -d DISTANCE    Check for collisions up to DISTANCE along path\n"
             "    -v             Verbose output\n"
             "    -h             Print this usage and exit\n"
             "\n\n", argv[0]);
}

int main (int argc, char *argv[])
{
    setlinebuf(stdout);
    state_t *self = (state_t*) calloc(1, sizeof(state_t));

    g_thread_init(NULL);
    int c;

    self->trash_wp = 0;
    self->perform_collision_check = 1;
    self->collision_check_path_distance = COLLISION_CHECK_PATH_DISTANCE;

    while ((c = getopt (argc, argv, "evhajDd:")) >= 0) {
        switch (c) {
            case 'v':
                self->verbose = 1;
                break;
            case 'D':
                self->perform_collision_check = 0;
                break;
            case 'd':
                self->collision_check_path_distance = strtod(optarg, 0);
            case 'a':
                self->trash_wp = 1;
                break;
            case 'e':
                self->envoy = 1;
                break;
		//case 'j':
                //self->subservient_to_joystick = 1;
                //break;
            case 'h':
            case '?':
                usage (argc, argv);
                return 1;
        }
    }

    if (self->collision_check_path_distance < self->line_collision_check_delta) {
        self->collision_check_path_distance = self->line_collision_check_delta;
        fprintf (stdout, "The collision check distance can't be less than %.2f, setting to %.2f\n", self->line_collision_check_delta,
                 self->collision_check_path_distance);
    }

    if (self->perform_collision_check)
        fprintf (stdout, "Checking the path %.2f meters ahead for collisions\n", self->collision_check_path_distance);

    self->error_mode = 0;
    self->tc_state = RIPL_TRAJECTORY_CONTROLLER_STATUS_T_STATE_IDLE;
    self->tc_error_mode = RIPL_TRAJECTORY_CONTROLLER_STATUS_T_ERROR_NONE;
    self->goal_state = RIPL_RRT_GOAL_STATUS_T_IDLE;

    self->lcm = bot_lcm_get_global(NULL);

    BotParam *param = bot_param_new_from_server (self->lcm, 0);

    if (!param) {
        fprintf (stderr, "Couldn't get BotParam instance\n");
        goto fail;
    }

    self->default_tv = bot_param_get_double_or_fail (param, "motion_planner.speed_design.default_tv");
    self->max_rv = bot_param_get_double_or_fail (param, "motion_planner.speed_design.max_rv");
    self->max_rv_in_place = bot_param_get_double_or_fail (param, "motion_planner.speed_design.max_rv_in_place");
    self->thresh_tv = bot_param_get_double_or_fail (param, "motion_planner.speed_design.thresh_tv");
    self->current_tv = self->default_tv;

    self->mp_cont = mp_control_new ();

    self->mp_cont->K_ct = bot_param_get_double_or_fail (param, "pp_controller.K_ct");
    self->mp_cont->K_ct_traj = bot_param_get_double_or_fail (param, "pp_controller.K_ct_traj");
    self->mp_cont->Kp = bot_param_get_double_or_fail (param, "pp_controller.Kp");
    self->mp_cont->Ki = bot_param_get_double_or_fail (param, "pp_controller.Ki");
    self->mp_cont->K_str = bot_param_get_double_or_fail (param, "pp_controller.K_str");
    self->mp_cont->K_str_traj = bot_param_get_double_or_fail (param, "pp_controller.K_str_traj");
    self->mp_cont->max_integral = bot_param_get_double_or_fail (param, "pp_controller.max_integral");
    self->mp_cont->default_integral_state = bot_param_get_double_or_fail (param, "pp_controller.default_integral_state");
    self->mp_cont->dt = bot_param_get_double_or_fail (param, "pp_controller.dt");



    self->lcmgl = bot_lcmgl_init (self->lcm, "TRAJECTORY_CONTROLLER");
    self->lcmgl_text = bot_lcmgl_init (self->lcm, "TRAJECTORY_CONTROLLER_TEXT");
    self->lcmgl_collision = bot_lcmgl_init (self->lcm, "TRAJECTORY_CONTROLLER_COLLISION");

    self->mp_prediction = mp_prediction_create (self->lcm);

    self->stopping_distance = bot_param_get_double_or_fail (param, "pp_controller.stopping_distance");
    self->stopping_distance_small = bot_param_get_double_or_fail (param, "pp_controller.stopping_distance_small");
    self->line_collision_check_delta = bot_param_get_double_or_fail (param, "pp_controller.line_collision_check_delta");

    self->mp_prediction->max_tv = self->default_tv;
    self->mp_prediction->max_steer = bot_param_get_double_or_fail (param, "pp_controller.max_steer");

    self->mp_prediction->wheelbase = bot_param_get_double_or_fail (param, "robot.wheelbase");
    self->mp_prediction->K_str = bot_param_get_double_or_fail (param, "pp_controller.K_str"); // 2.0;
    self->mp_prediction->K_ct = bot_param_get_double_or_fail (param, "pp_controller.K_ct"); //1.75;

    double length = bot_param_get_double_or_fail (param, "robot.length");
    double width = bot_param_get_double_or_fail (param, "robot.width");
    self->vehicle_turning_footprint = sqrt((width/2)*(width/2) + (length/2)*(length/2));
    self->vehicle_width = width;

    self->obstacles_last = NULL;
    self->sim_rect_last = NULL;
    self->path_obstructed = FALSE;

    self->bot_pose_last = NULL;
    self->robot_status_last = NULL;
    /****************************************************/
    self->new_ref_point = 1;
    self->num_ref_points = -1;
    self->current_ref_point = 0;
    self->ref_point_list = NULL;
    /****************************************************/

    self->guide_pos = NULL;

    bot_core_pose_t_subscribe (self->lcm, "POSE", on_bot_pose, self);

    ripl_guide_info_t_subscribe (self->lcm, "GUIDE_POS", on_guide_pose, self);

    // subscribe to the robot_status message
    ripl_robot_status_t_subscribe (self->lcm, "ROBOT_STATUS", on_robot_status, self);

    ripl_speech_cmd_t_subscribe (self->lcm, "WAYPOINT_NAVIGATOR", on_motion_command, self);

    // subscribe to the ref_point list message
    ripl_ref_point_list_t_subscribe (self->lcm, "GOAL_REF_LIST", on_ref_point_list, self);

    // subscribe to the ref_point list message
    ripl_ref_point_list_t_subscribe (self->lcm, "ELEVATOR_GOAL_REF_LIST", on_ref_point_list, self);



    /******************************************************************/

    obs_rect_list_t_subscribe(self->lcm, "SIM_RECTS", on_sim_rects, self);

    obs_obstacle_list_t_subscribe (self->lcm, "OBSTACLES", on_obstacles, self);

    /* if (self->subservient_to_joystick) */
    /*     ripl_joystick_state_t_subscribe (self->lcm, "PS3_JS_CMD", on_joystick_state, self); */


    self->mutex = g_mutex_new();
    if (!self->mutex) {
        fprintf (stderr, "Failed to create GMutex\n");
        goto fail;
    }

    self->mainloop = g_main_loop_new (NULL, FALSE);
    if (!self->mainloop) {
        fprintf (stderr, "Couldn't create glib main loop\n");
        goto fail;
    }

    bot_glib_mainloop_attach_lcm (self->lcm);

    //dont think we need this one - we aren't doing manipulation - least not for now
    self->status_update_timer_id = g_timeout_add (50, on_status_timer, self);
    if (!self->status_update_timer_id) {
        fprintf (stderr, "Couldn't create status update timer\n");
        goto fail;
    }


    //this timer call does the rrt following
    self->trajectory_controller_timer_id = g_timeout_add (50, on_trajectory_controller_timer, self);
    if (!self->trajectory_controller_timer_id) {
        fprintf (stderr, "Couldn't create the vehicle controller timer\n");

        goto fail;
    }

    g_main_loop_run (self->mainloop);

    // cleanup
    if (self->mainloop)
        g_main_loop_unref (self->mainloop);

    /*
      if (self->manipulation_planner)
      mp_destroy ( self->manipulation_planner);
    */

    free (self);

    return 0;

 fail:
    if (self->mainloop) g_main_loop_unref (self->mainloop);
    free (self);
    return -1;

}
