#include "mp_prediction.h"

#define MAX_VEL 0.5//0.1
#define MAX_STEER 9.0

#define VEHICLE_SECURITY_RADIUS 1.3          // If any obstacle gets withing the security radius, the vehicle will stop manipulation
#define DISTANCE_FROM_CENTER_TO_TINE_TIP 0.0 // Distance from the center of the vehicle to the tip of the tines
#define PRUNING_DISTANCE 1.5


mp_state_t* mp_prediction_new_state (mp_prediction_t *self) {
    
    mp_state_t *state_new = malloc (sizeof (mp_state_t));
    
    return state_new;    
}


mp_input_t* mp_prediction_new_input (mp_prediction_t *self) {

    mp_input_t *input_new = malloc (sizeof (mp_input_t));
    
    return input_new;
}
        

int mp_prediction_delete_state (mp_prediction_t *self, mp_state_t *state) {
    if (!state)
        return 0;
    free (state);
    return 1;
}


int mp_prediction_delete_input (mp_prediction_t *self, mp_input_t *input) {
    if (!input)
        return 0;
    free (input);
    return 1;
}


int mp_prediction_propagate_step (mp_prediction_t *self, mp_state_t *state, mp_input_t *input, double del_time,
                       mp_state_t *state_out) {
    
    // Calculate the velocities

    double del_x = input->v * cos (state->t) * del_time;
    double del_y = input->v * sin (state->t) * del_time;
    double del_t = (tan (input->d) / self->wheelbase) * del_time;

    state_out->x = state->x + del_x; 
    state_out->y = state->y + del_y; 
    state_out->t = state->t + del_t; 
    
    return 1;
}


int mp_prediction_propagate_compute_control (mp_prediction_t *self, 
                                             mp_state_t *state_curr, mp_state_t *state_fin, double *steering_out) {
   
    // Translate/rotate the current state such that the final state is (0, 0, 0)   
    double x_diff = state_curr->x - state_fin->x;
    double y_diff = state_curr->y - state_fin->y;
    
    double cos_t_fin = cos(-state_fin->t);
    double sin_t_fin = sin(-state_fin->t);

    double x_curr_in_fin_frame = cos_t_fin * x_diff - sin_t_fin * y_diff;
    if (x_curr_in_fin_frame > -PRUNING_DISTANCE) // wrong side of the pallet 
        return 0;
    
    // Compute the errors and check whether the control applies
    
    // Compute cross track error
    double error_e = sin_t_fin * x_diff + cos_t_fin * y_diff; // (= y_curr_in_fin_frame)
    
    // Compute orientation error
    double error_t = state_curr->t - state_fin->t; // (= t_curr_in_fin_frame)
    while (error_t < - M_PI)
        error_t += 2 * M_PI;
    while (error_t > M_PI)
        error_t -= 2 * M_PI;
    
    if ( (error_t > M_PI_2) || (error_t < -M_PI_2) ) // pallet is on the wrong side
        return 0;
    
    double steering = - self->K_str * (atan(self->K_ct * error_e) + error_t);
    if (steering < -MAX_STEER * M_PI/180.0)
        steering = -MAX_STEER * M_PI/180.0;

    if (steering > MAX_STEER * M_PI/180.0)
        steering = MAX_STEER * M_PI/180.0;
    
    *steering_out = steering;
    
    return 1;
}




// Given the initial and final states, returns a sequence states that represents the trajectory of the system 
//       under the controller.
int mp_prediction_propagate_trajectory (mp_prediction_t *self, mp_state_t *state_ini, mp_state_t *state_fin,
                                        GSList **states_out, GSList **inputs_out) {
    
    // Starting from the initial state calculate the controls and propagate
    // Store the resulting state and inputs into a gslist
    
    GSList *states = NULL;
    GSList *inputs = NULL;

    double del_time = 0.1;  // Fix a delta_time
    
    mp_state_t *state_prev = state_ini;

    int i = 0;


    while (1) {
       
 
        double steering;
        if (!mp_prediction_propagate_compute_control (self, state_prev, state_fin, &steering)) {
            *states_out = g_slist_reverse (states);
            *inputs_out = g_slist_reverse (inputs);
            
            return 1;
        }
        mp_state_t *state_curr = mp_prediction_new_state (self);
        mp_input_t *input_curr = mp_prediction_new_input (self);
       
        
        input_curr->v = MAX_VEL;
        input_curr->d = steering;
        
        mp_prediction_propagate_step (self, state_prev, input_curr, del_time, state_curr);
        
        states = g_slist_prepend (states, state_curr);
        inputs = g_slist_prepend (inputs, input_curr);


        if (i++ > 10000) {
            printf ("ERR: Prediction exceeded max iterations (more than 100 sec)\n");
            return 0;
        }

        state_prev = state_curr;
    }

}


int mp_prediction_destroy_trajectory (mp_prediction_t *self, GSList *states, GSList *inputs) {
    
    GSList *states_ptr = states;
    while (states_ptr) {
        mp_state_t *state_curr = states_ptr->data;
        mp_prediction_delete_state (self, state_curr);
        states_ptr = g_slist_next (states_ptr);
    }
    g_slist_free (states);

    GSList *inputs_ptr = inputs;
    while (inputs_ptr) {
        mp_input_t *input_curr = inputs_ptr->data;
        mp_prediction_delete_input (self, input_curr);
        inputs_ptr = g_slist_next (inputs_ptr);
    }
    g_slist_free (inputs);
    
    return 1;
}


int mp_prediction_draw_trajectory (mp_prediction_t *self, GSList *states, GSList *inputs, double height) {
    
    if (self->lcmgl) {
        bot_lcmgl_t *lcmgl = self->lcmgl;

        lcmglColor3f (1.0, 1.0, 0.2); 
        lcmglLineWidth (3.0);
        lcmglBegin (GL_LINE_STRIP);

        
        GSList *states_ptr = states;
        while (states_ptr) {
            mp_state_t *state_curr = (mp_state_t *)(states_ptr->data);
            lcmglVertex3d (state_curr->x, state_curr->y, height);
            states_ptr = g_slist_next (states_ptr);
        }

        lcmglEnd ();

     bot_lcmgl_switch_buffer (self->lcmgl);
    }
    
    return 1;
}


erlcm_rect_t* mp_prediction_check_collision_with_rect_list (mp_prediction_t *self, GSList *states, GSList *inputs,
                                                            erlcm_rect_list_t *rect_list) {
    
    if ( (!rect_list) || (!states) )
        return 0;

    erlcm_rect_t *colliding_rect = NULL;

    int j = 0;
    
    GSList *states_ptr = states;
    while (states_ptr) {

        if (j++ % 10 == 0) {
            states_ptr = g_slist_next (states_ptr);
            continue;
        }
        
        mp_state_t *state_curr = states_ptr->data;

        // Translate the pose from the tip of the tine to the center of the vehicle
        double cos_t = cos (state_curr->t);
        double sin_t = sin (state_curr->t);
        double state_curr_x = state_curr->x
            - DISTANCE_FROM_CENTER_TO_TINE_TIP * cos_t;
        double state_curr_y = state_curr->y
            - DISTANCE_FROM_CENTER_TO_TINE_TIP * sin_t;
        
        // Check this state with all the rects. 
        for (int i = 0; i < rect_list->num_rects; i++) {
            erlcm_rect_t *rect_curr =  &(rect_list->rects[i]);
            
            double max_rect_size 
                = (rect_curr->size[0] > rect_curr->size[1]) ? rect_curr->size[0] : rect_curr->size[1];
            
            double absolute_distance_x = rect_curr->dxy[0] - state_curr_x;
            double absolute_distance_y = rect_curr->dxy[1] - state_curr_y;
            double absolute_distance = sqrt (absolute_distance_x * absolute_distance_x 
                                             + absolute_distance_y * absolute_distance_y);
            
            if (absolute_distance < max_rect_size + VEHICLE_SECURITY_RADIUS) {
                colliding_rect = erlcm_rect_t_copy (rect_curr);
                goto terminate_obstacle_check;
            }
            
        }
        
        states_ptr = g_slist_next (states_ptr);
    }


terminate_obstacle_check:

    return colliding_rect;
}


int mp_prediction_draw_colliding_rect (mp_prediction_t *self, erlcm_rect_t *colliding_rect) {


    if (!colliding_rect) {
        return 0;
    }

    if (self->lcmgl) {

//         printf ("Rect : %5.5lf, %5.5lf\n", colliding_rect->dxy[0], colliding_rect->dxy[1]);
        
        bot_lcmgl_t *lcmgl = self->lcmgl;
    
        lcmglColor3f (1.0, 0.2, 0.2); 
        
        lcmglBegin (GL_POINTS);
        lcmglVertex3d (colliding_rect->dxy[0], colliding_rect->dxy[1], 0.5);
        lcmglEnd ();
        
        bot_lcmgl_switch_buffer (self->lcmgl);
    }
    
    return 1;
} 


int mp_prediction_check_proximity_map (mp_prediction_t *self, bot_core_planar_lidar_t *proximity_map) {
    
    
    return 0;
}


mp_prediction_t* mp_prediction_create (lcm_t *lcm) {

    mp_prediction_t *self = calloc (sizeof (mp_prediction_t), 1);
    
    if (lcm) {
        self->lcmgl = bot_lcmgl_init (lcm, "MANIPULATION_PLANNER_PREDICTION");    
    }
    
    return self;
}

