
#pragma once

#include "parser.h"
#include "utils.h"
#include "open_set.h"
#include "obstacle_marker.h"
#include "hom_classes.h"
#include "cost_funcs.h"

/**************** A_star_homotopies ***************
 * 
 * Inputs:
 *      Inputs for regular A*:
 *          - Params struct containing all the necessary information regarding the environment
 *          - Config containing the relative weights a, b for uncertainty and path length respectively in the cost funtion
 *          - pointer to the heuristic function
 *          - pointer to the cost function
 *          - tolerance value for float comparisons
 *      Additional inputs for homotopy classes search:
 *          - absolute value tolerance when comparing real and imaginary components of homotopy class descriptors
 *          - obstacle marker function parameters (opaque type handled in obstacle_marker.c)
 *          - pointer of list of target homotopy classes to be updated in place
 *          - number of target homotopy classes to search
 * 
 * Outputs:
 *          -  Collateral: The list of target homotopy classes is updated in place with the detected homotopy classes into the goal,
 *  
 * NOTE: Caller responsible for properly allocating and placing all parameters into the dedicated struct.
 *       This function will not change any of the parameters in the given struct, except for updating in place the location 
 *          pointed to by target_hom_classes_ptr with the updated homotopy classes for the goal.
 *       The caller is then responsible for deallocating all the input parameters.
 * 
 * IMPORTANT: The target homotopy class list is allocated and set to the goal's if it points to NULL (intended initialization).
 *            Otherwise, the function updates in place each homotopy class of the target list. Therefore, the homotopy classes
 *            are still reacheable at the same addresses, but with the new goal point information copied in.
 *            It is thus the responsibility of the caller to either pass again the target homotopy list for another round,
 *            or eventually free it when finished using it.
 *            This design allows to extract the paths to the goal at every iteration
 * 
 * Detailed usage and testing example shown in the implementation file. 
*/
struct A_star_homotopies_args {
    Params * params;
    Config * config;
    float (*h) (int, int, int, int, Params *, Config * );
    float (*c) (int, int, int, int, Params *, Config * );
    float float_tol;
    /* Homotopy parameters */
    double abs_tol;
    F_t * F;
    hom_classes_list_t ** target_hom_classes_ptr; 
    int max_hom_classes;
    int filled_hom_classes;
    int max_expandible_states_mult;
    char * json_filepath;
};
void A_star_homotopies(struct A_star_homotopies_args * args);


/************** helpers ******************/
hom_vertex_t *** allocate_grid(struct A_star_homotopies_args * args);
void deallocate_grid(struct A_star_homotopies_args * args, hom_vertex_t *** S, bool target_hom_classes_init, int x_goal_step, int y_goal_step);
void print_A_star_homotopies_args(struct A_star_homotopies_args * args);
void print_A_star_homotopies_goal_info(hom_class_t * curr_hom_class);

/***************    MACROS  **************/
#define A_STAR_ARGS_CHECK(args) \
    (!args || !args->params || !args->params->g_image || \
     !args->config || !args->h || !args->c || \
     !args->F || !args->target_hom_classes_ptr || !args->json_filepath || \
     args->abs_tol <= 0 || args->float_tol <= 0)
    

