
#pragma once

#include "parser.h"
#include "utils.h"
#include "hom_vertex.h"
#include "open_set.h"
#include "commonmacros.h"
#include "obstacle_marker.h"
#include "hom_classes.h"

/*** Configuration parameters wrapper ***/
typedef struct {
    float a;
    float b;
} Config;

/*** Default cost function ***/
float edge_cost(
    int x_1, int y_1, int x_2, int y_2,
    Params * params,
    Config * config
);

/*** Single homotopy class heuristic ***/
float heuristic(
    int x_n, int y_n, int x_g, int y_g,
    Params * params,
    Config * config
);

/*** Multiple homotopy class heuristic (Dijkstra's) ***/
float zero_heuristic(
    int x_n, int y_n, int x_g, int y_g,
    Params * params,
    Config * config
);

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
 *          - number of target homotopy classes
 * 
 * Outputs:
 *          -  Possible return value: The number of filled homotopy classes, which may or may not be the input maximum homotopy classes
 *          -  Collateral: The list of target homotopy classes is updated with the detected homotopy classes into the goal,
 *                         as explained in the sub-routine A_star_goal_check.
 * 
 * 
 * NOTE: Caller responsible for properly initializing both Params and F_t structs before passing them to A_star_homotopies
 *       Caller responsible for later free'ing target_hom_classes (i.e. by calling free_hom_classes_list(target_hom_classes))
 *          and other already-initialized structs passed such as params and F
 * 
 * 
 * Usage example:
 * 
 *  Params * params = load_json(input_json_path);
    F_t * F = initialize_obstacle_marker_func_params(params, float_tol);
    const int num_iterations = 4;   // A* runs
    const int max_hom_classes = params->num_obstacles;  // how many homotopy classes we want A* to keep track of
    hom_classes_list_t * target_hom_classes = NULL;     // list of homotopic classes, initialized to NULL

    for (int i = 0; i < num_iterations; i++) {
        A_star_homotopies(
        params, config,
        zero_heuristic, // this essentially turns A* into Dijkstra's for uniform exploration
        edge_cost, float_tol,
        0.1, F,
        &target_hom_classes,
        max_hom_classes
        );
        // Tweak the cost function to change the order in which homotopy classes are discovered
        config->a *= powf(16, i);
    }
    // Free the final resulting homtopy classes list
    free_hom_classes_list(target_hom_classes);
    // Other clean-up routines
 *
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
    int max_expandible_states_mult;
};
void A_star_homotopies(struct A_star_homotopies_args * args);

