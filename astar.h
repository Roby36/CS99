
#pragma once

#include "parser.h"
#include "utils.h"
#include "vertex.h"
#include "open_set.h"
#include "commonmacros.h"
#include "obstacle_marker.h"
#include "hom_classes.h"

/* Keep configuration parameters in a wrappable, versatile way */
typedef struct {
    float a;
    float b;
} Config;

/* Standard cost function */
float edge_cost(
    int x_1, int y_1, int x_2, int y_2,
    Params * params,
    Config * config
);

/* Standard heuristic function */
float heuristic(
    int x_n, int y_n, int x_g, int y_g,
    Params * params,
    Config * config
);

/* Zero heuristic function for homotopy graph searches */
float zero_heuristic(
    int x_n, int y_n, int x_g, int y_g,
    Params * params,
    Config * config
);

/**************** A* main algorithm ****************/
void A_star_homotopies(
    Params * params,
    Config * config,
    float (*h) (int, int, int, int, Params *, Config * ),
    float (*c) (int, int, int, int, Params *, Config * ),
    float float_tol,
    /* Homotopy parameters */
    double abs_tol,     // decimal tolerance for resolving between homotopy classes
    F_t * F,            // obstacle marker function parameters
    hom_classes_list_t ** target_hom_classes_ptr, // pointer to the target homotopy classes list, will be iteratively updated
    const int max_hom_classes   // total number of homotopic classes within target_hom_classes_ptr to fill 
);

/*** Backtracking ***/
void expand_backtrack(Params *params, hom_class_t * hom_class);
void free_backtrack(hom_class_t * hom_class);



