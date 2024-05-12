
#pragma once

#include "parser.h"
#include "utils.h"
#include "vertex.h"
#include "open_set.h"
#include "commonmacros.h"
#include "obstacle_marker.h"
#include "Lval_list.h"

/* Keep configuration parameters in a wrappable, versatile way */
typedef struct {
    float a;
    float b;
} Config;

/* Details of path returned by A* */
typedef struct {
    int l;
    float ** point_list;
    float len;
    float g_image_riemann_tot;
    /* Homotopy addition */
    Complex Lval;

} Optimal_Path;

/* Given already-malloc'd path struct with SET length, deallocates internal point list */
void free_point_array(Optimal_Path * opt_path); 

/* Standard cost function */
float edge_cost(
    int x_1, int y_1, int x_2, int y_2,
    Params * params,
    Config * config
);

/* Standar heuristic function */
float heuristic(
    int x_n, int y_n, int x_g, int y_g,
    Params * params,
    Config * config
);

/**************** A* main algorithm ****************/
void A_star(
    Params * params,
    Config * config,
    Optimal_Path * opt_path,
    float (*h) (int, int, int, int, Params *, Config * ),
    float (*c) (int, int, int, int, Params *, Config * ),
    float float_tol,
    /* Homotopy parameters */
    Lval_list_t * B,    // blocked homotopy classes for the search
    double abs_tol, // decimal tolerance for resolving between homotopy classes
    F_t * F             // obstacle marker
);


