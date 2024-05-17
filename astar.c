
#include "astar.h"

/* Name of parameter containing path written to .json file */
static const char * output_path_key_format = "a=%.2f, b=%.2f, L=(%.2f + %.2fi)";

/********** static / private function headers *******************/

/****** len ******
 * 
 * This function calculates the absolute value, relative to the map, of the distance between two arbitrary grid coordinates.
 * 
 * Inputs:
 *      Two discrete, adjacent points (x_1,y_1) and (x_2,y_2), in the s-step sized state grid for the robot
 *      Params struct containing the scaling factor s for the grid
 * Outputs:
 *      the resulting float-valued distance between (x_1,y_1) and (x_2,y_2), in map (not grid) units
 * 
 * This function will be applied on a series of adjacent points to calculate path lenths,
 * which will contribute to the path length linear component of the overall cost function (b-coefficient)
*/
static inline float len(
    int x_1, int y_1, int x_2, int y_2,
    Params * params) 
{
    return (params->s * D_ADJACENT(x_1, y_1, x_2, y_2)); // Use s as scaling factor
}

/****** g_image_riemann_sum ******
 * 
 * This function computes the Riemann sum, according to the (uncertainty) function g, 
 * between two arbitrary adjacent points (x_1,y_1) and (x_2,y_2), in the s-step sized state grid for the robot
 * 
 * Inupts:
 *      Two discrete, adjacent points (x_1,y_1) and (x_2,y_2), in the s-step sized state grid for the robot
 *      Params struct containing: 
 *          - scaling factors s and r for the grid and function g respectively
 *          - g_image containing the pre-computed values for g at r-steps resolution, stored in a 2-dimensional matrix
 *          - bounds xr_steps, yr_steps of the matrix containing g_image
 * 
 * Output:
 *      The contribution of taking a path integral of g, between (x_1,y_1) and (x_2,y_2), scaled to map units.
 * 
 * This function will be applied on a series of adjacent points in the domain of g to calculate path integrals numerically,
 * which will contribute to the uncertainty accumulation linear component of the overall cost function (a-coefficient)
 *     
 */
static float g_image_riemann_sum(
    int x_1, int y_1, int x_2, int y_2,
    Params * params) 
{
    // Determine what dimensions changed
    int dx = (x_1 != x_2);
    int dy = (y_1 != y_2);
    // Scale up the discrete s-relative coordinates to the corresponding r-relative indices of g_image
    int x_0 = (params->s_div_r) * x_1;
    int y_0 = (params->s_div_r) * y_1;

    // Now compute the sum of the function at all the (s/r) steps
    float rs = 0.0f;
    for (int i = 0; i < params->s_div_r; i++) {
        /** ensure that we are not accessing g_image out of bounds */
        int y = y_0 + (i * dy);
        int x = x_0 + (i * dx);
        if (x < 0 || x >= params->xr_steps || y < 0 || y >= params->yr_steps) {
            #ifdef ASTDBG
            DEBUG_ERROR("Attempt to access g_image matrix out of bounds");
            #endif
            continue; // skip iteration
        }
        rs += params->g_image[y][x];
    }
    // Multiply by the average lengh of each r-step to get the uncertainty accumulated 
    float u = params->r * D_ADJACENT(x_1, y_1, x_2, y_2) * rs;

    return u;
}

/****** edge_cost ******
 * 
 * This is the resulting cost function combining linearly uncertainty accumulation and path lenth,
 * which are computed by g_image_riemann_sum and len respectively
 * 
 * Inputs:
 *      Two discrete, adjacent points (x_1,y_1) and (x_2,y_2), in the s-step sized state grid for the robot
 *      Params struct containing the fixed environment parameters, passed as input for
 *          - g_image_riemann_sum
 *          - len
 *      Config struct containing the relative weights a and b for uncertainty accumulation and path length accordingly
 * 
 * Output:
 *      The resulting cost between the points (x_1,y_1) and (x_2,y_2) in the grid. Note that this is expressed in map units (floats)
 * 
 * This function will be passed as input for the A* search algorithm, serving as the main cost function for the graph search.
 * 
 */
float edge_cost(
    int x_1, int y_1, int x_2, int y_2,
    Params * params,
    Config * config) 
{
    float u = g_image_riemann_sum(x_1, y_1, x_2, y_2, params); // uncertainty accumulation
    float l = len(x_1, y_1, x_2, y_2, params);  // path length

    // Finally take a linear combination using the fixed parameters a,b
    float c = (config->a * u) + (config->b * l);
    return c;
}

/****** heuristic ******
 * 
 * This is the main heuristic used for the basic (not homotopy class) version of A*.
 * It consists in aiming towards the goal as fast as possible, firstly by exploiting 
 * all the faster diagonal transitions available, and then finishing with the remaining straight ones.
 * For each step, we cancel the Riemann sum factor of the cost function, thereby guaranteeing
 * admissibility and swifter computation. 
 * 
 * Inputs:
 *      current point on the grid (x_n, y_n), and goal point (x_g, y_g)
 *      Config struct holding b coefficient of the cost function (the coefficient determining the weight for path length) 
 *      Params struct holding scaling factor s for the grid
 * 
 * Outputs:
 *      Resulting value of the heuristic between (x_n, y_n) and (x_g, y_g), 
 *          guaranteed to be lower than the lower cost path between the two points
 * 
 * This function is passed as parameter to the A* search algorithm, when this is merely looking for 
 * the lowest cost path (not the more general version keeping track of all the homotopy classes). The tightness of this
 * heuristic significantly accelerates the graph search procedure towards the goal
 */
float heuristic(
    int x_n, int y_n, int x_g, int y_g,
    Params * params,
    Config * config) 
{
    // First determine the total steps in the x and y directions
    int dx = abs(x_g - x_n);
    int dy = abs(y_g - y_n);
    // Then we take as many steps possible diagonally, and the rest in the remaining straight line
    float l = params->s * ((SQRT2 - 1) * MIN(dx,dy) + MAX(dx, dy)); // each step is also of size s
    // We multiply by the parameter b to ensure a tight heuristic compared to the cost function defined above
    float h = config->b * l;
    return h;
}

/****** zero_heuristic ******
 * 
 * This is a trivial zero heuristic, used to turn A* into Dijkstra's algorithm in the case where
 * we are concerned in keeping track of all the possible homotopy classes.
 * In that case, we want to guarantee as much uniform and unbiased coverage as possible of the environment.
 * 
 * The dummy arguments make the function signature conform to the A* algorithm main function, 
 * thereby making the function reusable with different heuristic
 */
float zero_heuristic(
    int x_n, int y_n, int x_g, int y_g,
    Params * params,
    Config * config)
{
    return 0.0f;
}


/************ A_star_homotopies static helper functions *************/

/****** expand_backtrack *******
 * 
 * This function uses the information stored within the hom_class_t by the A* procedure
 * to carefully backtrack and allocate the path of the homotopy class. It is convenient because
 * it allows us to arbitrarily determine when a path is worth allocating and expanding at any time
 * we want to do so, by just using lightweight information stored in the homotopy class struct.
 * 
 * Inputs:
 *      Params struct for the given environment
 *      homotopy class struct pointer for which we want to expand and store the path
 * 
 * Outputs:
 *      The nested struct member hom_class->backtrack is updated with all the information on the path
 * 
 * NOTE: Caller must then call the corresponding clean-up function free_backtrack 
 *       which carefully free's all the memory malloc'd by this function
 * 
*/
void expand_backtrack(Params *params, hom_class_t * hom_class) {
    
    int edge_number     = 0;       // number of discrete edges
    float abs_len_cumul = 0.0f;    // cumulative length of path, scaled up
    float riem_cumul    = 0.0f;    // cumulative rieamnn sum for g_image buildup, scaled up
    int x_curr, y_curr;            // backtracking coordinates
    int x_next, y_next;         

    // First, find the length of the path by traversing from goal to start
    hom_class_t * curr_hom_class = hom_class;
    while (curr_hom_class != NULL) {
        edge_number++;
        curr_hom_class = curr_hom_class->parent;
    }

    // Allocate backtracking path structure from here 
    if (hom_class->backtrack == NULL) {
        hom_class->backtrack = (Backtrack_Path *) malloc (sizeof(Backtrack_Path));
    }

    Backtrack_Path * bt = hom_class->backtrack;

    // Set length for path struct
    bt->total_edges = edge_number;

    // Allocate memory for the points list
    bt->point_list = malloc(sizeof(float *) * edge_number);
    for (int i = 0; i < edge_number; i++) {
        bt->point_list[i] = malloc(sizeof(float) * 2); // Each point has two coordinates (x, y)
        if (!bt->point_list[i]) {
            fprintf(stderr, "Memory allocation failed for point_list[%d].\n", i);
            return;
        }
    }

    // Backtrack from goal to start and store the path in reverse
    curr_hom_class = hom_class;
    for (int i = edge_number - 1; i >= 0; i--) {

        // Extract coordinates from current vertex and parent if there is one
        x_curr = x_next = curr_hom_class->endpoint_vertex->x_s;
        y_curr = y_next = curr_hom_class->endpoint_vertex->y_s;
        if (curr_hom_class->parent != NULL) {
            x_next = curr_hom_class->parent->endpoint_vertex->x_s;
            y_next = curr_hom_class->parent->endpoint_vertex->y_s;
        }

        // Re-compute length and Riemann sum as they were originally computed, this time without the constants a,b
        abs_len_cumul  +=                 len(x_next, y_next, x_curr, y_curr, params);
        riem_cumul     += g_image_riemann_sum(x_next, y_next, x_curr, y_curr, params);

        // Record points in the array containing the path
        bt->point_list[i][0] = params->x_min + (x_curr * params->s); // Convert x_s back to x
        bt->point_list[i][1] = params->y_min + (y_curr * params->s); // Convert y_s back to y

        #ifdef ASTDBG
        printf("Backtracking path point[%d]: (%.2f, %.2f)\n", i, bt->point_list[i][0], bt->point_list[i][1]);
        #endif

        curr_hom_class = curr_hom_class->parent;    // iterate backwards through vertices on optimal path
    }

    // Set elements for the path
    bt->g_image_riemann_tot = riem_cumul;
    bt->absolute_length     = abs_len_cumul;

}

/****** write_path *******
 * 
 * This function writes the points of the output path corresponding to some 
 * cost coefficients a,b and homotopy class to the .json output file,
 * so that it can be parsed and rendered by graphing software (i.e. matplotlib)
 * 
 * Inputs:
 *      homotopy class object containing the path
 *      config struct containing a,b coefficients for cost function on which path was evaluated
 * 
 * Outputs:
 *      Path points written out as a matrix to ./params.json file (name hardcoded)
 * 
 * NOTE: The caller must first invoke expand_backtrack(params, hom_class)
 *       to ensure that the backtrack path is both computed and stored in hom_class->backtrack,
 *       and, preferably, invoke write_path before free_backtrack which deletes the path
 * 
 * Usage example:
 * 
 *  expand_backtrack(params, hom_class);
 *  // do something with the path
 *  write_path(hom_class, config);
 *  // do something with the path
 *  free_backtrack(hom_class);
 * 
 */
void write_path(hom_class_t * hom_class, Config * config) {

    Backtrack_Path * bt = hom_class->backtrack;
    if (bt == NULL) return;
    const int str_size = 256;
    char * Lval_string = (char*) malloc(str_size);

    // Format the entry for the path in the output .json file
    snprintf(
        Lval_string, str_size, 
        output_path_key_format, 
        config->a, config->b, creal(hom_class->Lval), cimag(hom_class->Lval)
    );
    #ifdef PATH_WRITE_DBG
    printf("Writing path to ./params.json file with key: %s\n", Lval_string);
    #endif
    write_json("./params.json", Lval_string, bt->point_list, bt->total_edges - 1, 2);
    free(Lval_string);
}

/****** free_backtrack ******
 * 
 * The corresponding clean-up routine to expand_backtrack.
 * 
 * Inputs:
 *      homotopy class struct whose backtrack data member we want to free
 * 
 * Outputs:
 *      The function carefully deallocates all the memory taken up by
 *      hom_class->backtrack 
*/
void free_backtrack(hom_class_t * hom_class) {

    Backtrack_Path * bt = hom_class->backtrack;

    // Verify that both path struct and internal point list are accessible 
    if (bt == NULL || bt->point_list == NULL || bt->total_edges == 0) {
        DEBUG_ERROR("free_point_array: opt_path = NULL OR opt_path->point_list = NULL OR opt_path->l = 0");
        return;
    }

    // Edge iteration through array
    for (int i = 0; i < bt->total_edges; i++) {
        // Verify that we are actually free'ing
        if (bt->point_list[i] == NULL) {
            DEBUG_ERROR("free_point_array: unexpected NULL value for opt_path->point_list[i]");
        }
        // Free the point
        free(bt->point_list[i]);
    }

    // Finally, free the backtrack structure pointer itself
    free(bt); 
}


/****** A_star_goal_check ******
 * 
 * This is a key helper function for the homotopy class generalization of A*.
 * It essentially checks if the just-dequeued state is the goal,
 * and then updates in place the list holding the target homotopy classes 
 * if we have found a target homotopy class. 
 * 
 * Inputs:
 *      goal grid coordinates (x_goal_step, y_goal_step)
 *      standard Params struct, obstacle marker struct F, tolerance for float comparisons abs_tol,
 *      number of target homotopy classes to fill max_hom_classes
 *      Collaterally changeable inputs: 
 *          - pointer to the list of target homotopy classes target_hom_classes_ptr
 *          - pointer to current number of successfully filled homotopy classes filled_hom_classes
 * 
 * Outputs:
 *      If we have identified a homotopy class to the goal within the list of target homotopy classes,
 *      we update its entry in the input list, and increase the count of successfully filled target homotopy classes.
 *      If we have a NULL input list, we fill it with the list corresponding to the goal vertex.
 *      Returns true iff we have reached the goal
 * 
 * NOTE: This function writes directly to the elements of the input list of homotopy classes. 
 *       While this may be useful when using the list itself as iterator for the different homotopy classes,  
 *       it has to be handled with care, as once a hom_classes_list_t is fed into A_star_homotopies it is guaranteed to be changed. 
 */
static bool A_star_goal_check(
    Params * params,  double abs_tol, F_t * F,
    hom_class_t * hom_class, int x_goal_step, int y_goal_step,
    hom_classes_list_t ** target_hom_classes_ptr, int * filled_hom_classes, int max_hom_classes) 
{
    int x_c = hom_class->endpoint_vertex->x_s;
    int y_c = hom_class->endpoint_vertex->y_s;

    if ((x_c != x_goal_step || y_c != y_goal_step)) {
        return false; // goal not found 
    } 

    #ifdef ASTDBG
    printf(
        "Goal reached at (%d, %d) from L-value (%.2f + %.2fi) with total path cost (g_score): %f\n", 
        x_c, y_c, creal(hom_class->Lval), cimag(hom_class->Lval), hom_class->g_score
    );
    #endif
    /** Reconstruct & assign path **/
    expand_backtrack(params, hom_class);

    if (*target_hom_classes_ptr == NULL) {
        // once the target number is reached have been reached, we simply fill in with the goal hom_classes_list 
        if (++(*filled_hom_classes) == max_hom_classes) {
            *target_hom_classes_ptr = hom_class->endpoint_vertex->hom_classes;
        }
    } else {
        // Search to see if we have discovered an Lvalue that is within the targets
        hom_class_t * target_hom_class = hom_class_get(*target_hom_classes_ptr, hom_class->Lval, abs_tol);
        // If we find some target homotopy class, we replace it with the newly discovered one
        // In the case of pre-filled *target_hom_classes_ptr we thus keep going until we have filled up the desired number of hom_classes
        if (target_hom_class != NULL) {
            free_backtrack(target_hom_class);               // first clean up the old homotopy class's path struct
            hom_class_t * next = target_hom_class->next;    /** CRITICAL: preserve pointer to next element, else you break *target_hom_classes_ptr list! */
            *(target_hom_class) = *(hom_class);             // copy the new value, inluding the just-allocated backtrack, into the target value
            target_hom_class->next = next;                  // reinstate the previous next that got nullified by the copy operation
            (*filled_hom_classes)++;                        // Mark the extra homotopy class added to the target list
        }
    }

    return true;    // goal was found and parameters updated accordingly if we make it here
}

/****** print_homotopy_classes ******
 * 
 * This is a debugging function, printing out each time the goal state is dequeued:
 *      A* algorithm's key inputs and outputs
 *      Details of most recently discovered homotopy class into the goal
 *      Path integrals (Lvalues) for all other homotopy classes into the goal  
*/
static void print_homotopy_classes(
    Params * params, Config * config, hom_class_t * hom_class, float float_tol, 
    hom_classes_list_t * B, double abs_tol, int expanded_states, int open_set_size) 
{
    #ifdef AST_HC_DBG
    char * B_str = complex_list_to_string(B);
    printf(
        "\nA* found new homotopic class to goal:\n"
        "\tL-value: (%.2f + %.2fi)\n"
        "\tcumulative length: %f\n" 
        "\tg_image Riemann sum (accumulated uncertainty): %f\n"
        "\nCurrent status of the graph search:\n"
        "\tTotal states expanded: %d\n"
        "\tHomotopy classes currently evaluated for the goal vertex: %s\n"
        "\tCurrent size of the open set (enqueued states): %d\n",
    creal(hom_class->Lval), cimag(hom_class->Lval), hom_class->backtrack->absolute_length,
    hom_class->backtrack->g_image_riemann_tot, expanded_states, B_str, open_set_size
    );
    free(B_str);
    #endif
}

/****** initialize_vertices ******
 * 
 * This helper handles the memory allocation and element initialization for each
 * vertex in the grid for A_star_homotopies. The vertices are deliberately cleared 
 * at each algorithm's run to ensure that all the flags on the states are properly reset,
 * and the main purpose of abstracting this method is to reduce clutter on the main algorithm.
 * 
 * Inputs:
 *      Params struct containing main information about the grid dimensions
 * 
 * Output:
 *      2-dimensional matrix of vertex pointers, vertex_t *** S, 
 *      where S[ys][xs] is the pointer to the vertex struct at the discrete grid coordinate (xs, ys)
 * 
 * NOTE: The large matrix returned must be carefully deallocated by calling delete_vertices
 *       This vertex initialization is only specific to A_star_homotopies, and would thus incorrectly 
 *       initialize the vertices if used for the other version of A*
 */
static vertex_t *** initialize_vertices(Params * params) {

    vertex_t *** S = malloc(params->ys_steps * sizeof(vertex_t **));  // row pointers
    if (!S) {
        fprintf(stderr, "Failed to allocate memory for grid rows.\n");
        exit(EXIT_FAILURE);
    }
    for (int ys = 0; ys < params->ys_steps; ys++) {
        S[ys] = (vertex_t **) malloc(params->xs_steps * sizeof(vertex_t *)); // allocate column
        if (!S[ys]) {
            fprintf(stderr, "Failed to allocate memory for grid columns at row %d.\n", ys);
            // Free previously allocated memory before exiting
            for (int j = 0; j < ys; j++) {
                free(S[j]);
            }
            free(S);
            exit(EXIT_FAILURE);
        }
        for (int xs = 0; xs < params->xs_steps; xs++) {
            // Initialize vertex located at (xs, ys) to default parameters
            vertex_t * new_vertex = malloc(sizeof(vertex_t));
            new_vertex->x_s = xs;
            new_vertex->y_s = ys;
            new_vertex->hom_classes = hom_classes_list_new(); // intialize empty list of Lvalues for each vertex
            // Save malloc'd vertex to matrix of vertex pointers 
            S[ys][xs] = new_vertex;
        }
    }
    return S;
}

/******* delete_vertices ********
 * 
 * This is the corresponding clean-up routine for initialize_vertices,
 * and must always be called to deallocate the memory allocated by the latter.
 * 
 * Inputs:
 *      vertex pointer 2-d matrix S to deallocate, Params struct containing information on grid size
 *      (x_goal_step, y_goal_step) grid coordinates for the goal
 * 
 * Output:
 *      The function free's all the memory allocated by initialize_vertices,
 *      except for the homotopy class linked list corresponding to the goal vertex.
 * 
 * NOTE: This linked list is crucially preserved, as it contains the information that gets stored
 *       by A_star_goal_check when writing the target homotopy classes into target_hom_classes_ptr,
 *       which is thus returned as an output of the main A_star_homotopies algorithm.
 *       The caller of A_star_homotopies is responsible for later deallocating the list of homotopy classes
 *          (for instance by recalling  A_star_homotopies) on it which re-writes on it:
 *  
 */
void delete_vertices(vertex_t *** S, Params * params, int x_goal_step, int y_goal_step) {

    for (int ys = 0; ys < params->ys_steps; ys++) {
        for (int xs = 0; xs < params->xs_steps; xs++) {
            vertex_t * curr_vert = S[ys][xs];
            // This free's all the dynamically allocated homotopy classes, except for the goal vertex
            if (xs != x_goal_step || ys != y_goal_step) {
                free_hom_classes_list(curr_vert->hom_classes);
            } 
            free(curr_vert); // free vertex * within matrix
        }
        free(S[ys]); // free row pointer (vertex **)
    } 
    free(S);

}

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
 *      The list of target homotopy classes is updated with the detected homotopy classes into the goal,
 *      as explained in the sub-routine A_star_goal_check.
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
 *  
*/
void A_star_homotopies(
    Params * params,
    Config * config,
    float (*h) (int, int, int, int, Params *, Config * ),
    float (*c) (int, int, int, int, Params *, Config * ),
    float float_tol,
    /* Homotopy parameters */
    double abs_tol,     
    F_t * F,            
    hom_classes_list_t ** target_hom_classes_ptr, 
    const int max_hom_classes  
    ) 
{
    int expanded_states = 0;    // we consider a state explored whenever it is added to the closed set
    int filled_hom_classes = 0; // tracks the number of homotopic classes that have been filled

    #ifdef ASTCLCK
    struct timeval start, end;
    long seconds, microseconds;
    double elapsed;
    gettimeofday(&start, NULL); // Start clock 
    #endif

    // Check first that we ahve properly computed the image of g
    if (params->g_image == NULL) {
        DEBUG_ERROR("params->g_image = NULL");
        return;
    }

    // Convert start and goal coordinates to the relative s-steps
    const int x_start_step = (int) (params->x_s - params->x_min) / params->s;
    const int x_goal_step  = (int) (params->x_g - params->x_min) / params->s;
    const int y_start_step = (int) (params->y_s - params->y_min) / params->s;
    const int y_goal_step  = (int) (params->y_g - params->y_min) / params->s;    

    /* This dynamic grid allocation within A* is deliberate to ensure that the states are perfectly blank at each run of the algorithm */
    vertex_t *** S = initialize_vertices(params);

    // Initialize starting homotopy class object
    vertex_t * start_vertex = S[y_start_step][x_start_step];
    hom_class_t * start_hom_class = (hom_class_t *) malloc(sizeof(hom_class_t));
    start_hom_class->Lval = CMPLX(0.0, 0.0);
    start_hom_class->next = NULL;   
    start_hom_class->parent = NULL;
    start_hom_class->g_score = 0.0f;
    start_hom_class->f_score = h(
        x_start_step, y_start_step, x_goal_step, y_goal_step, params, config
    );
    start_hom_class->is_evaluated = false;
    start_hom_class->endpoint_vertex = start_vertex;
    start_hom_class->minheap_node = NULL;
    start_hom_class->backtrack = NULL;

    // Insert starting homotopy class object into the list of homotopy classes into the start vertex 
    insert_hom_class(start_vertex->hom_classes, start_hom_class, abs_tol);  

    // Initialize open set with desired accessor functions and data type for homotopy classes
    open_set_t * open_set = open_set_new(
        hom_class_get_f_score,
        hom_class_set_minheap_node
    ); /** CAREFUL: not checking for NULL pointer return here */

    // Now insert the starting homotopy class object into the open set as a generic void *
    insert_sorted(open_set, (void *) start_hom_class);

    // Start main iteration for the A* algorithm
    while (!open_set_is_empty(open_set)) {
        hom_class_t * curr_hom_class = dequeue_min(open_set);
        curr_hom_class->is_evaluated = true; // dequeue homotopy class & add it to the closed set
        expanded_states++;

        // keep the current x,y grid coordinates as local variables for clarity
        int x_c = curr_hom_class->endpoint_vertex->x_s;
        int y_c = curr_hom_class->endpoint_vertex->y_s;

        // Here we would generally check for the goal state and break 
        if (A_star_goal_check(
                params, abs_tol, F, curr_hom_class, x_goal_step, y_goal_step, 
                target_hom_classes_ptr, &filled_hom_classes, max_hom_classes)) 
        {
            print_homotopy_classes(
                params, config, curr_hom_class, float_tol, curr_hom_class->endpoint_vertex->hom_classes, 
                abs_tol, expanded_states, open_set_size(open_set)); // show the current path
            write_path(curr_hom_class, config); // write the path to ./params.json
            // Break here if we have reached the desired number of homotopy classes
            if (filled_hom_classes == max_hom_classes) {
                #ifdef AST_HC_DBG
                char * hom_classes_str = complex_list_to_string(*target_hom_classes_ptr);
                printf(
                    "A* filled %d homotopy classes with Lvalues \n\t%s.\nBreaking main loop...\n", 
                    filled_hom_classes, hom_classes_str);
                free(hom_classes_str);
                #endif
                break;
            }
            // free_backtrack(curr_hom_class); // leave path allocated here
        }

        /** Now start generating outneighbors, considering that 
         * 1) If we have a g_image under FLOAT_TOL, signaling a zero value, the state is inaccessible
         * 2) If the boundaries go beyond matrix capacity, the state is inaccessible
        */
        for (int dx = -1; dx <= 1; dx++) {  // Loop over x offsets
        for (int dy = -1; dy <= 1; dy++) {  // Loop over y offsets
            // Note that the same state is automatically skipped by the later is_evaluated condition
            int x_n = x_c + dx; // fetch step coordinates of neighbouring state
            int y_n = y_c + dy;
            // Check accessibility conditions for the neighbor
            if (!is_accessible(params, x_n, y_n, float_tol)) {
                continue; 
            }
            // Now that we know that the vertex can be accessed, we can extract it 
            vertex_t * neighb_vertex = S[y_n][x_n];
            // Compute L-step for the current transition
            Complex L_step = get_Lval(F, x_c, y_c, x_n, y_n);
            // Add the L-step to the L-value for the current homotopy class to obtain the neighboring L value
            Complex neighb_Lval = curr_hom_class->Lval + L_step;
            // Search for the homtopy class corresponding to the given endpoint vertex and the neighboring L value
            hom_class_t * neighb_hom_class = hom_class_get(neighb_vertex->hom_classes, neighb_Lval, abs_tol);

            /** LOOPCHECKS: would go here, but for now we use sufficiently low max_hom_classes to avoid loops  */

            // If we haven't found it, then we have just discovered a new homotopy class, and thus we initialize it 
            if (neighb_hom_class == NULL) {
                neighb_hom_class = (hom_class_t *) malloc(sizeof(hom_class_t));
                neighb_hom_class->Lval = neighb_Lval;
                neighb_hom_class->next = NULL; 
                neighb_hom_class->parent = NULL;
                neighb_hom_class->g_score = INFINITY;
                neighb_hom_class->f_score = INFINITY;
                neighb_hom_class->is_evaluated = false;
                neighb_hom_class->endpoint_vertex = neighb_vertex;
                neighb_hom_class->minheap_node = NULL;
                neighb_hom_class->backtrack = NULL; 
                // Insert it into the list of homotopy classes for the neighbor vertex 
                insert_hom_class(neighb_vertex->hom_classes, neighb_hom_class, abs_tol);
            }

            // Check if the homotopy class is already in the closed set 
            if (neighb_hom_class->is_evaluated) {
                continue;
            }
            // Calculate tentative g_score for homotopy class
            float tent_g = curr_hom_class->g_score + c(
                x_c, y_c, x_n, y_n, params, config
            );
            // If no better path found for this homotopy class, continue
            if (tent_g >= neighb_hom_class->g_score) {
                continue;
            }
            // If we are here, then we have found a new shortest path for this homotopy class
            #ifdef ASTDBG
            printf("Updating some homotopy class at vertex (%d, %d) from g_score: %f to g_score: %f\n", neighb_vertex->x_s, neighb_vertex->y_s, neighb_vertex->g_score, tent_g);
            #endif
            neighb_hom_class->parent  = curr_hom_class;
            neighb_hom_class->g_score = tent_g;
            neighb_hom_class->f_score = tent_g + h(x_n, y_n, x_goal_step, y_goal_step, params, config);

            /** CRITICAL: If the homotopy class is already in the min heap, dedrease key, otherwise add it */
            if (neighb_hom_class->minheap_node) {
                decrease_key(open_set, neighb_hom_class->minheap_node, neighb_hom_class->f_score);
            } else {
                insert_sorted(open_set, neighb_hom_class);
            }
        }
        }
    }

    // Add timestamp under general print statement
    #ifdef ASTCLCK
    CALCULATE_ELAPSED_TIME(start, end, elapsed);
    printf(
        "\nA* terminated with the inputs:\n"
        "\t(x_g, y_g) = (%f, %f)\n"
        "\tr = %f, s = %f\n"
        "\ta = %f, b = %f\n"
        "\tfloat tolerance = %f\n"
        "\tabsolute tolerance for homotopy classes: %f\n"
        "\tTotal states expanded: %d\n"
        "\tCurrent size of the open set (enqueued states): %d\n"
        "\nTime elapsed since invoking A_star_homotopies:\n\t%.5f seconds\n\n",
    params->x_g, params->y_g, params->r, params->s, config->a, config->b, 
    float_tol, abs_tol, expanded_states, open_set_size(open_set), elapsed
    );
    #endif

    // Clean up 
    #ifdef ASTDBG
    printf("Cleaning up resources...\n");
    #endif

    free_open_set(open_set); 
    /** IMPORTANT: Grid deallocation, saving the hom_classes_list for the goal vertex as this is the collateral output */ 
    delete_vertices(S, params, x_goal_step, y_goal_step);
    
}


/******* Work in progress:  unseuccessful loop testing functions ********
 * 
 * Below are some functions that I tried to implement to detect loop formation
 * dynamically within A_star_homotopies, a phenomenon occurring after generating 
 * a number of homotopy classes roughly twice the number of obstacles, due to the fact
 * that loops have nonzero path integrals, thereby generating new homotopy classes.
 * 
 * However, these functions had various shortcomings hence have not been integrated 
 * in A_star_homotopies. The problem has been tackled by limiting the number of generated
 * homtopy classes by setting an appropriate value for max_hom_classes.
 * This approach is not perfect, as it does not exhaust non-looping homotopy classes,
 * and does never lead A* to termination with all the vertices dequeued from the open set,
 * but it gives good results and is more efficient since we can skip all the tedious loop-checking conditions.
 * 
*/

static bool simple_check_loop(hom_class_t * curr_hom_class, int x_n, int y_n)
{
    /** LOOPTEST: Backtracing (too inefficient) */
    bool loop = false;
    hom_class_t * curr_hom_class_it = curr_hom_class->parent;
    while (curr_hom_class_it != NULL) {
        if (
            curr_hom_class_it->endpoint_vertex->x_s == x_n &&
            curr_hom_class_it->endpoint_vertex->y_s == y_n
        ) {
            loop = true;
            break;
        }
        curr_hom_class_it = curr_hom_class_it->parent;
    }

    return loop;
}

static bool check_loop(
    Params * params, F_t * F, vertex_t * neighb_vertex, 
    int x_c, int y_c, int x_n, int y_n,
    Complex neighb_Lval, float abs_tol) 
{

   /** LOOPTEST: Clockwise vs anticlockwise (too inefficient)
     * Also this analysis doesn't work since any loop could always
     * come in clockwise or anticlockwise forms 
    */
    bool loop = false;
    Complex * A = get_residues(F);
    hom_class_t * nbr_hom_class_it = neighb_vertex->hom_classes->head;
    
    // Iteration through all homotopy classes into the neighbor
    while (!loop && nbr_hom_class_it != NULL) {
        // Iteration through the residue for each obstacle
        for (int i = 0; i < params->num_obstacles; i++) {
            // Key comparison to determine if we have potentially detected a loop
            Complex res_off = 2*M_PI*CMPLX(0.0, 1.0) * A[i];    // 2(pi)iRes(\zeta_i)
            if (
                !cmplx_compare(nbr_hom_class_it->Lval + res_off, neighb_Lval, abs_tol) &&
                !cmplx_compare(nbr_hom_class_it->Lval - res_off, neighb_Lval, abs_tol)
            ) {
                continue; // skips directly to the next
            }
            /** Now that we have detected a potential loop, we want to use the following information:
             * 1) vector (x_c, y_c) -> (x_n, y_n)
             * 2) center of identified obstacle
             * to determine whether the potential loop would be 
             * - anticlockwise: --> test cmplx_compare(nbr_hom_class_curr->Lval + res_off, neighb_Lval, abs_tol)
             * - clockwise:     --> test cmplx_compare(nbr_hom_class_curr->Lval - res_off, neighb_Lval, abs_tol) 
             * If this final test fails, then we have another disjoint homotopic class without loops,
             * which goes around the same obstacle 
            */

            float x0 = creal((get_obstacle_markers(F))[i]);  // extract corresponding obstacle center (x0, y0)
            float y0 = cimag((get_obstacle_markers(F))[i]);           

            float xc = params->x_min + (x_c * params->s);    // extract map coordinates for 
            float yc = params->y_min + (y_c * params->s);    // current (xc, yc) and neighbor (xn, yn)
            float xn = params->x_min + (x_n * params->s);
            float yn = params->y_min + (y_n * params->s);

            float theta_c = atan2(yc - y0, xc - x0);    // compute relative angles with respect to elliptical centre
            float theta_n = atan2(yn - y0, xn - x0);

            // Normalize angles to be between 0 and 2*pi
            theta_c = fmod(theta_c + 2.0*M_PI, 2.0*M_PI);
            theta_n = fmod(theta_n + 2.0*M_PI, 2.0*M_PI);  

            // set mult flag to +1 if anticlockwise or -1 if clockwise
            double mult = 0.0f;
            if (theta_n > theta_c  && (theta_n - theta_c) < M_PI)  mult =  1.0f;
            if (theta_n > theta_c  && (theta_n - theta_c) >= M_PI) mult = -1.0f; 
            if (theta_n <= theta_c && (theta_c - theta_n) < M_PI)  mult = -1.0f;
            if (theta_n <= theta_c && (theta_c - theta_n) >= M_PI) mult =  1.0f;

            // Now perform the final check taking rotation direction into account
            if (cmplx_compare(nbr_hom_class_it->Lval + (mult * res_off), neighb_Lval, abs_tol)) {
                loop = true;
                break;
            }
        }
        nbr_hom_class_it = nbr_hom_class_it->next;  // proceed to the next homotopic class
    }
    return loop;
}


/*********** Unit testing area *****************
 * Demonstrating some use-cases and initializations required 
 * to run the main algorithm
*/

#ifdef AST_UT

/* Main function & testing from here below */
/* Takes as argument the path to the json file to read parameters from */
int main(int argc, char *argv[]) {

    #ifdef ASTCLCK
    struct timeval start, end;
    long seconds, microseconds;
    double elapsed;
    gettimeofday(&start, NULL); // Start clock 
    #endif

    const float float_tol = 0.1f;
    char * input_json_path;
    char * endptr;  // Pointer to check conversion status
    float a, b;
    Config * config;
    Params * params;

    // Validate input 
    if (argc != 4) {  // Check if the number of arguments is correct
        fprintf(stderr, "Usage: %s input_json_path a b\n", argv[0]);
        return 1;  // Return an error code
    }

    // Allocate memory for the input path
    input_json_path = (char *) malloc(strlen(argv[1]) + 1);
    if (input_json_path == NULL) {
        fprintf(stderr, "Memory allocation failed\n");
        return 1;
    }

    // Copy the input argument to our new variable
    strcpy(input_json_path, argv[1]);

    #ifdef ASTDBG
    printf("JSON input path: %s\n", input_json_path);
    #endif

    // Convert first float
    a = strtod(argv[2], &endptr);
    if (*endptr != '\0') {  // Check if conversion consumed the whole input
        fprintf(stderr, "Invalid float for parameter a: %s\n", argv[2]);
        return 1;
    }
    // Convert second float
    b = strtod(argv[3], &endptr);
    if (*endptr != '\0') {  // Check if conversion consumed the whole input
        fprintf(stderr, "Invalid float for parameter b: %s\n", argv[3]);
        return 1;
    }

    #ifdef ASTDBG
    printf("Running A-star with parameters a = %f, b = %f\n", a, b); 
    #endif

    #ifdef ASTCLCK
    CALCULATE_ELAPSED_TIME(start, end, elapsed);
    printf("\nInvoking load_json %.5f seconds from start of astar main() program execution\n", elapsed);
    #endif

    // Extract parameters
    params = load_json(input_json_path);

    if (params == NULL) {
        DEBUG_ERROR("extract_parameters returned NULL, exiting...");
        exit(1);
    }

    /* Homotopic addition: Extract obstacle marker parameters from parameters, and define blocked homotopy classes */
    F_t * F = initialize_obstacle_marker_func_params(params, float_tol);
    // hom_classes_list_t * B = Lval_list_new();    // start empty here, allowing all homotopy classes
    // Hard-coding some previously identified homotopy classes that we want to try to block:
    // insert_Lval(B, CMPLX(-19.80, -61.57), 0.1);

    // Test if we actually have the right matrix by printing it to the file under other key
    #ifdef ASTDBG
    // write_json(input_json_path, "g_image_test", params->g_image, params->yr_steps, params->xr_steps); 
    #endif

    config = malloc(sizeof(Config));
    config->a = a; 
    config->b = b;

    #ifdef ASTDBG
    printf("Initializing A* with grid size: %dx%d\n", params->xs_steps, params->ys_steps);
    #endif

    #ifdef ASTCLCK
    CALCULATE_ELAPSED_TIME(start, end, elapsed);
    printf("\nInvoking A* %.5f seconds from start of astar main() program execution\n", elapsed);
    #endif

    /** Here is an example usage of the A* algorithm homotopy extension:
     * All we need is initialize the list of target homotopy classes to NULL,
     * set a number of desired homotopy classes that we want to fill,
     * (this will typically be around the number of obstacle markers to avoid loops)
     * and let the algorithm run for different interations, at different costs, 
     * each time refining this list
     */
    const int num_iterations = 6;   // A* runs
    const int max_hom_classes = params->num_obstacles + (params->num_obstacles / 2); // how many homotopy classes we want A* to keep track of
    hom_classes_list_t * target_hom_classes = NULL; // list of homotopic classes, initialized to NULL

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

    // Write the path to the .json file
    // write_json(input_json_path, output_path_param_name, opt_path->point_list, opt_path->l, 2); 

    // Clean up
    free(input_json_path);
    free(config);
    // free_Lval_list(B);
    delete_obstacle_marker_func_params(F);
    /** TODO: Find ways to clean up the params struct; probably manual thorough cleanup required for grids */
    free(params);

    return 0;
}

#endif



