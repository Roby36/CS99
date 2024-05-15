
#include "astar.h"

/* Name of parameter containing path written to .json file */
static const char * output_path_param_name = "A_star_output_path";

/** Cost function implementation 
 * Now we have two discrete, adjacent points (x_1,y_1) and (x_2,y_2), in the s-step sized state grid for the robot.
 * Supposing that we have data, stored in a g_image matrix, at smaller r-step intervals, we are going to 
 * compute the Riemann sum for the integral between the two adjacent points on the grid by accesing matrix entries,
 * and linearly combine it with path length to compute overall cost function between two adjacent states in the grid. 
*/
static inline float len(
    int x_1, int y_1, int x_2, int y_2,
    Params * params) 
{
    return (params->s * D_ADJACENT(x_1, y_1, x_2, y_2)); // Use s as scaling factor
}

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

/* Heuristic function between any point (x_n, y_n) and the goal (x_g, y_g) */
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

/* Try defining here a "zero heuristic" for Dijkstra, more appropriate to explore several homotopy classes spread across the environment */
float zero_heuristic(
    int x_n, int y_n, int x_g, int y_g,
    Params * params,
    Config * config)
{
    return 0.0f;
}

/* A* helpers */

static bool A_star_goal_check(
    Params * params, /* hom_classes_list_t * B, */ double abs_tol, F_t * F,
    hom_class_t * hom_class, int x_goal_step, int y_goal_step) 
{
    int x_c = hom_class->endpoint_vertex->x_s;
    int y_c = hom_class->endpoint_vertex->y_s;

    if ((x_c == x_goal_step && y_c == y_goal_step)
        /* For now no homotopy constraints checks 
        && (B == NULL || !Lval_list_contains(B, current_vertex->L_values, abs_tol) )
        */
        ) {
            #ifdef ASTDBG
            printf(
                "Goal reached at (%d, %d) from L-value (%.2f + %.2fi) with total path cost (g_score): %f\n", 
                x_c, y_c, creal(hom_class->Lval), cimag(hom_class->Lval), hom_class->g_score
            );
            #endif
            /** Reconstruct & assign path **/
            expand_backtrack(params, hom_class);
            return true;    // goal was found
    }
    return false;   // goal not found 
}

static void print_path(
    Params * params, Config * config, hom_class_t * hom_class, float float_tol, 
    hom_classes_list_t * B, double abs_tol, int expanded_states) 
{
    char * B_str = complex_list_to_string(B);
    printf("\nA* found path to goal with inputs:\n"
        "\t(x_g, y_g) = (%f, %f)\n"
        "\tr = %f, s = %f\n"
        "\ta = %f, b = %f\n"
        "\tHomotopy classes B: %s\n"
        "\tfloat tolerance = %f\n"
        "\tabsolute tolerance for homotopy classes: %f\n"
        "\nOutput path:\n"
        "\tcumulative length: %f\n" 
        "\tg_image Riemann sum (accumulated uncertainty): %f\n"
        "\tL-value: (%.2f + %.2fi)\n"
        "\tTotal states expanded: %d\n",
    params->x_g, params->y_g, params->r, params->s, config->a, config->b, B_str,
    float_tol, abs_tol, hom_class->backtrack->absolute_length, hom_class->backtrack->g_image_riemann_tot, 
    creal(hom_class->Lval), cimag(hom_class->Lval), expanded_states
    );
    free(B_str);
}

/**************** Main A* algorithm implementation ****************/
void A_star(
    Params * params,
    Config * config,
    float (*h) (int, int, int, int, Params *, Config * ),
    float (*c) (int, int, int, int, Params *, Config * ),
    float float_tol,
    /* Homotopy parameters */
    // hom_classes_list_t * B,    // blocked homotopy classes for the search
    double abs_tol,     // decimal tolerance for resolving between homotopy classes
    F_t * F             // obstacle marker function parameters
    ) 
{
    int expanded_states = 0;    // we consider a state explored whenever it is added to the closed set

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

            /** NOTE:
            * In the case of a homotopic class based search, vertices are merely lists of homotopy classes
            * corresponding to each point of the grid, and hence these properties are removed from vertices 
            * as they are specific to homotopy class objects:
            *
                new_vertex->parent = NULL;
                new_vertex->g_score = INFINITY;
                new_vertex->f_score = INFINITY;
                new_vertex->is_evaluated = false; // this means that element wasn't yet added to closed set
            */

            // Save malloc'd vertex to matrix of vertex pointers 
            S[ys][xs] = new_vertex;
        }
    }

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
        if (A_star_goal_check(params, /*B*/ abs_tol, F, curr_hom_class, x_goal_step, y_goal_step)) {
            print_path(params, config, curr_hom_class, float_tol, curr_hom_class->endpoint_vertex->hom_classes, abs_tol, expanded_states); // show the current path
            free_backtrack(curr_hom_class); // reset the path, since it was allocated at true return of A_star_goal_check
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
            // Check accessibility conditions
            if (x_n < 0 || x_n >= params->xs_steps || y_n < 0 || y_n >= params->ys_steps ||
                params->g_image[(params->s_div_r) * y_n][(params->s_div_r) * x_n] < float_tol) {
                continue; 
            }
            // Now that we know that the vertex can be accessed, we can extract it 
            vertex_t * neighb_vertex = S[y_n][x_n];

            // Compute L-step for the current transition
            Complex L_step = L(
                CMPLX(params->x_min + (x_c * params->s), params->y_min + (y_c * params->s)), 
                CMPLX(params->x_min + (x_n * params->s), params->y_min + (y_n * params->s)),
                F
            );

            // Add the L-step to the L-value for the current homotopy class to obtain the neighboring L value
            Complex neighb_Lval = curr_hom_class->Lval + L_step;

            // Search for the homtopy class corresponding to the given endpoint vertex and the neighboring L value
            hom_class_t * neighb_hom_class = hom_class_get(neighb_vertex->hom_classes, neighb_Lval, abs_tol);

            /* Loop test probably to be introduced here, before creating & enqueuing new homotopy class*/
            
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

    // Clean up (memory safe?)
    #ifdef ASTDBG
    printf("Cleaning up resources...\n");
    #endif

    free_open_set(open_set); 

    // Deallocation of the grid
    for (int ys = 0; ys < params->ys_steps; ys++) {
        for (int xs = 0; xs < params->xs_steps; xs++) {
            vertex_t * curr_vert = S[ys][xs];
            // This free's all the dynamically allocated homotopy classes
            free_hom_classes_list(curr_vert->hom_classes);    
            free(curr_vert); // free vertex * within matrix
        }
        free(S[ys]); // free row pointer (vertex **)
    } 
    free(S);

    // print_path(params, config, opt_path, float_tol, B, abs_tol, expanded_states);
    // Add timestamp under general print statement
    #ifdef ASTCLCK
    CALCULATE_ELAPSED_TIME(start, end, elapsed);
    printf("\nTime elapsed:\n\t%.5f seconds\n\n", elapsed);
    #endif
}


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

        #ifdef PATH_WRITE_DBG
        const int str_size = 64;
        char * Lval_string = (char*) malloc(str_size);
        snprintf(Lval_string, str_size, "(%.2f + %.2fi)", creal(hom_class->Lval), cimag(hom_class->Lval));
        write_json("./params.json", Lval_string, bt->point_list, edge_number - 1, 2);
        free(Lval_string);
        #endif

        curr_hom_class = curr_hom_class->parent;    // iterate backwards through vertices on optimal path
    }

    // Set elements for the path
    bt->g_image_riemann_tot = riem_cumul;
    bt->absolute_length     = abs_len_cumul;

}

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



/* Unit testing area */

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
    F_t * F = extract_obstacle_marker_func_params(params);
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

    // Invoke the A* algorithm on all the inputs generated
    A_star(
        params, config,
        zero_heuristic, // this essentially turns A* into Dijkstra's for uniform exploration
        edge_cost, 0.1,
        /* Homotopy additions */
            /*B,*/ 0.1, F
    );

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



