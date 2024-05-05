
#include "astar.h"

// #define ASTDBG 1 // Debug flag to activate print statements 

/* Name of parameter containing path written to .json file */
const char * output_path_param_name = "A_star_output_path"; // name of parameter holding path

/* Keep configuration parameters in a wrappable, versatile way */
typedef struct {
    float a;
    float b;
} Config;

typedef struct {
    int l;
    float ** point_list;
} Optimal_Path;

/* Internal functions prototypes & helpers */
Optimal_Path * backtrack_path(Params * params, vertex_t * goal_vertex);
void sqrt_dist_test();

/** Cost function implementation 
 * Now we have two discrete, adjacent points (x_1,y_1) and (x_2,y_2), in the s-step sized state grid for the robot.
 * Supposing that we have data, stored in a g_image matrix, at smaller r-step intervals, we are going to 
 * compute the Riemann sum for the integral between the two adjacent points on the grid by accesing matrix entries,
 * and linearly combine it with path length to compute overall cost function between two adjacent states in the grid. 
*/
float edge_cost(
    int x_1, int y_1, int x_2, int y_2,
    Params * params,
    Config * config) {
    
    // Calculate the total distance between the two discrete points 
    float l = params->s * D_ADJACENT(x_1, y_1, x_2, y_2); // Use s as scaling factor

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

    // Finally take a linear combination using the fixed parameters a,b
    float c = (config->a * u) + (config->b * l);
    return c;
}

/* Heuristic function between any point (x_n, y_n) and the goal (x_g, y_g) */
float heuristic(
    int x_n, int y_n, int x_g, int y_g,
    Params * params,
    Config * config) {
    // First determine the total steps in the x and y directions
    int dx = abs(x_g - x_n);
    int dy = abs(y_g - y_n);
    // Then we take as many steps possible diagonally, and the rest in the remaining straight line
    float l = params->s * ((SQRT2 - 1) * MIN(dx,dy) + MAX(dx, dy)); // each step is also of size s
    // We multiply by the parameter b to ensure a tight heuristic compared to the cost function defined above
    float h = config->b * l;
    return h;
}


/**************** Main A* algorithm implementation ****************/
Optimal_Path * A_star(
    Params * params,
    Config * config,
    float (*h) (int, int, int, int, Params *, Config * ),
    float (*c) (int, int, int, int, Params *, Config * ))
{
    #ifdef ASTCLCK
    struct timeval start, end;
    long seconds, microseconds;
    double elapsed;
    gettimeofday(&start, NULL); // Start clock 
    #endif

    // Initialize optimal path to default 
    Optimal_Path * opt_path = NULL;

    // Convert start and goal coordinates to the relative s-steps
    const int x_start_step = (int) (params->x_s - params->x_min) / params->s;
    const int x_goal_step  = (int) (params->x_g - params->x_min) / params->s;
    const int y_start_step = (int) (params->y_s - params->y_min) / params->s;
    const int y_goal_step  = (int) (params->y_g - params->y_min) / params->s;    

    // Next, based on number of rows (ys_steps), and columns (xs_steps), allocate the grid
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
            new_vertex->parent = NULL;
            new_vertex->g_score = INFINITY;
            new_vertex->f_score = INFINITY;
            new_vertex->fib_node = NULL;
            // new_vertex->next = NULL; 
            new_vertex->is_evaluated = false; // this means that element wasn't yet added to closed set
            // Save malloc'd vertex to matrix of vertex pointers 
            S[ys][xs] = new_vertex;
        }
    }

    // Handle start vertex parameter intitializations 
    vertex_t * start_vertex = S[y_start_step][x_start_step];
    start_vertex->g_score = 0.0f;
    start_vertex->f_score = h(
        x_start_step, y_start_step, x_goal_step, y_goal_step, params, config
    );

    // Start priority queue, intitialized to the start vertex
    open_set_t * open_set = open_set_new(); /** CAREFUL: not checking for NULL pointer return */
    insert_sorted(open_set, start_vertex);
    // Start main iteration, checking whether the open set is empty 
    while (!open_set_is_empty(open_set)) {
        vertex_t * current_vertex = dequeue_min(open_set);
        // flag the current vertex as evaluated (like adding it to closed set)
        current_vertex->is_evaluated = true;
        // keep the current x,y coordinates as local variables for clarity
        int x_c = current_vertex->x_s;
        int y_c = current_vertex->y_s;
        if (x_c == x_goal_step && y_c == y_goal_step) {
            #ifdef ASTDBG
            printf("Goal reached at (%d, %d) with path cost: %f\n", x_c, y_c, current_vertex->g_score);
            #endif
            /** Reconstruct & assign path **/
            opt_path = backtrack_path(params, current_vertex);
            break; // break out of while loop so that you can first clean up, and then return the saved path
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
                params->g_image[(params->s_div_r) * y_n][(params->s_div_r) * x_n] < FLOAT_TOL) {
                continue; 
            }
            // Now that we know that the vertex can be accessed, we can extract it 
            vertex_t * neighb_vertex = S[y_n][x_n];
            // Check if the vertex is already in the closed set
            if (neighb_vertex->is_evaluated) {
                continue;
            }
            // Calculate tentative g_score
            float tent_g = current_vertex->g_score + c(
                x_c, y_c, x_n, y_n, params, config
            );
            // If no better path found, continue
            if (tent_g >= neighb_vertex->g_score) {
                continue;
            }
            // If we are here, then we have found a new shortest path to neighb_vertex
            #ifdef ASTDBG
            printf("Updating vertex (%d, %d) from g_score: %f to g_score: %f\n", neighb_vertex->x_s, neighb_vertex->y_s, neighb_vertex->g_score, tent_g);
            #endif
            neighb_vertex->parent = current_vertex;
            neighb_vertex->g_score = tent_g;
            neighb_vertex->f_score = tent_g + h(x_n, y_n, x_goal_step, y_goal_step, params, config);
            // Ensure the vertex is extracted to reflect the update in f_score (no collateral effects if not in open_set)
            extract_sorted(open_set, neighb_vertex);
            // Now we are guaranteed that the vertex is out of the open set, thus can add it unconditionally
            insert_sorted(open_set, neighb_vertex); 
        }
        }
    }

    // Clean up (memory safe?)
    #ifdef ASTDBG
    printf("Cleaning up resources...\n");
    #endif

    // free_open_set(open_set); vertices are already deallocated later on 

    // Deallocation of the grid
    for (int ys = 0; ys < params->ys_steps; ys++) {
        for (int xs = 0; xs < params->xs_steps; xs++) {
            free(S[ys][xs]); // free vertex * within matrix
        }
        free(S[ys]); // free row pointer (vertex **)
    } 
    free(S);

    #ifdef ASTCLCK
    CALCULATE_ELAPSED_TIME(start, end, elapsed);

    printf("\nA* with parameters\n"
        "\t(x_g, y_g) = (%f, %f)\n"
        "\tr = %f, s = %f\n"
        "\ta = %f, b = %f\n"
        "took %.5f seconds\n", 
    params->x_g, params->y_g, params->r, params->s, config->a, config->b, elapsed);
    #endif

    return opt_path;

}


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
    Optimal_Path * opt_path;

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

    // Test if we actually have the right matrix by printing it to the file under other key
    #ifdef ASTDBG
    write_json(input_json_path, "g_image_test", params->g_image, params->yr_steps, params->xr_steps); 
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
    opt_path = A_star(params, config, heuristic, edge_cost);
    if (opt_path == NULL) {
        DEBUG_ERROR("A_star returned NULL path. Exiting...");
        exit(2);
    }

    // Write the path to the .json file
    write_json(input_json_path, output_path_param_name, opt_path->point_list, opt_path->l, 2); 

    // Clean up
    free(input_json_path);

    free(config);

    free(opt_path);

    /** TODO: Find ways to clean up the params struct; probably manual thorough cleanup required for grids */
    free(params);

    return 0;
}

/** Path backtracking **/

Optimal_Path * backtrack_path(Params *params, vertex_t *goal_vertex) {
    // First, find the length of the path by traversing from goal to start
    int path_length = 0;
    vertex_t *current_vertex = goal_vertex;
    while (current_vertex != NULL) {
        path_length++;
        current_vertex = current_vertex->parent;
    }

    // Allocate memory for the path struct
    Optimal_Path * opt_path = malloc(sizeof(Optimal_Path));

    // Allocate memory for the points list
    opt_path->point_list = malloc(sizeof(float *) * path_length);
    for (int i = 0; i < path_length; i++) {
        opt_path->point_list[i] = malloc(sizeof(float) * 2); // Each point has two coordinates (x, y)
        if (!opt_path->point_list[i]) {
            fprintf(stderr, "Memory allocation failed for point_list[%d].\n", i);
            return NULL;
        }
    }

    // Backtrack from goal to start and store the path in reverse
    current_vertex = goal_vertex;
    for (int i = path_length - 1; i >= 0; i--) {
        opt_path->point_list[i][0] = params->x_min + (current_vertex->x_s * params->s); // Convert x_s back to x
        opt_path->point_list[i][1] = params->y_min + (current_vertex->y_s * params->s); // Convert y_s back to y

        #ifdef ASTDBG
        printf("Backtracking path point[%d]: (%.2f, %.2f)\n", i, opt_path->point_list[i][0], opt_path->point_list[i][1]);
        #endif
        
        current_vertex = current_vertex->parent;
    }

    opt_path->l = path_length;
    return opt_path;
}




void sqrt_dist_test() {
    printf("Distance (same points): %f\n", D_ADJACENT(1, 1, 1, 1));
    printf("Distance (horizontal adjacent): %f\n", D_ADJACENT(1, 1, 2, 1));
    printf("Distance (vertical adjacent): %f\n", D_ADJACENT(1, 1, 1, 2));
    printf("Distance (diagonal adjacent): %f\n", D_ADJACENT(1, 1, 2, 2));
}

/* Euclidean distance between two adjacent points in a UNIT sized grid, taking in integers */
/* If applied on two ADJACENT s-step points, requires multiplication by s etc. */
/* Later converted to macro-like function */
/*
float d_adjacent(int x_1, int y_1, int x_2, int y_2) {
    int state = (x_1 != x_2) + (y_1 != y_2);
    switch (state) {
        case 0: return 0; // same coordinates
        case 1: return 1; // only displaced in one dimension
        case 2: return SQRT2; // displaced in both dimensions
    }
}
*/


