
#include "obstacle_marker.h"

/*** This module encapsulates the essential complex analysis framework to work with homotopy classes ***/

/*** struct containing parameters for obstacle marker function ***/
struct F {

    /* Input parameters extracted from the .json configuration file */
    Complex BL, TR;
    int N, a, b;
    Complex * obstacle_markers;

    /* Keep a pointer of the main params struct, and a value for the float tolerance for easier access */
    Params * params;
    float float_tol;

    /* These are additional precomputed parameters for the homotopies */
    Complex * A;        // array of Complex-valued residues of size N
    Complex **** Lvals; // 4-D array storing the L values between any two points on the grid
};

/*** Analytic dividend component of obstacle marker function ***/
#define F_0(z, BL, TR, a, b) (cpow((z) - (BL), (a)) * cpow((z) - (TR), (b)))


/********** static / private helper functions *************/

/******** extract_obstacle_marker_func_params *********
 * 
 * This is a private helper for the initialization sequence of the F struct,
 * which initializes the input parameters from the .json configuration file
 * 
 * Inputs:
 *  - An already-malloc'd F struct
 * 
 * Outputs:
 *  - Initializes N, BL, TR, a, b, obstacle_markers elements for the input struct
 * 
 * NOTE: The F struct must already be malloc'd, but potential mishandling is prevented 
 *       by keeping this as a static sub-routine, called within the
 *       full public constructor initialize_obstacle_marker_func_params later 
 */
static void extract_obstacle_marker_func_params(F_t * F) {

    F->N = F->params->num_obstacles;
    F->BL = CMPLX(F->params->x_min, F->params->y_min);
    F->TR = CMPLX(F->params->x_max, F->params->y_max);
    /* We assume f_0 to be a polynomial of order N - 1 */
    F->a = (F->N) / 2;
    F->b = ((F->N) - 1) - F->a;

    // Now fill up an array containing the obstacle marker points for each obstacle (must be free'd...)
    F->obstacle_markers = malloc((F->N) * sizeof(Complex));
    for (int obs = 0; obs < (F->N); obs++) {
        (F->obstacle_markers)[obs] = CMPLX(F->params->elliptical_obstacles[obs][0],
                                           F->params->elliptical_obstacles[obs][1]);
    }
}

/******** compute_residues ********
 * 
 * This is another private helper for the initialization sequence of the F struct,
 * which initializes the array A of the residues corresponding to each obstacle in the enviornment 
 * 
 * Inputs:
 *  - An already-malloc'd F struct, on which extract_obstacle_marker_func_params(F) was executed
 * 
 * Outputs:
 *  - Initializes and fills array A of the residues
 * 
 * NOTE: Like in the previous method, there could be clearly potential mishandling 
 *       if the method was not a static inline wrapped inside the
 *       full public constructor initialize_obstacle_marker_func_params 
*/
static void compute_residues(F_t * F) {

    Complex prod;
    Complex zeta_l;

    /** First allocate the array, assuming that N is already computed */
    if (F->obstacle_markers == NULL || F->N <= 0) {
        DEBUG_ERROR("compute_residues: F->obstacle_markers == NULL or F->N <= 0");
    }
    (F->A) = (Complex *) malloc(F->N * sizeof(Complex));

    /* Iterate throughout all the obstacles, and fill up A[l] as the residue of the lth obstacle */
    for (int l = 0; l < (F->N); l++) {
        prod = CMPLX(1.0, 0.0); // Initialize running product
        zeta_l = (F->obstacle_markers)[l]; // current obstacle marker
        // Iterate through obstacles
        for (int j = 0; j < (F->N); j++) {
            if (j == l) continue; // ignore when we are at the lth obstacle
            prod *= (zeta_l - (F->obstacle_markers)[j]);
        }
        (F->A)[l] = F_0(zeta_l, (F->BL), (F->TR), (F->a), (F->b)) / prod;
    }
}


/*********** L **************
 * 
 * This method computes analytically the L-value, or Riemann sum path integral,
 * of the obstacle marker function between two arbitrary points on the complex plane
 * 
 * Inputs:
 *  - malloc'd obstacle marker function parameters struct F, on which both 
 *      extract_obstacle_marker_func_params(F)
 *      compute_residues(F)
 *    have been called to initialize the necessary inner elements
 * 
 *  - Complex-valued Initial point and terminal point z1, z2 respectively
 * 
 * Outputs:
 *  - complex-valued L-value, corresponding to the obstacle marker function 
 *    Riemann sum path integral contribution between the points z1, z2
 * 
 * NOTE: Highly unsafe unless used as within the wrapper initialize_obstacle_marker_func_params
 */
static Complex L(Complex z1, Complex z2, F_t * F) {

    Complex zeta_l;
    double log_diff, delta_arg, k_l, minimized_arg_diff;
    Complex l_val = CMPLX(0.0, 0.0); // initialize running L-value

    for (int l = 0; l < (F->N); l++) {
        zeta_l = (F->obstacle_markers)[l]; // current obstacle marker
        // Compute log differences
        log_diff =  log(cabs(z2 - zeta_l)) - log(cabs(z1 - zeta_l));
        // Now compute the minimized arg difference between arg(z_2 - \zeta_l) - arg(z_1 - \zeta_l) + 2(k_l)pi 
        delta_arg = carg(z2 - zeta_l) - carg(z1 - zeta_l); 
        // Normalize delta_arg to be within [-pi, pi]
        delta_arg += ((delta_arg <= -M_PI) - (delta_arg > M_PI)) * 2 * M_PI;
        // Calculate the best k_l to minimize the expression
        k_l = round(-delta_arg / (2 * M_PI));
        // Calculate the minimized absolute value of the expression
        minimized_arg_diff = delta_arg + 2 * M_PI * k_l;
        // Increase the sum 
        l_val += (F->A)[l] * CMPLX(log_diff, minimized_arg_diff);
    }

    return l_val;
}


/********** compute_Lvals ************
 * 
 * This function computes all the possible L-values for the input grid 
 * at a preprocessing stage, storing them in a 4-dimensional array F->Lvals where
 *     The L-value between the neighboring grid edges ((xs, ys), (xn, yn)) is stored in the entry
 *     (F->Lvals) [ys][xs][(yn - ys) + 1][(xn - xs) + 1] 
 * 
 * Because the A* algorithm typically leads to the same transitions between states being performed multiple times,
 * precomputing the L-values for each one of these transitions saves several redundant computations, 
 * taking only a relatively low fixed time at the start of program execution 
 * (for instance, roughly 2.5 seconds for a large 1-million state grid), thereby proving to be an efficient solution. 
 * 
 * The function, taking as its only input the F struct, dynamically allocates the large matrix whilst simultaneously 
 * calculating the L-values, and fills it up within the F->Lvals attribute. 
 * It is well-encapsulated and inaccessible by outside users of the module, 
 * as it is wrapped as the last subroutine in initialize_obstacle_marker_func_params.
 * 
 * NOTE: The caller is responsible for later calling free_Lvals, which carefully deallocates F->Lvals 
 */
static void compute_Lvals(F_t * F) {

    #ifdef LVALCLCK
    struct timeval start, end;
    long seconds, microseconds;
    double elapsed;
    gettimeofday(&start, NULL); // Start clock 
    printf("Started pre-computation of Lvalues for the grid...\n");
    #endif

    F->Lvals = malloc(F->params->ys_steps * sizeof(Complex ***));  // allocate row pointers
    if (!F->Lvals) {
        DEBUG_ERROR("Failed to allocate memory for F->Lvals");
        return;
    }
    for (int ys = 0; ys < F->params->ys_steps; ys++) {
        (F->Lvals)[ys] = (Complex ***) malloc(F->params->xs_steps * sizeof(Complex **)); // allocate column for each row
        if (!(F->Lvals)[ys]) {
        /**** CLEANUP: ****/
            DEBUG_ERROR("Failed to allocate memory for F->Lvals[ys]");
            // Cleanup previously allocated memory for F->Lvals
            for (int cleanup = 0; cleanup < ys; cleanup++) {
                free((F->Lvals)[cleanup]);
            }
            free(F->Lvals);
            F->Lvals = NULL;
            return;
        }
        for (int xs = 0; xs < F->params->xs_steps; xs++) {
            // For each point (xs, ys), allocate space for the nested 2x2 matrix of the 8 neighboring states
            (F->Lvals)[ys][xs] = (Complex **) malloc((NBR_OFF_COUNT) * sizeof(Complex *));
            if (!(F->Lvals)[ys][xs]) {
            /**** CLEANUP: ****/
                DEBUG_ERROR("Failed to allocate memory for F->Lvals[ys][xs]");
                // Free previously allocated memory before exiting
                // Cleanup previous allocations in F->Lvals
                for (int cleanup_ys = 0; cleanup_ys <= ys; cleanup_ys++) {
                    for (int cleanup_xs = 0; cleanup_xs < (cleanup_ys == ys ? xs : F->params->xs_steps); cleanup_xs++) {
                        free((F->Lvals)[cleanup_ys][cleanup_xs]);
                    }
                    free((F->Lvals)[cleanup_ys]);
                }
                free(F->Lvals);
                F->Lvals = NULL;
                return;
            }
            for (int dy = -1; dy <= 1; dy++) {
                int yn = ys + dy;
                (F->Lvals)[ys][xs][dy + 1] = (Complex *) malloc((NBR_OFF_COUNT) * sizeof(Complex)); // save with dy offset
                if (!(F->Lvals)[ys][xs][dy + 1]) {
                /**** CLEANUP: ****/
                    DEBUG_ERROR("Failed to allocate memory for F->Lvals[ys][xs][dy + 1]");
                    // Cleanup for all allocated memory
                    for (int cleanup_ys = 0; cleanup_ys <= ys; cleanup_ys++) {
                        for (int cleanup_xs = 0; cleanup_xs <= xs; cleanup_xs++) {
                            for (int cleanup_dy = -1; cleanup_dy <= (cleanup_xs == xs && cleanup_ys == ys ? dy : 1); cleanup_dy++) {
                                free((F->Lvals)[cleanup_ys][cleanup_xs][cleanup_dy + 1]);
                            }
                            free((F->Lvals)[cleanup_ys][cleanup_xs]);
                        }
                        free((F->Lvals)[cleanup_ys]);
                    }
                    free(F->Lvals);
                    F->Lvals = NULL;
                    return;
                }
            for (int dx = -1; dx <= 1; dx++) {
                int xn = xs + dx; // x-neighbor matrix position
                // Default value in case of non accessibility or same state
                (F->Lvals)[ys][xs][dy + 1][dx + 1] = CMPLX(0.0, 0.0);
                // Check that the neighbor state is accessible, and that it's not the same state
                if (is_accessible(F->params, xn, yn, F->float_tol) && (dx != 0 || dy != 0)) {
                    // L-value between (xs, ys) and (xs + dx, ys + dy) in case of accessibility is stored in this matrix entry
                    (F->Lvals)[ys][xs][dy + 1][dx + 1] = L(
                        CMPLX(F->params->x_min + (xs * F->params->s), F->params->y_min + (ys * F->params->s)), 
                        CMPLX(F->params->x_min + (xn * F->params->s), F->params->y_min + (yn * F->params->s)),
                        F
                    );
                }
            }
            }
        }
    }

    #ifdef LVALCLCK
    CALCULATE_ELAPSED_TIME(start, end, elapsed);
    printf("\nLvalues precomputation for entire grid took %.5f seconds\n\n", elapsed);
    #endif
}


/******** free_Lvals **********
 * 
 * Corresponding free'ing routine for compute_Lvals 
 */
static void free_Lvals(F_t * F) {
    
    for (int ys = 0; ys < F->params->ys_steps; ys++) {
        for (int xs = 0; xs < F->params->xs_steps; xs++) {        
            for (int dy = -1; dy <= 1; dy++) { 
                free((F->Lvals)[ys][xs][dy + 1]);
            }
            free((F->Lvals)[ys][xs]);
        }
        free((F->Lvals)[ys]);
    }
    free(F->Lvals);
}


/**************** Public functions *******************/

/*********** initialize_obstacle_marker_func_params ****************
 * 
 * This function essentially acts as the public constructor for the F struct, 
 * wrapping the three private / static methods:
 * 
 *  1) extract_obstacle_marker_func_params
 *  2) compute_residues
 *  3) compute_Lvals
 * 
 * Inputs:
 *  - A fully initialized and filled params struct, containing the configuration parameters for the environment
 *  - A float tolerance value which will be necessary for floating point numbers comparison involving F
 * 
 * Outputs:
 *  - The resulting opaque struct pointer F, containing obstacle marker parameters,
 *    as well as residues and L-values between any two neighboring states on the grid 
 * 
 * NOTE: Caller responsible for providing a fully initialized and filled params struct.
 *       This should be constructed by first using the parser module to extract data 
 *       from the .json configuration file:
 *              params = load_json(input_json_path);
 *          
 *       Caller responsible for later calling the public destructor delete_obstacle_marker_func_params
 *       which wraps and handles all the tedious deallocations for the struct F
 */
F_t * initialize_obstacle_marker_func_params(Params * params, float float_tol) {
    // Main memory allocation for the struct here
    F_t * F = (F_t *) malloc(sizeof(F_t));
    F->params = params;
    F->float_tol = float_tol;
    // First extract the input parameters from the file using the parser
    extract_obstacle_marker_func_params(F);
    // Next compute the residues stored within the struct
    compute_residues(F);
    // Finally allocate and calculate the Lvalue matrix
    compute_Lvals(F);

    return F;
}

/***** delete_obstacle_marker_func_params ********
 * 
 * This function essentially acts as the public destructor for the F struct,
 * carefully free'ing all the memory allocated by initialize_obstacle_marker_func_params
 */
void delete_obstacle_marker_func_params(F_t * F) {
    free_Lvals(F);
    free(F->A);     
    free(F->obstacle_markers);
    free(F);
}


/*********** get_Lval **************
 * 
 * This is the main public function exposed by the module,
 * allowing the caller to seamlessly extract the L-value between two arbitary
 * gridpoints with minimal overhead.
 * 
 * Inputs:
 *  - opaque (to the caller) obstacle marker struct F
 *  - initial and terminal neighboring points (x_c, y_c), (x_n, y_n) for an edge in the search graph
 * 
 * Outputs:
 *  - L-value between the gridpoints if these are valid
 *  - 0 + 0i and error if the neighbor is not accessible or out of range
 *  
 */
Complex get_Lval(F_t * F, int x_c, int y_c, int x_n, int y_n) {

    // Redundant accessibility check 
    if (!is_accessible(F->params, x_n, y_n, F->float_tol)) {
        DEBUG_ERROR("(x_n, y_n) is not accessible");
        return CMPLX(0.0, 0.0);
    }

    // Check the bouds for the matrix 
    if (abs(y_n - y_c) > 1 || abs(x_n - x_c) > 1) {
        DEBUG_ERROR("neighbor out of range");
        return CMPLX(0.0, 0.0);
    }

    // Return Lvalue based on matrix storing conventions
    return ((F->Lvals) [y_c][x_c][(y_n - y_c) + 1][(x_n - x_c) + 1]);
}

/*********** calculate_path_integral **************
 *  
 * Inputs:
 *  - 2-dimensional matrix float ** test_path containing the float cooordinates in the map environment 
 *    for the path
 *  - number of points num_points contained in the test_path matrix
 *  - obstacle marker function struct F
 * 
 * Output:
 *  - Complex-valued path integral along the path 
 * 
 * NOTE: Assuming float **test_path is organized such that 
 *       test_path[i][0] is the real part and test_path[i][1] 
 *       is the imaginary part of the ith point in the path
 */
Complex calculate_path_integral(float ** test_path, int num_points, F_t * F) {

    Complex sum = CMPLX(0.0, 0.0);
    Complex z1, z2, integral_part;

    for (int i = 0; i < num_points - 1; i++) {
        // Convert the float coordinates to complex numbers
        z1 = CMPLX(test_path[i][0],   test_path[i][1]);
        z2 = CMPLX(test_path[i+1][0], test_path[i+1][1]);

        // Calculate the integral between the current point and the next
        integral_part = L(z1, z2, F);

        // Accumulate the result
        sum += integral_part;
    }
    return sum;
}

/* Simple getters */
Complex * get_residues(F_t * F) {
    return (F->A);
}

Complex * get_obstacle_markers(F_t * F) {
    return (F->obstacle_markers);
}

/*********** Unit testing area **************/

#ifdef HC_UT

int main(int argc, char *argv[]) {

    // Hard-coded for testing purposes
    float float_tol = 0.1;

    // Validate input 
    if (argc != 2) {  // Check if the number of arguments is correct
        fprintf(stderr, "Usage: %s input_json_path\n", argv[0]);
        return 1;  // Return an error code
    }

    // Allocate memory for the input path
    char * input_json_path = (char *) malloc(strlen(argv[1]) + 1);
    if (input_json_path == NULL) {
        fprintf(stderr, "Memory allocation failed\n");
        return 1;
    }

    // Copy the input argument to our new variable
    strcpy(input_json_path, argv[1]);

    // Extract parameters from json, and from corresponding struct
    Params * params = load_json(input_json_path);
    // Extract obstacle marker parameters from parameters
    F_t * F = initialize_obstacle_marker_func_params(params, float_tol);

    // not testing calculate_path_integral since it does not manipulate memory

    // Clean up
    delete_obstacle_marker_func_params(F);
    free_params(params);
    free(input_json_path);

    return 0;
}

#endif





