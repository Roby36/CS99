
#include "obstacle_marker.h"

/* This module develops the framework necessary to apply complex analysis to work with homotopy constraints */

/* Define struct F containing parameters for obstacle marker function */
struct F {
    /* These are the input parameters from the .json */
    Complex BL, TR;
    int N, a, b;
    Complex * obstacle_markers;

    /* Keep a pointer of the params from the json, and a value for the float tolerance */
    Params * params;
    float float_tol;

    /* These are additional computed parameters for the homotopies */
    Complex * A;            // array of Complex-valued residues of size N
    Complex **** Lvals;     // 4-D array storing the L values between any two points on the grid
};

/* We define an appropriate obstacle marker dividend analytic component f_0 */
#define F_0(z, BL, TR, a, b) (cpow((z) - (BL), (a)) * cpow((z) - (TR), (b)))

/* Function prototypes (static) */
static void extract_obstacle_marker_func_params(F_t * F);
static void compute_residues(F_t * F);
static Complex L(Complex z1, Complex z2, F_t * F); 
static void compute_Lvals(F_t * F);
static void free_Lvals(F_t * F);

/* Constructor for the F struct */
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

/* Destructor for the F struct */
void delete_obstacle_marker_func_params(F_t * F) {
    free_Lvals(F);
    free(F->A);     
    free(F->obstacle_markers);
    free(F);
}

/* This function extracts the relevant paramters for this module, and return F struct */
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

/* Calculating the A_l expression, given array of obstacle markers */
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

/** Lvals:
 * The L-value between the neighboring edges ((xs, ys), (xn, yn)) is stored in the entry
 * (F->Lvals) [ys][xs][(yn - ys) + 1][(xn - xs) + 1] 
 * of the (F->Lvals) 4-dimensional array
 */
static void compute_Lvals(F_t * F) {

    /** A long nested allocation of the matrix (F->Lvals), 
     * converting each discrete grid coordinate to its corresponding complex value,
     * and then using the L function to calculate the corresponding Lvalue for the edge
    */

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
            DEBUG_ERROR("Failed to allocate memory for F->Lvals[ys]");
            // Free previously allocated memory before exiting
            return;
        }
        for (int xs = 0; xs < F->params->xs_steps; xs++) {
            // For each point (xs, ys), allocate space for the nested 2x2 matrix of the 8 neighboring states
            (F->Lvals)[ys][xs] = (Complex **) malloc((NBR_OFF_COUNT) * sizeof(Complex *));
            if (!(F->Lvals)[ys][xs]) {
            DEBUG_ERROR("Failed to allocate memory for F->Lvals[ys][xs]");
            // Free previously allocated memory before exiting
            return;
            }
            for (int dy = -1; dy <= 1; dy++) {
                int yn = ys + dy;
                (F->Lvals)[ys][xs][dy + 1] = (Complex *) malloc((NBR_OFF_COUNT) * sizeof(Complex)); // save with dy offset
                if (!(F->Lvals)[ys][xs][dy + 1]) {
                DEBUG_ERROR("Failed to allocate memory for F->Lvals[ys][xs][dy + 1]");
                // Free previously allocated memory before exiting
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

/* This is the getter that returns the L-value for the edge ((x_c, y_c), (x_n, y_n)) */
Complex get_Lval(F_t * F, int x_c, int y_c, int x_n, int y_n) {

    // Redundant accessibility check 
    if (!is_accessible(F->params, x_n, y_n, F->float_tol)) {
        DEBUG_ERROR("(x_n, y_n) is not accessible");
        return CMPLX(0.0, 0.0);
    }

    // Return Lvalue based on matrix storing conventions
    return ((F->Lvals) [y_c][x_c][(y_n - y_c) + 1][(x_n - x_c) + 1]);
}

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


// Assuming float **test_path is organized such that test_path[i][0] is the real part and test_path[i][1] is the imaginary part of the ith point.
// The 'num_points' is the number of points in the path.
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

Complex * get_residues(F_t * F) {
    return (F->A);
}


#ifdef HC_UT

int main(int argc, char *argv[]) {

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
    F_t * F = extract_obstacle_marker_func_params(params);

    Complex integral_path_1 = calculate_path_integral(params->test_path_1, params->steps_path1, F);
    Complex integral_path_2 = calculate_path_integral(params->test_path_2, params->steps_path2, F);

    printf("integral_path_1 = %f + %f * i\n", creal(integral_path_1), cimag(integral_path_1));
    printf("integral_path_2 = %f + %f * i\n", creal(integral_path_2), cimag(integral_path_2));

    // Clean up
    delete_obstacle_marker_func_params(F);

    return 0;
}

#endif





