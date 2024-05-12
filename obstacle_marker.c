
#include "obstacle_marker.h"

/* This module develops the framework necessary to apply complex analysis to work with homotopy constraints */

/* Define struct F containing parameters for obstacle marker function */
struct F {
    Complex BL, TR;
    int N, a, b;
    Complex * obstacle_markers;
};

/* We define an appropriate obstacle marker dividend analytic component f_0 */
#define F_0(z, BL, TR, a, b) (cpow((z) - (BL), (a)) * cpow((z) - (TR), (b)))

/* This function extracts the relevant paramters for this module, and return F struct */
F_t * extract_obstacle_marker_func_params(Params * params) {

    F_t * F = (F_t *) malloc(sizeof(F_t));

    F->N = params->num_obstacles;
    F->BL = CMPLX(params->x_min, params->y_min);
    F->TR = CMPLX(params->x_max, params->y_max);
    /* We assume f_0 to be a polynomial of order N - 1 */
    F->a = (F->N) / 2;
    F->b = ((F->N) - 1) - F->a;

    // Now fill up an array containing the obstacle marker points for each obstacle (must be free'd...)
    F->obstacle_markers = malloc((F->N) * sizeof(Complex));
    for (int obs = 0; obs < (F->N); obs++) {
        (F->obstacle_markers)[obs] = CMPLX(params->elliptical_obstacles[obs][0],
                                           params->elliptical_obstacles[obs][1]);
    }

    return F;
}

/* Corresponding clean up function*/
void delete_obstacle_marker_func_params(F_t * F) {
    free(F->obstacle_markers);
    free(F);
}

/* Calculating the A_l expression, given array of obstacle markers */
Complex L(Complex z1, Complex z2, F_t * F) {

    Complex prod;
    Complex A_l;
    Complex zeta_l;
    double log_diff, delta_arg, k_l, minimized_arg_diff;

    Complex l_val = CMPLX(0.0, 0.0); // initialize running L-value
    for (int l = 0; l < (F->N); l++) {

        // Compute A_l
        prod = CMPLX(1.0, 0.0); // Initialize running product
        zeta_l = (F->obstacle_markers)[l]; // current obstacle marker
        // Iterate through obstacles
        for (int j = 0; j < (F->N); j++) {
            if (j == l) continue; // ignore when we are at the lth obstacle
            prod *= (zeta_l - (F->obstacle_markers)[j]);
        }
        A_l = F_0(zeta_l, (F->BL), (F->TR), (F->a), (F->b)) / prod;
        // Compute log differences
        log_diff =  log(cabs(z2 - zeta_l)) - log(cabs(z1 - zeta_l));

        /* Now compute the minimized arg difference between arg(z_2 - \zeta_l) - arg(z_1 - \zeta_l) + 2(k_l)pi */
        delta_arg = carg(z2 - zeta_l) - carg(z1 - zeta_l); 
        // Normalize delta_arg to be within [-pi, pi]
        delta_arg += ((delta_arg <= -M_PI) - (delta_arg > M_PI)) * 2 * M_PI;
        // Calculate the best k_l to minimize the expression
        k_l = round(-delta_arg / (2 * M_PI));
        // Calculate the minimized absolute value of the expression
        minimized_arg_diff = delta_arg + 2 * M_PI * k_l;

        // Increase the sum 
        l_val += A_l * CMPLX(log_diff, minimized_arg_diff);
    }

    return l_val;
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





