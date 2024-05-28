
#include "ast_hom_bo.h"
#include "parser.h"

/************ Boundary optimization testing progran******************
 * 
 * This is the main method for the boundary optimization method on the 
 * homotopy class extension of A* with the tailored cost metric,
 * as designed and implemented in the other modules.
 * 
 * It is meant to serve as an illustrative example of how to properly use 
 * the APIs to allocate and deallocate resources, building a simple 
 * command-line interface to run the program.
*/

int main(int argc, char *argv[]) {

    // Hard-coded constants 
    const double abs_tol   = 0.1;
    const float var_start  = 0.01;
    const float scale_fact = 1.0f;
  
    // Inputs from command line 
    char * input_json_path;
    char * table_title;
    int max_expandible_states_mult = 0;
    int max_hom_classes = 0;
    float float_tol = 0.0f;
    float bound = 0.0f;
    float var_mult = 0.0f;
    int MAX_IT = 0;

    // Parsing command-line arguments
    if (argc != 9) { 
        fprintf(stderr, "Usage: %s <input_json_path> <table_title> <float_tol> <bound> <var_mult> <MAX_IT> <max_expandible_states_mult> <max_hom_classes> \n", argv[0]);
        return 1; 
    }

    PARSE_STRING(input_json_path, argv[1], "<input_json_path>");
    PARSE_STRING(table_title, argv[2], "<table_title>");
    PARSE_FLOAT(argv[3], float_tol);
    PARSE_FLOAT(argv[4], bound);
    PARSE_FLOAT(argv[5], var_mult);
    PARSE_INT(argv[6], MAX_IT);
    PARSE_INT(argv[7], max_expandible_states_mult);
    PARSE_INT(argv[8], max_hom_classes);

    // Local variables for A*
    Params * params = load_json(input_json_path);
    if (params == NULL) {
        DEBUG_ERROR("load_json / extract_parameters returned NULL, exiting...");
        exit(1);
    }
    Config * config = malloc(sizeof(Config));
    F_t * F = initialize_obstacle_marker_func_params(params, float_tol);
    hom_classes_list_t * target_hom_classes = NULL; // initialization setting

    // Log parsed parameters
    #ifdef AST_HC_DBG
    printf(
        "Running boundary_optimization with inputs from command-line\n" 
        "\tbound = %f\n\tvar_mult = %f\n\tMAX_IT = %d\n\tmax_expandible_states_mult = %d\n\tmax_hom_classes = %d\n\n", 
        bound, var_mult, MAX_IT, max_expandible_states_mult, max_hom_classes
    ); 
    #endif

    // A* parameters
    struct A_star_homotopies_args  * astar_args = malloc(sizeof(struct A_star_homotopies_args));
    astar_args->params = params;
    astar_args->config = config;
    astar_args->h = zero_heuristic;
    astar_args->c = edge_cost;
    astar_args->float_tol = float_tol;
    astar_args->abs_tol = abs_tol;
    astar_args->F = F;
    astar_args->target_hom_classes_ptr = &target_hom_classes;
    astar_args->max_hom_classes = max_hom_classes;
    astar_args->max_expandible_states_mult = max_expandible_states_mult;
    astar_args->json_filepath = input_json_path;

    // boundary optimization parameters (initialize only input section)
    struct BO_Params * bo = malloc(sizeof(struct BO_Params));
    bo->var_config = &config->a;
    bo->fix_config = &config->b;
    bo->var_path_met = g_image_getter;
    bo->fix_path_met = path_length_getter;
    bo->bound = bound;
    bo->var_start = var_start;
    bo->var_mult = var_mult;
    bo->scale_fact = scale_fact;
    bo->MAX_IT = MAX_IT;
    bo->table_title = table_title;

    // main program
    boundary_optimization(astar_args, bo);

    // Logging and immediate clean up of BO_Params struct (alternatively could be used for something else)
    #ifdef BO_LATEX
    print_LaTex_table(astar_args, bo, table_title);
    #endif
    bo_clean_up(astar_args, bo);

    /** IMPORTANT: order in which we free is crucial, since some structs depend on others! */
    free_hom_classes_list(target_hom_classes); // done at same scope of declaration, but could also be handled elsewhere
    free(bo);
    free(astar_args);
    delete_obstacle_marker_func_params(F);
    free(config);
    free_params(params);
    free(table_title);
    free(input_json_path);

    return 0;
}

