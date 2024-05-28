
#include "ast_hom_bo.h"


void boundary_optimization(
    struct A_star_homotopies_args * astar_args,
    struct BO_Params * bo) 
{
    bool homotopy_classes_updated; // flag signaling whether any homotopic classes have been updated, thus whether further iterations are still needed
    int curr_hom_index;

    //  map: (iteration # -> (homotopy class -> Backtrack path *))
    bo->bt_paths = malloc((bo->MAX_IT + 1) * sizeof(Backtrack_Path **));
    // <config_type> (iteration # -> config value)
    bo->fix_config_vals = malloc((bo->MAX_IT + 1) * sizeof(float));
    bo->var_config_vals = malloc((bo->MAX_IT + 1) * sizeof(float));
 
    // Initialize configuration parameters for iteration, and write them to the arrays 
    bo->curr_it = 0; /** IMPORTANT: We will faithfully assume that A* does not modify the config struct */    
    *(bo->fix_config) = (1.0f / bo->scale_fact);  // e.g. b = 1, a = 0 to initialize fixed and variable configuration parameters
    *(bo->var_config) = 0.0f;
    
    while (true) {
        if (bo->curr_it == 1) {
            *(bo->var_config) = (bo->var_start / bo->scale_fact);
        }

        // Update cnfiguration values for current iteration
        bo->fix_config_vals[bo->curr_it] = *(bo->fix_config);
        bo->var_config_vals[bo->curr_it] = *(bo->var_config);

        // Allocate arrays of path pointers for each homotopy class of this iteration
        bo->bt_paths[bo->curr_it] = malloc((astar_args->max_hom_classes) * sizeof(Backtrack_Path *));

        // Check documentation in astar_homotopies.h to see how homotopy class lists are updated in place        
        A_star_homotopies(astar_args);
        
        // Update each homotopy class within the arrays with the updated path metrics from A*
        hom_class_t * hom_class_it = (*(astar_args->target_hom_classes_ptr))->head;
        curr_hom_index = 0;
        homotopy_classes_updated = false; 

        while (hom_class_it != NULL) {
            // Get currently not-yet-allocated entries for current iteration and homotopy class
            Backtrack_Path ** curr_path_ptr = &(bo->bt_paths[bo->curr_it][curr_hom_index]);

            // Check that this homotopy class's fixed path metric is within benchmark bounds
            if ((bo->curr_it != 0) && // ensure that we skip the condition for the benchmark itself 
                *(bo->fix_path_met(hom_class_it->backtrack)) > ((1.0f + bo->bound) * (* (bo->fix_path_met(bo->bt_paths[0][curr_hom_index])))) ) 
            {
                #ifdef BO_LOG
                printf(
                    "\nBoundary optimization: " 
                    " skipping homotopy class (%.2f + %.2fi) / index %d, setting Backtrack_Path = NULL, "
                    " at iteration %d becuase the fixed path metric exceeded bounds, "
                    " with variable configuration parameter %.2f\n",
                    creal(hom_class_it->Lval), cimag(hom_class_it->Lval), curr_hom_index,
                    bo->curr_it, *(bo->var_config)
                );
                #endif

                // Set path to NULL to show that the boundary condition was not satisfied, and skip over the allocation
                *curr_path_ptr = NULL;
                goto hom_class_inner_loop_end;  // continue to next iteration
            }

            // Next, check that the homotopy class has actually been updated by A* (and not just left in place after state limit exceeded) 
            if (!hom_class_it->updated) 
            {
                #ifdef BO_LOG
                printf(
                    "\nBoundary optimization: " 
                    " skipping homotopy class (%.2f + %.2fi) / index %d, setting Backtrack_Path = NULL, "
                    " at iteration %d becuase the homotopy class was not updated by A* (likely due to exceeding maximum expandible states), "
                    " with variable configuration parameter %.2f\n",
                    creal(hom_class_it->Lval), cimag(hom_class_it->Lval), curr_hom_index,
                    bo->curr_it, *(bo->var_config)
                );
                #endif

                // Set path to NULL to show that the boundary condition was not satisfied, and skip over the allocation
                *curr_path_ptr = NULL;
                goto hom_class_inner_loop_end;  // continue to next iteration
            }
            /** OPTIONAL: Check for path convergence or unexpected rise by comparing with previous values of variable path metric*/

            // If the bound condition was passed, then allocate paths, and set them to their current values 
            *curr_path_ptr = malloc(sizeof(Backtrack_Path));
            *(bo->fix_path_met(*curr_path_ptr)) = *(bo->fix_path_met(hom_class_it->backtrack));  // benchmark for curr_hom_index = fix_met_vals[0][curr_hom_index] 
            *(bo->var_path_met(*curr_path_ptr)) = *(bo->var_path_met(hom_class_it->backtrack));
            homotopy_classes_updated = true;    // if we make it to here, flag an update in at least one homotopy class

            // End of loop: iterating to next homotopy class
            hom_class_inner_loop_end:
            curr_hom_index++;
            hom_class_it = hom_class_it->next;
        }

        /* End-of-loop routine: check two loop-break conditions */
        if (++(bo->curr_it) > bo->MAX_IT) {
            #ifdef BO_LOG
            printf("\nBoundary optimization terminated as max_it = %d was reached\n", bo->MAX_IT);
            #endif
            break;
        }
        if (!homotopy_classes_updated) {
            #ifdef BO_LOG
            printf("\nBoundary optimization terminated as no homotopy clases have been updated at iteration %d\n", bo->curr_it);
            #endif
            break;
        }
        // Guarantees that A* is performed for the updated value 
        *(bo->var_config) *= bo->var_mult;  
    }


    /** ALGOEND: Outputting results and cleaning up from here below */
    #ifdef BO_LOG 
    printf(
        "\nboundary_optimization terminated with inputs:\n"
        "\tbound = %.2f; var_start = %.2f; var_mult = %.2f\n",
        bo->bound, bo->var_start, bo->var_mult
    );
    #endif

    // End of main boundary optimization procedure
    // still requires printing and cleaning up, performed by the caller
    
}


void print_LaTex_table(
    struct A_star_homotopies_args * astar_args,
    struct BO_Params * bo,
    char * table_title)
{
    printf("\n\n---------------- LATEX TABLES START ----------------\n\n");
    hom_class_t *hom_class_it = (*(astar_args->target_hom_classes_ptr))->head;
    int curr_hom_index = 0; /** NOTE: This variable was already used previously at this scope */
    int total_hom_classes = astar_args->max_hom_classes; // Assuming this gives the total number of homotopy classes
    int table_count = 0;

    while (hom_class_it != NULL) {
        int num_classes_in_table = 0;
        
        printf("\\begin{table}[h!]\n");
        printf("\\centering\n");
        printf("\\begin{tabularx}{\\textwidth}{|X|X|X|");  // Start tabularx with the specified width

        hom_class_t *temp_it = hom_class_it;    // holds intial homotopy class pointer for this table
        int start_hom_index = curr_hom_index;   
        for (int j = 0; j < MAX_HC_PER_TABLE && hom_class_it != NULL; j++) {
            printf("X|X|"); // Two columns per homotopy class
            num_classes_in_table++;
            hom_class_it = hom_class_it->next;  // advance main homotopy class iterator with index
            curr_hom_index++;
        }
        printf("}\n\\hline\n");
        
        // Print headers for the iteration and configuration columns
        printf("It & f.c. & v.c. ");
        for (int i = 0; i < num_classes_in_table && temp_it != NULL ; i++) {
            printf("& \\multicolumn{2}{c|}{%.2f + %.2fi} ", creal(temp_it->Lval), cimag(temp_it->Lval));
            temp_it = temp_it->next;    // advance until we either terminate classes in table or hit the null ptr
        }
        printf("\\\\ \\cline{4-%d}\n", 3 + 2 * num_classes_in_table);

        // Adding subheaders for fixed and variable path metrics
        printf("& & ");
        for (int j = 0; j < num_classes_in_table; j++) {
            printf("& f.p.m. & v.p.m. ");
        }
        printf("\\\\ \\hline\n");

        // Iterating over each iteration and each homotopy class
        for (int it = 0; it < bo->curr_it; it++) {
            printf("%d & %.2f & %.2f", it, bo->fix_config_vals[it], bo->var_config_vals[it]);
            for (int temp_hom_index = start_hom_index; 
                     temp_hom_index < curr_hom_index; 
                     temp_hom_index++) 
                {
                if (bo->bt_paths[it][temp_hom_index] == NULL) {    // path out of bounds
                    printf(" & oob & oob");
                } else {
                    printf(
                        " & %.2f & %.2f", 
                        *(bo->fix_path_met(bo->bt_paths[it][temp_hom_index])), 
                        *(bo->var_path_met(bo->bt_paths[it][temp_hom_index]))
                    );
                }
            }
            printf("\\\\ \\hline\n");
        }
        printf("\\end{tabularx}\n");
        printf("\\caption{%s; table %d}\n", table_title, ++table_count);
        printf("\\label{tab:metrics_summary_%d}\n", table_count);
        printf("\\end{table}\n");
    }
    printf("\n\n---------------- LATEX TABLES END ----------------\n\n");
}


void bo_clean_up(
    struct A_star_homotopies_args * astar_args, 
    struct BO_Params * bo)
{
    /* Clean-up procedure */
    for (int it = 0; it < bo->curr_it; it++) {
        hom_class_t * hom_class_it = (*(astar_args->target_hom_classes_ptr))->head;
        int curr_hom_index = 0;
        while (hom_class_it != NULL) {
            // Check that this Backtrack path was actually allocated 
            if (bo->bt_paths[it][curr_hom_index] != NULL) {
                free(bo->bt_paths[it][curr_hom_index]);
            }
            // Slide forward
            curr_hom_index++;
            hom_class_it = hom_class_it->next;
        }
        free(bo->bt_paths[it]);
    }
    free(bo->bt_paths);
    free(bo->fix_config_vals); 
    free(bo->var_config_vals);
}

/* Getters / setters for fixed and variable path metrics */
float * g_image_getter(Backtrack_Path * bt) {
    float * g_image_ptr = &(bt->g_image_riemann_tot);
    return g_image_ptr;
}

float * path_length_getter(Backtrack_Path * bt) {
    float * length_ptr = &(bt->absolute_length);
    return length_ptr;
}

