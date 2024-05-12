
#pragma once

#include <complex.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "commonmacros.h"

typedef double complex Complex;

/* These structs must be accessible to A* */
typedef struct hom_class {

    Complex Lval;
    struct hom_class * next;

    /* Homotopy class relative properties, updated for each homotopy class */
    struct vertex * parent_vertex; 
    float g_score; 
    float f_score;

    /* When backtracing, we will also need to keep track of the parent's hom_class */
    // struct hom_class * parent_hom_class;

} hom_class;

typedef struct hom_classes_list {
    
    hom_class *  head;
    hom_class ** last_ptr;

} hom_classes_list_t;


hom_classes_list_t *hom_classes_list_new();
void free_hom_classes_list(hom_classes_list_t *hom_classes_list);
hom_class * hom_class_get(hom_classes_list_t *hom_classes_list, Complex Lval, double abs_tol);
bool insert_Lval(hom_classes_list_t *hom_classes_list, Complex Lval, double abs_tol);
hom_class * hom_class_get_min_f(hom_classes_list_t *hom_classes_list);
char* complex_list_to_string(hom_classes_list_t *list);

