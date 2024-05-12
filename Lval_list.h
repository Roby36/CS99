
#include <complex.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "commonmacros.h"

typedef double complex Complex;

/* Opaques */
typedef struct Lval_node Lval_node;
typedef struct Lval_list Lval_list_t;

Lval_list_t *Lval_list_new();
void free_Lval_list(Lval_list_t *Lval_list);
bool Lval_list_contains(Lval_list_t *Lval_list, Lval_list_t * other_list, double abs_tol);
void insert_Lval(Lval_list_t *Lval_list, Complex Lval, double abs_tol);
void merge_Lvalues(Lval_list_t *Lval_list, Lval_list_t * other_list, Complex L_offset, double abs_tol);
void print_complex_list(Lval_list_t *list);
char* complex_list_to_string(Lval_list_t *list);


