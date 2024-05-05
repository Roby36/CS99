
#include "set.h"

/**************** local types ****************/
typedef struct element {
  char* key;  
  void* item;                 
  struct element* next;       
} element_t;

/**************** global types ****************/
typedef struct set{
    struct element* head;
} set_t;  // opaque to users of the module

/**************** functions ****************/
static element_t* __attribute__ ((noinline)) key_find(set_t* setp, const char* key);
static void set_merge_helper(void* arg, const char* key, void* item);
static void set_intersect_helper(void* arg, const char* key, void* item);

/**************** set_new ****************/
/* see set.h for description */
set_t* 
set_new(void){
    set_t* new_set = malloc(sizeof(set_t));
    if (new_set != NULL)
        new_set->head = NULL; 
    return new_set;
}

/**************** set_insert ****************/
/* see set.h for description */
bool 
set_insert(set_t* set, const char* key, void* item){

    if (set == NULL || key == NULL || item == NULL) 
        return false;

    // First check if key exists already in set:
    if (key_find(set, key) != NULL) 
        return false; 

    // If key does not already exist in set, then we allocate space for it:
    int key_length = strlen(key) + 1;
    char* key_copy = malloc(key_length);
    if (key_copy == NULL) 
        return false;
    strncpy(key_copy, key, key_length);

    // Initialize new element, using copied string:
    element_t* new_element = malloc(sizeof(element_t));
    if (new_element == NULL) 
        return false;
    new_element->key  = key_copy;
    new_element->item = item;

    // Insert element on top of set
    new_element->next = set->head;
    set->head         = new_element;

    return true;
}

/**************** set_find ****************/
/* see set.h for description */
void* 
set_find(set_t* set, const char* key) {
    // key_find already validates the arguments
    element_t* el = key_find(set, key);
    if (el == NULL) 
        return NULL; 
    return (el->item);
}

/**************** set_iterate ****************/
/* see set.h for description */
void 
set_iterate(set_t* set, void* arg,
                 void (*itemfunc)(void* arg, const char* key, void* item) ){
                    
    if (set == NULL || itemfunc == NULL) 
        return;
    for (element_t* it = set->head; 
                    it != NULL; 
                    it = it->next) {
        (*itemfunc)(arg, it->key, it->item);
    }
}

/**************** set_print ****************/
/* see set.h for description */
void 
set_print(set_t* set, FILE* fp, 
            void (*itemprint)(FILE* fp, const char* key, void* item) ) {
    
    if (fp == NULL)
        return;
    if (set == NULL) {
        fprintf(fp, "(null)");
        return;
    }

    fprintf(fp, "{");
    set_iterate(set, fp, itemprint);
    fprintf(fp, "}");
}

/**************** set_delete ****************/
/* see set.h for description */
void 
set_delete(set_t* set, void (*itemdelete)(void* item) ){

    if (set == NULL) 
        return;
    for (element_t* it = set->head; 
                    it != NULL;) {
        // shortcut possible when compiling
        if (itemdelete != NULL) {
            (*itemdelete)(it->item);
        }
        // Key strings freed even if itemdelete is NULL:
        free(it->key);
        element_t* next = it->next;
        free(it);
        it = next;
    }
    free(set);
}

/**************** key_find ****************/
/* Return pointer to element with desired key if key exists in set,
 * and NULL if key does not exist in set, or if a given pointer is invalid: */
element_t*
key_find (set_t* set, const char* key) {

    if (set == NULL || key == NULL) 
        return NULL; 

    for (element_t* it = set->head; 
                    it!= NULL; 
                    it = it->next) {
        if (strncmp(it->key, key, strlen(it->key) + 1) == 0)
            return it; 
    }

    return NULL;
}

void set_merge(set_t* setA, set_t* setB)
{
    set_iterate(setB, setA, set_merge_helper);
}

void set_merge_helper(void* arg, const char* key, void* item)
{
    set_t * setA = arg;
    int * B_val  = item;
    int * A_val;

    /* if key found in setA, increment total value by value in B */
    if ((A_val = set_find(setA, key)) != NULL) 
        *A_val += *B_val;
    /* Otherwise insert in A key with value in B (duplicated!)*/
    else {
        int * B_val_copy = malloc(sizeof(int));
        *B_val_copy      = *B_val;
        set_insert(setA, key, B_val_copy);
    }     
}

typedef struct set_intsct_strct {
    set_t* setA;    // non-iterating set of intersection
    set_t* setC;    // resulting set from intersection
} set_inter_t;

set_t* set_intersect(set_t* setA, set_t* setB)
{
    /* Load struct on stack*/
    set_t * setC = set_new();
    set_inter_t sit;
    sit.setA   = setA;
    sit.setC   = setC;

    /* Iterate through setB */
    set_iterate(setB, &sit, set_intersect_helper); 

    /* return resulting set */
    return sit.setC;
}

void set_intersect_helper(void* arg, const char* key, void* item)
{
    /* Extract struct set pointers */
    set_inter_t * sit = arg;
    set_t * setA      = sit->setA;
    set_t * setC      = sit->setC;

    /* Check if the key is present in setA */
    int * A_val;
    int * B_val = item;
    if ((A_val = set_find(setA, key)) == NULL)
        return;
    /* Insert in setC the minimum value */
    int* min = malloc(sizeof(int));
    *min     = (*A_val < *B_val) ? *A_val : *B_val;
    set_insert(setC, key, min);
}
