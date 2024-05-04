
#pragma once

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <strings.h>
#include <stdlib.h>

/**************** global types ****************/
typedef struct set set_t;  // opaque to users of the module

/**************** functions ****************/

/**************** set_new ****************/
/* Create a new (empty) set.
 * 
 * We return:
 *   pointer to a new set, or NULL if error.
 * We guarantee:
 *   The set is initialized empty.
 * Caller is responsible for:
 *   later calling set_delete.
 */
extern set_t* set_new(void);

/**************** set_insert ****************/
/* Insert item, identified by a key (string), into the given set.
 *
 * Caller provides:
 *   valid set pointer, valid string pointer, and pointer to item.
 * We return:
 *  false if key exists, any parameter is NULL, or error;
 *  true iff new item was inserted.
 * Caller is responsible for:
 *   later calling set_delete
 * Notes:
 *   The key string is copied for use by the set; that is, the module
 *   is responsible for allocating memory for a copy of the key string, and
 *   later deallocating that memory; thus, the caller is free to re-use or 
 *   deallocate its key string after this call.  
 */
extern bool set_insert(set_t* set, const char* key, void* item);

/**************** set_find ****************/
/* Return the item associated with the given key.
 *
 * Caller provides:
 *   valid set pointer, valid string pointer.
 * We return:
 *   a pointer to the desired item, if found; 
 *   NULL if set is NULL, key is NULL, or key is not found.
 * Notes:
 *   The item is *not* removed from the set.
 *   Thus, the caller should *not* free the pointer that is returned.
 */
extern void* set_find(set_t* set, const char* key);

/**************** set_iterate ****************/
/* Iterate over the set, calling a function on each item.
 * 
 * Caller provides:
 *   valid set pointer,
 *   arbitrary argument (pointer) that is passed-through to itemfunc,
 *   valid pointer to function that handles one item.
 * We do:
 *   nothing, if set==NULL or itemfunc==NULL.
 *   otherwise, call the itemfunc on each item, with (arg, key, item).
 * Notes:
 *   the order in which set items are handled is undefined.
 *   the set and its contents are not changed by this function,
 *   but the itemfunc may change the contents of the item.
 */
extern void __attribute__ ((noinline)) set_iterate(set_t* set, void* arg,
                 void (*itemfunc)(void* arg, const char* key, void* item) );

/**************** set_print ****************/
/* Print the whole set; provide the output file and func to print each item.
 *
 * Caller provides:
 *   valid set pointer,
 *   FILE open for writing,
 *   valid pointer to function that prints one item.
 * We print:
 *   nothing if NULL fp. Print (null) if NULL set.
 *   print a set with no items if NULL itemprint. 
 *  otherwise, 
 *   print a comma-separated list of items surrounded by {brackets}.
 * Notes:
 *   The set and its contents are not changed.
 *   The 'itemprint' function is responsible for printing (key,item).
 */
extern void set_print(set_t* set, FILE* fp, 
               void (*itemprint)(FILE* fp, const char* key, void* item) );

/**************** set_delete ****************/
/* Delete set, calling a delete function on each item.
 *
 * Caller provides:
 *   valid set pointer,
 *   valid pointer to function that handles one item (may be NULL).
 * We do:
 *   if set==NULL, do nothing.
 *   otherwise, unless itemfunc==NULL, call the itemfunc on each item.
 *   free all the key strings, and the set itself.
 * Notes:
 *   We free the strings that represent key for each item, because 
 *   this module allocated that memory in set_insert.
 */
extern void set_delete(set_t* set, void (*itemdelete)(void* item) );


/* Utility functions for a set */

/* ******************* set_merge ************************************** */
/* This function is used to evaluate the union between two sets.
 *
 * We assume:
 *   Caller provides two non-NULL pointers to sets, with integer items.
 * 
 * We return:
 *   Nothing (void).
 *   
 * We guarantee:
 *   The fist set is changed to contain the union of both sets.
 *   The second set is unchanged.
 *   
 * Caller is responsible for:
 *   Later free'ing the set(s) of required.
 * 
 * Imporant:
 *   The value for each key in the union is given by the SUM of the
 *   key's values in the original sets.
 */
extern void set_merge(set_t* setA, set_t* setB);

/* ******************* set_intersect ************************************** */
/* This function is used to evaluate the intersection between two sets.
 *
 * We assume:
 *   Caller provides two non-NULL pointers to sets, with integer items.
 * 
 * We return:
 *   A set pointer to a new set containing the intersection of both sets.
 * 
 * We guarantee:
 *   Memory is malloc'd for the newly generated intersection set,
 *   and both sets are unchanged.
 *   
 * Caller is responsible for:
 *   Later free'ing the newly generated sets, 
 *   and the other sets if required.
 *   
 * Imporant:
 *   The value for each key in the intersection is given by the MINIMUM 
 *   of the key's values in the original sets.
 */
extern set_t* set_intersect(set_t* setA, set_t* setB);


