#ifndef _ZARRAY_H
#define _ZARRAY_H

#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Defines a structure which acts as a resize-able array ala Java's ArrayList.
 */
typedef struct zarray zarray_t;

/**
 * Creates and returns a variable array structure capable of holding elements of
 * the specified size. It is the caller's responsibility to call zarray_destroy()
 * on the returned array when it is no longer needed.
 */
zarray_t *zarray_create(size_t el_sz);

/**
 * Frees all resources associated with the variable array structure which was
 * created by zarray_create(). After calling, 'za' will no longer be valid for storage.
 */
void zarray_destroy(zarray_t *za);

/** Allocate a new zarray that contains a copy of the data in the argument. **/
zarray_t *zarray_copy(const zarray_t *za);

/**
 * Retrieves the number of elements currently being contained by the passed
 * array, which may be different from its capacity. The index of the last element
 * in the array will be one less than the returned value.
 */
int zarray_size(const zarray_t *za);

/**
 * Allocates enough internal storage in the supplied variable array structure to
 * guarantee that the supplied number of elements (capacity) can be safely stored.
 */
void zarray_ensure_capacity(zarray_t *za, int capacity);

/**
 * Adds a new element to the end of the supplied array, and sets its value
 * (by copying) from the data pointed to by the supplied pointer 'p'.
 * Automatically ensures that enough storage space is available for the new element.
 */
void zarray_add(zarray_t *za, const void *p);

/**
 * Retrieves the element from the supplied array located at the zero-based
 * index of 'idx' and copies its value into the variable pointed to by the pointer
 * 'p'.
 */
void zarray_get(const zarray_t *za, int idx, void *p);

/**
 * Similar to zarray_get(), but returns a "live" pointer to the internal
 * storage, avoiding a memcpy. This pointer is not valid across
 * operations which might move memory around (i.e. zarray_remove_value(),
 * zarray_remove_index(), zarray_insert(), zarray_sort(), zarray_clear()).
 * 'p' should be a pointer to the pointer which will be set to the internal address.
 */
void zarray_get_volatile(const zarray_t *za, int idx, void *p);

/**
 * Remove the entry whose value is equal to the value pointed to by 'p'.
 * If shuffle is true, the last element in the array will be placed in
 * the newly-open space; if false, the zarray is compacted. At most
 * one element will be removed.
 *
 * Note that objects will be compared using memcmp over the full size
 * of the value. If the value is a struct that contains padding,
 * differences in the padding bytes can cause comparisons to
 * fail. Thus, it remains best practice to bzero all structs so that
 * the padding is set to zero.
 *
 * Returns the number of elements removed (0 or 1).
 */
int zarray_remove_value(zarray_t *za, const void *p, int shuffle);

/**
 * Removes the entry at index 'idx'.
 * If shuffle is true, the last element in the array will be placed in
 * the newly-open space; if false, the zarray is compacted.
 */
void zarray_remove_index(zarray_t *za, int idx, int shuffle);

/**
 * Creates a new entry and inserts it into the array so that it will have the
 * index 'idx' (i.e. before the item which currently has that index). The value
 * of the new entry is set to (copied from) the data pointed to by 'p'. 'idx'
 * can be one larger than the current max index to place the new item at the end
 * of the array, or zero to add it to an empty array.
 */
void zarray_insert(zarray_t *za, int idx, const void *p);

/**
 * Sets the value of the current element at index 'idx' by copying its value from
 * the data pointed to by 'p'. The previous value of the changed element will be
 * copied into the data pointed to by 'outp' if it is not null.
 */
void zarray_set(zarray_t *za, int idx, const void *p, void *outp);

/**
 * Calls the supplied function for every element in the array in index order.
 * The map function will be passed a pointer to each element in turn and must
 * have the following format:
 *
 * void map_function(element_type *element)
 */
void zarray_map(zarray_t *za, void (*f)());

/**
 * Calls the supplied function for every element in the array in index order.
 * HOWEVER values are passed to the function, not pointers to values. In the
 * case where the zarray stores object pointers, zarray_vmap allows you to
 * pass in the object's destroy function (or free) directly. Can only be used
 * with zarray's which contain pointer data. The map function should have the
 * following format:
 *
 * void map_function(element_type *element)
 */
void zarray_vmap(zarray_t *za, void (*f)());

/**
 * Removes all elements from the array and sets its size to zero. Pointers to
 * any data elements obtained i.e. by zarray_get_volatile() will no longer be
 * valid.
 */
void zarray_clear(zarray_t *za);

/**
 * Determines whether any element in the array has a value which matches the
 * data pointed to by 'p'.
 *
 * Returns 1 if a match was found anywhere in the array, else 0.
 */
int zarray_contains(const zarray_t *za, const void *p);

/**
 * Uses qsort() to sort the elements contained by the array in ascending order.
 * Uses the supplied comparison function to determine the appropriate order.
 *
 * The comparison function will be passed a pointer to two elements to be compared
 * and should return a measure of the difference between them (see strcmp()).
 * I.e. it should return a negative number if the first element is 'less than'
 * the second, zero if they are equivalent, and a positive number if the first
 * element is 'greater than' the second. The function should have the following format:
 *
 * int comparison_function(const element_type *first, const element_type *second)
 *
 * zstrcmp() can be used as the comparison function for string elements, which
 * will call strcmp() internally.
 */
void zarray_sort(zarray_t *za, int (*compar)(const void*, const void*));

/**
 * A comparison function for comparing strings which can be used by zarray_sort()
 * to sort arrays with char* elements.
 */
int zstrcmp(const void * a_pp, const void * b_pp);

/**
  * Find the index of an element, or return -1 if not found. Remember that p is
  * a pointer to the element.
 **/
 int zarray_index_of(const zarray_t *za, const void *p);

#ifdef __cplusplus
}
#endif

#endif
