#ifndef UID_LIST_H_
#define UID_LIST_H_

#include <Arduino.h>

/**
 * A bounded list of UIDs, backed by a caller-provided array.
 */
typedef struct {
    uint64_t *items;
    uint8_t count;
    uint8_t capacity;
} UidList;

/**
 * Check if a UID is in the list.
 *
 * @param list  The list to search.
 * @param uid   UID to look for in the list.
 */
bool uid_list_contains(const UidList *list, uint64_t uid);

/**
 * Add a UID to the list.
 *
 * @param list  The list to add to.
 * @param uid   UID to add.
 */
void uid_list_add(UidList *list, uint64_t uid);

/**
 * Remove a UID from the list.
 *
 * @param list  The list to remove from.
 * @param uid   UID to remove from the list.
 */
void uid_list_remove(UidList *list, uint64_t uid);

#endif
