#include "uid_list.h"

bool uid_list_contains(const UidList *list, uint64_t uid) {
    for (int i = 0; i < list->count; i++) {
        if (list->items[i] == uid) {
            return true;
        }
    }
    return false;
}

void uid_list_add(UidList *list, uint64_t uid) {
    if (list->count < list->capacity) {
        list->items[list->count++] = uid;
    }
}

void uid_list_remove(UidList *list, uint64_t uid) {
    for (int i = 0; i < list->count; i++) {
        if (list->items[i] == uid) {
            list->count--;
            for (int j = i; j < list->count; j++) {
                list->items[j] = list->items[j + 1];
            }
        }
    }
}
