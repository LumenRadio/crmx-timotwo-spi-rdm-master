#include "uid_list.h"

bool uid_list_radio_is_in_list(uint64_t uid) {
    for (int i = 0; i < n_radios; i++) {
        if (radios[i] == uid) {
            return true;
        }
    }
    return false;
}

void uid_list_add_radio_to_list(uint64_t uid) {
    if (n_radios < UID_LIST_N_RADIOS) {
        radios[n_radios++] = uid;
    }
}

void uid_list_remove_radio_from_list(uint64_t uid) {
    for (int i = 0; i < n_radios; i++) {
        if (radios[i] == uid) {
            n_radios--;
            for (int j = i; j < n_radios; j++) {
                radios[j] = radios[j + 1];
            }
        }
    }
}

bool uid_list_rdm_device_is_in_list(uint64_t uid) {
    for (int i = 0; i < n_devices; i++) {
        if (rdm_devices[i] == uid) {
            return true;
        }
    }
    return false;
}

void uid_list_add_rdm_device_to_list(uint64_t uid) {
    if (n_devices < UID_LIST_N_DEVICES) {
        rdm_devices[n_devices++] = uid;
    }
}

void uid_list_remove_rdm_device_from_list(uint64_t uid) {
    for (int i = 0; i < n_devices; i++) {
        if (rdm_devices[i] == uid) {
            n_devices--;
            for (int j = i; j < n_devices; j++) {
                rdm_devices[j] = rdm_devices[j + 1];
            }
        }
    }
}
