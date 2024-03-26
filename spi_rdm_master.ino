/**
 * This example provides a very simple RDM controller using the TimoTwo module's SPI RDM TX function.
 * It can be used in CRMX mode or W-DMX G4S mode, and is controlled be a variable set below.
 * This example does not implements any error checking of missed responses, etc. This must be 
 * added in a real-world implementation.
 */

#include <SPI.h>
#include "timo_spi.h"
#include "e120.h"

/* This variable sets if we are going to run in G4S mode or CRMX mode, uncomment the one you want */
uint8_t rf_protocol = TIMO_RF_PROTO_G4S;
//uint8_t rf_protocol = TIMO_RF_PROTO_CRMX;

/* The UID to use for this controller */
uint8_t my_uid[6] = {0x4c, 0x55, 0x00, 0x00, 0x00, 0x12};

/* Define list sizes */
#define N_RADIOS 20
#define N_DEVICES 50

#define  MIN( A,  B )      ( ( A )  <  ( B )   ?   ( A )   :   ( B ) )

#define SWAP16(x) ((uint16_t)(((uint16_t)x >> 8) | ((uint16_t)x << 8)))

/* Define where the TimoTwo module is connected */
static timo_t timo = { .csn_pin =  5, .irq_pin = 3 };

/* SPI communication buffers, make sure these are aligned to 32 bit words */
static uint32_t tx_buffer32[300/4];
static uint32_t rx_buffer32[300/4];
/* point the working byte pointers to the aligned buffers */
static uint8_t* tx_buffer = (uint8_t*)tx_buffer32;
static uint8_t* rx_buffer = (uint8_t*)rx_buffer32;

static bool has_set_up = false;

typedef enum {
  DiscUid,
  DiscCollission,
  DiscNone,
} DiscoveryResponseType;

/* Set up the radio and RDM device lists */
uint64_t radios[N_RADIOS] = {0, 0};
uint8_t n_radios = 0;
uint64_t rdm_devices[N_DEVICES] = {0, 0};
uint8_t n_devices = 0;

/* This variable holds the RDM transaction number, shall be incremented for each new message */
uint8_t rdm_tn = 0;

/**
 * Check if a radio is in our list.
 *
 * @param uid   UID of the receiver to look for in list.
 */
bool radio_is_in_list(uint64_t uid) {
  for (int i=0; i<n_radios; i++) {
    if (radios[i] == uid) {
      return true;
    }
  }
  return false;
}

/**
 * Add a receiver to the list.
 *
 * @param uid   UID of the receiver to add.
 */
void add_radio_to_list(uint64_t uid) {
  if (n_radios < N_RADIOS) {
    radios[n_radios++] = uid;
  }
}

/**
 * Remove a receiver from the list.
 *
 * @param uid   UID of the receiver to remove from list.
 */
void remove_radio_from_list(uint64_t uid) {
  for (int i=0; i<n_radios; i++) {
    if (radios[i] == uid) {
      n_radios--;
      for (int j=i; j<n_radios; j++) {
        radios[j] = radios[j+1];
      }
    }
  }
}

/**
 * Check if RDM device is in our list.
 *
 * @param uid   UID of the RDM device to look for in list.
 */
bool rdm_device_is_in_list(uint64_t uid) {
  for (int i=0; i<n_devices; i++) {
    if (rdm_devices[i] == uid) {
      return true;
    }
  }
  return false;
}

/**
 * Add a RDM device to the list.
 *
 * @param uid   UID of the RDM device to add to list.
 */
void add_rdm_device_to_list(uint64_t uid) {
  if (n_devices < N_DEVICES) {
    rdm_devices[n_devices++] = uid;
  }
}

/**
 * Remove a RDM device from the list.
 *
 * @param uid   UID of the RDM device to remove from list.
 */
void remove_rdm_device_from_list(uint64_t uid) {
  for (int i=0; i<n_devices; i++) {
    if (rdm_devices[i] == uid) {
      n_devices--;
      for (int j=i; j<n_devices; j++) {
        rdm_devices[j] = rdm_devices[j+1];
      }
    }
  }
}

void irq_pin_handler() {
  timo.irq_pending = 1;
}

bool irq_is_pending() {
  noInterrupts();
  bool pending = timo.irq_pending;
  timo.irq_pending = false;
  interrupts();
  return pending;
}

/**
 * This is the Arduino setup function, it's called when the Arduino starts up 
 */
void setup() {
  int16_t irq_flags;
  
  /* Initiate serial port to 115200 bps */
  Serial.begin(115200);

  /* Initiate SPI */
  SPI.begin();

  /* Setup IRQ and CS pins */
  pinMode(timo.irq_pin, INPUT);
  pinMode(timo.csn_pin, OUTPUT);
  digitalWrite(timo.csn_pin, HIGH);
  attachInterrupt(digitalPinToInterrupt(timo.irq_pin), irq_pin_handler, FALLING);

  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));

  delay(1000);

  /* Clear the Serial port from any garbage bytes */
  while(Serial.available() > 0) {
    Serial.read();
  }

  Serial.println("Running");
  Serial.flush();

  /* Wait here until module has booted and IRQ signal is high */
  while (irq_is_pending()) {
    ;
  }

  Serial.println("Version:");
  irq_flags = timo_transfer(TIMO_READ_REG_COMMAND(TIMO_VERSION_REG), rx_buffer, tx_buffer, 9);
  print_response(irq_flags, rx_buffer, 8);

  /* Making sure module is in TX mode */
  Serial.println("Config:");
  irq_flags = timo_transfer(TIMO_READ_REG_COMMAND(TIMO_CONFIG_REG), rx_buffer, tx_buffer, 2);
  print_response(irq_flags, rx_buffer, 1);
  while ((rx_buffer[0] & TIMO_CONFIG_RADIO_TX_RX_MODE) == 0) {
    Serial.println("In RX mode - changing mode");
    /* in RX mode - change to TX */
    tx_buffer[0] = rx_buffer[0] | TIMO_CONFIG_RADIO_TX_RX_MODE;
    timo_transfer(TIMO_WRITE_REG_COMMAND(TIMO_CONFIG_REG), rx_buffer, tx_buffer, 2);
    delay(3000);
    irq_flags = timo_transfer(TIMO_READ_REG_COMMAND(TIMO_CONFIG_REG), rx_buffer, tx_buffer, 2);
    print_response(irq_flags, rx_buffer, 1);
  }

  /* Making sure we have the RDM SPI TX option installed, assume it being among the first 5 options, so only read 10 bytes */
  Serial.println("Options:");
  irq_flags = timo_transfer(TIMO_READ_REG_COMMAND(TIMO_INSTALLED_OPTIONS_REG), rx_buffer, tx_buffer, 10);
  print_response(irq_flags, rx_buffer, 1);
  Serial.print("Module has "); Serial.print(rx_buffer[0]); Serial.println(" installed option(s)."); 
  int option_counter;
  for (option_counter=0; option_counter<rx_buffer[0]; option_counter++) {
    uint16_t option = ((uint16_t)rx_buffer[1+option_counter*2+1] << 8) | (uint16_t)rx_buffer[1+option_counter*2];
    if (option == TIMO_INSTALLED_OPTION_RDM_SPI) {
      Serial.println("RDM SPI TX option found."); 
      break;
    }
  }
  if (option_counter == rx_buffer[0]) {
    /* Reached end of list - our option not found in list */
    Serial.println("Option for SPI RDM TX not found...");
    while (1) {
      ;
    }
  }

  /* Turning off BLE - this is important when running W-DMX as old receivers does not deal with BLE */
  Serial.println("BLE:");
  irq_flags = timo_transfer(TIMO_READ_REG_COMMAND(TIMO_BLE_STATUS_REG), rx_buffer, tx_buffer, 2);
  print_response(irq_flags, rx_buffer, 1);
  while (rx_buffer[0] & TIMO_BLE_ENABLED) {
    Serial.println("BLE is on - turning off");
    tx_buffer[0] = 0x00;
    timo_transfer(TIMO_WRITE_REG_COMMAND(TIMO_BLE_STATUS_REG), rx_buffer, tx_buffer, 2);
    delay(3000);
    irq_flags = timo_transfer(TIMO_READ_REG_COMMAND(TIMO_BLE_STATUS_REG), rx_buffer, tx_buffer, 2);
    print_response(irq_flags, rx_buffer, 1);
  }

  /* Making sure the module is configured for the correct protocol */
  Serial.println("RF Protocol:");
  irq_flags = timo_transfer(TIMO_READ_REG_COMMAND(TIMO_RF_PROTOCOL_REG), rx_buffer, tx_buffer, 2);
  print_response(irq_flags, rx_buffer, 1);
  while (rx_buffer[0] != rf_protocol) {
    Serial.println("Configured for wrong protocol - changing to G4S");
    tx_buffer[0] = rf_protocol;
    timo_transfer(TIMO_WRITE_REG_COMMAND(TIMO_RF_PROTOCOL_REG), rx_buffer, tx_buffer, 2);
    delay(3000);
    irq_flags = timo_transfer(TIMO_READ_REG_COMMAND(TIMO_RF_PROTOCOL_REG), rx_buffer, tx_buffer, 2);
    print_response(irq_flags, rx_buffer, 1);
  }


  Serial.println("Status:");
  irq_flags = timo_transfer(TIMO_READ_REG_COMMAND(TIMO_STATUS_REG), rx_buffer, tx_buffer, 2);
  print_response(irq_flags, rx_buffer, 1);

  /* Enabling the extended IRQs */
  Serial.println("IRQ mask:");
  tx_buffer[0] = TIMO_IRQ_EXTENDED_FLAG;
  irq_flags = timo_transfer(TIMO_WRITE_REG_COMMAND(TIMO_IRQ_MASK_REG), rx_buffer, tx_buffer, 2);
  irq_flags = timo_transfer(TIMO_READ_REG_COMMAND(TIMO_IRQ_MASK_REG), rx_buffer, tx_buffer, 2);
  print_response(irq_flags, rx_buffer, 1);

  /* Enabling interrupts for Discovery (Radio and RDM), Mute and RDM */
  Serial.println("Extended IRQ mask:");
  tx_buffer[0] = 0;
  tx_buffer[1] = 0;
  tx_buffer[2] = 0;
  tx_buffer[3] = TIMO_EXTIRQ_SPI_RADIO_DISC_FLAG | TIMO_EXTIRQ_SPI_RADIO_MUTE_FLAG | TIMO_EXTIRQ_SPI_RDM_DISC_FLAG | TIMO_EXTIRQ_SPI_RDM_FLAG;
  irq_flags = timo_transfer(TIMO_WRITE_REG_COMMAND(TIMO_EXT_IRQ_MASK_REG), rx_buffer, tx_buffer, 5);
  irq_flags = timo_transfer(TIMO_READ_REG_COMMAND(TIMO_EXT_IRQ_MASK_REG), rx_buffer, tx_buffer, 5);
  print_response(irq_flags, rx_buffer, 4);

  disc_all(false);
}

/**
 * This function waits for an RDM response to be indicated. 
 */
void wait_for_rdm_response(void) {
  int16_t irq_flags;
  
  while (1) {
    /* module indicates IRQ */
    if (!digitalRead(timo.irq_pin)) {
      /* Issue NOP - this returs the IRQ flags */
      irq_flags = timo_transfer(TIMO_NOP_COMMAND, rx_buffer, tx_buffer, 0);
  
      while (!irq_is_pending()) {
        ;
      }
  
      /* if it's an extended IRQ it can be the RDM IRQ */
      if (irq_flags & TIMO_IRQ_EXTENDED_FLAG) {
        uint32_t ext_flags;
        uint8_t response_length;
        bzero(tx_buffer, 5);
        /* read the extended flags register */
        irq_flags = timo_transfer(TIMO_READ_REG_COMMAND(TIMO_EXT_IRQ_FLAGS_REG), rx_buffer, tx_buffer, 5);
        ext_flags = ((uint32_t)rx_buffer[0] << 24) | ((uint32_t)rx_buffer[1] << 16) | ((uint32_t)rx_buffer[2] << 8) | ((uint32_t)rx_buffer[3]);
        if (ext_flags & TIMO_EXTIRQ_SPI_RDM_FLAG) {
          /* it was the RDM IRQ */
          while (!irq_is_pending()) {
            ;
          }
          return;
        }
      }
    }
  }
}


void wait_for_radio_discovery_response(void) {
  int16_t irq_flags;
  
  while (1) {
    if (!digitalRead(timo.irq_pin)) {
      irq_flags = timo_transfer(TIMO_NOP_COMMAND, rx_buffer, tx_buffer, 0);
  
      while (!irq_is_pending()) {
        ;
      }
  
      if (irq_flags & TIMO_IRQ_EXTENDED_FLAG) {
        uint32_t ext_flags;
        uint8_t response_length;
        bzero(tx_buffer, 5);
        irq_flags = timo_transfer(TIMO_READ_REG_COMMAND(TIMO_EXT_IRQ_FLAGS_REG), rx_buffer, tx_buffer, 5);
        ext_flags = ((uint32_t)rx_buffer[0] << 24) | ((uint32_t)rx_buffer[1] << 16) | ((uint32_t)rx_buffer[2] << 8) | ((uint32_t)rx_buffer[3]);
        if (ext_flags & TIMO_EXTIRQ_SPI_RADIO_DISC_FLAG) {
          while (!irq_is_pending()) {
            ;
          }
  
          return;
        }
      }
    }
  }
}

/**
 * This function waits for a radio mute response to be indicated. 
 */
void wait_for_radio_mute_response(void) {
  int16_t irq_flags;
  
  while (1) {
    if (!digitalRead(timo.irq_pin)) {
      irq_flags = timo_transfer(TIMO_NOP_COMMAND, rx_buffer, tx_buffer, 0);
  
      while (!irq_is_pending()) {
        ;
      }
  
      if (irq_flags & TIMO_IRQ_EXTENDED_FLAG) {
        uint32_t ext_flags;
        uint8_t response_length;
        bzero(tx_buffer, 5);
        irq_flags = timo_transfer(TIMO_READ_REG_COMMAND(TIMO_EXT_IRQ_FLAGS_REG), rx_buffer, tx_buffer, 5);
        ext_flags = ((uint32_t)rx_buffer[0] << 24) | ((uint32_t)rx_buffer[1] << 16) | ((uint32_t)rx_buffer[2] << 8) | ((uint32_t)rx_buffer[3]);
        if (ext_flags & TIMO_EXTIRQ_SPI_RADIO_MUTE_FLAG) {
          while (!irq_is_pending()) {
            ;
          }
  
          return;
        }
      }
    }
  }
}

/**
 * This function waits for an RDM discovery response to be indicated. 
 */
void wait_for_rdm_discovery_response(void) {
  int16_t irq_flags;
  
  while (1) {
    if (!digitalRead(timo.irq_pin)) {
      irq_flags = timo_transfer(TIMO_NOP_COMMAND, rx_buffer, tx_buffer, 0);
  
      while (!irq_is_pending()) {
        ;
      }
  
      if (irq_flags & TIMO_IRQ_EXTENDED_FLAG) {
        uint32_t ext_flags;
        uint8_t response_length;
        bzero(tx_buffer, 5);
        irq_flags = timo_transfer(TIMO_READ_REG_COMMAND(TIMO_EXT_IRQ_FLAGS_REG), rx_buffer, tx_buffer, 5);
        ext_flags = ((uint32_t)rx_buffer[0] << 24) | ((uint32_t)rx_buffer[1] << 16) | ((uint32_t)rx_buffer[2] << 8) | ((uint32_t)rx_buffer[3]);
        if (ext_flags & TIMO_EXTIRQ_SPI_RDM_DISC_FLAG) {
          while (!irq_is_pending()) {
            ;
          }
  
          return;
        }
      }
    }
  }
}

/**
 * This function waits for an RDM response to be indicated. 
 *
 * @param uid   The UID to print
 */
void print_uid(uint64_t uid) {
  for (int i = 11; i >= 0; i--) {
    Serial.print(nibble_to_hex(uid >> (4*i)));
  }
   Serial.flush();
}

/**
 * Issue a radio DISCOVERY UNIQUE BRANCH. 
 *
 * Return values:
 *    DiscNone         Empty response
 *    DiscCollission   Collission
 *    DiscUid          Single UID response
 *
 * @param lower   The lower end of the search span
 * @param upper   The upper end of the search span
 * @param *uid    Pointer to where to store the UID to in case of a signel response
 */
DiscoveryResponseType radio_dub(uint64_t lower, uint64_t upper, uint64_t *uid) {
  
  Serial.print("Radio DUB (");
  print_uid(lower);
  
  Serial.print(":");
  print_uid(upper);
  Serial.println(")...");
  
  tx_buffer[0] = lower >> 40;
  tx_buffer[1] = lower >> 32;
  tx_buffer[2] = lower >> 24;
  tx_buffer[3] = lower >> 16;
  tx_buffer[4] = lower >> 8;
  tx_buffer[5] = lower;
  tx_buffer[6] = upper >> 40;
  tx_buffer[7] = upper >> 32;
  tx_buffer[8] = upper >> 24;
  tx_buffer[9] = upper >> 16;
  tx_buffer[10] = upper >> 8;
  tx_buffer[11] = upper;
  timo_transfer(TIMO_RADIO_DISCOVERY, rx_buffer, tx_buffer, 13);

  wait_for_radio_discovery_response();

  timo_transfer(TIMO_RADIO_DISCOVERY_RESULT, rx_buffer, tx_buffer, 8);

  if (rx_buffer[0] == 1) {
    Serial.println("No response.");
    return DiscNone;
  } else if (rx_buffer[0] == 2) {
    Serial.println("Collission.");
    return DiscCollission;
  } else if (rx_buffer[0] == 3) {
    uint64_t found_uid = 0;
    Serial.print("Found dev: ");
    for (int i = 0; i<6; i++) {
      found_uid = found_uid << 8;
      found_uid |= rx_buffer[1+i];
    }
    print_uid(found_uid);
    Serial.println();
    *uid = found_uid;
    return DiscUid;
  }

  return DiscNone;
}

/**
 * Try to mute a radio. 
 *
 * Return values:
 *    false         Mute failed
 *    true          Radio muted
 *
 * @param uid   The UID of the radio to mute
 */
bool mute_radio(uint64_t uid) {
  
  Serial.print("Trying to mute ");
  print_uid(uid);
  Serial.println("...");
  
  tx_buffer[0] = uid >> 40;
  tx_buffer[1] = uid >> 32;
  tx_buffer[2] = uid >> 24;
  tx_buffer[3] = uid >> 16;
  tx_buffer[4] = uid >> 8;
  tx_buffer[5] = uid;
  tx_buffer[6] = 1;
  timo_transfer(TIMO_RADIO_MUTE, rx_buffer, tx_buffer, 8);

  wait_for_radio_mute_response();

  timo_transfer(TIMO_RADIO_MUTE_RESPONSE, rx_buffer, tx_buffer, 2);

  if (rx_buffer[0]) {
    Serial.println("Muted.");
    return true;
  } else {
    Serial.println("Failed to mute.");
    return false;
  }
}

/**
 * Unmute all radios. 
 */
void unmute_all_radios(void) {
  
  Serial.println("Unmuting all radios...");
  
  tx_buffer[0] = 0xFF;
  tx_buffer[1] = 0xFF;
  tx_buffer[2] = 0xFF;
  tx_buffer[3] = 0xFF;
  tx_buffer[4] = 0xFF;
  tx_buffer[5] = 0xFF;
  tx_buffer[6] = 0;
  timo_transfer(TIMO_RADIO_MUTE, rx_buffer, tx_buffer, 8);

  wait_for_radio_mute_response();

  timo_transfer(TIMO_RADIO_MUTE_RESPONSE, rx_buffer, tx_buffer, 2);
}

/**
 * Performes a radio discovery on a part of the binary search tree. 
 *
 * Return value:  The number of radios found in this part of the tree.
 *
 * @param lower   The lower end of the search span
 * @param upper   The upper end of the search span
 */
uint16_t radio_discover_sub_tree(uint64_t lower, uint64_t upper) {
  DiscoveryResponseType dub_resp;
  uint16_t n_found = 0;
  uint64_t uid;
  
  uint8_t n_retries = 1;

  do {
    /* Send a DUB. If we get no response, try again if there are retries left */
    do {
      dub_resp = radio_dub(lower, upper, &uid);
    } while ((dub_resp == DiscNone) && (n_retries-- > 0));
  
    /* If we still did not get any result - there is no radios in this part of the binary tree. */
    if (dub_resp == DiscNone) {
      return n_found;
    }

    /* If there was collissions we need to branch further down in the binary tree */
    if (dub_resp == DiscCollission) {
 	    uint64_t mid = (lower + upper) / 2;
      /* the total amount of devices found in this part of the tree is the sum of:
       * the number of devices we already found + the number of devices in each half of the tree */
      return n_found + radio_discover_sub_tree(lower, mid) + radio_discover_sub_tree(mid + 1, upper);
    }

    /* if we got a single response we will try to mute it to verify it's a true response */
    if (dub_resp == DiscUid) {
      if (mute_radio(uid)) {
        /* if we could mute it and we do not already have it in the list - add it. */
        if (!radio_is_in_list(uid)) {
          add_radio_to_list(uid);
          n_found++;
        }
      }
    }
    
  } while (dub_resp == DiscUid); /* keep trying while we're getting single responses */

  return n_found;
}

/**
 * Issue a radio DISCOVERY UNIQUE BRANCH. 
 *
 * Return values:
 *    DiscNone         Empty response
 *    DiscCollission   Collission
 *    DiscUid          Single UID response
 *
 * @param rx      The UID of the receiver that we are performing the discover via.
 * @param lower   The lower end of the search span
 * @param upper   The upper end of the search span
 * @param *uid    Pointer to where to store the UID to in case of a signel response
 */
DiscoveryResponseType rdm_dub(uint64_t rx, uint64_t lower, uint64_t upper, uint64_t *uid) {
  
  Serial.print("RDM DUB (");
  print_uid(lower);
  
  Serial.print(":");
  print_uid(upper);
  Serial.println(")...");

  tx_buffer[0] = rx >> 40;
  tx_buffer[1] = rx >> 32;
  tx_buffer[2] = rx >> 24;
  tx_buffer[3] = rx >> 16;
  tx_buffer[4] = rx >> 8;
  tx_buffer[5] = rx;
  tx_buffer[6] = lower >> 40;
  tx_buffer[7] = lower >> 32;
  tx_buffer[8] = lower >> 24;
  tx_buffer[9] = lower >> 16;
  tx_buffer[10] = lower >> 8;
  tx_buffer[11] = lower;
  tx_buffer[12] = upper >> 40;
  tx_buffer[13] = upper >> 32;
  tx_buffer[14] = upper >> 24;
  tx_buffer[15] = upper >> 16;
  tx_buffer[16] = upper >> 8;
  tx_buffer[17] = upper;
  timo_transfer(TIMO_RDM_DISCOVERY, rx_buffer, tx_buffer, 19);

  wait_for_rdm_discovery_response();

  timo_transfer(TIMO_RDM_DISCOVERY_RESULT, rx_buffer, tx_buffer, 8);

  if (rx_buffer[0] == 1) {
    Serial.println("No response.");
    return DiscNone;
  } else if (rx_buffer[0] == 2) {
    Serial.println("Collission.");
    return DiscCollission;
  } else if (rx_buffer[0] == 3) {
    uint64_t found_uid = 0;
    Serial.print("Found dev: ");
    for (int i = 0; i<6; i++) {
      found_uid = found_uid << 8;
      found_uid |= rx_buffer[1+i];
    }
    print_uid(found_uid);
    Serial.println();
    *uid = found_uid;
    return DiscUid;
  }

  return DiscNone;
}

/**
 * Fills an RDM packet with all the standard values. 
 *
 * @param *r      Pointer to the request message.
 * @param dest    The UID of the device we want to send this request to.
 */
void fill_rdm_packet(RdmRequest *r, uint64_t dest) {
  r->startCode = SC_RDM;
  r->subStartCode = SC_SUB_MESSAGE;
  r->transactionNumber = ++rdm_tn;
  r->portId = 1;
  r->messageCount = 0;
  r->subDevice = 0;
  memcpy(r->sourceUid, my_uid, 6);
  r->destinationUid[0] = dest >> 40;
  r->destinationUid[1] = dest >> 32;
  r->destinationUid[2] = dest >> 24;
  r->destinationUid[3] = dest >> 16;
  r->destinationUid[4] = dest >> 8;
  r->destinationUid[5] = dest ;
}

/**
 * Calculates the total RDM message length and checksum. 
 *
 * @param *r      Pointer to the request message.
 */
void set_rdm_length_and_checksum(RdmRequest *r) {
  uint16_t sum = 0;

  r->messageLength = 24 + r->parameterDataLength;

  for(int i=0; i<r->messageLength; i++) {
    sum += ((uint8_t*)r)[i];
  }

  ((uint8_t*)r)[r->messageLength] = sum >> 8;
  ((uint8_t*)r)[r->messageLength + 1] = sum;
}

/**
 * Unmute all RDM devices. 
 */
void unmute_all() {
  RdmRequest req;

  fill_rdm_packet(&req, 0x00FFFFFFFFFFFF);
  
  req.commandClass = DISCOVERY_COMMAND;
  req.parameterId = SWAP16(DISC_UN_MUTE);
  req.parameterDataLength = 0;

  set_rdm_length_and_checksum(&req);

  tx_buffer[0] = 0xFF;
  tx_buffer[1] = 0xFF;
  tx_buffer[2] = 0xFF;
  tx_buffer[3] = 0xFF;
  tx_buffer[4] = 0xFF;
  tx_buffer[5] = 0xFF;

  memcpy(tx_buffer+6, &req, req.messageLength+2);

  timo_transfer(TIMO_WRITE_RDM_COMMAND, rx_buffer, tx_buffer, 1+6+req.messageLength+2);

  wait_for_rdm_response();

  timo_transfer_rdm_response(TIMO_READ_RDM_COMMAND, rx_buffer, tx_buffer, 260);

}

/**
 * Prints the MANUFACTURER_LABEL of an RDM device. 
 *
 * @param rx      The UID of the receiver the RDM device is connected to.
 * @param uid     The UID of the RDM device.
 */
void print_manufacturer_label(uint64_t rx, uint64_t uid) {
  RdmRequest req;
  RdmResponse resp;

  fill_rdm_packet(&req, uid);
  
  req.commandClass = GET_COMMAND;
  req.parameterId = SWAP16(MANUFACTURER_LABEL);
  req.parameterDataLength = 0;

  set_rdm_length_and_checksum(&req);

  tx_buffer[0] = rx >> 40;
  tx_buffer[1] = rx >> 32;
  tx_buffer[2] = rx >> 24;
  tx_buffer[3] = rx >> 16;
  tx_buffer[4] = rx >> 8;
  tx_buffer[5] = rx ;

  memcpy(tx_buffer+6, &req, req.messageLength+2);

  timo_transfer(TIMO_WRITE_RDM_COMMAND, rx_buffer, tx_buffer, 1+6+req.messageLength+2);

  wait_for_rdm_response();

  timo_transfer_rdm_response(TIMO_READ_RDM_COMMAND, rx_buffer, tx_buffer, 260);

  /* result code 1 means we got an answer */
  if (rx_buffer[0] == 1) {
    memcpy(&resp, rx_buffer+1, rx_buffer[3]+2);
    resp.parameterId = SWAP16(resp.parameterId);

    /* check for corrrect response */
    if ((resp.responseType == RESPONSE_TYPE_ACK) && (resp.parameterId == MANUFACTURER_LABEL)) {
      for (int i=0; i<resp.parameterDataLength; i++) {
        Serial.print((char)resp.parameterData[i]);
      }
    }
  }
}

/**
 * Prints the DEVICE_MODEL_DESCRIPTION of an RDM device. 
 *
 * @param rx      The UID of the receiver the RDM device is connected to.
 * @param uid     The UID of the RDM device.
 */
void print_device_model_description(uint64_t rx, uint64_t uid) {
  RdmRequest req;
  RdmResponse resp;

  fill_rdm_packet(&req, uid);
  
  req.commandClass = GET_COMMAND;
  req.parameterId = SWAP16(DEVICE_MODEL_DESCRIPTION);
  req.parameterDataLength = 0;

  set_rdm_length_and_checksum(&req);

  tx_buffer[0] = rx >> 40;
  tx_buffer[1] = rx >> 32;
  tx_buffer[2] = rx >> 24;
  tx_buffer[3] = rx >> 16;
  tx_buffer[4] = rx >> 8;
  tx_buffer[5] = rx ;

  memcpy(tx_buffer+6, &req, req.messageLength+2);

  timo_transfer(TIMO_WRITE_RDM_COMMAND, rx_buffer, tx_buffer, 1+6+req.messageLength+2);

  wait_for_rdm_response();

  timo_transfer_rdm_response(TIMO_READ_RDM_COMMAND, rx_buffer, tx_buffer, 260);

  /* result code 1 means we got an answer */
  if (rx_buffer[0] == 1) {
    memcpy(&resp, rx_buffer+1, rx_buffer[3]+2);
    resp.parameterId = SWAP16(resp.parameterId);

    /* check for corrrect response */
    if ((resp.responseType == RESPONSE_TYPE_ACK) && (resp.parameterId == DEVICE_MODEL_DESCRIPTION)) {
      for (int i=0; i<resp.parameterDataLength; i++) {
        Serial.print((char)resp.parameterData[i]);
      }
    }
  }
}

/**
 * Sends IDENTIFY_DEVICE to an RDM device. 
 *
 * @param rx      The UID of the receiver the RDM device is connected to.
 * @param uid     The UID of the RDM device.
 * @param state   Identify state - true = on, false = off
 */
void identify(uint64_t rx, uint64_t uid, bool state) {
  RdmRequest req;
  RdmResponse resp;

  fill_rdm_packet(&req, uid);
  
  req.commandClass = SET_COMMAND;
  req.parameterId = SWAP16(IDENTIFY_DEVICE);
  req.parameterDataLength = 1;
  req.parameterData[0] = state ? 1 : 0;

  set_rdm_length_and_checksum(&req);

  tx_buffer[0] = rx >> 40;
  tx_buffer[1] = rx >> 32;
  tx_buffer[2] = rx >> 24;
  tx_buffer[3] = rx >> 16;
  tx_buffer[4] = rx >> 8;
  tx_buffer[5] = rx ;

  memcpy(tx_buffer+6, &req, req.messageLength+2);

  timo_transfer(TIMO_WRITE_RDM_COMMAND, rx_buffer, tx_buffer, 1+6+req.messageLength+2);

  wait_for_rdm_response();

  timo_transfer_rdm_response(TIMO_READ_RDM_COMMAND, rx_buffer, tx_buffer, 260);
}

/**
 * Tries to mute an RDM device. 
 *
 * Return value:  Returns if device could successfully be muted.
 *
 * @param rx      The UID of the receiver the RDM device is connected to.
 * @param uid     The UID of the RDM device.
 */
bool mute_rdm_device(uint64_t rx, uint64_t uid) {
  RdmRequest req;
  RdmResponse resp;

  fill_rdm_packet(&req, uid);
  
  req.commandClass = DISCOVERY_COMMAND;
  req.parameterId = SWAP16(DISC_MUTE);
  req.parameterDataLength = 0;

  set_rdm_length_and_checksum(&req);

  tx_buffer[0] = rx >> 40;
  tx_buffer[1] = rx >> 32;
  tx_buffer[2] = rx >> 24;
  tx_buffer[3] = rx >> 16;
  tx_buffer[4] = rx >> 8;
  tx_buffer[5] = rx ;

  memcpy(tx_buffer+6, &req, req.messageLength+2);

  timo_transfer(TIMO_WRITE_RDM_COMMAND, rx_buffer, tx_buffer, 1+6+req.messageLength+2);

  wait_for_rdm_response();

  timo_transfer_rdm_response(TIMO_READ_RDM_COMMAND, rx_buffer, tx_buffer, 260);

  /* result code 1 means we got an answer */
  if (rx_buffer[0] == 1) {
    memcpy(&resp, rx_buffer+1, rx_buffer[3]+2);
    resp.parameterId = SWAP16(resp.parameterId);

    /* check for corrrect response */
    if ((resp.responseType == RESPONSE_TYPE_ACK) && (resp.parameterId == DISC_MUTE)) {
      return true;
    }
  }

  return false;
}

/**
 * Performes a RDM discovery on a part of the binary search tree. 
 *
 * Return value:  The number of radios found in this part of the tree.
 *
 * @param rx      The receiver to perform the discover via.
 * @param lower   The lower end of the search span
 * @param upper   The upper end of the search span
 */
uint16_t rdm_discover_sub_tree(uint64_t rx, uint64_t lower, uint64_t upper) {
  DiscoveryResponseType dub_resp;
  uint16_t n_found = 0;
  uint64_t uid;
  
  uint8_t n_retries = 1;

  do {
    /* If lower and upper are the same, it means we are at the bottom of the tree. then try to mute that device */
    if (lower == upper) {
      Serial.print("At leaf... Try to mute "); print_uid(lower); Serial.println();
      uid = lower;
      if (mute_rdm_device(rx, uid)) {
        /* add the device to the list if it was not already in the list */
        if (!rdm_device_is_in_list(uid)) {
          add_rdm_device_to_list(uid);
          return 1;
        }
      }
      /* if mute was not successful there was no device with this UID */
      return 0;
    } else {
      /* Send a DUB. If we get no response, try again if there are retries left */
      do {
        dub_resp = rdm_dub(rx, lower, upper, &uid);
      } while ((dub_resp == DiscNone) && (n_retries-- > 0));
    }
  
    /* If still no more responses then this part of the tree is done, return the number of devices found */
    if (dub_resp == DiscNone) {
      return n_found;
    }

    /* If there was collissions we need to branch further down in the binary tree */
    if (dub_resp == DiscCollission) {
 	    uint64_t mid = (lower + upper) / 2;
      /* the total amount of devices found in this part of the tree is the sum of:
       * the number of devices we already found + the number of devices in each half of the tree */
      return n_found + radio_discover_sub_tree(lower, mid) + radio_discover_sub_tree(mid + 1, upper);
    }

     /* if we got a single response we will try to mute it to verify it's a true response */
    if (dub_resp == DiscUid) {
      if (mute_rdm_device(rx, uid)) {
        if (!rdm_device_is_in_list(uid)) {
          /* if device could be muted and did not exist in list before, then we found a new device */
          add_rdm_device_to_list(uid);
          n_found++;
        } else {
          /* if device was already in the list, but could be muted - this means we got a rough response caused by collissions */
          dub_resp = DiscCollission;
        }
      } else {
        /* if device could not be muted - this means we got a rough response caused by collissions */
        dub_resp = DiscCollission;
      }
    }

    /* If there was collissions we need to branch further down in the binary tree */
    if (dub_resp == DiscCollission) {
      uint64_t mid = (lower + upper) / 2;
      /* the total amount of devices found in this part of the tree is the sum of:
       * the number of devices we already found + the number of devices in each half of the tree */
      return n_found + rdm_discover_sub_tree(rx, lower, mid) + rdm_discover_sub_tree(rx, mid + 1, upper);
    }
    
  } while (dub_resp == DiscUid); /* keep trying while we're getting single responses */

  return n_found;
}

/**
 * Fetches a list of downstream RDM devices from a W-DMX G4S receiver. 
 *
 * Return value:  The number of RDM devices found, or -1 if fetching list failed.
 *                Some older W-DMX receivers does not return a list at all when the list is empty.
 *
 * @param rx      The receiver to fetch RDM devie list from.
 */
int8_t fetch_devices_from_wdmx_receiver(uint64_t rx) {
  RdmRequest req;
  RdmResponse resp;
  uint8_t n_dev = 0;

  fill_rdm_packet(&req, rx);
  
  req.commandClass = GET_COMMAND;
  req.parameterId = SWAP16(WDMX_RADIO_QUEUED_MESSAGE);
  req.parameterDataLength = 0;

  set_rdm_length_and_checksum(&req);

  tx_buffer[0] = rx >> 40;
  tx_buffer[1] = rx >> 32;
  tx_buffer[2] = rx >> 24;
  tx_buffer[3] = rx >> 16;
  tx_buffer[4] = rx >> 8;
  tx_buffer[5] = rx ;

  memcpy(tx_buffer+6, &req, req.messageLength+2);

  timo_transfer(TIMO_WRITE_RDM_COMMAND, rx_buffer, tx_buffer, 1+6+req.messageLength+2);

  wait_for_rdm_response();

  timo_transfer_rdm_response(TIMO_READ_RDM_COMMAND, rx_buffer, tx_buffer, 260);

  /* return code 1 means we got a proper result */
  if (rx_buffer[0] == 1) {
    memcpy(&resp, rx_buffer+1, rx_buffer[3]+2);
    resp.parameterId = SWAP16(resp.parameterId);

    if ((resp.responseType == RESPONSE_TYPE_ACK) && (resp.parameterId == WDMX_RADIO_PROXIED_DEVICES)) {
      for (int i=0; i<(resp.parameterDataLength / 8); i++) {
        uint64_t uid;

        uid = resp.parameterData[i*8];
        uid <<= 8;
        uid |= resp.parameterData[i*8+1];
        uid <<= 8;
        uid |= resp.parameterData[i*8+2];
        uid <<= 8;
        uid |= resp.parameterData[i*8+3];
        uid <<= 8;
        uid |= resp.parameterData[i*8+4];
        uid <<= 8;
        uid |= resp.parameterData[i*8+5];
        /* next two bytes are mute flasg, ignore these */
        
        add_rdm_device_to_list(uid);
        n_dev++;
      }
    }
  } else {
    /* no list returned - could be either lost packet or older W-DMX receiver that doesn't return if empty */
    return -1;
  }

  return n_dev;
}

/**
 * Performes a discovery of radio and RDM devices. 
 *
 * @param incremental  If an incremental discovery should be used instead of a full.
 */
void disc_all(bool incremental) {
  uint64_t uid;
  uint16_t n_found = 0;

  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println();

  Serial.println("Discovering radio receivers....");
  /* Unmuting all receivers */
  unmute_all_radios();
  unmute_all_radios();

  if (incremental) {
    /* if it's an incremental discovery we start off by muting all known radios */
    for (int i=0; i<n_radios; i++) {
      if(!mute_radio(radios[i])) {
        /* if we could not mute it, it's gone now - remove from list */
        remove_radio_from_list(radios[i]);
        i--;
      }
    }
  } else {
    /* if full discovery - just clear the list */
    n_radios = 0;
  }

  /* run discovery ofer the full binary search tree - this functions will branch itself when needed */
  n_found = radio_discover_sub_tree(0, 0x0000FFFFFFFFFFFF);

  Serial.print("Found "); Serial.print(n_found); Serial.print(" new radios ("); Serial.print(n_radios); Serial.println(" in total).");

  /* There's some differences in how to get the RDM devices between W-DMX and CRMX */
  if (rf_protocol == TIMO_RF_PROTO_G4S) {

    /* For W-DMX the receivers perform the local discovery, so we do not care about incremental/full discover here */
    n_devices = 0;
    /* go through all found receivers and fethc the devices */
    for (int i=0; i<n_radios; i++) {
      int8_t new_found = -1, retries = 2;
      Serial.print("Fetch devices from "); print_uid(radios[i]); Serial.print("... ");
      while (new_found < 0) {
        new_found = fetch_devices_from_wdmx_receiver(radios[i]);
        /* if negative means that we failed fetching, or that an old W-DMX receiver ignored the request when no devices found */
        if (new_found < 0) {
          if (retries-- == 0) {
            new_found = 0;
            break;
          }
        }
      }
      Serial.print("Got "); Serial.print(new_found); Serial.println(" devices...");
    }

    Serial.print("Found "); Serial.print(n_devices); Serial.println(" devices in total.");
  } else { // CRMX
    /* Loop over all receivers to do RDM discovery downstream */
    unmute_all();
    unmute_all();
    if (incremental) {
      /* if it's an incremental discovery we start off by muting all known radios */
      for (int i=0; i<n_devices; i++) {
        if(!mute_rdm_device(0x00FFFFFFFFFFFF, rdm_devices[i])) {
          /* if we could not mute it, it's gone now - remove from list */
          remove_rdm_device_from_list(rdm_devices[i]);
          i--;
        }
      }
    } else {
      n_devices = 0;
    }
    
    /* now discover any unmuted RDM devices */
    n_found = 0;
    for (int i=0; i<n_radios; i++) {
      /* run discovery on each receiver */
      uint16_t new_found = 0;
      Serial.print("Running discovery on "); print_uid(radios[i]); Serial.println("...");
      new_found += rdm_discover_sub_tree(radios[i], 0, 0x0000FFFFFFFFFFFF);
      Serial.print("Found "); Serial.print(new_found); Serial.println(" new...");
      n_found += new_found;
    }

    Serial.print("Found "); Serial.print(n_found); Serial.print(" new devices ("); Serial.print(n_devices); Serial.println(" in total).");
  }
}

/**
 * This is the Arduino main loop function, it's called repeatedly 
 */
void loop() {
  uint16_t selection;
  int i;

  Serial.println();
  Serial.println();

  /* Print a list of the discovered devices */
  for (i=1; i<(n_devices+1); i++) {
    Serial.print(i); Serial.print(") "); print_uid(rdm_devices[i-1]); Serial.print(" ");
    print_manufacturer_label(0x00FFFFFFFFFFFF, rdm_devices[i-1]); Serial.print(" ");
    print_device_model_description(0x00FFFFFFFFFFFF, rdm_devices[i-1]);
    Serial.println();
  }

  Serial.print(i++); Serial.println(") Find all devices");
  Serial.print(i++); Serial.println(") Find new devices");
  
  Serial.setTimeout(0xFFFFFFFF);

  /* Wait for input from the user */
  selection = Serial.parseInt();

  if (selection == (n_devices + 1)) {
    /* selection was to Find all devices (full discovery) */
    disc_all(false);
  } else if (selection == (n_devices + 2)) {
    /* selection was to Find new devices (incremental discovery) */
    disc_all(true);
  } else if ((selection > 0) && (selection <= n_devices)) {
    /* a device was selected from the list - we will identify it for 5 seconds */
    Serial.print("Identifying "); print_uid(rdm_devices[selection-1]); Serial.println("...");
    identify(0x00FFFFFFFFFFFF, rdm_devices[selection-1], true);
    for (i=0; i<5; i++) {
      Serial.print(5-i); Serial.print("... "); Serial.flush();
      delay(1000);
    }
    identify(0x00FFFFFFFFFFFF, rdm_devices[selection-1], false);
    Serial.println("done");
  }
}

/**
 * Makes a complete SPI transaction with the TimoTwo module
 *
 * Return value:   The content of the IRQ flags register, or -1 if there was no response.
 *
 * @param command   The TimoTwo SPI command.
 * @param *dst      Pointer to the buffer where to store the returned data
 * @param *src      Pointer to the buffer containing data to transfer
 * @param len       Length in bytes. IRQ flags is included. Example: Use length 9 when reading the version register.
 */
int16_t timo_transfer(uint8_t command, uint8_t *dst, uint8_t *src, uint32_t len) {
  uint8_t irq_flags;

  uint32_t start_time = millis();

  /* Perform the transfer of the command byte */
  digitalWrite(timo.csn_pin, LOW);
  irq_flags = SPI.transfer(command);
  irq_is_pending();
  digitalWrite(timo.csn_pin, HIGH);

  /* If no bytes to transfer, this was a NOP command - just wait for IRQ or timeout */
  if (len == 0) {
    start_time = millis();
    while ((!digitalRead(timo.irq_pin)) && (!irq_is_pending())) {
      if (millis() - start_time > 10) {
        break;
      }
    }
    return irq_flags;
  }

  /* wait for IRQ or timeout */
  while(!irq_is_pending()) {
    if (millis() - start_time > 1000) {
      return -1;
    }
  }

  /* start the payload transfer */
  digitalWrite(timo.csn_pin, LOW);
  irq_flags = SPI.transfer(0xff);

  /* If busy flag is set we can't do the transfer now, cancel */
  if (irq_flags & TIMO_SPI_DEVICE_BUSY_IRQ_MASK) {
    digitalWrite(timo.csn_pin, HIGH);
    return irq_flags;
  }

  /* Transfer the data */
  for (uint32_t i = 0; i < len - 1; i++) {
    *dst++ = SPI.transfer(*src++);
  }

  /* End transfer */
  digitalWrite(timo.csn_pin, HIGH);

  /* wair for IRQ or timeout */
  while (!digitalRead(timo.irq_pin)) {
    if (millis() - start_time > 50) {
      break;
    }
  }
  return irq_flags;
}

/**
 * This is a specialized version of the SPI transfer function for RDM responses. 
 * It checks the length field of the RDM response to know how much data to transfer.
 *
 * Return value:   The number of bytes read, or -1 if there was no response.
 *
 * @param command   The TimoTwo SPI command.
 * @param *dst      Pointer to the buffer where to store the returned data
 * @param *src      Pointer to the buffer containing data to transfer
 * @param max_len   Maximum length in bytes, that is - how large is the dst buffer.
 */
int16_t timo_transfer_rdm_response(uint8_t command, uint8_t *dst, uint8_t *src, uint32_t max_len) {
  uint8_t irq_flags;

  uint32_t start_time = millis();

  /* write command */
  digitalWrite(timo.csn_pin, LOW);
  irq_flags = SPI.transfer(command);
  irq_is_pending();
  digitalWrite(timo.csn_pin, HIGH);

  /* if we don't accept any data this was just to issue the command - quit after IRQ was received */
  if (max_len == 0) {
    start_time = millis();
    while ((!digitalRead(timo.irq_pin)) && (!irq_is_pending())) {
      if (millis() - start_time > 10) {
        break;
      }
    }
    return 0;
  }

  /* otherwise wait for IRQ or timeout */
  while(!irq_is_pending()) {
    if (millis() - start_time > 1000) {
      return -1;
    }
  }

  /* start reading the data */
  digitalWrite(timo.csn_pin, LOW);
  irq_flags = SPI.transfer(0xff);

  /* If busy flag is set we can't do the transfer now, cancel */
  if (irq_flags & TIMO_SPI_DEVICE_BUSY_IRQ_MASK) {
    digitalWrite(timo.csn_pin, HIGH);
    return 0;
  }

  /* read the data */
  for (uint32_t i = 0; i < max_len - 1; i++) {
    uint8_t data;
    data = SPI.transfer(*src++);
    if ((i == 0) && (data == 0)) {
      /* if first byte (result code) is 0, that means there was no response */
      *dst++ = data;
      break;
    }
    if (i == 3) {
      /* if we gotten here, this is the RDM packet length field, calculate the actual number of bytes to read */
      if ((data + 2) < (max_len - 1)) {
        max_len = data + 2 + 1;
      }
    }
    *dst++ = data;
  }

  /* end transfer */
  digitalWrite(timo.csn_pin, HIGH);

  /* wait for IRQ or timeout */
  while (!digitalRead(timo.irq_pin)) {
    if (millis() - start_time > 50) {
      break;
    }
  }
  return max_len - 1;
}

/**
 * Converts a ASCII character as a single HEX digit to integer.
 *
 * Return value:   The value, 0-15.
 *
 * @param ascii    The hex character.
 */
uint8_t hex_nibble_to_val(char ascii) {
  if (ascii >= '0' && ascii <= '9') {
    return ascii - '0';
  } else if (ascii >= 'A' && ascii <= 'F') {
    return ascii - 'A' + 10;
  } else if (ascii >= 'a' && ascii <= 'f') {
    return ascii - 'a' + 10;
  }
  Serial.print("! ");
  Serial.print("0x");
  Serial.print(ascii, HEX);
  Serial.println(" is not a valid hex symbol");
  return 0;
}

/**
 * Converts a an integer 0-15 to ASCII character as a single HEX digit.
 *
 * Return value:   The hex digit as ASCII char, '0'-'F'.
 *
 * @param nibble   The integer.
 */
char nibble_to_hex(uint8_t nibble) {
  nibble &= 0x0f;
  if (nibble >= 0 && nibble <= 9) {
    return '0' + nibble;
  } else {
    return 'A' + nibble - 10;
  }
}

/**
 * Prints the response from the module as hex, including the IRQ flags.
 *
 * @param irq_flags  The IRQ flags
 * @param *data      Pointer to the data
 * @param len        Length of data
 */
void print_response(int16_t irq_flags, uint8_t *data, uint32_t len) {
  if (irq_flags < 0) {
    Serial.println("! Timeout");
    return;
  }
  Serial.print("< ");
  print_irq_flags(irq_flags);
  Serial.print(" ");
  Serial.flush();
  for (uint32_t i = 0; i < len; i++) {
    Serial.print(nibble_to_hex(0x0f & (data[i] >> 4)));
    Serial.print(nibble_to_hex(0x0f & data[i]));
    Serial.print(" ");
    Serial.flush();
  }
  Serial.println();
}

/**
 * Prints IRQ flags.
 *
 * @param irq_flags  The IRQ flags
 */
void print_irq_flags(int16_t irq_flags) {
  for (int i = 7; i >= 0; i--) {
    if (irq_flags & (1 << i)) {
      Serial.print('1');
    } else {
      Serial.print('0');
    }
  }
  Serial.flush();
}
