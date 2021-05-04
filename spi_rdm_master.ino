#include <SPI.h>
#include "timo_spi.h"
#include "e120.h"

uint8_t my_uid[6] = {0x4c, 0x55, 0x00, 0x00, 0x00, 0x12};

#define N_RADIOS 20
#define N_DEVICES 50

#define  MIN( A,  B )      ( ( A )  <  ( B )   ?   ( A )   :   ( B ) )

#define TIMO_IDENTIFY_IRQ (1 << 5)

#define TIMO_SPI_DEVICE_BUSY_IRQ_MASK (1 << 7)

#define SWAP16(x) ((uint16_t)(((uint16_t)x >> 8) | ((uint16_t)x << 8)))

static timo_t timo = { .csn_pin =  7, .irq_pin = 6 };

static uint32_t tx_buffer32[256];
static uint32_t rx_buffer32[256];

static uint8_t* tx_buffer = (uint8_t*)tx_buffer32;
static uint8_t* rx_buffer = (uint8_t*)rx_buffer32;
static bool has_set_up = false;

typedef enum {
  DiscUid,
  DiscCollission,
  DiscNone,
} DiscoveryResponseType;

uint64_t radios[N_RADIOS] = {0, 0};
uint8_t n_radios = 0;

uint64_t rdm_devices[N_DEVICES] = {0, 0};
uint8_t n_devices = 0;

uint8_t rdm_tn = 0;

bool radio_is_in_list(uint64_t uid) {
  for (int i=0; i<n_radios; i++) {
    if (radios[i] == uid) {
      return true;
    }
  }

  return false;
}

void add_radio_to_list(uint64_t uid) {
  if (n_radios < N_RADIOS) {
    radios[n_radios++] = uid;
  }
}

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


bool rdm_device_is_in_list(uint64_t uid) {
  for (int i=0; i<n_devices; i++) {
    if (rdm_devices[i] == uid) {
      return true;
    }
  }

  return false;
}

void add_rdm_device_to_list(uint64_t uid) {
  if (n_devices < N_DEVICES) {
    rdm_devices[n_devices++] = uid;
  }
}

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

void setup() {
  int16_t irq_flags;
  
  Serial.begin(115200);

  SPI.begin();

  pinMode(timo.irq_pin, INPUT);
  pinMode(timo.csn_pin, OUTPUT);
  digitalWrite(timo.csn_pin, HIGH);

  attachInterrupt(digitalPinToInterrupt(timo.irq_pin), irq_pin_handler, FALLING);

  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));

  delay(1000);

  while(Serial.available() > 0) {
    Serial.read();
  }

  Serial.println("Running");
  Serial.flush();

  while (irq_is_pending()) {
    ;
  }

  Serial.println("Version:");
  irq_flags = timo_transfer(TIMO_READ_REG_COMMAND(TIMO_VERSION_REG), rx_buffer, tx_buffer, 9);
  print_response(irq_flags, rx_buffer, 8);

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

  Serial.println("Status:");
  irq_flags = timo_transfer(TIMO_READ_REG_COMMAND(TIMO_STATUS_REG), rx_buffer, tx_buffer, 2);
  print_response(irq_flags, rx_buffer, 1);

  Serial.println("IRQ mask:");
  tx_buffer[0] = TIMO_IRQ_EXTENDED_FLAG;
  irq_flags = timo_transfer(TIMO_WRITE_REG_COMMAND(TIMO_IRQ_MASK_REG), rx_buffer, tx_buffer, 2);
  irq_flags = timo_transfer(TIMO_READ_REG_COMMAND(TIMO_IRQ_MASK_REG), rx_buffer, tx_buffer, 2);
  print_response(irq_flags, rx_buffer, 1);

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

void wait_for_rdm_response(void) {
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
        if (ext_flags & TIMO_EXTIRQ_SPI_RDM_FLAG) {
          while (!irq_is_pending()) {
            ;
          }
  
          return;
        }
      }
    }
  }
}

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

void print_uid(uint64_t uid) {
  for (int i = 11; i >= 0; i--) {
    Serial.print(nibble_to_hex(uid >> (4*i)));
  }
   Serial.flush();
}

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

uint16_t radio_discover_sub_tree(uint64_t lower, uint64_t upper) {
  DiscoveryResponseType dub_resp;
  uint16_t n_found = 0;
  uint64_t uid;
  
  uint8_t n_retries = 1;

  do {
    do {
      dub_resp = radio_dub(lower, upper, &uid);
    } while ((dub_resp == DiscNone) && (n_retries-- > 0));
  
    if (dub_resp == DiscNone) {
      return n_found;
    }

    if (dub_resp == DiscCollission) {
      uint64_t mid = (lower + upper) / 2;
      return n_found + radio_discover_sub_tree(lower, mid) + radio_discover_sub_tree(mid + 1, upper);
    }

    if (dub_resp == DiscUid) {
      if (mute_radio(uid)) {
        if (!radio_is_in_list(uid)) {
          add_radio_to_list(uid);
          n_found++;
        }
      }
    }
    
  } while (dub_resp == DiscUid);

  return n_found;
}

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

void set_rdm_length_and_checksum(RdmRequest *r) {
  uint16_t sum = 0;

  r->messageLength = 24 + r->parameterDataLength;

  for(int i=0; i<r->messageLength; i++) {
    sum += ((uint8_t*)r)[i];
  }

  ((uint8_t*)r)[r->messageLength] = sum >> 8;
  ((uint8_t*)r)[r->messageLength + 1] = sum;
}

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

  if (rx_buffer[0] == 1) {
    memcpy(&resp, rx_buffer+1, rx_buffer[3]+2);
    resp.parameterId = SWAP16(resp.parameterId);

    if ((resp.responseType == RESPONSE_TYPE_ACK) && (resp.parameterId == MANUFACTURER_LABEL)) {
      for (int i=0; i<resp.parameterDataLength; i++) {
        Serial.print((char)resp.parameterData[i]);
      }
    }
  }
}

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

  if (rx_buffer[0] == 1) {
    memcpy(&resp, rx_buffer+1, rx_buffer[3]+2);
    resp.parameterId = SWAP16(resp.parameterId);

    if ((resp.responseType == RESPONSE_TYPE_ACK) && (resp.parameterId == DEVICE_MODEL_DESCRIPTION)) {
      for (int i=0; i<resp.parameterDataLength; i++) {
        Serial.print((char)resp.parameterData[i]);
      }
    }
  }
}

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

  if (rx_buffer[0] == 1) {
    memcpy(&resp, rx_buffer+1, rx_buffer[3]+2);
    resp.parameterId = SWAP16(resp.parameterId);

    if ((resp.responseType == RESPONSE_TYPE_ACK) && (resp.parameterId == DISC_MUTE)) {
      return true;
    }
  }

  return false;
}

uint16_t rdm_discover_sub_tree(uint64_t rx, uint64_t lower, uint64_t upper) {
  DiscoveryResponseType dub_resp;
  uint16_t n_found = 0;
  uint64_t uid;
  
  uint8_t n_retries = 1;

  do {
    if (lower == upper) {
      Serial.print("At leaf... Try to mute "); print_uid(lower); Serial.println();
      uid = lower;
      if (mute_rdm_device(rx, uid)) {
        if (!rdm_device_is_in_list(uid)) {
          add_rdm_device_to_list(uid);
          return 1;
        }
      }
      return 0;
    } else {
      do {
        dub_resp = rdm_dub(rx, lower, upper, &uid);
      } while ((dub_resp == DiscNone) && (n_retries-- > 0));
    }
  
    if (dub_resp == DiscNone) {
      return n_found;
    }

    if (dub_resp == DiscUid) {
      if (mute_rdm_device(rx, uid)) {
        if (!rdm_device_is_in_list(uid)) {
          add_rdm_device_to_list(uid);
          n_found++;
        } else {
          dub_resp = DiscCollission;
        }
      } else {
        dub_resp = DiscCollission;
      }
    }

    if (dub_resp == DiscCollission) {
      uint64_t mid = (lower + upper) / 2;
      return n_found + rdm_discover_sub_tree(rx, lower, mid) + rdm_discover_sub_tree(rx, mid + 1, upper);
    }
    
  } while (dub_resp == DiscUid);

  return n_found;
}

void disc_all(bool incremental) {
  uint64_t uid;
  uint16_t n_found = 0;

  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println();

  Serial.println("Discovering radio receivers....");
  unmute_all_radios();
  unmute_all_radios();
  if (incremental) {
    for (int i=0; i<n_radios; i++) {
      if(!mute_radio(radios[i])) {
        remove_radio_from_list(radios[i]);
        i--;
      }
    }
  } else {
    n_radios = 0;
  }

  n_found = radio_discover_sub_tree(0, 0x0000FFFFFFFFFFFF);
  

  Serial.print("Found "); Serial.print(n_found); Serial.print(" new radios ("); Serial.print(n_radios); Serial.println(" in total).");

  /* Loop over all receivers to do RDM discovery downstream */
  unmute_all();
  unmute_all();
  if (incremental) {
    for (int i=0; i<n_devices; i++) {
      if(!mute_rdm_device(0x00FFFFFFFFFFFF, rdm_devices[i])) {
        remove_rdm_device_from_list(rdm_devices[i]);
        i--;
      }
    }
  } else {
    n_devices = 0;
  }
  
  n_found = 0;
  for (int i=0; i<n_radios; i++) {
    uint16_t new_found = 0;
    Serial.print("Running discovery on "); print_uid(radios[i]); Serial.println("...");
    new_found += rdm_discover_sub_tree(radios[i], 0, 0x0000FFFFFFFFFFFF);
    Serial.print("Found "); Serial.print(new_found); Serial.println(" new...");
    n_found += new_found;
  }
  Serial.print("Found "); Serial.print(n_found); Serial.print(" new devices ("); Serial.print(n_devices); Serial.println(" in total).");

}


void loop() {
  uint16_t selection;
  int i;

  Serial.println();
  Serial.println();

  for (i=1; i<(n_devices+1); i++) {
    Serial.print(i); Serial.print(") "); print_uid(rdm_devices[i-1]); Serial.print(" ");
    print_manufacturer_label(0x00FFFFFFFFFFFF, rdm_devices[i-1]); Serial.print(" ");
    print_device_model_description(0x00FFFFFFFFFFFF, rdm_devices[i-1]);
    Serial.println();
  }

  Serial.print(i++); Serial.println(") Find all devices");
  Serial.print(i++); Serial.println(") Find new devices");
  
  Serial.setTimeout(0xFFFFFFFF);

  selection = Serial.parseInt();

  if (selection == (n_devices + 1)) {
    disc_all(false);
  } else if (selection == (n_devices + 2)) {
    disc_all(true);
  } else if ((selection > 0) && (selection <= n_devices)) {
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
 * @param len   Length in bytes. IRQ flag is counted. Example: Use length 9 when reading the version register.
 */
int16_t timo_transfer(uint8_t command, uint8_t *dst, uint8_t *src, uint32_t len) {
  uint8_t irq_flags;

  uint32_t start_time = millis();

  digitalWrite(timo.csn_pin, LOW);
  irq_flags = SPI.transfer(command);
  irq_is_pending();
  digitalWrite(timo.csn_pin, HIGH);

  if (len == 0) {
    start_time = millis();
    while ((!digitalRead(timo.irq_pin)) && (!irq_is_pending())) {
      if (millis() - start_time > 10) {
        break;
      }
    }
    return irq_flags;
  }

  while(!irq_is_pending()) {
    if (millis() - start_time > 1000) {
      return -1;
    }
  }

  digitalWrite(timo.csn_pin, LOW);
  irq_flags = SPI.transfer(0xff);

  if (irq_flags & TIMO_SPI_DEVICE_BUSY_IRQ_MASK) {
    digitalWrite(timo.csn_pin, HIGH);
    return irq_flags;
  }

  for (uint32_t i = 0; i < len - 1; i++) {
    *dst++ = SPI.transfer(*src++);
  }

  digitalWrite(timo.csn_pin, HIGH);

  while (!digitalRead(timo.irq_pin)) {
    if (millis() - start_time > 50) {
      break;
    }
  }
  return irq_flags;
}

int16_t timo_transfer_rdm_response(uint8_t command, uint8_t *dst, uint8_t *src, uint32_t max_len) {
  uint8_t irq_flags;

  uint32_t start_time = millis();

  digitalWrite(timo.csn_pin, LOW);
  irq_flags = SPI.transfer(command);
  irq_is_pending();
  digitalWrite(timo.csn_pin, HIGH);

  if (max_len == 0) {
    start_time = millis();
    while ((!digitalRead(timo.irq_pin)) && (!irq_is_pending())) {
      if (millis() - start_time > 10) {
        break;
      }
    }
    return 0;
  }

  while(!irq_is_pending()) {
    if (millis() - start_time > 1000) {
      return -1;
    }
  }

  digitalWrite(timo.csn_pin, LOW);
  irq_flags = SPI.transfer(0xff);

  if (irq_flags & TIMO_SPI_DEVICE_BUSY_IRQ_MASK) {
    digitalWrite(timo.csn_pin, HIGH);
    return 0;
  }

  for (uint32_t i = 0; i < max_len - 1; i++) {
    uint8_t data;
    data = SPI.transfer(*src++);
    if ((i == 0) && (data == 0)) {
      *dst++ = data;
      break;
    }
    if (i == 3) {
      if ((data + 2) < (max_len - 1)) {
        max_len = data + 2 + 1;
      }
    }
    *dst++ = data;
  }

  digitalWrite(timo.csn_pin, HIGH);

  while (!digitalRead(timo.irq_pin)) {
    if (millis() - start_time > 50) {
      break;
    }
  }
  return max_len - 1;
}

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

char nibble_to_hex(uint8_t nibble) {
  nibble &= 0x0f;
  if (nibble >= 0 && nibble <= 9) {
    return '0' + nibble;
  } else {
    return 'A' + nibble - 10;
  }
}

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
