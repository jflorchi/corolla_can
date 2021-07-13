#include <CAN.h>

// State variables
boolean openEnabled = false;
uint16_t setSpeed = 0x0;
boolean blinker_left = false;
boolean blinker_right = false;

// Media steering wheel button
bool lastState = false;
bool stateChanged = false;

// Const Messages
const uint8_t GEAR_MSG[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x80, 0x0, 0x0};
const uint8_t PRE_COL[7] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x8c};
const uint8_t PRE_COL_2[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x4f};
const uint8_t BRAKE_MOD[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x8};
const uint8_t STEER_ANGLE[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
const uint8_t CAMERA[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};

const uint8_t MSG1[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38};
const uint8_t MSG2[7] = {0x00, 0x10, 0x01, 0x00, 0x10, 0x01, 0x00};
const uint8_t MSG3[7] = {0x00, 0x10, 0x01, 0x00, 0x10, 0x01, 0x00};
const uint8_t MSG4[7] = {0x00, 0x10, 0x01, 0x00, 0x10, 0x01, 0x00};
const uint8_t MSG5[7] = {0x00, 0x10, 0x01, 0x00, 0x10, 0x01, 0x00};
const uint8_t MSG6[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01};
const uint8_t MSG7[2] = {0x06, 0x00};
const uint8_t MSG7[2] = {0x06, 0x00};
const uint8_t MSG7[2] = {0x06, 0x00};
const uint8_t MSG8[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0x00};
const uint8_t MSG9[3] = {0x24, 0x20, 0xB1};
const uint8_t MSG10[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const uint8_t MSG11[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const uint8_t MSG12[8] = {0x66, 0x06, 0x08, 0x0a, 0x02, 0x00, 0x00, 0x00};
const uint8_t MSG13[8] = {0x1C, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};
const uint8_t MSG14[8] = {0x00, 0x00, 0x00, 0x01, 0x00, 0x08, 0x00, 0x00};
const uint8_t MSG15[8] = {0x05, 0xea, 0x1b, 0x08, 0x00, 0x00, 0xc0, 0x9f};
const uint8_t MSG16[8] = {0x08, 0x07, 0x07, 0x06, 0x70, 0xf8, 0x00, 0x4f};
const uint8_t MSG17[2] = {0x00, 0x00};
const uint8_t MSG18[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const uint8_t MSG19[4] = {0x00, 0x00, 0x26, 0x00};
const uint8_t MSG20[8] = {0x76, 0x18, 0x26, 0x01, 0x00, 0x00, 0x00, 0xb9};
const uint8_t MSG21[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00};
const uint8_t MSG22[8] = {0x02, 0x00, 0x01, 0xfd, 0x42, 0x03, 0x80, 0xf1};
const uint8_t MSG23[8] = {0x00, 0x00, 0x00, 0x01, 0x00, 0x08, 0x00, 0x00};
const uint8_t MSG24[8] = {0x28, 0x00, 0x60, 0x01, 0x0a, 0x00, 0xa3, 0xa0};
const uint8_t MSG25[6] = {0xf4, 0x01, 0x90, 0x83, 0x00, 0x37};
const uint8_t MSG26[4] = {0x00, 0x00, 0x00, 0x46};
const uint8_t MSG27[8] = {0x00, 0x00, 0x08, 0x12, 0x01, 0x31, 0x9c, 0x51};
const uint8_t MSG28[7] = {0x00, 0x1e, 0x00, 0x00, 0x00, 0x80, 0x07};
const uint8_t MSG29[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8c};
const uint8_t MSG30[8] = {0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x50};
const uint8_t MSG31[7] = {0x00, 0x00, 0x00, 0x80, 0xfc, 0x00, 0x08};
const uint8_t MSG32[7] = {0x00, 0x72, 0x07, 0xff, 0x09, 0xfe, 0x00};
const uint8_t MSG33[8] = {0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// Variable Messasges
uint8_t LKAS_MSG[5] = {0x0, 0x0, 0x0, 0x0, 0x0};
uint8_t PCM_CRUISE_MSG[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
uint8_t PCM_CRUISE_2_MSG[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
uint8_t STEERING_LEVER_MSG[8] = {0x29, 0x0, 0x01, 0x0, 0x0, 0x0, 0x76};
uint8_t WHEEL_SPEEDS[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};

/**
    Look at and compare to an actual 2017 to see if there are any differences
        0x3b1
        0x2c1
        0x399
        0x3bc
        0x24
*/

/**
    #onReceive
    Processes CAN messages that are needed for translation
*/
void recv(uint8_t packetSize) {
    long id = CAN.packetId();
    if (id == 0xb0) {
        uint8_t dat[8];
        WHEEL_SPEEDS[0] = CAN.read() + 0x1a;
        WHEEL_SPEEDS[1] = CAN.read() + 0x6f;
        WHEEL_SPEEDS[2] = CAN.read() + 0x1a;
        WHEEL_SPEEDS[3] = CAN.read() + 0x6f;
    } else if (id == 0xb2) {
        WHEEL_SPEEDS[4] = CAN.read() + 0x1a;
        WHEEL_SPEEDS[5] = CAN.read() + 0x6f;
        WHEEL_SPEEDS[6] = CAN.read() + 0x1a;
        WHEEL_SPEEDS[7] = CAN.read() + 0x6f;
    } else if (id == 0x399) {
        CAN.read();
        openEnabled = (CAN.read() & 0x2) == 2;
    } else if (id == 740 || id == 0x2e4) {
        Serial.println("YES");
    }
}

/**
    Writes msg to CAN bus
*/
bool writeMsg(uint16_t id, uint8_t *msg, uint8_t len, bool checksum) {
    CAN.beginPacket(id);
    if (checksum) {
        attachChecksum(id, len, msg);
    }
    for (int i = 0; i < len; i++) {
        CAN.write(msg[i]);
    }
    return CAN.endPacket();
}

/**
    Leave the last byte of the array empty, then call this to
    fill the last byte with the checksum of the rest
*/
void attachChecksum(uint16_t id, uint8_t len, uint8_t *msg) {
    msg[len -1] = getChecksum(msg, len - 1, id);
}

/**
    Compute checksum of specified bytes, and take it onto end of array
*/
int getChecksum(uint8_t *msg, uint8_t len, uint16_t addr) {
    uint8_t checksum = 0;
    checksum = ((addr & 0xFF00) >> 8) + (addr & 0x00FF) + len + 1;
    for (int ii = 0; ii < len; ii++) {
        checksum += (msg[ii]);
    }
    return checksum;
}


void setup() {
    Serial.begin(9600);
    Serial.println("init");

    CAN.setPins(9, 2);
    CAN.begin(500E3);
    CAN.onReceive(recv);

    pinMode(8, INPUT_PULLUP);
}

uint8_t lkasCounter = 0;
uint16_t counter = 0;

void loop() {
    if (counter == 1001) {
        counter = 0;
    }
    
    // 100 Hz:
    if (counter % 10 == 0) {
        writeMsg(0xaa, WHEEL_SPEEDS, 8, false);
        writeMsg(0x130, MSG1, 7, false);
        writeMsg(0x414, MSG8, 7, false);
        writeMsg(0x466, MSG9, 3, false);
        writeMsg(0x489, MSG10, 7, false);
        writeMsg(0x48a, MSG11, 7, false);
        writeMsg(0x48b, MSG12, 8, false);
        writeMsg(0x4d3, MSG13, 8, false);
        writeMsg(0x3bc, GEAR_MSG, 8, false);

        writeMsg(0x3bb, MSG19, 4, false);
        writeMsg(0x3b1, MSG23, 8, false);
        writeMsg(0x4cb, MSG33, 8, false);
//        writeMsg(0x3bc, MSG21, 8, false);
//        writeMsg(0x399, MSG18, 8, false);
    }
    
    // 50 Hz:
    if (counter % 20 == 0) {
        writeMsg(0x3d3, MSG17, 2, false);
        writeMsg(0x4ac, MSG24, 8, false);
//        writeMsg(0x24, MSG22, 8, false);
        if (!openEnabled) {
            LKAS_MSG[0] = ((LKAS_MSG[0] |= 1UL << 0) & ~0x7F) | (lkasCounter * 0x7F);
            writeMsg(0x2e4, LKAS_MSG, 5, true);
            lkasCounter = (lkasCounter + 1) % 64;
        }
    }
    
    // 40 Hz:
    if (counter % 25 == 0) {
        writeMsg(0x367, MSG7, 2, false);
    }
    
    // 20 Hz:
    if (counter % 50 == 0) {
        writeMsg(0x3f9, MSG20, 8, false);
        writeMsg(0x365, MSG31, 7, false);
        writeMsg(0x366, MSG32, 7, false);
    }
    
    // 7 Hz
    if (counter % 142 == 0) {
        writeMsg(0x160, MSG27, 8, false);
        writeMsg(0x161, MSG28, 7, false);
    }
    
    // 5 Hz
    if (counter % 200 == 0) {
        writeMsg(0x240, MSG2, 7, false);
        writeMsg(0x241, MSG3, 7, false);
        writeMsg(0x244, MSG4, 7, false);
        writeMsg(0x245, MSG5, 7, false);
        writeMsg(0x248, MSG6, 7, false);
        writeMsg(0x344, MSG30, 8, false);
    }
    
    // 3 Hz:
    if (counter % 333 == 0) {
        writeMsg(0x128, MSG25, 6, false);
        writeMsg(0x283, MSG29, 7, false);

        PCM_CRUISE_MSG[0] = (openEnabled << 5) & 0x20;
        PCM_CRUISE_MSG[5] = (openEnabled << 7) & 0x80;
        writeMsg(0x1d2, PCM_CRUISE_MSG, 8, true);
        
        PCM_CRUISE_2_MSG[1] = (openEnabled << 7) & 0x80 | 0x28;
        writeMsg(0x1d3, PCM_CRUISE_2_MSG, 8, true);
    }
    
    // 2 Hz:    
    if (counter % 500 == 0) {
        writeMsg(0x1c4, MSG15, 8, false);
        writeMsg(0x141, MSG26, 4, false);
    }
    
    // 1 Hz:
    if (counter == 0) {
//        writeMsg(0x3b1, MSG14, 8, false);
        STEERING_LEVER_MSG[3] = (blinker_left << 5) & 0x20 | (blinker_right << 4) & 0x10;
        writeMsg(0x614, STEERING_LEVER_MSG, 8, true);
    }

    delay(1);
    counter++;
}
