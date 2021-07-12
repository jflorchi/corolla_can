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
const uint8_t LEAD_INFO_MSG[8] = {0xff, 0xf8, 0x00, 0x08, 0x7f, 0xe0, 0x00, 0x4e};
const uint8_t GEAR_MSG[8] = {0x0, 0x1, 0x0, 0x0, 0x0, 0x80, 0x0, 0x0};
const uint8_t PRE_COL[7] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x8c};
const uint8_t PRE_COL_2[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x4f};
const uint8_t BRAKE_MOD[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x8};
const uint8_t STEER_ANGLE[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
const uint8_t CAMERA[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};

// Variable Messasges
uint8_t PCM_CRUISE_MSG[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
uint8_t PCM_CRUISE_2_MSG[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
uint8_t STEERING_LEVER_MSG[8] = {0x29, 0x0, 0x01, 0x0, 0x0, 0x0, 0x76};

uint8_t WHEEL_SPEEDS[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};

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

void loop() {

    // Media Button Handling
//    bool buttonPressed = digitalRead(8) == 0;
//    if (buttonPressed) {
//        if (lastState != buttonPressed) {
//            if (!stateChanged) {
//                openEnabled = !openEnabled;
//            }
//            stateChanged = true;
//        } else {
//            stateChanged = false;
//        }
//    }
//    lastState = buttonPressed;

    //0x2e6 LEAD_INFO
    writeMsg(0x2e6, LEAD_INFO_MSG, 8, false);
    
    //0xaa WHEEL_SPEED
    writeMsg(0xaa, WHEEL_SPEEDS, 8, false);

    //0x1d2 msg PCM_CRUISE
    PCM_CRUISE_MSG[0] = (openEnabled << 5) & 0x20;
    PCM_CRUISE_MSG[5] = (openEnabled << 7) & 0x80;
    writeMsg(0x1d2, PCM_CRUISE_MSG, 8, true);

    //0x1d3 msg PCM_CRUISE_2
    PCM_CRUISE_2_MSG[1] = (openEnabled << 7) & 0x80 | 0x28;
    writeMsg(0x1d3, PCM_CRUISE_2_MSG, 8, true);

    // 0x3bc msg GEAR_PACKET
    writeMsg(0x3bc, GEAR_MSG, 8, false);

    //0x614 msg steering_levers
    STEERING_LEVER_MSG[3] = (blinker_left << 5) & 0x20 | (blinker_right << 4) & 0x10;
    writeMsg(0x614, STEERING_LEVER_MSG, 8, true);

    writeMsg(0x400, CAMERA, 8, false);

    delay(8);
    
}
