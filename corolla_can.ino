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


//unsigned char data1[1] = {0x11};
//unsigned char data2[2] = {0x11, 0x11};
//unsigned char data3[4] = {0x11, 0x11, 0x11, 0x11};
//unsigned char data4[5] = {0x11, 0x11, 0x11, 0x11, 0x1};
//unsigned char data5[6] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x0};
//unsigned char data6[7] = {0x11, 0x11, 0x11, 0x11, 0x0, 0x0, 0x0};
//unsigned char data7[8] = {0x11, 0x11, 0x11, 0x11, 0x0, 0x0, 0x0, 0x0};
unsigned char data1[1] = {0x00};
unsigned char data2[2] = {0x0, 0x0};
unsigned char data3[4] = {0x0, 0x0, 0x0, 0x0};
unsigned char data4[5] = {0x0, 0x0, 0x0, 0x0, 0x0};
unsigned char data5[6] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
unsigned char data6[7] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
unsigned char data7[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};

unsigned int len2 = 3;
unsigned int len3 = 1;
unsigned int len4 = 2;
unsigned int len5 = 3;
unsigned int len6 = 4;
unsigned int len7 = 45;

//unsigned int array_1[] = {0x423};
unsigned int array_2[] = {0x367, 0x394, 0x3d3};
unsigned int array_3[] = {0x3bb};
unsigned int array_4[] = {0x2e4, 0x3e6};
unsigned int array_5[] = {0x1aa, 0x384, 0x386};
unsigned int array_6[] = {0x283, 0x365, 0x366, 0x3e7};
unsigned int array_7[] = {0x1c4, 0x1d0, 0x320, 0x343, 0x344, 0x381, 0x389, 0x38f, 0x3a5, 0x3e8, 0x3e9, 0x3f9, 0x411, 0x412, 0x413, 0x414, 0x45a, 0x489, 0x48a, 0x48b, 0x4ac, 0x4cb, 0x4d3, 0x4ff, 0x615, 0x619, 0x61a, 0x623, 0x63c, 0x63d, 0x680, 0x6f3, 0x770, 0x778, 0x7c6, 0x7ce, 0x7e0, 0x7e1, 0x7e2, 0x7e3, 0x7e4, 0x7e5, 0x7e6, 0x7e7, 0x7e8};

unsigned char enable1[] = {0xf8, 0x24, 0x02, 0xf8, 0x00, 0x01, 0x80, 0x72};
unsigned char enable2[] = {0x00, 0xa8, 0x43, 0x10, 0xee, 0x00, 0x00, 0xc5};
unsigned char not_in_d[] = {0x0, 0x20, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};


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
    } else if (id == 0x2e4 || id == 740) {
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

void loop() {

//    for (uint8_t i = 0; i < len2; i++) {
//        delay(100);
//        writeMsg(array_2[i], data2, 2, false);
//    }
//    for (uint8_t i = 0; i < len3; i++) {
//        delay(50);
//        writeMsg(array_3[i], data3, 4, false);
//    }
//    for (uint8_t i = 0; i < len4; i++) {
//        delay(30);
//        writeMsg(array_4[i], data4, 5, false);
//    }
//    for (uint8_t i = 0; i < len5; i++) {
//        delay(20);
//        writeMsg(array_5[i], data5, 6, false);
//    }
//    for (uint8_t i = 0; i < len6; i++) {
//        delay(10);
//        writeMsg(array_6[i], data6, 7, false);
//    }
//    for (uint8_t i = 0; i < len7; i++) {
//        writeMsg(array_7[i], data7, 7, false);
//        writeMsg(0xaa, WHEEL_SPEEDS, 8, false);
//    }
    // Media Button Handling
    bool buttonPressed = digitalRead(8) == 0;
    if (buttonPressed) {
        if (lastState != buttonPressed) {
            if (!stateChanged) {
                openEnabled = !openEnabled;
            }
            stateChanged = true;
        } else {
            stateChanged = false;
        }
    }
    lastState = buttonPressed;

    //0x2e6 LEAD_INFO
    writeMsg(0x2e6, LEAD_INFO_MSG, 8, false);
    
    //0xaa WHEEL_SPEED
    writeMsg(0xaa, WHEEL_SPEEDS, 8, false);

//    PCM_CRUISE_2_MSG[1] = (openEnabled << 7) & 0x80 | 0x28;
//    if (openEnabled) {
//        writeMsg(0x1d1, enable1, 8, false);
//        writeMsg(0x1d2, enable2, 8, false);
//        writeMsg(0x1d3, PCM_CRUISE_2_MSG, 8, true);
//    } else {
//        writeMsg(0x1d1, data7, 8, false);
//        writeMsg(0x1d2, data7, 8, false);
//        writeMsg(0x1d3, PCM_CRUISE_2_MSG, 8, true);
//    }
//
    //0x1d2 msg PCM_CRUISE
    PCM_CRUISE_MSG[0] = (openEnabled << 5) & 0x20;
    PCM_CRUISE_MSG[5] = (openEnabled << 7) & 0x80;
    writeMsg(0x1d2, PCM_CRUISE_MSG, 8, true);

    //0x1d3 msg PCM_CRUISE_2
    PCM_CRUISE_2_MSG[1] = (openEnabled << 7) & 0x80 | 0x28;
    writeMsg(0x1d3, PCM_CRUISE_2_MSG, 8, true);

    // 0x3bc msg GEAR_PACKET
    writeMsg(0x3bc, data7, 8, false);

    //0x614 msg steering_levers
    STEERING_LEVER_MSG[3] = (blinker_left << 5) & 0x20 | (blinker_right << 4) & 0x10;
    writeMsg(0x614, STEERING_LEVER_MSG, 8, true);

    writeMsg(0x400, CAMERA, 8, false);

    delay(8);
    
}
