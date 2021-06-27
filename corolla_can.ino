#include <CAN.h>

// State variables
boolean openEnabled = false;
uint8_t setSpeed = 0x0;
boolean blinker_left = false;
boolean blinker_right = false;

// Media steering wheel button
bool lastState = false;
bool stateChanged = false;

// Wheel speeds
short fr1 = -1, fr2 = -1, fl1 = -1, fl2 = -1, br1 = -1, br2 = -1, bl1 = -1, bl2 = -1;
uint8_t lkasCounter = 0;

// Const Messages
const uint8_t LEAD_INFO_MSG[8] = {0xff, 0xf8, 0x00, 0x08, 0x7f, 0xe0, 0x00, 0x4e};
const uint8_t GEAR_MSG[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x80, 0x0, 0x0};

// Variable Messasges
uint8_t LKAS_MSG[5] = {0x0, 0x0, 0x0, 0x0, 0x0};
uint8_t PCM_CRUISE_MSG[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
uint8_t PCM_CRUISE_2_MSG[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
uint8_t STEERING_LEVER_MSG[8] = {0x29, 0x0, 0x01, 0x0, 0x0, 0x0, 0x76};

/**
    #onReceive
    Processes CAN messages that are needed for translation
*/
void recv(uint8_t packetSize) {
    long id = CAN.packetId();
    
    if (id == 0xb0) {
        uint8_t dat[8];
        dat[0] = CAN.read();
        dat[1] = CAN.read();
        dat[2] = CAN.read();
        dat[3] = CAN.read();
        if (br1 != -1) {
            uint8_t dat[8];
            dat[4] = br1;
            dat[5] = br2;
            dat[6] = bl1;
            dat[7] = bl2;
            writeMsg(0xaa, dat, 8, false);
            br1 = br2 = bl1 = bl2 = -1;
        } else {
            fr1 = dat[0];
            fr2 = dat[1];
            fl1 = dat[2];
            fl2 = dat[3];
        }
    } else if (id == 0xb2) {
        uint8_t dat[8];
        dat[4] = CAN.read();
        dat[5] = CAN.read();
        dat[6] = CAN.read();
        dat[7] = CAN.read();
        if (fr1 != -1) {
            dat[0] = fr1;
            dat[1] = fr2;
            dat[2] = fl1;
            dat[3] = fl2;
            writeMsg(0xaa, dat, 8, false);
            br1 = br2 = bl1 = bl2 = -1;
        } else {
            br1 = dat[4];
            br2 = dat[5];
            bl1 = dat[6];
            bl2 = dat[7];
        }
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

    //0x2e4 STERING_LKAS
    LKAS_MSG[0] = ((LKAS_MSG[0] |= 1UL << 0) & ~0x7F) | (lkasCounter * 0x7F);
    writeMsg(0x2e4, LKAS_MSG, 5, true);
    lkasCounter = (lkasCounter + 1) % 64;

    //0x1d2 msg PCM_CRUISE
    PCM_CRUISE_MSG[0] = (openEnabled << 5) & 0x20;
    PCM_CRUISE_MSG[5] = (openEnabled << 7) & 0x80;
    writeMsg(0x1d2, PCM_CRUISE_MSG, 8, true);

    //0x1d3 msg PCM_CRUISE_2
    PCM_CRUISE_2_MSG[1] = (openEnabled << 7) & 0x80 | 0x28;
    PCM_CRUISE_2_MSG[2] = setSpeed;
    writeMsg(0x1d3, PCM_CRUISE_2_MSG, 8, true);

    // 0x3bc msg GEAR_PACKET
    writeMsg(0x3bc, GEAR_MSG, 8, false);

    //0x614 msg steering_levers
    STEERING_LEVER_MSG[3] = (blinker_left << 5) & 0x20 | (blinker_right << 4) & 0x10;
    writeMsg(0x614, STEERING_LEVER_MSG, 8, true);

    delay(8);
}
