#include <CAN.h>

boolean openEnabled = false;
uint8_t setSpeed = 0x0;
boolean blinker_left = false;
boolean blinker_right = false;

bool lastState = false;
bool stateChanged = false;

// wheel speeds - front right 1 front right 2 etc... each wheel speed is 2 bytes
short fr1 = -1, fr2 = -1, fl1 = -1, fl2 = -1, br1 = -1, br2 = -1, bl1 = -1, bl2 = -1;
uint8_t lkasCounter = 0;

// Messages
const uint8_t LEAD_INFO_MSG[8] PROGMEM = {0xff, 0xf8, 0x00, 0x08, 0x7f, 0xe0, 0x00, 0x4e};
const uint8_t GEAR_MSG[8] PROGMEM = {0x0, 0x0, 0x0, 0x0, 0x0, 0x80, 0x0, 0x0};

uint8_t LKAS_MSG[5] = {0x0, 0x0, 0x0, 0x0, 0x0};
uint8_t PCM_CRUISE_MSG[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
uint8_t PCM_CRUISE_2_MSG[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
uint8_t STEERING_LEVER_MSG[8] = {0x29, 0x0, 0x01, 0x0, 0x0, 0x0, 0x76};

/**
    #onReceive
    Processes CAN messages that are needed for translation
*/
void onReceive(uint8_t packetSize) {
    long id = CAN.packetId();
    
    if (id == 0xb0) {
        uint8_t ffr1 = CAN.read();
        uint8_t ffr2 = CAN.read();
        uint8_t ffl1 = CAN.read();
        uint8_t ffl2 = CAN.read();
        if (br1 != -1) {
            uint8_t dat[8];
            dat[0] = ffr1;
            dat[1] = ffr2;
            dat[2] = ffl1;
            dat[3] = ffl2;
            dat[4] = br1;
            dat[5] = br2;
            dat[6] = bl1;
            dat[7] = bl2;
            CAN.beginPacket(0xaa);
            for (uint8_t i = 0; i < 8; i++) {
                CAN.write(dat[i]);
            }
            CAN.endPacket();
            br1 = br2 = bl1 = bl2 = -1;
        } else {
            fr1 = ffr1;
            fr2 = ffr2;
            fl1 = ffl1;
            fl2 = ffl2;
        }
        for (uint8_t i = 0; i < 4; i++) {
            CAN.read();
        }
    } else if (id == 0xb2) {
        uint8_t bbr1 = CAN.read();
        uint8_t bbr2 = CAN.read();
        uint8_t bbl1 = CAN.read();
        uint8_t bbl2 = CAN.read();
        if (fr1 != -1) {
            uint8_t dat[8];
            dat[0] = fr1;
            dat[1] = fr2;
            dat[2] = fl1;
            dat[3] = fl2;
            dat[4] = bbr1;
            dat[5] = bbr2;
            dat[6] = bbl1;
            dat[7] = bbl2;
            // Write Packet
            CAN.beginPacket(0xaa);
            for (uint8_t i = 0; i < 8; i++) {
                CAN.write(dat[i]);
            }
            CAN.endPacket();
            br1 = br2 = bl1 = bl2 = -1;
        } else {
            br1 = bbr1;
            br2 = bbr2;
            bl1 = bbl1;
            bl2 = bbl2;
        }
        for (uint8_t i = 0; i < 4; i++) {
            CAN.read();
        }
    }
}

bool writeMsg(uint8_t id, uint8_t *msg, bool checksum) {
    CAN.beginPacket(id);
    if (checksum) {
        attachChecksum(id, msg);
    }
    for (int i = 0; i < sizeof(msg); i++) {
        CAN.write(msg[i]);
    }
    return CAN.endPacket();
}

/**
    Leave the last byte of the array empty, then call this to
    fill the last byte with the checksum of the rest
*/
void attachChecksum(uint8_t id, uint8_t *msg) {
    getChecksum(msg, sizeof(msg) - 1, id);
}

int getChecksum(uint8_t *dat, uint8_t len, uint16_t addr) {
    uint8_t checksum = 0;
    checksum = ((addr & 0xFF00) >> 8) + (addr & 0x00FF) + len + 1;
    for (int ii = 0; ii < len; ii++) {
        checksum += (dat[ii]);
    }
    return checksum;
}


void setup() {
    Serial.begin(9600);
    Serial.println("init");

    CAN.setPins(9, 2);
    CAN.onReceive(onReceive);
    CAN.begin(500E3);

    pinMode(8, INPUT_PULLUP);
}

void loop() {

    // Media Button Handling
    bool buttonPressed = digitalRead(8) == 0;
    if (buttonPressed) {
        if (lastState != buttonPressed) {
            if (!stateChanged) {
                if (openEnabled) {
                    openEnabled = false;
                } else {
                    openEnabled = true;
                }
            }
            stateChanged = true;
        } else {
            stateChanged = false;
        }
    }
    lastState = buttonPressed;

    // Optimization: can pre-create arrays with values that don't change
    //                  then just set the values that do change

    //0x2e6 LEAD_INFO
    writeMsg(0x2e6, LEAD_INFO_MSG, false);

    //0x2e4 STERING_LKAS
    LKAS_MSG[0] = ((LKAS_MSG[0] |= 1UL << 0) & ~0x7F) | (lkasCounter * 0x7F);
    writeMsg(0x2e4, LKAS_MSG, true);
    lkasCounter = (lkasCounter + 1) % 64;

    //0x1d2 msg PCM_CRUISE
    PCM_CRUISE_MSG[0] = (openEnabled << 5) & 0x20;
    PCM_CRUISE_MSG[5] = (openEnabled << 7) & 0x80;
    writeMsg(0x1d2, PCM_CRUISE_MSG, true);

    //0x1d3 msg PCM_CRUISE_2
    PCM_CRUISE_2_MSG[1] = (openEnabled << 7) & 0x80 | 0x28;
    PCM_CRUISE_2_MSG[2] = setSpeed;
    writeMsg(0x1d3, PCM_CRUISE_2_MSG, true);

    // 0x3bc msg GEAR_PACKET
    writeMsg(0x3bc, GEAR_MSG, false);

    //0x614 msg steering_levers
    STEERING_LEVER_MSG[3] = (blinker_left << 5) & 0x20 | (blinker_right << 4) & 0x10;
    writeMsg(0x614, STEERING_LEVER_MSG, true);

    delay(8);
}
