#include <CAN.h>

//______________BUTTONS AND SWITCHES
int button4 = 8;
int button3 = 7;
int button2 = 6;
int button1 = 9;
int pedal = A4;
int buttonstate4;
int lastbuttonstate4;
int buttonstate3;
int lastbuttonstate3;
int buttonstate2;
int lastbuttonstate2;
int buttonstate1;
int lastbuttonstate1;

//______________VALUES SEND ON CAN
boolean OP_ON = false;
uint8_t set_speed = 0x0;
int gas_pedal_state = 0;   // TODO: Remove gas_pedal_state
int brake_pedal_state = 0; // TODO: Remove brake_pedal_state
double average = 0;
boolean blinker_left = true;
boolean blinker_right = true;

// wheel speeds - front right 1 front right 2 etc... each wheel speed is 2 bytes
short fr1 = -1, fr2 = -1, fl1 = -1, fl2 = -1, br1 = -1, br2 = -1, bl1 = -1, bl2 = -1;
uint8_t lkasCounter = 0;

void setup() {

    CAN.begin(500E3);
    pinMode(7, INPUT);
    pinMode(8, INPUT);

}

void loop() {

    int modeState = analogRead(A0);
    int quadState = analogRead(A1);

    if (modeState > 2.5) {
        if (OP_ON) {
            OP_ON = false;
        } else {
            OP_ON = true;
        }
    }

    // TODO: get actual values
    if (quadState >= 1 && quadState <= 1) {  // volume +

    } else if (quadState >= 1 && quadstate <= 1) {  // volume -

    } else if (quadState >= 1 && quadstate <= 1) {  // seek +

    } else if (quadState >= 1 && quadstate <= 1) {  // seek -
    
    }

//    buttonstate4 = digitalRead(button4);
//    buttonstate3 = digitalRead(button3);
//    buttonstate2 = digitalRead(button2);
//    buttonstate1 = digitalRead(button1);
//
//    if (buttonstate4 != lastbuttonstate4) {
//        if (buttonstate4 == LOW) {
//            if (OP_ON == true) {
//                OP_ON = false;
//            } else if (OP_ON == false) {
//                OP_ON = true;
//                set_speed = (average += 3);
//            }
//        }
//    }
//
//    blinker_right = buttonstate3 == LOW;
//    blinker_left = buttonstate2 == LOW;
//
//    if (buttonstate1 != lastbuttonstate1 && buttonstate1 == LOW) {
//        set_speed += 5;
//    }
//
//    lastbuttonstate1 = buttonstate1;
//    lastbuttonstate2 = buttonstate2;
//    lastbuttonstate3 = buttonstate3;
//    lastbuttonstate4 = buttonstate4;


    // TODO: read steering wheel angle over time and compute degrees per second

    long id = CAN.packetId();
    
    if (id == 0x399) { // read PCM_CRUISE_SM for if cruise control is enabled or not
        uint8_t byte1 = CAN.read();
        uint8_t byte2 = CAN.read();
        uint8_t cruiseEnabled = (byte2 & 0x2);
        OP_ON = cruiseEnabled == 1;        
    }

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
            br1 = -1;
            br2 = -1;
            bl1 = -1;
            bl2 = -1;    
        } else {
            fr1 = ffr1;
            fr2 = ffr2;
            fl1 = ffl1;
            fl2 = ffl2;
        }
        for (uint8_t i = 0; i < 4; i++) {
            CAN.read();
        }
        id = CAN.packetId();
    }

    if (id == 0xb2) {
        // Read wheel speeds from CAN, the first 4 bytes are wheel speeds
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
            br1 = -1;
            br2 = -1;
            bl1 = -1;
            bl2 = -1;    
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

    //______________SENDING_CAN_MESSAGES


    // just to test if OpenPilot will acknowledge the car as a 2017
    //0x2e4 STERING_LKAS
    uint8_t lkasDat[8];
    uint8_t val = 0;
    val = val |= 1UL << 0; // set first bit to zero
    val = (val & ~0x7F) | (lkasCounter * 0x7F); // set interal 6 bits to counter value
    lkasDat[0] = val;
    lkasDat[1] = 0x0;
    lkasDat[2] = 0x0;
    lkasDat[3] = 0x0;
    lkasDat[4] = can_cksum(lkasDat, 4, 0x2e4);
    CAN.beginPacket(0x2e4);
    for (uint8_t i = 0; i < 5; i++) {
        CAN.write(lkasDat[i]);
    }
    CAN.endPacket();
    lkasCounter = (lkasCounter + 1) % 64;

    //0x1d2 msg PCM_CRUISE
    uint8_t dat[8];
    dat[0] = (OP_ON << 5) & 0x20 | (!gas_pedal_state << 4) & 0x10; // tie this into PCM_CRUISE_SM for openpilot on
    dat[1] = 0x0;
    dat[2] = 0x0;
    dat[3] = 0x0;
    dat[4] = 0x0;
    dat[5] = 0x0;
    dat[6] = (OP_ON << 7) & 0x80;
    dat[7] = can_cksum(dat, 7, 0x1d2);
    CAN.beginPacket(0x1d2);
    for (int ii = 0; ii < 8; ii++) {
        CAN.write(dat[ii]);
    }
    CAN.endPacket();

    //0x1d3 msg PCM_CRUISE_2
    uint8_t dat2[8];
    dat2[0] = 0x0;
    dat2[1] = (OP_ON << 7) & 0x80 | 0x28; // need to tie this into cruise control PCM_CRUISE_SM for openpilot on
    dat2[2] = set_speed;
    dat2[3] = 0x0;
    dat2[4] = 0x0;
    dat2[5] = 0x0;
    dat2[6] = 0x0;
    dat2[7] = can_cksum(dat2, 7, 0x1d3);
    CAN.beginPacket(0x1d3);
    for (int ii = 0; ii < 8; ii++) {
        CAN.write(dat2[ii]);
    }
    CAN.endPacket();

    // 0x3bc msg GEAR_PACKET
    uint8_t dat7[8];
    dat7[0] = 0x0;
    dat7[1] = 0x0;
    dat7[2] = 0x0;
    dat7[3] = 0x0;
    dat7[4] = 0x0;
    dat7[5] = 0x80;
    dat7[6] = 0x0;
    dat7[7] = 0x0;
    CAN.beginPacket(0x3bc);
    for (int ii = 0; ii < 8; ii++) {
        CAN.write(dat7[ii]);
    }
    CAN.endPacket();

    //0x614 msg steering_levers
    uint8_t dat614[8];
    dat614[0] = 0x29;
    dat614[1] = 0x0;
    dat614[2] = 0x01;
    dat614[3] = (blinker_left << 5) & 0x20 | (blinker_right << 4) & 0x10;
    dat614[4] = 0x0;
    dat614[5] = 0x0;
    dat614[6] = 0x76;
    dat614[7] = can_cksum(dat614, 7, 0x614);
    CAN.beginPacket(0x614);
    for (int ii = 0; ii < 8; ii++) {
        CAN.write(dat614[ii]);
    }
    CAN.endPacket();
}

int can_cksum(uint8_t *dat, uint8_t len, uint16_t addr) {
    uint8_t checksum = 0;
    checksum = ((addr & 0xFF00) >> 8) + (addr & 0x00FF) + len + 1;
    for (int ii = 0; ii < len; ii++) {
        checksum += (dat[ii]);
    }
    return checksum;
}
