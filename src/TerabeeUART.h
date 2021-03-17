#ifndef TerabeeUART_H
#define TerabeeUART_H

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <HardwareSerial.h>


/** 
 * @param TEXT
 *
 * XXXX(, XXXX, XXXX, XXXX )/n
 *
 * - Outputs distance in mm
 *
 *  @param BINARY
 *
 * T XX XX XX XX CRC8
 *
 * - Header (two characters): T (84 decimal / 0x54 hex)
 * - Distance reading in millimeters** (2 bytes per sensor): XX
 * - Checksum (1 byte) of previous 19 bytes: CRC8
 * Evo Mini return T01CRC8 as an error messageif the sensor is unable to meassure the distance
 * 
 */
enum style {
    TEXT,
    BINARY
};

const byte styleList[2][4]       = {{0x00, 0x11, 0x01, 0x45},
                                    {0x00, 0x11, 0x02, 0x4C}};
/**
 * @param LONG
 * Meassures distances from 0.03m to 3.30m in 1-Pixel mode @ 20Hz
 * Meassures distances from 0.03m to 2.30m in 2-Pixel mode @ 8Hz
 * Meassures distances from 0.03m to 1.65m in 4-Pixel mode @ 4Hz
 * @param SHORT
 * Meassures distances from 0.03m to 1.35m in 1-Pixel mode @ 40Hz
 * Meassures distances from 0.03m to 1.35m in 2-Pixel mode @ 13Hz
 * Meassures distances from 0.03m to 1.35m in 4-Pixel mode @ 6Hz
*/
enum rangeMode {
    LONG,
    SHORT
};

const byte rangeModeList[2][4]   = {{0x00, 0x61, 0x03, 0xE9},
                                    {0x00, 0x61, 0x01, 0xE7}};

/**
 * @param SINGLE_PIXEL  Reads one distance 
 * @param TWO_PIXEL     Reads two distances (Left - Right)
 * @param QUAD_PIXEL    Reads four distances (Upper Left - Upper Right - Lower Left - Lower Right)
*/
enum pixelMode {
    SINGLE_PIXEL,
    TWO_PIXEL,
    QUAD_PIXEL
};

const byte pixelModeList[3][4]   = {{0x00, 0x21, 0x01, 0xBC},
                                    {0x00, 0x21, 0x03, 0xB2},
                                    {0x00, 0x21, 0x02, 0xB5}};

/**
 * Baud rate of the Evo Mini UART. 
 */
#define UART_BAUD            115200

class TerabeeUart {

    public:
        TerabeeUart(HardwareSerial& serial, byte rangeMode, byte pixelMode, byte style = BINARY);
        TerabeeUart(SoftwareSerial& serial, byte rangeMode, byte pixelMode, byte style = BINARY);
        TerabeeUart(HardwareSerial& serial);
        TerabeeUart(SoftwareSerial& serial);
        void read(int &pixel1);
        void read(int &pixel1, int &pixel2);
        void read(int &pixel1, int &pixel2, int &pixel3);
        void read(int &pixel1, int &pixel2, int &pixel3, int &pixel4);
        void update();

    private:
        uint8_t crc8(uint8_t *p, uint8_t len);
        uint8_t Framereceived[10];// The variable "Framereceived[]" will contain the frame sent by the TeraRanger
        int ind;
        bool hwSerial;
        void begin();
        
        Stream& _port;
        byte _range;
        byte _pixelMode;
        byte _style;

        char inData[10]; // Allocate some space for the string
        char inChar; // Where to store the character read
        bool tempBool = false;

        int dist, dist1, dist2, dist3 = 0;

};


#endif