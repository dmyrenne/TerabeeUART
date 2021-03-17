#include <TerabeeUART.h>
#include <crc8.h>

/**
 * @brief Setup communication between microcontroller and Terabee EvoHub
 * @param serial Select which Serial _port you want to use. Hardware- and SoftwareSerial are supported.
 * @param rangeMode Select 
 * @param pixelMode select how the sensors are read by the hub
 * @param style Text or Binary mode
 */
TerabeeUART::TerabeeUART(HardwareSerial& serial, byte rangeMode, byte pixelMode, byte style) : _port(serial) {
  hwSerial = true;
  _style = style;
  _range = rangeMode;
  _pixelMode = pixelMode;
  serial.begin(UART_BAUD);
  this->_port = serial;
  begin();
}

TerabeeUART::TerabeeUART(SoftwareSerial& serial, byte rangeMode, byte pixelMode, byte style) : _port(serial) {
  hwSerial = false;
  _style = style;
  _range = rangeMode;
  _pixelMode = pixelMode;
  this->_port = serial;
  serial.begin(UART_BAUD);
  begin();
}

/**
 * @brief Setup communication between microcontroller and Terabee EvoHub
 * @param serial Select which Serial _port you want to use. Hardware- and SoftwareSerial are supported.
 */
TerabeeUART::TerabeeUART(HardwareSerial& serial) : _port(serial) {
  hwSerial = true;
  this->_port = serial;
  serial.begin(UART_BAUD);
}

TerabeeUART::TerabeeUART(SoftwareSerial& serial) : _port(serial) {
  hwSerial = false;
  this->_port = serial;
  serial.begin(UART_BAUD);
}

void TerabeeUART::begin(){
  _port.write(pixelModeList[_pixelMode], 4);
  _port.write(rangeModeList[_range], 4);
  _port.write(styleList[_style], 4);

  ind = 0;
}

/**
 * @brief Reads, translates and returns distance(s) from the sensor when using BINARY mode
 * @param[out] pixel1 in mm. 0 if CRC check fails on reading
 */
void TerabeeUART::read(int &pixel1){

  pixel1 = dist;

}
/**
 * @brief Reads, translates and returns distance(s) from the sensor when using BINARY mode
 * @param[out] pixel1 in mm. 0 if CRC check fails on reading
 * @param[out] pixel2 in mm. -1 if you use Singel-Pixel mode.
 */
void TerabeeUART::read(int &pixel1, int &pixel2){

  pixel1 = dist;
  pixel2 = dist1;

}
/**
 * @brief Reads, translates and returns distance(s) from the sensor when using BINARY mode
 * @param[out] pixel1 in mm. 0 if CRC check fails on reading
 * @param[out] pixel2 in mm. -1 if you use Singel-Pixel mode.
 * @param[out] pixel3 in mm. -1 if you use Singel-Pixel mode.
 * @param[out] pixel4 in mm. -1 if you use Singel-Pixel mode.
 */
void TerabeeUART::read(int &pixel1, int &pixel2, int &pixel3, int &pixel4){

  pixel1 = dist;
  pixel2 = dist1;
  pixel3 = dist2;
  pixel4 = dist3;

}

void TerabeeUART::update(){
  if (_port.available() > 0) {
    // Send data only when you receive data
    uint8_t inChar = _port.read();  
    //Serial.println(inChar);                      
    if (ind == 0) {
      if (inChar == 'T')
      {
        //Looking for frame start 'T'
        Framereceived[ind++] = inChar;
      }
      else return;
    }
    else if ((ind > 0) && (ind < 10))
    {
      //Gathering data
      Framereceived[ind++] = inChar;
    }

   if(ind == 4){
      if (crc8(Framereceived, 3) == Framereceived[3]){
        dist  = (Framereceived[1]<<8) + Framereceived[2];
        dist1 = dist2 = dist3 = -1;
        
        ind = 0;
        Framereceived[0] = 0;
      }
    }
    else if(ind == 6){
      if (crc8(Framereceived, 5) == Framereceived[5]){
        dist  = (Framereceived[1]<<8) + Framereceived[2];
        dist1 = (Framereceived[3]<<8) + Framereceived[4];
        dist2 = dist3 = -1;

        ind = 0;
        Framereceived[0] = 0;
      }
    }
    else if(ind == 10){
      if (crc8(Framereceived, 9) == Framereceived[9]){
        dist  = (Framereceived[1]<<8) + Framereceived[2];
        dist1 = (Framereceived[3]<<8) + Framereceived[4];
        dist2 = (Framereceived[5]<<8) + Framereceived[6];
        dist3 = (Framereceived[7]<<8) + Framereceived[8];
        
        ind = 0;
        Framereceived[0] = 0;
      }
    }
    else {
      dist = dist1 = dist2 = dist3 = -1;
    }
  }
}

uint8_t TerabeeUART::crc8(uint8_t *p, uint8_t len) {
  uint8_t i;
  uint8_t crc = 0x0;
  while (len--) {
    i = (crc ^ *p++) & 0xFF;
    crc = (crc_table[i] ^ (crc << 8)) & 0xFF;
  }
  return crc & 0xFF;
}