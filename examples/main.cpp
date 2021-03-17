#include <Arduino.h>
#include <TerabeeUART.h>
#include <Ticker.h>

TerabeeUart sensor(Serial1);

void update(){
  int pixel;
  sensor.read(pixel);
  Serial.printf("Distance in mm: %i\r\n", pixel);
}

Ticker timer(update, 500);

void setup(){
    Serial.begin(115200);
    timer.start();
    
}

void loop(){
  timer.update();
  sensor.update();
}
