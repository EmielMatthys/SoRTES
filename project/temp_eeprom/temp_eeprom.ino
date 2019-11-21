#include <LoRa.h>
#include <Arduino_FreeRTOS.h>

//LoR32u4II 868MHz or 915MHz (black board)
  #define SCK     15
  #define MISO    14
  #define MOSI    16
  #define SS      8
  #define RST     4
  #define DI0     7
  #define BAND    868700000  // 915E6
  #define PABOOST true 

// Description:
//   This FreeRTOS program will wait for incoming beacon, save and send temperature data and go into sleep mode (timer still active)
//   After 20 consecutive connections, deep sleep will be activated. An external interupt can wake up the board.
//   TODO: commands
//   Zet temp sturen op hogere prioriteit dan opslaan task (kan worden gedaan na het sturen).
//   Houd bij hoe lang het duurde om tasks uit te voeren --> om slaap beter te berekenen.

void setup() {
  
  Serial.begin(9600);
  while(!Serial) {;}
  
  LoRa.setPins(SS,RST,DI0); // CS, reset, IRQ pin

  if (!LoRa.begin(BAND,PABOOST)) {         
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                   // if failed, do nothing
  }

  
}

void loop() // Remember that loop() is simply the FreeRTOS idle task. Something to do, when there's nothing else to do.
{
  
}
