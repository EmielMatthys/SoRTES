#include <EDB.h>
#include <LoRa.h>
#include <Arduino_FreeRTOS.h>
#include <EEPROM.h>
#include <semphr.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

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
  // Digital Input Disable on Analogue Pins
  // When this bit is written logic one, the digital input buffer on the corresponding ADC pin is disabled.
  // The corresponding PIN Register bit will always read as zero when this bit is set. When an
  // analogue signal is applied to the ADC7..0 pin and the digital input from this pin is not needed, this
  // bit should be written logic one to reduce power consumption in the digital input buffer.
   
  #if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) // Mega with 2560
  DIDR0 = 0xFF;
  DIDR2 = 0xFF;
  #elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega1284PA__) // Goldilocks with 1284p
  DIDR0 = 0xFF;
   
  #elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega8__) // assume we're using an Arduino with 328p
  DIDR0 = 0x3F;
   
  #elif defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__) // assume we're using an Arduino Leonardo with 32u4
  DIDR0 = 0xF3;
  DIDR2 = 0x3F;
  #endif
  // Analogue Comparator Disable
  // When the ACD bit is written logic one, the power to the Analogue Comparator is switched off.
  // This bit can be set at any time to turn off the Analogue Comparator.
  // This will reduce power consumption in Active and Idle mode.
  // When changing the ACD bit, the Analogue Comparator Interrupt must be disabled by clearing the ACIE bit in ACSR.
  // Otherwise an interrupt can occur when the ACD bit is changed.
  ACSR &= ~_BV(ACIE);
  ACSR |= _BV(ACD);
   
  // There are several macros provided in the header file to actually put
  // the device into sleep mode.
  // SLEEP_MODE_IDLE (0)
  // SLEEP_MODE_ADC (_BV(SM0))
  // SLEEP_MODE_PWR_DOWN (_BV(SM1))
  // SLEEP_MODE_PWR_SAVE (_BV(SM0) | _BV(SM1))
  // SLEEP_MODE_STANDBY (_BV(SM1) | _BV(SM2))
  // SLEEP_MODE_EXT_STANDBY (_BV(SM0) | _BV(SM1) | _BV(SM2))
   
  set_sleep_mode( SLEEP_MODE_IDLE );
   
  portENTER_CRITICAL();
  sleep_enable();
   
  // Only if there is support to disable the brown-out detection.
  #if defined(BODS) && defined(BODSE)
  sleep_bod_disable();
  #endif
   
  portEXIT_CRITICAL();
  sleep_cpu(); // good night.
   
  // Ugh. I've been woken up. Better disable sleep mode.
  sleep_reset(); // sleep_reset is faster than sleep_disable() because it clears all sleep_mode() bits.
}
