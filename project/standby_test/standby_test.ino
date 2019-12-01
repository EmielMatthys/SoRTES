 #include <LoRa.h>

#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

#define SCK     15
#define MISO    14
#define MOSI    16
#define SS      8
#define RST     4
#define DI0     7
#define BAND    868700000  // 915E6
#define PABOOST true 
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while(!Serial) {;}
  
  LoRa.setPins(SS,RST,DI0); // CS, reset, IRQ pin

  if (!LoRa.begin(BAND,PABOOST)) {         
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                   // if failed, do nothing
  }
  pinMode(LED_BUILTIN,OUTPUT);

  Serial.println("Pre sleep");
  delay(1000);
  cli();

  USBDevice.detach();
  setup_watchdog();
  //shallow_sleep();
  USBDevice.attach();
  sei();
  
  Serial.begin(9600);
  while(!Serial) { // Should be stuck in this loop until second monitor window is opened, but just skips instead...
    delay(1000);
    digitalWrite(LED_BUILTIN, digitalRead(LED_BUILTIN) ^ 1);
  }
    
  delay(2000);
  Serial.println("Script started"); // This is executed but appears nowhere
  delay(100); 
}

void setup_watchdog()
{
  // The MCU Status Register (MCUSR) is used to tell the cause of the last
  // reset, such as brown-out reset, watchdog reset, etc.
  // NOTE: for security reasons, there is a timed sequence for clearing the
  // WDE and changing the time-out configuration. If you don't use this
  // sequence properly, you'll get unexpected results.

  // Clear the reset flag on the MCUSR, the WDRF bit (bit 3).
  MCUSR &= ~(1<<WDRF);

  // Configure the Watchdog timer Control Register (WDTCSR)
  // The WDTCSR is used for configuring the time-out, mode of operation, etc

  // In order to change WDE or the pre-scaler, we need to set WDCE (This will
  // allow updates for 4 clock cycles).

  // Set the WDCE bit (bit 4) and the WDE bit (bit 3) of the WDTCSR. The WDCE
  // bit must be set in order to change WDE or the watchdog pre-scalers.
  // Setting the WDCE bit will allow updates to the pre-scalers and WDE for 4
  // clock cycles then it will be reset by hardware.
  WDTCSR |= (1<<WDCE) | (1<<WDE);

  /**
   *  Setting the watchdog pre-scaler value with VCC = 5.0V and 16mHZ
   *  WDP3 WDP2 WDP1 WDP0 | Number of WDT | Typical Time-out at Oscillator Cycles
   *  0    0    0    0    |   2K cycles   | 16 ms
   *  0    0    0    1    |   4K cycles   | 32 ms
   *  0    0    1    0    |   8K cycles   | 64 ms
   *  0    0    1    1    |  16K cycles   | 0.125 s
   *  0    1    0    0    |  32K cycles   | 0.25 s
   *  0    1    0    1    |  64K cycles   | 0.5 s
   *  0    1    1    0    |  128K cycles  | 1.0 s
   *  0    1    1    1    |  256K cycles  | 2.0 s
   *  1    0    0    0    |  512K cycles  | 4.0 s
   *  1    0    0    1    | 1024K cycles  | 8.0 s
  */
  WDTCSR  = (0<<WDP3) | (1<<WDP2) | (1<<WDP1) | (1<<WDP0);
  // Enable the WD interrupt (note: no reset).
  WDTCSR |= _BV(WDIE);
}

void set_timer1()
{
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  OCR1A = 31250;
  TCCR1B |= _BV(WGM12) | _BV(CS12) ;
  TIMSK1 |= _BV(OCIE1A);
}

void shallow_sleep()
{
  cli(); 
  #if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) // Mega with 2560
  DIDR0 = 0xFF;
  DIDR2 = 0xFF;
  #elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega1284PA__) // Goldilocks with 1284p
  DIDR0 = 0xFF;
   
  #elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega8__) // assume we're using an Arduino with 328p
  DIDR0 = 0x3F;
   
  #elif defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__) // assume we're using an Arduino Leonardo with 32u4
  DIDR0 = 0xF3;
  DIDR2 = 0x3F; // Digital input Disable Register
  #endif
  // Analogue Comparator Disable
  // When the ACD bit is written logic one, the power to the Analogue Comparator is switched off.
  // This bit can be set at any time to turn off the Analogue Comparator.
  // This will reduce power consumption in Active and Idle mode.
  // When changing the ACD bit, the Analogue Comparator Interrupt must be disabled by clearing the ACIE bit in ACSR.
  // Otherwise an interrupt can occur when the ACD bit is changed.
  ACSR &= ~_BV(ACIE);
  ACSR |= _BV(ACD);


  // We need to compare benefits of turning ADC off versus the extra cost of extended conversion => first conversion takes twice as long
  //PRR0 |= _BV(PRADC);
  // - Brown-out detector can be turned off
  // Is done in critical section later =>
  // - WatchDogTimer can also be turned off
  //wdt_reset();
  //wdt_disable();
  // - OCD can be turned off by writing one to the JTD bit in MCUCR
  //MCUCR |= _BV(JTD);
   
  // There are several macros provided in the header file to actually put
  // the device into sleep mode.
  // SLEEP_MODE_IDLE (0)
  // SLEEP_MODE_ADC (_BV(SM0))
  // SLEEP_MODE_PWR_DOWN (_BV(SM1))
  // SLEEP_MODE_PWR_SAVE (_BV(SM0) | _BV(SM1))
  // SLEEP_MODE_STANDBY (_BV(SM1) | _BV(SM2))
  // SLEEP_MODE_EXT_STANDBY (_BV(SM0) | _BV(SM1) | _BV(SM2))
  
//  set_sleep_mode( SLEEP_MODE_IDLE ); // This should potentially go down to EXT_STANDBY or PWR_DOWN
  set_sleep_mode( SLEEP_MODE_PWR_DOWN );
  sleep_enable();
   
  sei();
  sleep_cpu(); // good night.
  // Ugh. I've been woken up. Better disable sleep mode.
  sleep_disable(); // sleep_reset is faster than sleep_disable() because it clears all sleep_mode() bits.
  sei();  
  //power_all_enable();
  //digitalWrite(LED_BUILTIN, digitalRead(LED_BUILTIN) ^ 1);//digitalRead(LED_BUILTIN) ^ 1);
}

ISR(WDT_vect)
{
  //SerialUSB.println("Watchdog ISR");
  digitalWrite(LED_BUILTIN, digitalRead(LED_BUILTIN) ^ 1);//digitalRead(LED_BUILTIN) ^ 1);
  delay(200);
  digitalWrite(LED_BUILTIN, digitalRead(LED_BUILTIN) ^ 1);//digitalRead(LED_BUILTIN) ^ 1);
  wdt_disable();
}

ISR(TIMER1_COMPA_vect)
{
  //digitalWrite(LED_BUILTIN, digitalRead(LED_BUILTIN) ^ 1);//digitalRead(LED_BUILTIN) ^ 1);
  //SerialUSB.println("Interrupt called!");
}

void loop() {
  delay(200);
    digitalWrite(LED_BUILTIN, digitalRead(LED_BUILTIN) ^ 1);//digitalRead(LED_BUILTIN) ^ 1);
}
