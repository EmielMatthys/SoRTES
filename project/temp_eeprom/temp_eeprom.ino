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

#define INCLUDE_vTaskSuspend 1;

//-------------- DB Structures START --------------
#define TABLE_SIZE 512
struct record {
  int id;
  double temp;
  int next_beacon;
} currentRecord;

void writer(unsigned long address, byte data)
{
  EEPROM.write(address, data);
}

byte reader(unsigned long address)
{
  return EEPROM.read(address);
}

EDB db(&writer, &reader);
int gRecNr = 1; // We start at index 1

//-------------- Semaphores/Mutex' START---------
SemaphoreHandle_t gSemDB;

//-------------- Task headers START -------------
void TaskListen(void*);
void TaskCommands(void*);
void print_record(int);
void print_whole_db();
void print_dberror(EDB_Status);

void setup() {
  
  Serial.begin(9600);
  while(!Serial) {;}
  
  LoRa.setPins(SS,RST,DI0); // CS, reset, IRQ pin

  if (!LoRa.begin(BAND,PABOOST)) {         
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                   // if failed, do nothing
  }

  // Attempt to open existing DB
  EDB_Status stat = db.open(0);

  if(stat != EDB_OK || db.count() == 0)
  {
    Serial.println("No table found, new will be created.");
    db.create(0, TABLE_SIZE, sizeof(record));
  }
  else
  {
    Serial.print("Found previous table! Record count: ");
    Serial.println(db.count());
    gRecNr = db.count() + 1; // index is [1,count(db)]
  }

  gSemDB = xSemaphoreCreateBinary();
  xSemaphoreGive(gSemDB);

  xTaskCreate(TaskListen,
  "TaskListen",
  128,
  NULL,
  2,
  NULL);

  xTaskCreate(TaskCommands,
  "CommandTask",
  128,
  NULL,
  0, // We dont need instant response for decent usability
  NULL);
}

/*******************************************************************************
* Name: TaskListen
* Description: Listens for incoming packets, records and saves their value. 
*               Afterwards, the current temperature is measured and sent back.
* Note: Currently does not listen to incoming signals but generates dummy values
*       instead.
*******************************************************************************/
int gBeaconCount = 0;
void TaskListen(void* pParams)
{
  (void) pParams;
  
  for(;;)
  {
    if(gBeaconCount++ >= 20)
    {
      power_down();
      //vTaskDelay(portMAX_DELAY);
      continue;
    }
    int packetSize = LoRa.parsePacket();
    String Message;
    String Delay;
    if (packetSize) {
      int charCounter = 0;
      while (charCounter<4) {
        charCounter++;
        char c = (char)LoRa.read();
        Message += c;
        Serial.write(c);
      }
      while (LoRa.available()) {
        Delay+=LoRa.read();
      }
    }
    int next_delay = Delay.toInt();
    TickType_t ticks= next_delay*1000/portTICK_PERIOD_MS;
    vTaskDelay(ticks-30);
    
    //char gw[5] = "GW04"; // moet worden uitgelezen uit packet, zie thibaut
//    Serial.print("Received from gateway ");
//    Serial.print(gw);
//    Serial.print(". Next beacon: ");
//    Serial.print(next_delay);
//    Serial.println(" seconds.");

    double temp = read_temp();
//    Serial.print("Temperature: ");
//    Serial.println(temp);

    LoRa.beginPacket();
    LoRa.print(temp);
    LoRa.endPacket();    

    xSemaphoreTake(gSemDB, portMAX_DELAY);
    currentRecord.id = gRecNr++;
    currentRecord.temp = temp;
    currentRecord.next_beacon = next_delay;
    EDB_Status result = db.appendRec( (byte*) &currentRecord );
    if(result != EDB_OK){print_dberror(result);}
    xSemaphoreGive(gSemDB);
    
    vTaskDelay(2000 / portTICK_PERIOD_MS );
  }
}

/*******************************************************************************
* Name: TaskCommands
* Description: Checks if a serial command has been sent (approx every second) 
*               and responds accordingly.
* Cases:
*   command == 1:   print the current record
*   command == 2:   print all records in EEPROM
*   command == 3:   enter deep sleep (powerdown)
*******************************************************************************/
void TaskCommands(void*)
{
  for(;;){
    if(Serial.available()){
      int command = Serial.parseInt();
      switch(command){
        case 1: 
         print_record(currentRecord.id);
         break;
       case 2:
        print_whole_db();
        break;
       case 3:
        power_down();
        break;
      }
    }
  }
  vTaskDelay(1000 / portTICK_PERIOD_MS); // Fast enough for usability
}

/*******************************************************************************
* Name: print_record
* Description: Prints the values (columns) of the record associated with the 
*               given record id.
* Params:
*   rec:    The ID associated with the record to be printed.
*******************************************************************************/
inline void print_record(int rec)
{
  Serial.print("recno = ");
  Serial.println(rec);
  xSemaphoreTake(gSemDB, portMAX_DELAY);
  EDB_Status result = db.readRec(rec, EDB_REC currentRecord);
  if(result != EDB_OK) print_dberror(result);
  Serial.print("Record ID: ");
  Serial.println(currentRecord.id);
  Serial.print("  temperature: ");
  Serial.println(currentRecord.temp);
  Serial.print("  next_beacon: ");
  Serial.println(currentRecord.next_beacon);
  xSemaphoreGive(gSemDB);
}

/*******************************************************************************
* Name: print_dberror
* Description: Prints an error message based on the given error status.
* Params:
*   err:    (EDB_Status) the status for which to print the error (see EDB library)
*******************************************************************************/
void print_dberror(EDB_Status err)
{
  Serial.print("DB ERROR: ");
  switch (err)
  {
    case EDB_OUT_OF_RANGE:
      Serial.println("Recno out of range");
      break;
    case EDB_TABLE_FULL:
      Serial.println("Table full");
      break;
    case EDB_OK:
    default:
      Serial.println("OK");
      break;
  }
}

/*******************************************************************************
* Name: print_whole_db
* Description: Prints all the records stored in EEPROM
*******************************************************************************/
inline void print_whole_db()
{
  for(int i = 1; i <= db.count(); i++) { print_record(i); }
}

/*******************************************************************************
* Name: loop
* Description: This is the idle task. It puts the board to shallow sleep. (IDLE)
*               Copied from https://feilipu.me/2015/11/24/arduino_freertos/
*******************************************************************************/
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
   
  set_sleep_mode( SLEEP_MODE_IDLE ); // This should potentially go down to EXT_STANDBY or PWR_DOWN
   
  portENTER_CRITICAL(); //Enter critical section
  sleep_enable();
   
  // Only if there is support to disable the brown-out detection.
  #if defined(BODS) && defined(BODSE)
  sleep_bod_disable();
  #endif
   
  portEXIT_CRITICAL(); // Exit critical section
  sleep_cpu(); // good night.
   
  // Ugh. I've been woken up. Better disable sleep mode.
  sleep_reset(); // sleep_reset is faster than sleep_disable() because it clears all sleep_mode() bits.
}

/*******************************************************************************
* Name: read_temp
* Description: Reads out temperature from the ATMEGA32u4 internal thermometer
*               Based on ATMEGA32u4 datasheet and 
*               https://playground.arduino.cc/Main/InternalTemperatureSensor/
*******************************************************************************/
inline double read_temp()
{
  unsigned int wADC;
  double t;

  // The internal temperature has to be used
  // with the internal reference of 1.1V.
  // Channel 8 can not be selected with
  // the analogRead function yet.

  // Set the internal reference and mux.
  ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX2) | _BV(MUX1)  | _BV(MUX0) );
  ADCSRA |= _BV(ADEN);  // enable the ADC
  ADCSRB |= _BV(MUX5);

  //delay(20);            // wait for voltages to become stable.
  vTaskDelay( 20 / portTICK_PERIOD_MS );


  ADCSRA |= _BV(ADSC);  // Start the ADC

  // Detect end-of-conversion
  while (bit_is_set(ADCSRA,ADSC));
  
  //while ( ADIF & 1 == 0 );

  // Reading register "ADCW" takes care of how to read ADCL and ADCH.
  wADC = ADCW;

  // The offset of 324.31 could be wrong. It is just an indication.
  t = (wADC- 273.15);

  // The returned temperature is in degrees Celcius.
  return (t);
}

/*******************************************************************************
* Name: power_down
* Description: Puts the board into deep sleep mode (powerdown).
*               See datasheet for waking up sources (mostly external int's).
*******************************************************************************/
void power_down()
{
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  cli();
  sleep_enable();
  #if defined(BODS) && defined(BODSE)
  sleep_bod_disable();
  #endif
  sei();
  sleep_cpu();
  sleep_disable();
  sei();  
}
