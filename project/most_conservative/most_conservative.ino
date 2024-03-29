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
//   After 20 consecutive packets, deep sleep will be activated. An external interupt can wake up the board.

#define INCLUDE_vTaskSuspend 1;
#define PACKET_SIZE 5
#define WAKEUP_MARGIN_TICKS 35
#define BEACON_MAX 20
//#define DEBUG 1
#define DEBUG_LED 1
#define DEBUG_DUMMY 1
//#define DEBUG_CLEAR_EEPROM 1

//-------------- DB Structures START --------------
#define TABLE_SIZE 1024
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
SemaphoreHandle_t gSemCommand;

//-------------- Task headers START -------------
void TaskListen(void*);
void TaskCommands(void*);
void print_record(int);
void print_whole_db();
void print_dberror(EDB_Status);

int gBeaconCount = 0;
bool bWarmingUp = false;
void setup() {
  
  Serial.begin(9600);
  while(!Serial) {;}
  
  LoRa.setPins(SS,RST,DI0); // CS, reset, IRQ pin

  if (!LoRa.begin(BAND,PABOOST)) {         
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                   // if failed, do nothing
  }

  pinMode(LED_BUILTIN,OUTPUT);

  #ifdef DEBUG_CLEAR_EEPROM
  for(int i = 0; i < EEPROM.length(); i++)
  {
    EEPROM.write(i, 0);
  }
  Serial.println("EEPROM CLEARED");
  #endif
  

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

  gSemCommand = xSemaphoreCreateBinary();
  xSemaphoreTake(gSemCommand, 0); // Take it if not taken already

  gBeaconCount = 0;
  digitalWrite(LED_BUILTIN, LOW); // Turn off the BuiltIn LED

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
int gLedCount = 0;
void TaskListen(void* pParams)
{
  (void) pParams;
  
  for(;;)
  {
    if(gBeaconCount >= BEACON_MAX)
    {
      Serial.println("Going into power down");
      power_down();
      //vTaskDelay(portMAX_DELAY);
      continue;
    }

    #ifdef DEBUG_LED
    enable_led();
    #endif
    
    // Parse incoming packet
    int packetSize = LoRa.parsePacket();
    #ifdef DEBUG_DUMMY
    packetSize = PACKET_SIZE;
    #endif
    
    if (packetSize == PACKET_SIZE )
    {
      
      // Read whole packet into char buffer of same size
      char inputBuffer[packetSize]; // Kan zijn dat dit niet werkt
      inputBuffer[packetSize] = 0;
      int i = 0;
      while (LoRa.available()) {
        inputBuffer[i++] = (char)LoRa.read();
      }

      #ifdef DEBUG_DUMMY
      dummyBeacon(inputBuffer, PACKET_SIZE);
      #endif

      #ifdef DEBUG
      Serial.print("Received packet: ");
      Serial.println(inputBuffer);
      #endif

      // Check if correct GW number
      if(inputBuffer[2] == '0' && inputBuffer[3] == '4')
      {
        
        // Convert next delay chars to integer and tickcount
        int iDelay = parseDelay(&inputBuffer[4]);
        //int iDelay = String(&inputBuffer[5]).toInt();
        TickType_t tDelay = iDelay*1000/portTICK_PERIOD_MS;

        #ifdef DEBUG
        Serial.print("Converted iDelay, tDelay: ");
        Serial.print(iDelay);
        Serial.print(", ");
        Serial.println(tDelay);
        #endif
        
        
        // Read the temperature of the chip and send the result back to the GW
        bWarmingUp = true;
        //vTaskDelay(15); // Allow ADC to recover from sleep
        
        double temp = read_temp();
        bWarmingUp = false;
  
        LoRa.beginPacket();
        LoRa.print(temp);
        LoRa.endPacket();  
        gBeaconCount++;
        
        // Before writing to DB, attempt to take the Semaphore for it
        
        if(xSemaphoreTake(gSemDB, 50) != pdTRUE)
        {
          Serial.println("SEMAPHORE DEALDOCK");
          continue;
        }

        // The currentRecord struct object is used to create and read records.
        currentRecord.id = gRecNr++;
        currentRecord.temp = temp;
        currentRecord.next_beacon = iDelay;

        // After filling out the record object, we attempt to append it to the database
        EDB_Status result = db.appendRec( (byte*) &currentRecord );
        if(result != EDB_OK){print_dberror(result);}

        // Give Semaphore back
        xSemaphoreGive(gSemDB);

        // Delay this task so the board can go to shallow sleep
        #ifdef DEBUG_LED
        disable_led();
        #endif
        if(gBeaconCount < BEACON_MAX) vTaskDelay(tDelay - WAKEUP_MARGIN_TICKS); 
        continue;
      }
      //disable_led();
    }
    #ifdef DEBUG_LED
        disable_led();
    #endif
    
    // This section only executes when something about the packet was incorrect (size, GW number...)
    //while (LoRa.available()) { LoRa.read(); }
    vTaskDelay(1); // Give lower tasks a chance ;)
  }
}

#ifdef DEBUG_LED
inline void enable_led()
{
  digitalWrite(LED_BUILTIN, HIGH);
  gLedCount++;
}

inline void disable_led()
{
  if(--gLedCount <= 0) digitalWrite(LED_BUILTIN, LOW);
}
#endif

/*******************************************************************************
* Name: parseDelay
* Description: Transforms the given characters (2) into an integer. 
*               0 <= result <= 99
*               0 if not parsable
*Parameter: char* buff :
*             Buffer containing the two characters to be parsed
*******************************************************************************/
inline int parseDelay(char* buff)
{
  #ifdef DEBUG
  Serial.print("Parser called with: ");
  Serial.print((int) buff[0]);
  Serial.println((int) buff[1]);
  #endif
  int result = 0;
  if((int) buff[0] >= 48 && (int) buff[0] <= 57)
  {
    result += (int) buff[0] - 48;
  }
  else return 0;

  return result;
}

/*******************************************************************************
* Name: dummyBeacon
* Description: Generates dummy input to test the ListenTask command
*         Result is of this form:
*           buff[0..1] == 'GW'
*           buff[2..3] == XX where XX represents the GW number (eg 04)
*           buff[4..5] == YY where YY represents the delay (eg 09)
*        Returns the size of the buffer
* Note: only works if size == PACKET_SIZE (== 6)
*******************************************************************************/
inline int dummyBeacon(char* buff, size_t size)
{
  if(size != PACKET_SIZE) {return;}
  buff[0] = 'G';
  buff[1] = 'W';
  buff[2] = '0';
  buff[3] = '4';

  buff[4] = '1';
  return PACKET_SIZE;
}

/*******************************************************************************
* Name: TaskCommands
* Description: Checks if a serial command has been sent and responds accordingly.
* Cases:
*   command == 1:   print the current record
*   command == 2:   print all records in EEPROM
*   command == 3:   enter deep sleep (powerdown)
*******************************************************************************/
void TaskCommands(void*) //Commands should only be supported in deep sleep => aka ADC can turn off
{
  for(;;){
    xSemaphoreTake(gSemCommand, portMAX_DELAY);
    if(Serial.available()){
      int command = Serial.parseInt();
      switch(command){
        case 0: // null char / eos
          break;
        case 1: 
         print_record(db.count());
         break;
       case 2:
        print_whole_db();
        break;
       case 3:
        taskENTER_CRITICAL();
        power_down();
        taskEXIT_CRITICAL();
        sei();
        break;
       default:
        Serial.print("Invalid command: ");
        Serial.println(command);
        break;
      }
      //Serial.flush();
    }
  }
  //vTaskDelay(1000 / portTICK_PERIOD_MS); // Fast enough for usability
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
  if(!bWarmingUp) power_adc_disable();
//  
//  power_spi_disable();
  power_twi_disable();
  power_timer0_disable();
  power_timer1_disable();
   
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__) // assume we're using an Arduino Leonardo with 32u4
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

  portENTER_CRITICAL();
  sleep_enable();
  
  portEXIT_CRITICAL();
  sleep_cpu(); // good night.
  portENTER_CRITICAL();

   
  // Ugh. I've been woken up. Better disable sleep mode.
  sleep_reset(); // sleep_reset is faster than sleep_disable() because it clears all sleep_mode() bits.
  power_adc_enable();
  portEXIT_CRITICAL();
  
  // Shouldnt more stuff be turned on again
  if(Serial.available())
    xSemaphoreGive(gSemCommand); // Signal Command Task

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
inline void power_down()
{
  LoRa.sleep(); // Make sure the radio is off
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  cli();
  sleep_enable();
  #if defined(BODS) && defined(BODSE)
  sleep_bod_disable();
  #endif
//  power_adc_disable();
//  power_spi_disable();
//  power_timer0_disable();
//  power_timer1_disable();
//  power_timer2_disable();
//  power_twi_disable();
  power_all_disable();
  sleep_cpu();
  sleep_disable();
  power_all_enable();
  gBeaconCount = 0;
}
