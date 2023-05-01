#include "main.h"

void drawDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  u8g2.print(":");
  if(digits < 10) {
    u8g2.print("0");
  }
  Serial.print(digits);
  u8g2.print(digits, DEC);
}


void drawTime(void) {
  uint8_t x = 5;
  u8g2.setFont(u8g_font_7x14);
   if (!rtc.isrunning()) {
    u8g2.setCursor(5, 20);
    u8g2.print("RTC:OFF");
    // rtc.adjust(DateTime(__DATE__, __TIME__));
  } else {
    char * stringDateBuffer = (char *)malloc(sizeof(char) * 10);
    if (sprintf(stringDateBuffer, "%04d/%02d/%02d", year(), month(), day()) > 0) {
      u8g2.setCursor(x, 11);
      u8g2.println(stringDateBuffer);
    }
    free(stringDateBuffer);
    char * stringTimeBuffer = (char *)malloc(sizeof(char) * 8);
    if (sprintf(stringDateBuffer, "%02d:%02d:%02d", hour(), minute(), second()) > 0) {
      u8g2.setCursor(x, 25);
      u8g2.println(stringTimeBuffer);
    }
    free(stringDateBuffer);
  }
}

time_t syncProvider() {
  return rtc.now().unixtime();
}

void draw_elecrow(void) {
  // u8g.drawXBMP(32, 0, 64, 64, bitmap_logo);
}


void drawflower() {
  uint8_t i;
  uint8_t plants = getPlants();
  uint8_t x_positions[4] = { 0, 32, 64, 96 };
  MoiustureSensorData sensor = readMoistureSensors();
  for (i = 0; i < plants; i++) {
    if (sensor.data[i] < 30) {
      u8g2.drawXBMP(x_positions[i], 0, 32, 30, bitmap_bad);
    } else {
      u8g2.drawXBMP(x_positions[i], 0, 32, 30, bitmap_good);
    }
  }
}

/*
 * Setup function
*/
void setup() {
  uint8_t i;
  Serial.begin(9600); // for debugging
  Alarm.delay(2000);

  Wire.begin();

  /* Sanity check */
  if (!rtc.begin(&Wire)) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    abort();
  } else if (!rtc.isrunning()) {
    Serial.println("RTC is not running resetting");
    Serial.flush();
    rtc.adjust(DateTime(__DATE__, __TIME__));
    abort();
  }

  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // comment out after use and reupload
  setPlants(4); // set amount of plants
  setPotSize(0, 4); // set pot size
  setPotSize(1, 4); // set pot size
  setPotSize(2, 4); // set pot size
  setPotSize(3, 4); // set pot size
  setLitersPumpedPerMinute(1); // set liters pumped per minute

  setSyncProvider(syncProvider);  //sets Time Library to RTC time
  setSyncInterval(5000);          //sync Time Library to RTC every 5 seconds


  if (timeStatus() != timeSet) {
    Serial.println("Unable to sync with the RTC");
  } else {
    Serial.println("RTC has set the system time");
  }

  // declare relay as output
  for (i = 0; i < sizeof(relays); i++) {
    pinMode(relays[i], OUTPUT);  
  }
  // declare pump as output
  pinMode(pump, OUTPUT);
  // declare switch as input
  pinMode(button, INPUT);
  // water_flower();
  Alarm.delay(2000); // Pause for 2 seconds
  
  // u8g.firstPage();
  // do {
  //   draw_elecrow();
  // } while (u8g.nextPage());

  Alarms *alarms = getAlarms();
  for (i = 0; i < MAX_ALARMS; i++) {
    boolean isActive = getActiveAlarm(i);
    if (isActive) {
        Serial.print("Alarm: [");
        Serial.print(i, DEC);
        Serial.println("] is active !");
        Serial.print("DOW:");
        Serial.print(printDow((timeDayOfWeek_t)alarms->alarm[i][0]));
        Serial.print("T:");
        Serial.print(alarms->alarm[i][1], DEC);
        Serial.print(":");
        Serial.print(alarms->alarm[i][2], DEC);
        Serial.print(":");
        Serial.print(alarms->alarm[i][3], DEC);
        Serial.print("\n");
    }
  }

  // Alarm.alarmRepeat(dowMonday, 14, 30, 00, pumpWater);  // 14:30:00 every Sunday
  // Alarm.alarmRepeat(dowWednesday, 14, 30, 00, pumpWater);  // 14:30:00 every Thesday
  // Alarm.alarmRepeat(dowFriday, 14, 30, 00, pumpWater);  // 14:30:00 every Thursday
  // // Test
  // Alarm.alarmRepeat(dowWednesday, 01, 49, 00, pumpWater);  // 14:30:00 every Thursday

  serialCommands.SetDefaultHandler(&cmdUnrecognized);
  serialCommands.AddCommand(&cmdSetTimeAndDate_);
  serialCommands.AddCommand(&cmdGetAlarms_);
  serialCommands.AddCommand(&cmdResetAlarms_);
  serialCommands.AddCommand(&cmdReboot_);
  serialCommands.AddCommand(&cmdHelp_);
  serialCommands.AddCommand(&cmdTestPump_);
  serialCommands.AddCommand(&cmdGetActiveAlarm_);
  serialCommands.AddCommand(&cmdGetActiveAlarms_);
  serialCommands.AddCommand(&cmdSetActiveAlarms_);
  serialCommands.AddCommand(&cmdToggleActiveAlarms_);

  u8g2.begin();


  // pumpWater();
}

char * printDow(timeDayOfWeek_t dow) {
  switch(dow) {
    case dowSunday: 
      return daysOfTheWeek[0];
    case dowMonday:
      return daysOfTheWeek[1];
    case dowTuesday: 
      return daysOfTheWeek[2];
    case dowWednesday:
      return daysOfTheWeek[3];
    case dowThursday: 
      return daysOfTheWeek[4];
    case dowFriday:
      return daysOfTheWeek[5];
    case dowSaturday: 
      return daysOfTheWeek[6];
    default:
      return NULL;
  }
}

void testPump(uint32_t delay = 60) {
  time_t currentTime = now(), orgTime = now(); // store the current time in time variable t
  
  digitalWrite(pump, HIGH);
  while(currentTime < orgTime + delay) {
    Alarm.delay(1000);
    u8g2.firstPage();
    u8g2.setFont(u8g_font_7x14);
    do {
      char * stringBuffer = (char *)malloc(sizeof(char) * 64);
      sprintf(stringBuffer, "pumping time: %02lu seconds", currentTime - orgTime);
      u8g2.setCursor(10, 10);
      u8g2.println(stringBuffer);
      free(stringBuffer);
    } while (u8g2.nextPage());
    currentTime = now(); // store the current time in time variable t
  }
  digitalWrite(pump, LOW);
}

// functions to be called when an alarm triggers:
void pumpWater() {
  uint8_t i;
  uint8_t plants = getPlants();
  uint8_t litersPerMinute = getLitersPerMinute();
  digitalWrite(pump, HIGH);
  for(i = 0; i < plants; i++) {
    time_t currentTime = now(), orgTime = now(); // store the current time in variable t
    float potLiters = (float)getPotSize(i) / 10;
    uint32_t relayDelay = ((potLiters / litersPerMinute) * 60); // to seconds
    digitalWrite(relays[i], HIGH);
    while(currentTime < orgTime + relayDelay) {
      u8g2.firstPage();
      u8g2.setFont(u8g_font_7x14);
      char * stringBuffer = (char *)malloc(sizeof(char) * 64);
      do {
        sprintf(stringBuffer, "plant: %d time: %02lu seconds", i, currentTime - orgTime);
        u8g2.setCursor(10, 10);
        u8g2.println("Watering");
        u8g2.setCursor(10, 20);
        u8g2.println(stringBuffer);
        free(stringBuffer);
      } while (u8g2.nextPage());
      Alarm.delay(1000);
      currentTime = now(); // store the current time in variable t
    }
    digitalWrite(relays[i], LOW);
  }
  digitalWrite(pump, LOW);
}

void pumpWater2() {
  uint8_t i;
  uint8_t plants = getPlants();
  uint8_t litersPerMinute = getLitersPerMinute();
  Serial.print("litersPerMinute: ");
  Serial.println(litersPerMinute, DEC);
  Serial.println("Alarm: - on - start");
  digitalWrite(pump, HIGH);
  for(i = 0; i < plants; i++) {
    Serial.print("watering plant: ");
    Serial.println(i, DEC);
    time_t currentTime = now(), orgTime = now(); // store the current time in time variable t
    float potLiters = (float)getPotSize(i) / 10;
    Serial.print("potLiters: ");
    Serial.println(potLiters, DEC);
    uint32_t relayDelay = ((potLiters / litersPerMinute) * 60); // to seconds
    Serial.print("relayDelay: ");
    Serial.println(relayDelay, DEC);
    Serial.print("orgTime: ");
    Serial.println(orgTime, DEC);
    Serial.print("expectedTime: ");
    Serial.println(orgTime + relayDelay, DEC);
    Serial.println(i, DEC);
    digitalWrite(relays[i], HIGH);
    while(currentTime < orgTime + relayDelay) {
      Serial.print("current:\t");
      Serial.println(currentTime, DEC);
      Serial.print("extra:\t\t");
      Serial.println(orgTime + relayDelay, DEC);
      Alarm.delay(1000);
      currentTime = now(); // store the current time in time variable t
    }
    digitalWrite(relays[i], LOW);
  }
  digitalWrite(pump, LOW);
  Serial.println("Alarm: - end");
}

// char * paddZero(uint8_t number, uint8_t padd = 2) {
//   char * stringBuffer = (char *)malloc(sizeof(char) * padd);
//   if(sprintf(stringBuffer, "%2d", number) > 0) {
//     return stringBuffer;
//   }
// }


void loop() {
  uint8_t plants = getPlants();
  uint8_t litersPumpedPerMinutePerPlant = getLitersPumpedPerMinutePerPlant(plants);
  uint8_t potSize = getPotSize(0);
  // water_flower(plants);
  /* Process incoming commands. */
  serialCommands.ReadSerial();
 
  u8g2.firstPage();
  do {
    int button_state = digitalRead(button);
    if (button_state == 1) {
      drawflower();
      drawTH(plants);
    } else {
      // testPump();
      drawTime();
      Serial.print("plants: ");
      Serial.println(plants, DEC);
      Serial.print("litersPumpedPerMinutePerPlant: ");
      Serial.println(litersPumpedPerMinutePerPlant, DEC);
      Serial.print("potSize: ");
      Serial.println(potSize, DEC);
      Serial.print("free ram: ");
      Serial.println(freeRam());
    }
  } while (u8g2.nextPage());
}

// v
//Set moisture value
struct MoiustureSensorData readMoistureSensors() {
  /************These is for capacity moisture sensor*********/
  struct MoiustureSensorData sensor;
  sensor.data[0]  = map(analogRead(A0), 590, 360, 0, 100); Alarm.delay(20);
  if(sensor.data[0] < 0){
    sensor.data[0] = 0;
  }
  sensor.data[1] = map(analogRead(A1), 600, 360, 0, 100); Alarm.delay(20);
  if(sensor.data[1] < 0) {
    sensor.data[1] = 0;
  }
  sensor.data[2] = map(analogRead(A2), 600, 360, 0, 100); Alarm.delay(20);
  if(sensor.data[2] < 0){
    sensor.data[2] = 0;
  }
  sensor.data[3] = map(analogRead(A3), 600, 360, 0, 100); Alarm.delay(20);
  if(sensor.data[3] < 0) {
    sensor.data[3] = 0;
  }
  return sensor;
}

void water_flower(uint8_t plants) {
  uint8_t i;
  MoiustureSensorData sensorData = readMoistureSensors();
  for( i = 0; i < plants; i++) {
    if (sensorData.data[i] < 30) {
      digitalWrite(relays[i], HIGH);
      relay_state_flag[i] = 1;
      delay(50);
      if (pump_state_flag == 0) {
        digitalWrite(pump, HIGH);
        pump_state_flag = 1;
        delay(50);
      }
    } else if (sensorData.data[i] > 55) {
      digitalWrite(relays[i], LOW);
      relay_state_flag[i] = 0;
      delay(50);
      if ((relay_state_flag[i] == 0) && (relay_state_flag[i + 1] == 0) && (relay_state_flag[i + 2] == 0) && (relay_state_flag[i + 3] == 0)) {
        digitalWrite(pump, LOW);
        pump_state_flag = 0;
        delay(50);
      }
    }
  }
}

void drawTH(uint8_t plants) {
  uint8_t i;
  int positions[4] = {0, 0, 64, 95};
  PrinterPos printerPos[4] = { 
    { 9, 60 },
    { 41, 60 },
    { 73, 60 },
    { 105, 60 }
  };
  char moisture_value_temp[4][5] = {
    { 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0 }
  };
  PrinterPosOther positions_screen[4] = {
    { 14, 7, 2 },
    { 46, 39, 32 },
    { 14, 7, 2 }
  };
  char inputNames[4] = {'A', 'B', 'C', 'D'};
  uint8_t percentage_pos[4] = { 23, 54, 23, 24 };
  MoiustureSensorData sensorData = readMoistureSensors();
  itoa(sensorData.data[0], moisture_value_temp[0], 10);
  itoa(sensorData.data[1], moisture_value_temp[1], 10);
  itoa(sensorData.data[2], moisture_value_temp[2], 10);
  itoa(sensorData.data[3], moisture_value_temp[3], 10);
  u8g2.setFont(u8g_font_7x14);
  for(i = 0; i < plants; i++) {
    u8g2.setCursor(printerPos[i].x, printerPos[i].y);
    u8g2.print(inputNames[i]);
    u8g2.print(i, DEC);
    if (sensorData.data[i] < 10)
    {
      //u8g.setPrintPos(A + 14, 45 );
      u8g2.drawStr(positions[i] + positions_screen[i].x, 45,  moisture_value_temp[i]);
      Alarm.delay(20);
      u8g2.drawStr(positions[i] + positions_screen[i].x, 45,  moisture_value_temp[i]);
      
    }
    else if (sensorData.data[0] < 100)
    {
      //u8g.setPrintPos(A + 7, 45);
      u8g2.drawStr(positions[i] + positions_screen[i].y, 45,  moisture_value_temp[i]);
      Alarm.delay(20);
      u8g2.drawStr(positions[i] + positions_screen[i].y, 45,  moisture_value_temp[i]);
    
    }
    else
    {
      //u8g.setPrintPos(A + 2, 45 );
      sensorData.data[i] = 100;
      itoa(sensorData.data[i],  moisture_value_temp[i], 10);
      u8g2.drawStr(positions[i] + positions_screen[i].other, 45,  moisture_value_temp[i]);
    }
    //u8g.print(moisture1_value);
    u8g2.setCursor(positions[i] + percentage_pos[i], 45 );
    u8g2.print("%");
  }
}


/*
 * Battery Ram setters
 */
uint8_t getLitersPerMinute() {
  return (rtc.readnvram(PUMP_LITERS_PER_MINUTE_ADDRESS));
};

uint8_t getLitersPumpedPerMinutePerPlant(uint8_t relays) {
  return (rtc.readnvram(PUMP_LITERS_PER_MINUTE_ADDRESS) / relays);
};

uint8_t getPlants() {
  return rtc.readnvram(PLANTS_ADDRESS);
};

uint8_t getPotSize(uint8_t pot) {
  return rtc.readnvram(POTSIZE_ADDRESS[pot]);
};

SensorCalibration getSensortCalibrationData(uint8_t sensor) {
  SensorCalibration sensorCalibration;
  uint8_t size = 2;
  sensorCalibration.min = rtc.readnvram(SENSOR_CALIBRATION_ADDRESS + (sensor * size));
  sensorCalibration.max = rtc.readnvram(SENSOR_CALIBRATION_ADDRESS + (sensor * size) + 1);
  return sensorCalibration;
};


SensorCalibration *getSensortCalibrationData2(uint8_t sensor) {
  uint8_t size = 2;
  struct SensorCalibration *sensorCalibration = (SensorCalibration *)malloc(sizeof(SensorCalibration));
  /* clear the newly allocated memory */
  memset(sensorCalibration, 0, sizeof(SensorCalibration));
  /* Do we have memory */
  if (sensorCalibration != NULL) {
    sensorCalibration->min = rtc.readnvram(SENSOR_CALIBRATION_ADDRESS + (sensor * size));
    sensorCalibration->max = rtc.readnvram(SENSOR_CALIBRATION_ADDRESS + (sensor * size) + 1);
  }
  return sensorCalibration;
};

/*
 * Battery Ram Setters
 */
void setLitersPumpedPerMinute(uint8_t liters) {
 rtc.writenvram(PUMP_LITERS_PER_MINUTE_ADDRESS, liters);
};

void setPlants(uint8_t number) {
  rtc.writenvram(PLANTS_ADDRESS, number);
};

void setPotSize(uint8_t pot, uint8_t size) {
  rtc.writenvram(POTSIZE_ADDRESS[pot], size);
};

void setAlarmRepeat(uint8_t alarmNumber, timeDayOfWeek_t DOW, int H, int M, int S) {
  rtc.writenvram((ALARM_MODES_ADDRESS * (ALARM_DATA_STORE + alarmNumber)) , DOW);
  rtc.writenvram((ALARM_MODES_ADDRESS * (ALARM_DATA_STORE + alarmNumber) + 1), H);
  rtc.writenvram((ALARM_MODES_ADDRESS * (ALARM_DATA_STORE + alarmNumber) + 2) , M);
  rtc.writenvram((ALARM_MODES_ADDRESS * (ALARM_DATA_STORE + alarmNumber) + 3) , S);
}


void cdmHelp(SerialCommands *sender) {
  sender->GetSerial()->println("some help");
}

void printDigits(int digits) {
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

// void cmdGetAlarm(SerialCommands *sender) {
//   getAlarms(sender);
// }

void cmdReboot(SerialCommands *sender) {
  return reboot();
}

void cmdUnrecognized(SerialCommands* sender, const char* cmd) {
	sender->GetSerial()->print("ERROR: Unrecognized command [");
	sender->GetSerial()->print(cmd);
	sender->GetSerial()->println("]");
}

/*
 * expects one single parameter
 * Commands
 */

void cmdHelp(SerialCommands* sender) {
  sender->GetSerial()->println(HELP_MESSAGE);
}

void cmdTestPump(SerialCommands* sender) {
  char* str = sender->Next();
  // if no arguments then just print time
	if (str == NULL) {
    sender->GetSerial()->print("ERR_ARGS");
		return;
	}

  sender->GetSerial()->print("Testing pump for ");
  sender->GetSerial()->print(str);
  sender->GetSerial()->println(" seconds");

  uint8_t seconds = atoi(str);
  testPump(seconds);
  return;
}

void cmdGetTimeAndDate(SerialCommands* sender) {
  sender->GetSerial()->print(hour());
  printDigits(minute());
  printDigits(second());
  sender->GetSerial()->print(" ");
  sender->GetSerial()->print(day());
  sender->GetSerial()->print(" ");
  sender->GetSerial()->print(month());
  sender->GetSerial()->print(" ");
  sender->GetSerial()->print(year()); 
  sender->GetSerial()->println(); 
}

void cmdSetTimeAndDate(SerialCommands *sender) {
	//Note: Every call to Next moves the pointer to next parameter

	char* str = sender->Next();
  // if no arguments then just print time
	if (str == NULL) {
		return cmdGetTimeAndDate(sender);
	}

  sender->GetSerial()->print("Adjusting time to: ");
  sender->GetSerial()->println(str);

  if (timeStatus() != timeNotSet) {
    // char * time = (char *)malloc(sizeof(char) * 10);
    // if (sprintf(time, "T:%02d:%02d:%02d", hour(), minute(), second())) {
    //   sender->GetSerial()->println(time);
    // }
    // rtc.adjust
    DateTime dateTime = DateTime(str);
    if (dateTime.isValid()) {
      rtc.adjust(dateTime);
      sender->GetSerial()->println("time adjusted to: ");
      return cmdGetTimeAndDate(sender);
    }
  }
}

void cmdResetAlarms(SerialCommands *sender) {
  resetAlarms();
  cmdGetAlarms(sender);
}

void printAlarm(SerialCommands *sender, Alarms *alarms, uint8_t index) {
  sender->GetSerial()->print("DOW:");
  sender->GetSerial()->print(alarms->alarm[index][0], DEC);
  sender->GetSerial()->print("T:");
  sender->GetSerial()->print(alarms->alarm[index][1], DEC);
  sender->GetSerial()->print(":");
  sender->GetSerial()->print(alarms->alarm[index][2], DEC);
  sender->GetSerial()->print(":");
  sender->GetSerial()->print(alarms->alarm[index][3], DEC);
  sender->GetSerial()->print("\n");
}

void cmdGetAlarms(SerialCommands *sender) {
  uint8_t i = 0;
  struct Alarms *alarms = getAlarms();

  if(alarms != NULL) {
    for (i = 0; i < MAX_ALARMS; i++) {
      printAlarm(sender, alarms, i);
    }
  }
  free(alarms);
}


uint8_t getActiveAlarm(uint8_t alarmNumber) {
  uint8_t activeAlarms = rtc.readnvram(ALARM_GET_ACTIVE_ADDRESS);
  return  bitRead(activeAlarms, alarmNumber);
}

void cmdGetActiveAlarms(SerialCommands *sender) {
  uint8_t i;
  sender->GetSerial()->println("ALARMS");
  for (i = 0; i < MAX_ALARMS; i++) {
    uint8_t isActive = getActiveAlarm(i);
    if (isActive > 0) {
      sender->GetSerial()->print("is active: ");
      sender->GetSerial()->println(i, DEC);
    }
  }
}


void cmdGetActiveAlarm(SerialCommands *sender) {
  char* str = sender->Next();
	if (str == NULL) {
		sender->GetSerial()->println("ERROR ALARM");
		return;
	}
  uint8_t alarm = atoi(str); //converts string to int
  sender->GetSerial()->print("ALARM: ");
  sender->GetSerial()->println(alarm, DEC);
  uint8_t isActive = getActiveAlarm(alarm);
  if (isActive > 0) {
    sender->GetSerial()->print("is active: ");
    sender->GetSerial()->print(alarm, DEC);
  }
}

void setActiveAlarms(uint8_t alarmNumber) {
  uint8_t activeAlarms = rtc.readnvram(ALARM_GET_ACTIVE_ADDRESS);
  bitSet(activeAlarms, alarmNumber);
  return rtc.writenvram(ALARM_GET_ACTIVE_ADDRESS, activeAlarms);
}

void cmdToggleActiveAlarms(SerialCommands *sender) {
  char* str = sender->Next();
	if (str == NULL) {
		sender->GetSerial()->println("ERROR NO_PORT");
		return;
	}
  uint8_t alarm = atoi(str); //converts string to int
  toggleActiveAlarms(alarm);
}

void toggleActiveAlarms(uint8_t alarmNumber) {
  uint8_t activeAlarms = rtc.readnvram(ALARM_GET_ACTIVE_ADDRESS);
  uint8_t isActive = bitRead(activeAlarms, alarmNumber);
  if (isActive) {
    bitClear(activeAlarms, alarmNumber);
  } else { 
    bitSet(activeAlarms, alarmNumber);
  }
  return rtc.writenvram(ALARM_GET_ACTIVE_ADDRESS, activeAlarms);
}


void cmdSetActiveAlarms(SerialCommands *sender) {
  char* str = sender->Next();
	if (str == NULL) {
		sender->GetSerial()->println("ERROR NO ARGS");
		return;
	}
  uint8_t alarm = atoi(str); // converts string to int
  setActiveAlarms(alarm);
}


Alarms * getAlarms() {
  /* out counters */
  uint8_t i = 0;
  uint8_t j = 0;

  /* alloc memory for the alarm struct */
  struct Alarms *alarms = (Alarms *)malloc(sizeof(Alarms));
  /* clear the newly allocated memory */
  memset(alarms, 0, sizeof(Alarms));
  /* Do we have memory */
  if (alarms != NULL) {
    for (i = ALARM_MODES_ADDRESS; i < ALARM_MODES_ADDRESS + (ALARM_DATA_STORE * MAX_ALARMS); i++) {
      for (j = 0; j < ALARM_DATA_STORE ; j++) {
        uint8_t startAddress =  i + j;
        alarms->alarm[i][j] = rtc.readnvram(startAddress);
      }
    }
  }
  /* return the alarms pointer */
  return alarms;
}

void resetAlarms() {
  uint8_t i = 0;
  uint8_t j = 0;

  for (i = ALARM_MODES_ADDRESS; i < ALARM_MODES_ADDRESS + (ALARM_DATA_STORE * MAX_ALARMS); i++) {
    for (j = 0; j < ALARM_DATA_STORE ; j++) {
      uint8_t startAddress =  i + j;
      rtc.writenvram(startAddress , 0);
    }
  }
}

void setAlarms(Alarms *alarms) {
  uint8_t i = 0;
  uint8_t j = 0;

  for (i = ALARM_MODES_ADDRESS; i < ALARM_MODES_ADDRESS + (ALARM_DATA_STORE * MAX_ALARMS); i++) {
    for (j = 0; j < ALARM_DATA_STORE ; j++) {
      uint8_t startAddress =  i + j;
      rtc.writenvram(startAddress , 0);
    }
  }
}

void reboot() {
  wdt_disable();
  wdt_enable(WDTO_15MS);
  while (1) {}
}


void cmdGetAlarms2(SerialCommands *sender) {
  Stream * serial = sender->GetSerial();
  uint8_t i;
  Alarms * alarms = getAlarms();

  for(i = 0; i < MAX_ALARMS; i++) {
    // serial->print(alarms->alarm);
    char * str = (char *)malloc(sizeof(char) * 10);
    if(sprintf(str, "%d:%0d:%0d:%0d:%0d", alarms->alarm[i][0], alarms->alarm[i][1], alarms->alarm[i][2], alarms->alarm[i][3], alarms->alarm[i][4])) {
      serial->println(str);
    }
  }
}