/* 
 * constants 
*/

#define MAX_PLANTS 4 /* Maximun amount of allowed plants */
#define MAX_ALARMS 4 /* Maximun amount of allowed alarms */
#define PLANTS_ADDRESS 7 /* Active plants (battery backed ram address) */
#define PUMP_LITERS_PER_MINUTE_ADDRESS 8 /* Pump liters per minute (battery backed ram address) */
#define SENSOR_CALIBRATION_ADDRESS 20 /* Moisture sensor calibration (battery backed ram address) */
#define ALARM_GET_ACTIVE_ADDRESS 30
#define ALARM_MODES_ADDRESS 31

const uint8_t POTSIZE_ADDRESS[8] = { 9, 10, 11, 12, 13, 14, 15, 16 };
const uint8_t ALARM_DATA_STORE = 4;
const char *HELP_MESSAGE =
    "Available commands:\n"
    "------------------------------\n"
    "tm         <get/set time>\n"
    "al         <get/set alarms>:\n"
    "rb         <hard reboot>\n"
    "ra         <reset all alarms>:\n"
    "tp         <test pump for x seconds>\n";


