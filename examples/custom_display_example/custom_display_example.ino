
#include "thijs_rplidar.h"

#define ILI9341displayAdafruit
//#define ILI9341displayTFT_eSPI
//#define DACdisplay  // untested
//#define compositedisplay   // TBD


struct lidarMotorHandler {  // not really needed (and (currently) very ESP32-bound) but somewhat futureproof
  const uint8_t pin;
  const uint32_t freq; //Hz
  //const uint8_t res; //bits (commented because i want to keep this thing simple, and changing variable sizes (templates?) is not
  const uint8_t channel; // an ESP32 ledc specific thing
  const bool activeHigh; // depends on your specific hardware setup (CTRL_MOTO should be driven to the same voltage as 5V_MOTO (which can range from 5 to 9V), i think)
  lidarMotorHandler(const uint8_t pin, const bool activeHigh=true, const uint32_t freq=500, /*const uint8_t res=8,*/ const uint8_t channel=0) : 
                    pin(pin), freq(freq), /*res(res),*/ channel(channel), activeHigh(activeHigh) {}
  void init() {
    ledcSetup(channel, freq, 8);
    ledcAttachPin(pin, channel);
    setPWM(0);
  }
  inline void setPWM(uint8_t newPWMval) {ledcWrite(channel, activeHigh ? newPWMval : (255-newPWMval));}
};


//struct universalLidarMeasurement {
//  uint16_t dist;
//  uint16_t angle_q6;
//  int8_t quality;
//}__attribute__((packed));
const uint16_t dataSetResPerDegree = 1; // measurements per degree
DRAM_ATTR struct _lidarDataSet { // a big-ass static array full of sensor data (stored in DataRAM for speed)
  uint16_t distArray[dataSetResPerDegree*360]; // a quick'n'dirty static array, enforcing a constant resolution (despite the lidar not really working like that)
  int8_t qualityArray[dataSetResPerDegree*360];
  volatile bool dataArrayWriting = false;
} lidarDataSet;


lidarMotorHandler motorHandler(27);
RPlidar lidar(Serial2);

bool keepSpinning = true;
uint16_t debugPrintCounter = 0;
const uint16_t debugPrintThreshold = 48; // print data every (this many) datapoints (if you are getting CRC errors, there may be buffer overflow, try setting this to like 48+ (or uncommenting printing entirely))

void dataHandler(RPlidar* lidarPtr, uint16_t dist, uint16_t angle_q6, uint8_t newRotFlag, int8_t quality) {
  float distFloat = dist; // unit is mm directly
  float angleDegreesFloat = angle_q6 * 0.015625; // angle comes in 'q6' format, so divide by (1<<6)=64 (or multiply by 1/64) (or bitshift to the right by 6) to get angle in degrees
  // alternatively, you could use bitshifting to divide the angleDegreesFloat slightly faster. Something like:
//  float angleDegreesFloat = angle_q6;   angleDegreesFloat = (float&)(((uint32_t&)angleDegreesFloat)-=(((uint32_t)6)<<23)); // subtract 6 from the float's exponent, thereby dividing it by 2^6=64

  //uint16_t arrayIndex = (uint16_t)(angleDegreesFloat * (float)dataSetResPerDegree);
  uint16_t arrayIndex = ((uint32_t)angle_q6 * dataSetResPerDegree)>>6; // keepin' it integer
  arrayIndex = constrain(arrayIndex, 0, (dataSetResPerDegree*360)-1); // for safety
  lidarDataSet.dataArrayWriting = true; // low effort semaphore
  lidarDataSet.distArray[arrayIndex] = dist; // store the data in a big (dumb) static array
  lidarDataSet.qualityArray[arrayIndex] = quality; // store the data in a big (dumb) static array
  lidarDataSet.dataArrayWriting = false; // low effort semaphore
}

void setup() {
  Serial.begin(115200);
  Serial.println();

  motorHandler.init();

  lidar.init(16, 17);
  lidar.postParseCallback = dataHandler; // set dat handler function

//  lidar.printLidarInfo();
//  lidar.printLidarHealth();
//  lidar.printLidarSamplerate();
//  lidar.printLidarConfig();
  Serial.println();

  if(!lidar.connectionCheck()) { Serial.println("connectionCheck() failed"); while(1) {} }

  delay(10);
  motorHandler.setPWM(200);
  //bool startSuccess = lidar.startStandardScan();  // 2kHz
  //bool startSuccess = lidar.startExpressScan(EXPRESS_SCAN_WORKING_MODE_LEGACY); // 4kHz
  bool startSuccess = lidar.startExpressScan(EXPRESS_SCAN_WORKING_MODE_BOOST); // 8kHz
//  Serial.print("startSuccess: "); Serial.println(startSuccess);
  
  rplidar_display_init(); // see last few lines of display_plot.ino
}

void loop() {
//  if(Serial.available()) { lidar.lidarSerial.write(Serial.read()); }
//  if(lidar.lidarSerial.available()) { Serial.write(lidar.lidarSerial.read()); }
//  if(millis() >= 5000) { motorHandler.setPWM(0); while(1) {} }
  
  if(keepSpinning) {
    uint32_t extraSpeedTimer = micros();
    int8_t datapointsProcessed = lidar.handleData(true, false); // read lidar data and send it to the callback function. Parameters are: (includeInvalidMeasurements, waitForChecksum)
    // includeInvalidMeasurements means sending data where the measurement failed (out of range or too close or bad surface, etc. it's when distance == 0)
    // waitForChecksum (only applies to express scans) means whether you wait for the whole packet to come, or to process data as it comes in (checksum is still checked when the whole packet is there, but the bad data may have already been sent to the callback)
//    extraSpeedTimer = micros() - extraSpeedTimer;
//    if(extraSpeedTimer > 40) { Serial.println(extraSpeedTimer); }

    if(datapointsProcessed < 0) { keepSpinning = false; lidar.stopScan(); } // handleData() returns -1 if it encounters an error
    //if(lidar.packetCount >= 200) { keepSpinning = false; lidar.stopScan(); }  // stop scanning after a while
  } else {
    motorHandler.setPWM(0);
  }
}
