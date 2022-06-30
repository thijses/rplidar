/*
todo:
 - constant name translation functions (going backwards from 0x84 to "RESP_DESCR_SENDMODE_DATATYPE_EXPRESS_EXTEND")
 - multicore example (use the ESP32's second core to process the data / show it on an SPI-display/oscilloscope)
 - motor PID?
 - make debug printing optional????? (nah)

NOTES:
if you're using the included serial adapter, the DTR pin is connected to the motor PWM control.
on boot (and on reset) it prints something like:
"
RP LIDAR System.
Firmware Ver 1.29 - rtm, HW Ver 7
Model: 18
"
to investigate: (taken from some random library's github
typedef struct _rplidar_ans_header_t {
    _u8  syncByte1; // must be RPLIDAR_ANS_SYNC_BYTE1
    _u8  syncByte2; // must be RPLIDAR_ANS_SYNC_BYTE2
    _u32 size:30;
    _u32 subType:2;       //(thijs) i wonder if the compiler knows what this means and will automatically apply bitmasks to it (efficiently?)
    _u8  type;
} __attribute__((packed)) rplidar_ans_header_t;

some data retrieved from the lidar (model A1M8, bought on Aliexpress in Feb 2022):
from CMD_GET_INFO:
model: 24
firmware: 1.29
hardware: 7
serialNumber (HEX): D8 E0 ED F9 C7 E2 9B D1 A7 E3 9E F2 48 33 43 1B 
serialNumber (32bit): 4193116376 3516654279 4070499239 457388872 
serialNumber (64bit): 15103915123836575960 1964470250864829351 
from CMD_GET_HEALTH:
status: 0
errorCode (HEX): 0
from CMD_GET_SAMPLERATE:
standard: 508
express: 254
from CMD_GET_LIDAR_CONF: (and sub-commands)
GET_CONF_SCAN_MODE_COUNT: 5
scan mode: 0
  name: Standard
  sampletime: 130048 == 508us
  maxDist: 3072      == 12m
  ansType (HEX): 81  == CONF_SCAN_MODE_ANS_TYPE_STANDARD
scan mode: 1
  name: Express
  sampletime: 65024  == 254us
  maxDist: 3072      == 12m
  ansType (HEX): 82  == CONF_SCAN_MODE_ANS_TYPE_EXPRESS (== legacy???)
scan mode: 2
  name: Boost
  sampletime: 32512  == 127us
  maxDist: 3072      == 12m
  ansType (HEX): 84  == CONF_SCAN_MODE_ANS_TYPE_EXPRESS_EXTEND ???
scan mode: 3
  name: Sensitivity
  sampletime: 32512  == 127us
  maxDist: 3072      == 12m
  ansType (HEX): 84  == CONF_SCAN_MODE_ANS_TYPE_EXPRESS_EXTEND ???
scan mode: 4
  name: Stability
  sampletime: 51456  == 201us
  maxDist: 3072      == 12m
  ansType (HEX): 84  == CONF_SCAN_MODE_ANS_TYPE_EXPRESS_EXTEND ???
*/

#ifndef THIJS_RPLIDAR
#define THIJS_RPLIDAR

#include "Arduino.h"

//#define lidarDebugSerial Serial

#if !defined(ESP32)
  #error("thijs_rplidar.h was designed for the ESP32. It could probably also run on other MCUs, but you need to check for yourself")
#endif

//struct lidarMotorHandler {  // not really needed (and (currently) very ESP32-bound) but somewhat futureproof
//  const uint8_t pin;
//  const uint32_t freq; //Hz
//  //const uint8_t res; //bits (commented because i want to keep this thing simple, and changing variable sizes (templates?) is not
//  const uint8_t channel; // an ESP32 ledc specific thing
//  const bool activeHigh; // depends on your specific hardware setup (CTRL_MOTO should be driven to the same voltage as 5V_MOTO (which can range from 5 to 9V), i think)
//
//  uint16_t targetRPM = 0;
//  uint16_t Kp = 150;
//  uint16_t lastRPM = 0;
//  //uint32_t RPMcontrolTimer;
//  //const uint32_t RPMcontrolInterval = 5000;
//  const uint32_t angleToRPMmult = (1000000*60)/(360<<6); // 60 million microseconds per minute, 360deg per rotation, lidar angle data comes in 'q6' format (divide by 64 to get degrees)
//  
//  lidarMotorHandler(const uint8_t pin, const bool activeHigh=true, const uint32_t freq=500, /*const uint8_t res=8,*/ const uint8_t channel=0) : 
//                    pin(pin), freq(freq), /*res(res),*/ channel(channel), activeHigh(activeHigh) {}
//  void init() {
//    ledcSetup(channel, freq, 8);
//    ledcAttachPin(pin, channel);
//    setPWM(0);
//  }
//  inline void setPWM(uint8_t newPWMval) {ledcWrite(channel, activeHigh ? newPWMval : (255-newPWMval));}
//  uint16_t RPM_PID(uint32_t angleDiff, uint32_t microsDiff) {
//    uint16_t currentRPM = (angleDiff*angleToRPMmult)/microsDiff; //calculate RPM in an integer friendly way (because it's fast)
//    //uint32_t newPWMval = Kp * targetRPM;
//    //newPWMval = constrain(newPWMval>>8, 0, 255); // divide by 256 and constrain
//    //setPWM((uint8_t)newPWMval);
//    lastRPM = currentRPM;
//    return(currentRPM);
//  }
//};

const uint8_t CMD_START_FLAG = 0xA5;
const uint8_t RESP_DESCR_START_FLAGS[2] = {0xA5, 0x5A};
//const uint8_t EXPRESS_DATA_LEGACY_SYNC_FLAGS[2] = {0xA0, 0x50};
//const uint8_t EXPRESS_DATA_LEGACY_SYNC_MASK = 0xF0; //only the first half of the 

//command (request) codes:
const uint8_t CMD_STOP = 0x25; //stop the lidar (no payload, no repsonse (wait at least 1ms))
const uint8_t CMD_RESET = 0x40; //reset the lidar core (no payload, no repsonse (wait at least 2ms)) (used for exiting "Protection Stop state")
const uint8_t CMD_SCAN = 0x20; //start scanning (once rotation speed is good and stable) (no payload, multiple response) (this is more of a legacy scan mode, the real good stuff is in EXPRESS_SCAN, i think)
const uint8_t CMD_EXPRESS_SCAN = 0x82; //start scanning at highest speed (idk if that's sample rate or rotation speed) (once rotation speed is good???) (has payload, multiple response) (requires firmware >= 1.17)
const uint8_t CMD_FORCE_SCAN = 0x21; //start scanning (regardless of rotation speed) (no payload, multiple response)
const uint8_t CMD_GET_INFO = 0x50; //get device info (no payload, single response)
const uint8_t CMD_GET_HEALTH = 0x52; //get device health info (no payload, single response)
const uint8_t CMD_GET_SAMPLERATE = 0x59; //get samplerate (no payload, single response) (requires firmware >= 1.17)
const uint8_t CMD_GET_LIDAR_CONF = 0x84; //request LIDAR configuration (lots of info about scan modes) (has payload, single response) (requires firmware >= 1.24)
//const uint8_t CMD_MOTOR_SPEED_CTRL = 0xA8; //set motor speed (has 16bit payload, no response?) (requires S1 model)

// response descriptor codes:
const uint8_t RESP_DESCR_SENDMODE_SINGLE_RESPONSE = 0b00000000; // single request - single response
const uint8_t RESP_DESCR_SENDMODE_MULTI_RESPONSE = 0b01000000; // single request - multiple response
const uint8_t RESP_DESCR_SENDMODE_DATATYPE_EXPRESS_LEGACY = 0x82;
//const uint8_t RESP_DESCR_SENDMODE_DATATYPE_EXPRESS_HW = 0x83; // taken from SDK code, not from documentation
const uint8_t RESP_DESCR_SENDMODE_DATATYPE_EXPRESS_EXTEND = 0x84;
const uint8_t RESP_DESCR_SENDMODE_DATATYPE_EXPRESS_DENSE = 0x85;

//CMD_GET_HEALTH specific response codes:
const uint8_t GET_HEALTH_STATUS_GOOD = 0; // (status byte of GET_HEALTH response)
const uint8_t GET_HEALTH_STATUS_WARNING = 1;
const uint8_t GET_HEALTH_STATUS_ERROR = 2; // 'protection stop state'

//GET_LIDAR_CONF specific codes:
const uint8_t GET_CONF_SCAN_MODE_COUNT = 0x70; // (no payload, 16bit response)
const uint8_t GET_CONF_SCAN_MODE_SAMPLETIME = 0x71; // (16bit payload, 32bit response) (response/256.0 = response>>8 = microseconds)
const uint8_t GET_CONF_SCAN_MODE_MAX_DIST = 0x74; // (16bit payload, 32bit response) (response/256.0 = response>>8 = microseconds)
const uint8_t GET_CONF_SCAN_MODE_ANS_TYPE = 0x75; // (16bit payload, 8bit response)
const uint8_t GET_CONF_SCAN_MODE_NAME = 0x7F; // (16bit payload, String response)
const uint8_t GET_CONF_SCAN_MODE_TYPICAL = 0x7C; // (no payload, 16bit response)

const uint8_t CONF_SCAN_MODE_ANS_TYPE_STANDARD = 0x81; // standard mode (presumably what i called struct lidarStandardData {} )
const uint8_t CONF_SCAN_MODE_ANS_TYPE_EXPRESS = 0x82; // express mode (legacy i think)
const uint8_t CONF_SCAN_MODE_ANS_TYPE_ULTRA_CAP = 0x83; // 'ultra capsulated' mode, used for 'boost, stability and sensitivity mode'... i'm pretty sure my A1M8 can't do those anyway
const uint8_t CONF_SCAN_MODE_ANS_TYPE_EXPRESS_EXTEND = 0x84; // express mode, extended version? (sometimes also called 'ultra capsulated'?)
const uint8_t CONF_SCAN_MODE_ANS_TYPE_EXPRESS_DENSE = 0x85; // express mode, dense version

//response description 'dataType'
const uint8_t RESP_TYPE_SCAN = 0x81; // also for FORCE_SCAN
const uint8_t RESP_TYPE_SCAN_EXPRESS_LEGACY = 0x82;
//const uint8_t RESP_TYPE_SCAN_HQ = 0x83; // taken from SDK code, but i can't find any real mention of 'HQ' mode (and the typedefs in the SDK code are unfamiliar
const uint8_t RESP_TYPE_SCAN_EXPRESS_EXTEND = 0x84; // a.k.a. 'ultra capsulated'
const uint8_t RESP_TYPE_SCAN_EXPRESS_DENSE = 0x85;
const uint8_t RESP_TYPE_GET_INFO = 0x04;
const uint8_t RESP_TYPE_GET_HEALTH = 0x06;
const uint8_t RESP_TYPE_GET_SAMPLERATE = 0x15;
const uint8_t RESP_TYPE_GET_LIDAR_CONF = 0x20;
const uint8_t RESP_TYPE_INVALID = 0x00; // not from the official documentation, this is soemthing i came up with the let me return objects even when the read fails (and identify 

//EXPRESS WORK MODES (to be put into EXPRESS_SCAN request payload (first byte))
const uint8_t EXPRESS_SCAN_WORKING_MODE_LEGACY = 0; //working mode for legacy mode, exists on all lidar models
const uint8_t EXPRESS_SCAN_WORKING_MODE_BOOST = 2; //see what comes up when you run GET_LIDAR_CONF
const uint8_t EXPRESS_SCAN_WORKING_MODE_SENSITIVITY = 3; //these values appear to be indices of scan modes
const uint8_t EXPRESS_SCAN_WORKING_MODE_STABILITY = 4; //the documentation regarding these is somewhat lacking

//bit masks for isolating various tangled data in packets (not used in all cases, so do check the data structs manually if you change anything here)
const uint8_t RESP_DESCR_SENDMODE_BITS = 0b11000000; //the responseLength is 30 bits long, the last 2 bits indicate sendMode
const uint8_t STANDARD_DATA_ROT_START_BITS = 0b00000011; //the lowest 2 bits of the first byte is the rotation_start flag(s)
const uint8_t STANDARD_DATA_CHECK_BIT = 0b00000001; //the LSBit of the second byte is a check but, should always be 1
const uint8_t EXPRESS_DATA_LEGACY_SYNC_BITS = 0xF0; //the MSB half of the first 2 legacy express measurement packet bytes are the sync thingies
const uint8_t EXPRESS_DATA_LEGACY_ROT_START_BIT = 0b10000000; //the MSBit of the 4th byte of the legacy express measurement packet is the rotation_start flag
const uint8_t EXPRESS_DATA_LEGACY_DIST_BITS = 0b11111100; //fuck it, just see page 22 of https://bucket-download.slamtec.com/f010c72be308cdc618e91746d643278185ed02b2/LR001_SLAMTEC_rplidar_protocol_v2.2_en.pdf
const uint16_t EXPRESS_DATA_EXTEND_MAJOR_BITS = 0x0FFF; //first 2 bits of the 4-byte 'ultra_cabin' (page 26)
//const uint16_t EXPRESS_DATA_EXTEND_PREDICT_ONE_BITS = 0x3FF0; //middle 10 bits of the of the 4-byte 'ultra_cabin' (page 26)
//const uint16_t EXPRESS_DATA_EXTEND_PREDICT_TWO_BITS = 0xFFC0; //last 10 bits of the of the 4-byte 'ultra_cabin' (page 26)


// for encoding requests to the lidar:
struct lidarCMD { // less of a struct and more of a wrapper around a single byte array, but whatever
  uint8_t* _data; // a (variable size) byte buffer for the data to be sent (initialized with malloc() on initialization, freed in destructor)
  lidarCMD(uint8_t CMD, uint8_t payloadSize) : _data((uint8_t*) calloc(payloadSize+4, 1)) {_data[0]=CMD_START_FLAG; _data[1]=CMD; _data[2]=payloadSize;}
  lidarCMD(uint8_t* _dataInput) { _data = _dataInput; } // not recommended (but at least you dont NEED to make a second buffer if you've (foolishly) chose to make your own byte buffer)
  ~lidarCMD() { delete _data; } //my constructor is too funky for the compiler to think of this. (make sure to check for memory leaks BTW...)
  inline uint8_t*& operator&() {return(_data);} //any attempts to retrieve the address of this object should be met with a (reference to) _data (it's the same address, just an implicit typecast)
  inline uint8_t& operator[](size_t index) {return(_data[index + 3]);} // the payload starts after the StartFlag, CMDbyte and payloadSize bytes.
  inline uint8_t& CMD() {return(_data[1]);}
  inline uint8_t& payloadSize() {return(_data[2]);}
  //inline uint8_t& checksum() {return(_data[_data[2]+3]);}
  inline uint32_t& payloadAsUint32_t(uint8_t index=0) {return((uint32_t&)_data[3+index]);} // used for GET_LIDAR_CONF 'type' value (fill with values like GET_CONF_SCAN_MODE_COUNT
  inline uint16_t& payloadAsUint16_t(uint8_t index=0) {return((uint16_t&)_data[3+index]);} // alternatively, you could do " uint16_t& temp = (uint16_t&)cmdObj[index]; temp = blabla; "
  uint8_t calcChecksum() {
    uint8_t checksumVal = 0;
    for(uint8_t i=0; i<(_data[2]+3); i++) { checksumVal ^= _data[i]; } // '^' is the XOR operator
    return(checksumVal);
  }
  inline uint16_t sendSize() { return(_data[2] ? (_data[2]+4) : 2); } //the size of _data is either payloadSize+4 {start,CMD,size,payload[],checksum} or 3 {start,CMD,size}, but if there is no payload, only SEND 2.
  uint8_t* send() {
    if(_data[2]) { _data[_data[2]+3] = calcChecksum(); } //if there is a payload, calculate and store checksum as the last byte of _data
    return(_data);
  }
};

template<uint8_t arraySize> struct byteArrayStruct {
  uint8_t _data[arraySize];
//  byteArrayStruct() {} // default initializer
//  byteArrayStruct(uint8_t* _dataInput) {for(uint8_t i=0;i<sizeof(_data);i++){_data[i]=_dataInput[i];}}
//  byteArrayStruct(uint8_t* _dataInput) { _data = _dataInput; }
  inline uint8_t* operator&() {return(_data);} //any attempts to retrieve the address of this object should be met with a byte pointer to _data (it's the same address, just an implicit typecast)
  inline uint8_t& operator[](size_t index) {return(_data[index]);} //i'm hoping the compiler will just know what i mean
}; // that saves a few lines through inheritance

// for decoding response descriptors from the lidar:
struct lidarRespDescr : public byteArrayStruct<5> { // less of a struct and more of a wrapper around a single byte array, but whatever
  uint32_t responseLength() {
    uint32_t responseLength;  uint8_t* bytePointer = (uint8_t*) &responseLength;
    memcpy(bytePointer, _data, sizeof(responseLength));
    bytePointer[sizeof(responseLength)-1] &= ~RESP_DESCR_SENDMODE_BITS;
    return(responseLength);
//    uint32_t& responseLength = (uint16_t&)_data[0];
//    return(responseLength & 0x3FFFFFFF);
//    //return(responseLength & (0xFFFFFFFF ^ (RESP_DESCR_SENDMODE_BITS<<24));
  }
  inline uint8_t sendMode() {return(_data[3] & RESP_DESCR_SENDMODE_BITS);}
  inline uint8_t& dataType() {return(_data[4]);} //???
  //inline const uint8_t& recvSize() {return(sizeof(_data));} // just use sizeof(lidarRespDescr), as the array is initialized statically (so the compiler knows the size)
};

// for decoding all of the different measurement response packet formats from the lidar:
struct lidarStandardData : public byteArrayStruct<5> {
  inline uint8_t quality() {return((_data[0]&(~STANDARD_DATA_ROT_START_BITS))>>2);} // (6bit number) quality of measurement ("reflected laser pulse strength")
  inline uint8_t rotStartFlag() {return(_data[0]&STANDARD_DATA_ROT_START_BITS);} // 1 when new rotation starts, 2 otherwise (if it's 0 or 3 then something is very wrong!)
  inline bool checkBit() {return(_data[1]&STANDARD_DATA_CHECK_BIT);} //should always be 1
  inline uint16_t angle() {uint16_t angle=(_data[2]<<7); return(angle|(_data[1]>>1));} //(15bit number) divide by 64 (bitshift by 6 (or multiply by 0.015625) if you want to be quick) to get degrees
  inline uint16_t& dist() {return((uint16_t&)_data[3]);} // divide by 4 (bitshift by 2 (or multiply by 0.25) if you want to be quick) to get millimeters
};

template<uint8_t bufferSize> struct _lidarExpressDataBase { //this is the parts all Express data packets have in common
  uint8_t _data[bufferSize]; // unfortunately, this template class can't seem to work with byteArrayStruct, so here are those functions again
  inline uint8_t* operator&() {return(_data);} //any attempts to retrieve the address of this object should be met with a (byte pointer) to _data (it's the same address, just an implicit typecast)
  inline uint8_t& operator[](size_t index) {return(_data[index]);} //i'm hoping the compiler will just know what i mean
  inline uint8_t sync() {return((_data[0]&EXPRESS_DATA_LEGACY_SYNC_BITS)|((_data[1]&EXPRESS_DATA_LEGACY_SYNC_BITS)>>4));} // sync 'byte' (two half bytes) should equal 0xA5 when put together like this
  inline uint8_t checksum() {return((_data[0]&(~EXPRESS_DATA_LEGACY_SYNC_BITS))|((_data[1]&(~EXPRESS_DATA_LEGACY_SYNC_BITS))<<4));} // checksum byte
  inline uint16_t startAngle() {uint16_t startAngle=(_data[3]&(~EXPRESS_DATA_LEGACY_ROT_START_BIT)); return((startAngle<<8)|_data[2]);} //(15bit number) divide by 64 (bitshift by 6 (or multiply by 0.015625) if you want to be quick) to get degrees
  inline bool rotStartFlag() {return(_data[3]&EXPRESS_DATA_LEGACY_ROT_START_BIT);}
  uint8_t calcChecksum() { //seems to work?
    uint8_t checksumVal = 0; //= 0xA0 ^ 0x50 = 0xF0   if the first 2 bytes are included in the checksum
    for(uint8_t i=2; i<bufferSize; i++) { checksumVal ^= _data[i]; } // '^' is the XOR operator
    return(checksumVal);
  }
  //the dist and angle extraction functions depend on the different types of packets. This struct is inherited into those
};

struct lidarExpressDataLegacy : public _lidarExpressDataBase<84> {
  inline uint16_t dist(uint8_t index) { // a 14bit number
    uint8_t offIndex=4+((index/2)*5)+((index%2)*2); //index runs from 0 to 31, offIndex runs from 4 to 81
    uint16_t dist=(_data[offIndex]&EXPRESS_DATA_LEGACY_DIST_BITS);
    return((_data[offIndex+1]<<6)|(dist>>2)); // return in direct millimeters (becuase that is the actual resolution of this packet type)
    //return((_data[offIndex+1]<<8)|dist); // return in 'q2' millimeters (mm*4), to conform with the standard data packets?
  } // express legacy distance measurements have only 14bit resolution, so they are in mm directly (a max value of 2^14= 16384mm = 16.384m >= the A1M8's 12M range)
  inline int8_t deltaAngle(uint8_t index) { //note: deltaAngle is a SIGNED 6bit number in 'q3' format (divide by (1<<3) or bitshift by 3 to get degrees) (from the formula on page 31 it seems like this number maybe indicate deviation from the expected angle progression)
    uint8_t offIndex=4+((index/2)*5); // just see page 22 of https://bucket-download.slamtec.com/f010c72be308cdc618e91746d643278185ed02b2/LR001_SLAMTEC_rplidar_protocol_v2.2_en.pdf
//    int8_t deltaAngle = (_data[offIndex+((index%2)*2)] & 0b00000001)<<4; //the last bit of the first/third byte is the 5th databit in deltaAngle
//    deltaAngle |= ((_data[offIndex+((index%2)*2)] & 0b00000010) ? 0b10000000 : 0); //the second bit of the first/third byte is the sign bit of deltaAngle????
    int8_t deltaAngle = (_data[offIndex+((index%2)*2)] & (~EXPRESS_DATA_LEGACY_DIST_BITS))<<6; deltaAngle = deltaAngle>>2; //bitshift all the way left, and then back again to right right to use signed bitshifting (fills in based on sign bit)
    if(index%2) { //if it's an odd numbered index
      return(deltaAngle|(_data[offIndex+4]>>4)); //i'm hoping it will fill the byte with 0th when shifting
    } else {
      return(deltaAngle|(_data[offIndex+4]&0x0F));
    }
  } // "in the format of q3 in the unit degrees" i dont really know
};

struct lidarExpressDataExtended : public _lidarExpressDataBase<132> {
  inline uint32_t& wholeCabin(uint8_t cabinIndex=0) {return((uint32_t&)_data[4+cabinIndex*4]);}
  inline uint16_t major(uint8_t cabinIndex) { //get the 'major' value of each 'ultra_cabin'. see page 26 of https://bucket-download.slamtec.com/f010c72be308cdc618e91746d643278185ed02b2/LR001_SLAMTEC_rplidar_protocol_v2.2_en.pdf
    uint16_t& firstTwoBytes = (uint16_t&)_data[4+cabinIndex*4]; //cabinIndex runs from 0 to 31
    return(firstTwoBytes & EXPRESS_DATA_EXTEND_MAJOR_BITS);
  }
  inline int32_t predictOne(uint8_t cabinIndex) { //get the 'predict1' value of each 'ultra_cabin'
    int32_t& combined_x3 = (int32_t&)_data[4+cabinIndex*4]; //cabinIndex runs from 0 to 31
    return((combined_x3<<10)>>22); // shift left and right to make use of signed shifting (pads left bits based on sign bit)
  }
  inline int32_t predictTwo(uint8_t cabinIndex) { //get the 'predict1' value of each 'ultra_cabin'
    int32_t& combined_x3 = (int32_t&)_data[4+cabinIndex*4]; //cabinIndex runs from 0 to 31
    return((combined_x3)>>22); // shift right to make use of signed shifting (pads left bits based on sign bit)
  }
};

struct lidarExpressDataDense : public _lidarExpressDataBase<84> { //the simplest format, but doesn't include Intermediary angles (you just have to calculate thise as (index/40)*(start_angle - last_start_angle))
  inline uint16_t& dist(uint8_t index) {return((uint16_t&)_data[4+(index*2)]);} // "unit is millimeters" so the max distance measured is 65meters??? page 12 of the protocol manual does state that the S1 lidars can do 40m in this mode...
};


// for decoding the information request response packets from the lidars
struct lidarGetInfoResponse : public byteArrayStruct<20> {
  inline uint8_t& model() {return(_data[0]);} //the first byte contains the model data
  inline uint8_t& firmwareMinor() {return(_data[1]);} //part after the decimal point
  inline uint8_t& firmwareMajor() {return(_data[2]);} //part before the decimal point
  inline uint8_t& hardware() {return(_data[3]);} //the fourth byte contains the model data
  inline uint8_t* serialNumber() {return(_data+4);} // serialNumber is a 16-byte array (128bit number) (this returns a pointer to it)
  inline uint8_t& serialNumber(uint8_t index) {return(_data[index+4]);} // serialNumber is a 16-byte array (128bit number) (hopefully the compiler makes efficient use of this)
};

struct lidarGetHealthResponse : public byteArrayStruct<3> {
  inline uint8_t& status() {return(_data[0]);} //the first byte contains the status data (0=good, 1=warning, 2=error)
  inline uint16_t& errorCode() {return((uint16_t&)_data[1]);} //the second and third bytes make the error code
};

struct lidarGetSamplerateResponse : public byteArrayStruct<4> {
  inline uint16_t& standard() {return((uint16_t&)_data[0]);} //the standard sample time (for using CMD_SCAN or CMD_FORCE_SCAN) in microseconds
  inline uint16_t& express() {return((uint16_t&)_data[2]);} //the express sample time (for using CMD_EXPRESS_SCAN) in microseconds
};

struct lidarGetLidarConfResponse { // i would have liked to set this at a constant size as well, but it must also be able to carry a String (otherwise i'd just spec it for the largest possible content)
  uint8_t* _data; // a (variable size) byte buffer for the data to be sent (initialized with malloc() on initialization, freed in destructor)
  lidarGetLidarConfResponse(uint8_t payloadSize) : _data((uint8_t*) malloc(payloadSize + sizeof(uint32_t))) {} //room for the payload and the (4byte) 'type' value
  ~lidarGetLidarConfResponse() { delete _data; } //my constructor is too funky for the compiler to think of this.
  inline uint8_t*& operator&() {return(_data);} //any attempts to retrieve the address of this object should be met with a (reference to) _data (it's the same address, just an implicit typecast)
  inline uint8_t& operator[](size_t index) {return(_data[index+4]); } // the payload starts after typeVal
  inline uint32_t& typeVal() {return((uint32_t&)_data[0]);} //GET_LIDAR_CONF returns a 'type' variable, although what this may contain is not entirely clear (it may be an echo of the request you sent it, idk)
  inline operator uint16_t&() {return((uint16_t&)_data[4]);} //cast to unsigned 16bit integer
  inline operator uint32_t&() {return((uint32_t&)_data[4]);} //cast to unsigned 32bit integer
  inline operator uint8_t&() {return(_data[4]);} //cast to unsigned 8bit integer
  //inline String operator() { TBD } //cant be done because this struct does NOT keep track of its own size (so it wouldn't know how long to make the string)
  inline char* charArray() {return((char*)_data+4);} // (really just a pointer to the real payload)
  inline char& charArray(uint8_t index) {return((char&)_data[index+4]);} // the String payload may be easier to read as a char array
};



class RPlidar {
public:
  HardwareSerial& lidarSerial;
  //lidarMotorHandler* motorHandlerPtr; // optional, don't pass this if you want to handle the RPM manually
  uint8_t scanResponseFormat = 0; // 0 if scanning has not been requested (correctly), RESP_TYPE_SCAN or similar if scanning has been requested
  void (*postParseCallback)(RPlidar* self, uint16_t dist, uint16_t angle_q6, uint8_t newRotFlag, int8_t quality) = NULL;
  const uint32_t slowSerialTimout = 10; // packets like lidarRespDescr may take a while to arrive
  const uint32_t fastSerialTimout = 0; // scan data packets should be quick to read out

  int16_t expressDataAngleDelta; // used to calculate intermediate angles (only needed for express packages)
  uint16_t expressDataLastStartAngle; // used to calculate expressDataAngleDelta
  uint32_t packetCount = 0; // a counter for how many packets have been received (used to calculate expressDataAngleDelta)
  uint32_t rotationCount = 0; // a counter to keep track of how many rotations the lidar has made (does NOT get reset when requesting scan start)
  uint32_t rotationSpeedTimer = 0; // a quick'n'dirty timer to calculate rotation speed (using expressDataAngleDelta)
  uint32_t rotationSpeedDt = 0; // (see 1 comment above) this holds the time between packets
  
  size_t _incompleteBytesRemaining = 0; // if you read only part of a packet (and there's still more to come), keep track of how many bytes are left
  lidarStandardData _incompleteStandardPackets[10]; // this is an array becuase i assume the overhead of calling lidarSerial.readBytes() is substantial
  lidarExpressDataLegacy _incompleteExpressLegacyPacket; // uses 84 bytes to store 32 measurements
  lidarExpressDataExtended _incompleteExpressExtendPacket; // uses 132 bytes to store 96 measurements
  uint32_t _incompleteExpressExtendCabinBuff; // the express extend data structure requires the next datapoint to calculate the current one. This holds the last 4byte 'cabin' value
  lidarExpressDataDense _incompleteExpressDensePacket; // uses 84 bytes to store 40 measurements
  
  RPlidar(HardwareSerial& lidarSerial /*, lidarMotorHandler* motorHandlerPtr=NULL*/ ) : lidarSerial(lidarSerial) /*, motorHandlerPtr(motorHandlerPtr)*/ {}
  //~RPlidar() { resetLidar(); }
  void init(uint8_t RXpin=16, uint8_t TXpin=17) {
    lidarSerial.setRxBufferSize(1024); // increase serial buffer size (becuase the lidar will send a lot of data and sometimes the CPU is busy) (must be done before .begin())
    lidarSerial.begin(115200, SERIAL_8N1, RXpin, TXpin);
    lidarSerial.setTimeout(slowSerialTimout);
//    if(motorHandlerPtr) {
//      Serial.println("motor handler found");
//      motorHandlerPtr->init();
//    }
    delay(25); //wait for the lidar to boot up
    while(lidarSerial.available()) { lidarSerial.read(); } // clear the serial buffer (the lidar prints a little ASCII on boot/reset, i would like to ignore that (running any command should also serve this, but still))
    // test connection by attempting to retrieve scan modes and store them somewhere?
  }
//private: // oh whatever, i trust the users of the library enough
  void _sendCommand(lidarCMD& commandToSend) {
    lidarSerial.write(commandToSend.send(), commandToSend.sendSize());
  //  commandToSend.send(); for(uint8_t i=0;i<commandToSend.sendSize();i++){lidarDebugSerial.print("rplidar: "); lidarDebugSerial.print(commandToSend._data[i], HEX); lidarDebugSerial.print(' ');} lidarDebugSerial.println();
  }
  
  size_t _recvRespDescr(lidarRespDescr& resp, uint32_t startFlagTimeout=5000) { // read serial data untill both RESP_DESCR_START_FLAGS are received. startFlagTimeout is in micros, returns the number of bytes read
    lidarSerial.setTimeout(slowSerialTimout);
    uint32_t startTime = micros();
    uint8_t syncProgress = 0;
    while(((micros() - startTime) < startFlagTimeout) && (syncProgress < sizeof(RESP_DESCR_START_FLAGS))) {
      if(lidarSerial.available()) {
        uint8_t potentialSyncByte = lidarSerial.read();
        if(potentialSyncByte == RESP_DESCR_START_FLAGS[syncProgress]) {
          syncProgress++;
    } } }
    if(syncProgress < sizeof(RESP_DESCR_START_FLAGS)) {
      #ifdef lidarDebugSerial
        lidarDebugSerial.print("rplidar: _recvRespDescr() "); lidarDebugSerial.print(syncProgress); lidarDebugSerial.print(" < "); lidarDebugSerial.println(sizeof(RESP_DESCR_START_FLAGS));
      #endif
      return(0); //sync bytes were not received (within startFlagTimeout)
    } // (else) //sync bytes were all received, time to read the message
    return(lidarSerial.readBytes(&resp, sizeof(resp)));
  }

  //size_t wiatForExpressPacketSync() { // TBD(?) a function that looks for the two sync half-bytes at the start of every express packet (and then maybe reads the packet to check the CRC?, although that would also just happen if you let handleData() run)
  
  lidarRespDescr _sendRequestNoPayload(uint8_t CMDbyte) {
    lidarCMD commandToSend(CMDbyte, 0);
    _sendCommand(commandToSend);
    lidarRespDescr resp;
    size_t bytesRead = _recvRespDescr(resp);
    if(bytesRead < sizeof(resp)) { // if something went wrong with the reading
      //memset(&resp, 0, sizeof(resp)); // set all zeros
      resp.dataType() = RESP_TYPE_INVALID;
      //lidarDebugSerial.print("rplidar: _sendRequestNoPayload() bytesRead < sizeof(resp)"); lidarDebugSerial.print(bytesRead); lidarDebugSerial.print(" < "); lidarDebugSerial.println(sizeof(resp));
    }
    return(resp);
  }
  
  template<typename returnType, const uint8_t CMDbyte, const uint8_t correct_resp_type_byte> // CMDbyte and correct_resp_type_byte dont need to be templated, theyt could also just be input variables
  returnType _getSomethingSimple() {
    lidarRespDescr resp = _sendRequestNoPayload(CMDbyte);
    returnType returnVal;
    if((resp.dataType() == correct_resp_type_byte) && (resp.responseLength() == sizeof(returnVal))) {
      lidarSerial.setTimeout(slowSerialTimout);
      size_t bytesRead = lidarSerial.readBytes(&returnVal, sizeof(returnVal));
      if(bytesRead < sizeof(returnVal)) {
        //memset(&returnVal, 0xFF, sizeof(returnVal)); // do something to indicate faillure
        #ifdef lidarDebugSerial
          /*lidarDebugSerial.print(funcName);*/ lidarDebugSerial.print("rplidar: _getSomethingSimple() bytesRead < sizeof(returnVal) "); lidarDebugSerial.print(bytesRead); lidarDebugSerial.print(" < "); lidarDebugSerial.println(sizeof(returnVal));
        #endif
      }
    } else {
      //memset(&returnVal, 0xFF, sizeof(returnVal)); // do something to indicate faillure
      #ifdef lidarDebugSerial
        /*lidarDebugSerial.print(funcName);*/ lidarDebugSerial.print("rplidar: _getSomethingSimple() resp wrong! "); 
        lidarDebugSerial.print(resp.dataType(), HEX); lidarDebugSerial.print(" != "); lidarDebugSerial.print(correct_resp_type_byte, HEX); lidarDebugSerial.print(" || ");
        lidarDebugSerial.print(resp.responseLength()); lidarDebugSerial.print(" != "); lidarDebugSerial.println(sizeof(returnVal));
      #endif
    }
    return(returnVal);
  }
  
  
  uint32_t _recvLidarConfRespDesc() {
    lidarRespDescr resp;
    size_t bytesRead = _recvRespDescr(resp);
    if((bytesRead < sizeof(resp)) || (resp.dataType() != RESP_TYPE_GET_LIDAR_CONF)) {
      #ifdef lidarDebugSerial
        lidarDebugSerial.print("rplidar: _recvLidarConfRespDesc() response issue: "); 
        lidarDebugSerial.print(bytesRead); lidarDebugSerial.print(" < "); lidarDebugSerial.print(sizeof(resp)); lidarDebugSerial.print(" || ");
        lidarDebugSerial.print(resp.dataType(), HEX); lidarDebugSerial.print(" != "); lidarDebugSerial.println(RESP_TYPE_GET_LIDAR_CONF, HEX);
      #endif
      return(0);
    } // else
    return(resp.responseLength());
  }
  
  lidarGetLidarConfResponse _recvLidarConfPayload(uint8_t payloadSize) { // payloadSize is INCLUDING 4 bytes for 'type' variable
    lidarGetLidarConfResponse lidarConfPayload(payloadSize - sizeof(uint32_t)); // (all lidarGetLidarConfResponse's include the first 4 bytes, so the constructor just takes how many bytes on top of that are needed)
    lidarSerial.setTimeout(slowSerialTimout);
    size_t bytesRead = lidarSerial.readBytes(&lidarConfPayload, payloadSize);
    if(bytesRead < payloadSize) {
      #ifdef lidarDebugSerial
        lidarDebugSerial.print("rplidar: _recvLidarConfPayload() lidarConfPayload read issue: "); 
        lidarDebugSerial.print(bytesRead); lidarDebugSerial.print(" < "); lidarDebugSerial.println(payloadSize);
      #endif
      lidarConfPayload.typeVal() = 0x00; //indicate that receiving failed
    } // else
    return(lidarConfPayload);
  }
  
  template<typename returnType, const uint8_t CONF_CMDbyte>
  returnType _getLidarConfAttributeBoth(lidarCMD& lidarConfCMD) {
    _sendCommand(lidarConfCMD);
    uint32_t currentRespLength = _recvLidarConfRespDesc();
    if(currentRespLength != (sizeof(uint32_t)+sizeof(returnType))) {
      #ifdef lidarDebugSerial
        lidarDebugSerial.print("rplidar: lidarConfCMD currentRespLength bad:"); lidarDebugSerial.println(currentRespLength);
      #endif
      return(0);
    } // else
    lidarGetLidarConfResponse lidarConfAttribute = _recvLidarConfPayload(sizeof(uint32_t)+sizeof(returnType));
    if(lidarConfAttribute.typeVal() != CONF_CMDbyte) {
      #ifdef lidarDebugSerial
        lidarDebugSerial.print("rplidar: _getLidarConfAttributeBoth() lidarConfAttribute read issue: ");
        lidarDebugSerial.print(lidarConfAttribute.typeVal()); lidarDebugSerial.print(" != "); lidarDebugSerial.println(CONF_CMDbyte);
      #endif
      return(0);
    } // else
    return((returnType)lidarConfAttribute);
  }
  
  template<typename returnType, const uint8_t CONF_CMDbyte>  // CONF_CMDbyte could also just be an input variable and returnType is (currently) always uint16_t
  returnType _getLidarConfAttributeNoPayload() {
    lidarCMD lidarConfCMD(CMD_GET_LIDAR_CONF, sizeof(uint32_t)+0);
    //lidarConfCMD[0] = CONF_CMDbyte; memset(&lidarConfCMD[1], 0, 3); // fill in the 'type' value (BTW, memset no longer needed thanks to calloc())
    lidarConfCMD.payloadAsUint32_t(0) = CONF_CMDbyte; // this is just a slightly faster way of filling the remaining 3 bytes of the 'type' value with 0s
    return(_getLidarConfAttributeBoth<returnType, CONF_CMDbyte>(lidarConfCMD));
  }
  
  template<typename returnType, typename requestPayloadType, const uint8_t CONF_CMDbyte>  // CONF_CMDbyte could be an input variable and payloadType is (currently) always uint16_t
  returnType _getLidarConfAttribute(requestPayloadType requestPayload) {
    lidarCMD lidarConfCMD(CMD_GET_LIDAR_CONF, sizeof(uint32_t)+sizeof(requestPayload));
    //lidarConfCMD[0] = CONF_CMDbyte; memset(&lidarConfCMD[1], 0, 3); // fill in the 'type' value (BTW, memset no longer needed thanks to calloc())
    lidarConfCMD.payloadAsUint32_t(0) = CONF_CMDbyte; // this is just a slightly faster way of filling the remaining 3 bytes of the 'type' value with 0s
    //lidarConfCMD.payloadAsUint16_t(sizeof(uint32_t)) = requestPayload; // if you ever remove requestPayloadType from the template (because it can only be a uint16_t), use this line
    //(requestPayloadType&)lidarConfCMD[sizeof(uint32_t)] = requestPayload; // i dont know if this works (but it reads like shit)
    requestPayloadType& payloadReference = (requestPayloadType&)lidarConfCMD[sizeof(uint32_t)];
    payloadReference = requestPayload; // should work
    return(_getLidarConfAttributeBoth<returnType, CONF_CMDbyte>(lidarConfCMD));
  }

  uint16_t _varbitscale_decode(uint16_t major, uint8_t& scaleLevel) { // taken from SDK (and modified slightly)
    static const int32_t VBS_SCALED_BASE[] = {
      3328, // SL_LIDAR_VARBITSCALE_X16_DEST_VAL // thijs: 3328 = 1101 0000 0000 = (13)<<8  ?
      1792, // SL_LIDAR_VARBITSCALE_X8_DEST_VAL // thijs: 1792 = 0111 0000 0000 = (7)<<8  ?
      1280, // SL_LIDAR_VARBITSCALE_X4_DEST_VAL // thijs: 1280 = 0101 0000 0000 = (5)<<8  ?
      512, // SL_LIDAR_VARBITSCALE_X2_DEST_VAL // thijs: 512 = 0010 0000 0000 = (2)<<8  ?
      0 };
    static const uint16_t VBS_TARGET_BASE[] = {
      (1<<14), // SL_LIDAR_VARBITSCALE_X16_SRC_BIT
      (1<<12), // SL_LIDAR_VARBITSCALE_X8_SRC_BIT
      (1<<11), // SL_LIDAR_VARBITSCALE_X4_SRC_BIT
      (1<<9), // SL_LIDAR_VARBITSCALE_X2_SRC_BIT
      0 };
    for (size_t i = 0; i < 5; ++i) {
      int32_t remain = ((int32_t)major - VBS_SCALED_BASE[i]);
      if (remain >= 0) {
        scaleLevel = 5-1-i;
        return(VBS_TARGET_BASE[i] + (remain << scaleLevel));
      }
    }
    return(0);
  }

public:
  inline lidarGetInfoResponse getInfo() {return(_getSomethingSimple<lidarGetInfoResponse, CMD_GET_INFO, RESP_TYPE_GET_INFO>());}
  inline lidarGetHealthResponse getHealth() {return(_getSomethingSimple<lidarGetHealthResponse, CMD_GET_HEALTH, RESP_TYPE_GET_HEALTH>());}
  inline lidarGetSamplerateResponse getSamplerate() {return(_getSomethingSimple<lidarGetSamplerateResponse, CMD_GET_SAMPLERATE, RESP_TYPE_GET_SAMPLERATE>());}
  
  void printLidarInfo() {
    lidarGetInfoResponse info = getInfo();
    Serial.print("model: "); Serial.println(info.model());
    Serial.print("firmware: "); Serial.print(info.firmwareMajor()); Serial.print('.'); Serial.println(info.firmwareMinor());
    Serial.print("hardware: "); Serial.println(info.hardware());
    Serial.print("serialNumber (HEX): "); for(uint8_t i=0;i<16;i++){Serial.print(info.serialNumber(i), HEX);Serial.print(' ');} Serial.println();
    Serial.print("serialNumber (32bit): "); for(uint8_t i=0;i<4;i++){Serial.print((uint32_t&)info.serialNumber(i*4));Serial.print(' ');} Serial.println();
    Serial.print("serialNumber (64bit): "); for(uint8_t i=0;i<2;i++){Serial.print((uint64_t&)info.serialNumber(i*8));Serial.print(' ');} Serial.println();
    //Serial.print("serialNumber (128bit): "); TBD Serial.println();
  }
  void printLidarHealth() {
    lidarGetHealthResponse health = getHealth();
    Serial.print("status: "); Serial.println(health.status());
    Serial.print("errorCode (HEX): "); Serial.println(health.errorCode(), HEX);
  }
  void printLidarSamplerate() {
    lidarGetSamplerateResponse samplerate = getSamplerate();
    Serial.print("standard: "); Serial.println(samplerate.standard());
    Serial.print("express: "); Serial.println(samplerate.express());
  }
  
  inline uint16_t getLidarConfScanModeCount() {return(_getLidarConfAttributeNoPayload<uint16_t, GET_CONF_SCAN_MODE_COUNT>());}
  inline uint16_t getLidarConfScanModeTypical() {return(_getLidarConfAttributeNoPayload<uint16_t, GET_CONF_SCAN_MODE_TYPICAL>());}
  inline uint32_t getLidarConfScanModeSampletime(uint16_t index) {return(_getLidarConfAttribute<uint32_t, uint16_t, GET_CONF_SCAN_MODE_SAMPLETIME>(index));}
  inline uint32_t getLidarConfScanModeMaxDist(uint16_t index) {return(_getLidarConfAttribute<uint32_t, uint16_t, GET_CONF_SCAN_MODE_MAX_DIST>(index));}
  inline uint8_t getLidarConfScanModeAnsType(uint16_t index) {return(_getLidarConfAttribute<uint8_t, uint16_t, GET_CONF_SCAN_MODE_ANS_TYPE>(index));}
  
  String getLidarConfScanModeName(uint16_t index) { // this one requires an extra special function (mostly becuase the length of the string is not stored in the lidarGetLidarConfResponse struct)
    String returnString = "";
    lidarCMD lidarConfCMD(CMD_GET_LIDAR_CONF, sizeof(uint32_t)+sizeof(index));
    lidarConfCMD.payloadAsUint32_t(0) = GET_CONF_SCAN_MODE_NAME; // this is just a slightly faster way of filling the remaining 3 bytes of the 'type' value with 0s
    lidarConfCMD.payloadAsUint16_t(sizeof(uint32_t)) = index;
    _sendCommand(lidarConfCMD);
    uint32_t currentRespLength = _recvLidarConfRespDesc();
    if((currentRespLength <= sizeof(uint32_t)) || (currentRespLength > 260)) {
      #ifdef lidarDebugSerial
        lidarDebugSerial.print("rplidar: lidarConfCMD currentRespLength bad:"); lidarDebugSerial.println(currentRespLength);
      #endif
      return(returnString);
    } // else
    lidarGetLidarConfResponse lidarConfAttribute = _recvLidarConfPayload(currentRespLength);
    if(lidarConfAttribute.typeVal() != GET_CONF_SCAN_MODE_NAME) {
      #ifdef lidarDebugSerial
        lidarDebugSerial.print("rplidar: _getLidarConfAttributeBoth() lidarConfAttribute read issue: ");
        lidarDebugSerial.print(lidarConfAttribute.typeVal()); lidarDebugSerial.print(" != "); lidarDebugSerial.println(GET_CONF_SCAN_MODE_NAME);
      #endif
      return(returnString);
    } // else
    // now remake string
    for(uint32_t i=0; i<(currentRespLength-sizeof(uint32_t)-1); i++) { returnString += lidarConfAttribute.charArray(i); } // -1 is to skip the last char (should be 0)
  //  // alternatively:
  //  lidarConfAttribute.charArray(currentRespLength-sizeof(uint32_t)-1) = 0; // make sure the last char is NULL (marks the end of the string). This should already be the case, but i NEED to be sure
  //  returnString = String(lidarConfAttribute.charArray()); // convert to String from char*
    return(returnString);
  }
  
  void printLidarConfig() {
    uint16_t scanModeCount = getLidarConfScanModeCount();
    //if((scanModeCount == 0) || (scanModeCount > 10)) {Serial.print("");Serial.println();return;}
    Serial.print("scan mode count: "); Serial.println(scanModeCount);
    for(uint16_t i=0; i<scanModeCount; i++) {
      Serial.print("scan mode: "); Serial.println(i);
      String modeName = getLidarConfScanModeName(i); Serial.print('\t'); Serial.print("name: "); Serial.println(modeName); // inefficient as heck, but it should work
      uint32_t sampletime = getLidarConfScanModeSampletime(i);  Serial.print('\t'); Serial.print("sampletime: "); Serial.print(sampletime); Serial.print(" = "); Serial.print((float)sampletime/256.0); Serial.println(" microseconds");
      uint32_t maxDist = getLidarConfScanModeMaxDist(i);  Serial.print('\t'); Serial.print("maxDist: "); Serial.print(maxDist); Serial.print(" = "); Serial.print((float)maxDist/256.0); Serial.println(" meters");
      uint32_t ansType = getLidarConfScanModeAnsType(i);  Serial.print('\t'); Serial.print("ansType (HEX): "); Serial.println(ansType, HEX); // todo: print string for CONF_SCAN_MODE_ANS_TYPE_STANDARD or one of those
    }
    uint16_t scanModeTypical = getLidarConfScanModeTypical();
    Serial.print("scan mode typical: "); Serial.println(scanModeTypical);
  }

  bool startStandardScan(bool force=false) {
    //if(postParseCallback == NULL) { lidarDebugSerial.println("rplidar: no postParseCallback function set, the data will go nowhere");
    _incompleteBytesRemaining = 0; // reset (and discard) partial packet data
    packetCount = 0; // reset packet counter
    lidarRespDescr resp = _sendRequestNoPayload(force ? CMD_FORCE_SCAN : CMD_SCAN);
    scanResponseFormat = resp.dataType(); // save what type of data the lidar is going to send out
    bool success = (resp.dataType() == RESP_TYPE_SCAN) && (resp.sendMode() == RESP_DESCR_SENDMODE_MULTI_RESPONSE);
    //if((motorHandlerPtr) && success) { //start motor }
    return(success);
  }
  bool startExpressScan(uint8_t extendMode=EXPRESS_SCAN_WORKING_MODE_LEGACY) {
    //if(postParseCallback == NULL) { lidarDebugSerial.println("rplidar: no postParseCallback function set, the data will go nowhere");
    _incompleteBytesRemaining = 0; // reset (and discard) partial packet data
    packetCount = 0; // reset packet counter
    lidarCMD commandToSend(CMD_EXPRESS_SCAN, 5);
    commandToSend[0] = extendMode; // first byte of CMD payload is 'working mode' (badly described in documentation how these values are derived
    _sendCommand(commandToSend);
    lidarRespDescr resp;
    size_t bytesRead = _recvRespDescr(resp);
    if((bytesRead < sizeof(resp)) || (resp.sendMode() != RESP_DESCR_SENDMODE_MULTI_RESPONSE) ||     // if something went wrong with the reading
       ((resp.dataType() != ((extendMode != EXPRESS_SCAN_WORKING_MODE_LEGACY) ? RESP_TYPE_SCAN_EXPRESS_EXTEND : RESP_TYPE_SCAN_EXPRESS_LEGACY)) && (resp.dataType() != RESP_TYPE_SCAN_EXPRESS_DENSE)) )  { // or if the datatype is not what you expected
      #ifdef lidarDebugSerial
        lidarDebugSerial.print("rplidar: startExpressScan error: ");
        lidarDebugSerial.print(bytesRead); lidarDebugSerial.print('<'); lidarDebugSerial.print(sizeof(resp)); lidarDebugSerial.print(" || "); 
        lidarDebugSerial.print(resp.sendMode()); lidarDebugSerial.print("!="); lidarDebugSerial.print(RESP_DESCR_SENDMODE_MULTI_RESPONSE); lidarDebugSerial.print(" || ");
        lidarDebugSerial.print(resp.dataType(),HEX); lidarDebugSerial.print("!="); 
                                        if(extendMode != EXPRESS_SCAN_WORKING_MODE_LEGACY){ lidarDebugSerial.print(RESP_TYPE_SCAN_EXPRESS_EXTEND,HEX); }
                                        else{lidarDebugSerial.print('(');lidarDebugSerial.print(RESP_TYPE_SCAN_EXPRESS_LEGACY,HEX);lidarDebugSerial.print("||");lidarDebugSerial.print(RESP_TYPE_SCAN_EXPRESS_DENSE,HEX);lidarDebugSerial.print(')');}
                                        lidarDebugSerial.println();
      #endif
      scanResponseFormat = 0; // save the fact that starting the scan failed
      return(false);
    } // else
    scanResponseFormat = resp.dataType(); // save what type of data the lidar is going to send out
//    expressDataAngleDelta = ;  // predicting this accurately ahead of time is too difficult (the samplerate and RPM vary too much)
//    expressDataLastStartAngle = 0; // instead, the first data packet (express data only) is skipped to record the startAngle
    //if(motorHandlerPtr) { //start motor }
    return(true);
  }
  
  int8_t handleData(bool includeInvalidMeasurements=true, bool waitForChecksum=false, bool doExtendAngleMath=false) { // returns how many measurements were sent to the postParseCallback()
    if(scanResponseFormat == 0) {
      #ifdef lidarDebugSerial
        lidarDebugSerial.println("rplidar: handleData() failed, scanResponseFormat == 0!"); 
      #endif
      return(-1);
    }
    //if(postParseCallback == NULL) { lidarDebugSerial.println("rplidar: no postParseCallback function set, the data will go nowhere");
    uint8_t measurementsHandled = 0;
    lidarSerial.setTimeout(fastSerialTimout);
/////////////////////////////////////////////////////////////////////////////////////////////////////////standard///////////////////////////////////////////////////////////////////////////////////
    if(scanResponseFormat == RESP_TYPE_SCAN) { // standard scan data
      if(!_incompleteBytesRemaining) { _incompleteBytesRemaining = sizeof(_incompleteStandardPackets); }   uint32_t startReadOffset = sizeof(_incompleteStandardPackets) - _incompleteBytesRemaining;
      size_t bytesRead = lidarSerial.readBytes((&_incompleteStandardPackets[0]) + startReadOffset, _incompleteBytesRemaining);   _incompleteBytesRemaining -= bytesRead;
      uint8_t whereToStopParse = ((sizeof(_incompleteStandardPackets)-_incompleteBytesRemaining)/sizeof(lidarStandardData)); // integer division should ensure automatic low-bound-rounding
      uint8_t whereToStartParse = startReadOffset / sizeof(lidarStandardData); // CHECK
      for(uint8_t i=whereToStartParse; i<whereToStopParse; i++) {
        if(_incompleteStandardPackets[i].checkBit() && ((_incompleteStandardPackets[i].rotStartFlag() != STANDARD_DATA_ROT_START_BITS) && (_incompleteStandardPackets[i].rotStartFlag() != 0))) {
          packetCount++;
          uint16_t dist = _incompleteStandardPackets[i].dist() >> 2; // standard angle data is 16bits, but in 'q2' format. right shifting discards that 0.25mm resolution (but the lidar has a +- 1mm accuracy anyway, 1mm resolution is perfectly adequate
          if(_incompleteStandardPackets[i].rotStartFlag() == 1) { rotationCount++; }
          if((postParseCallback) && (includeInvalidMeasurements ? true : (dist > 0))) { postParseCallback(this, dist, _incompleteStandardPackets[i].angle(), (_incompleteStandardPackets[i].rotStartFlag() == 1), _incompleteStandardPackets[i].quality()); }
        } else {
          #ifdef lidarDebugSerial
            lidarDebugSerial.print("rplidar: handleData() standard scan check error: "); lidarDebugSerial.print(_incompleteStandardPackets[i].checkBit()); lidarDebugSerial.print(" or "); lidarDebugSerial.println(_incompleteStandardPackets[i].rotStartFlag()); return(-1);
          #endif
          //if(motorHandlerPtr) { //stop motor }
        }
      }
      if(_incompleteBytesRemaining == 0) { // once the buffer is full (mostly to avoid spamming these functions)
        int16_t newStartAngle = _incompleteStandardPackets[0].angle(); // idk, why not
        expressDataAngleDelta = newStartAngle - expressDataLastStartAngle; // JUST FOR CALCULATING ROTATION SPEED, expressDataLastStartAngle is not actually required for standard scans
        if(newStartAngle < expressDataLastStartAngle) { expressDataAngleDelta += (360<<6); rotationCount++; } // angle rollover (only needed in one direction, becuase the lidar always spins in the same direction
        expressDataLastStartAngle = newStartAngle;
        rotationSpeedDt = micros() - rotationSpeedTimer;  rotationSpeedTimer = micros(); // for calculating rotation speed (quickly, not accurately)
        //if(motorHandlerPtr) { motorHandlerPtr->RPM_PID(uint32_t angleDiff, uint32_t microsDiff); } // TODO?
      }
/////////////////////////////////////////////////////////////////////////////////////////////////////////express legacy///////////////////////////////////////////////////////////////////////////////////
    } else if(scanResponseFormat == RESP_TYPE_SCAN_EXPRESS_LEGACY) { // express legacy scan data
      if(!_incompleteBytesRemaining) { _incompleteBytesRemaining = sizeof(_incompleteExpressLegacyPacket); }   uint32_t startReadOffset = sizeof(_incompleteExpressLegacyPacket) - _incompleteBytesRemaining;
      size_t bytesRead = lidarSerial.readBytes((&_incompleteExpressLegacyPacket) + startReadOffset, _incompleteBytesRemaining);   _incompleteBytesRemaining -= bytesRead;
      if((_incompleteBytesRemaining == 0) || ((!waitForChecksum) && ((sizeof(_incompleteExpressLegacyPacket) - _incompleteBytesRemaining) > sizeof(_lidarExpressDataBase<4>)))) { //if the packet is complete or if you don't care about checking data (measurements start after 4byte preamble)
        if(_incompleteBytesRemaining == 0) { // if the whole packet is read
          if(_incompleteExpressLegacyPacket.calcChecksum() != _incompleteExpressLegacyPacket.checksum()) {
            #ifdef lidarDebugSerial
              lidarDebugSerial.print("rplidar: handleData() express legacy packet checksum error: "); lidarDebugSerial.print(_incompleteExpressLegacyPacket.calcChecksum()); lidarDebugSerial.print(" != "); lidarDebugSerial.println(_incompleteExpressLegacyPacket.checksum());  return(-1);
            #endif
            //if(motorHandlerPtr) { //stop motor }
          }
          // updating expressDataAngleDelta doesnt require the whole packet, but i only want to do it once per packet. This delays the function, but since the samplerate is so high, the RPM should change very little between packets
          if(packetCount == 0) { expressDataLastStartAngle = _incompleteExpressLegacyPacket.startAngle(); packetCount++;
                                 rotationSpeedTimer = micros();  return(0); } // skip the first packet entirely, just to get the expressDataAngleDelta
          expressDataAngleDelta = _incompleteExpressLegacyPacket.startAngle() - expressDataLastStartAngle; // update the expressDataAngleDelta
          if(_incompleteExpressLegacyPacket.startAngle() < expressDataLastStartAngle) { expressDataAngleDelta += (360<<6); rotationCount++; } // angle rollover (only needed in one direction, becuase the lidar always spins in the same direction
          expressDataLastStartAngle = _incompleteExpressLegacyPacket.startAngle();
          packetCount++;
          rotationSpeedDt = micros() - rotationSpeedTimer;  rotationSpeedTimer = micros(); // for calculating rotation speed (quickly, not accurately)
          //if(motorHandlerPtr) { motorHandlerPtr->RPM_PID(uint32_t angleDiff, uint32_t microsDiff); } // TODO? (this will make motor PID dependent on packet checksum success and therefore potentially inconsistent)
        }
        uint8_t whereToStopParse = (sizeof(_incompleteExpressLegacyPacket)-_incompleteBytesRemaining-sizeof(_lidarExpressDataBase<4>)) / 5; // CHECK 0 to 16
        uint8_t whereToStartParse = (startReadOffset>sizeof(_lidarExpressDataBase<4>)) ? ((startReadOffset-sizeof(_lidarExpressDataBase<4>)) / 5) : 0; // CHECK
        for(uint8_t i=whereToStartParse; i<whereToStopParse; i++) {
          for(uint8_t j=0; j<2; j++) {
            int16_t angle_q6 = _incompleteExpressLegacyPacket.startAngle() + ((expressDataAngleDelta*((i*2)+j))>>5) - (((int16_t)_incompleteExpressLegacyPacket.deltaAngle((i*2)+j))<<3); // angle = start + (speed*dt)/32 + extraDelta
            angle_q6 = angle_q6 % (360<<6); //if(angle > (360<<6)) { angle -= (360<<6); } // handle rollover
            bool startFlag = _incompleteExpressLegacyPacket.rotStartFlag() && (((i*2)+j) == 0);
            uint16_t dist = _incompleteExpressLegacyPacket.dist((i*2)+j); //express legacy dist is 14bit, and directly in mm.
            if((postParseCallback) && (includeInvalidMeasurements ? true : (dist > 0))) { postParseCallback(this, dist, angle_q6, startFlag, -1); }
          }
        }
      }
/////////////////////////////////////////////////////////////////////////////////////////////////////////express extended///////////////////////////////////////////////////////////////////////////////////
    } else if(scanResponseFormat == RESP_TYPE_SCAN_EXPRESS_EXTEND) { // express extended scan data
      if(!_incompleteBytesRemaining) { _incompleteBytesRemaining = sizeof(_incompleteExpressExtendPacket); }   uint32_t startReadOffset = sizeof(_incompleteExpressExtendPacket) - _incompleteBytesRemaining;
      size_t bytesRead = lidarSerial.readBytes((&_incompleteExpressExtendPacket) + startReadOffset, _incompleteBytesRemaining);   _incompleteBytesRemaining -= bytesRead;
      if((_incompleteBytesRemaining == 0) || ((!waitForChecksum) && ((sizeof(_incompleteExpressExtendPacket) - _incompleteBytesRemaining) > sizeof(_lidarExpressDataBase<4>)))) { //if the packet is complete or if you don't care about checking data (measurements start after 4byte preamble)
        if(_incompleteBytesRemaining == 0) { // if the whole packet is read
          if(_incompleteExpressExtendPacket.calcChecksum() != _incompleteExpressExtendPacket.checksum()) {
            #ifdef lidarDebugSerial
              lidarDebugSerial.print("rplidar: handleData() express extend packet checksum error: "); lidarDebugSerial.print(_incompleteExpressExtendPacket.calcChecksum()); lidarDebugSerial.print(" != "); lidarDebugSerial.println(_incompleteExpressExtendPacket.checksum());  return(-1);
            #endif
            //if(motorHandlerPtr) { //stop motor }
          }
          // updating expressDataAngleDelta doesnt require the whole packet, but i only want to do it once per packet. This delays the function, but since the samplerate is so high, the RPM should change very little between packets
          if(packetCount == 0) { expressDataLastStartAngle = _incompleteExpressExtendPacket.startAngle();  packetCount++;
                                     _incompleteExpressExtendCabinBuff = _incompleteExpressExtendPacket.wholeCabin(31);   // (only for express extend) save the last cabin of the discarded packet (because this data scheme needs a cabin buffer)
                                     rotationSpeedTimer = micros();  return(0); } // skip the first packet entirely, just to get the expressDataAngleDelta
          expressDataAngleDelta = _incompleteExpressExtendPacket.startAngle() - expressDataLastStartAngle; // update the expressDataAngleDelta
          if(_incompleteExpressExtendPacket.startAngle() < expressDataLastStartAngle) { expressDataAngleDelta += (360<<6); rotationCount++; } // angle rollover (only needed in one direction, becuase the lidar always spins in the same direction
          expressDataLastStartAngle = _incompleteExpressExtendPacket.startAngle();
          packetCount++;
          rotationSpeedDt = micros() - rotationSpeedTimer;  rotationSpeedTimer = micros(); // for calculating rotation speed (quickly, not accurately)
          //if(motorHandlerPtr) { motorHandlerPtr->RPM_PID(uint32_t angleDiff, uint32_t microsDiff); } // TODO? (this will make motor PID dependent on packet checksum success and therefore potentially inconsistent)
        }
        int8_t whereToStopParse = (sizeof(_incompleteExpressExtendPacket)-_incompleteBytesRemaining-sizeof(_lidarExpressDataBase<4>)) / 4; // CHECK 0 to 32
        int8_t whereToStartParse = (startReadOffset>sizeof(_lidarExpressDataBase<4>)) ? ((startReadOffset-sizeof(_lidarExpressDataBase<4>)) / 4) : 0; // CHECK
        // buffering measurements is required for this data format to work
        for(int8_t i=whereToStartParse; i<whereToStopParse; i++) {
          uint16_t dist[3]; // a little array to hold the distances ( CHECK whether this is in fact enough)
          uint8_t predictShiftOne, predictShiftTwo; // how much the data was scaled (shifted), i think...
          // decode with the var bit scale ...
          uint16_t major = _incompleteExpressExtendCabinBuff & EXPRESS_DATA_EXTEND_MAJOR_BITS; //
          dist[0] = _varbitscale_decode(major, predictShiftOne); // 'varbitscale' decode, taken straight from the SDK...
          uint16_t dist_major_next = _varbitscale_decode(_incompleteExpressExtendPacket.major(i), predictShiftTwo); // presumably, this results in a measurement with a full range, but reduced resolution (since the only input is a 12bit value)
          uint16_t dist_base1 = dist[0]; // used to calculate second measurement
          if ((!dist_base1) && dist_major_next) { // this one doesnt entirely make sense to me, but maybe it's a quirk of the communication protocol... idk
            dist_base1 = dist_major_next;
            predictShiftOne = predictShiftTwo;
          }
          int32_t predictOne = (((int32_t)(_incompleteExpressExtendCabinBuff << 10)) >> 22); // shifting signed values to the right pads left bits with the sign bit (to keep it positive/negative)
          int32_t predictTwo = (((int32_t)_incompleteExpressExtendCabinBuff) >> 22); // signed cast is important!
          if ((predictOne == 0xFFFFFE00) || (predictOne == 0x1FF)) { dist[1] = 0; } // -512 or 511 means no measurement
          else { dist[1] = dist_base1 + (predictOne << predictShiftOne); } // calculated based on first distance and a prediction (equally scaled)
          if ((predictTwo == 0xFFFFFE00) || (predictTwo == 0x1FF)) { dist[2] = 0; } // -512 or 511 means no measurement
          else { dist[2] = dist_major_next + (predictTwo << predictShiftTwo); } // calculated based on next packet's distance and a prediction (equally scaled)
          
          for(uint8_t j=0; j<3; j++) {
            if(includeInvalidMeasurements ? true : (dist[j] > 0)) { // a little timesaver (avoid calculating the angle if you're not gonna use it anyway)
              int32_t angle_q6 = _incompleteExpressExtendPacket.startAngle() + (expressDataAngleDelta*((i*3)+j))/96; // angle = start + (speed*dt)/40 + extraDelta
              if(doExtendAngleMath) {
                // calculate angle (taken from the SDK and altered slightly)
                // NOTE: this math shifts the angle based on the distance (for some reason), and may therefore result in non-chronological angles (angles dropping below the last one for no reason)
                int32_t offsetAngleMean_q16;
                if(dist[j] >= 50) { // note: in the SDK they use (50*4), but they also use dist_q2, so (50*4)>>2 = 50*4/4 = 50
                  int32_t k2 = (int32_t)98361 / ((int32_t)dist[j] << 2); // presumably, any odd value divided by any dist_Q2 would be equivalent to (the value's nearest multiple of 4) divided by dist_q2
                  static const float extraThingy = radians((1<<19)); // from SDK to mine: (8 * 3.1415926535 * (1 << 16) / 180) = radians(8*(1<<16) = radians(1<<19)
                  offsetAngleMean_q16 = degrees(extraThingy - (k2 << 6) - (k2*k2*k2)/98304); // i'm converting to degrees here (instead of the SDK, where they do it a little later)
                  /* i've done some napkin-math, and the largest numbers to come out of these formulas are: 
                  *  k2 = 98361/(50<<2)=491;
                  *  offset_q16 = degrees(9150 - (491*64) - (491^3)/98304) = degrees(9150 - 31424 - 1204) = degrees(-23478) = -1345190
                  *  offset_q6 = -1314
                  *  offset_degrees = -20.5 degrees
                  * and on the other side of the spectrum:
                  *  k2 = 98361 / (25000<<2) = 0  // max range for some lidar models is 25m in scan modes which (on the A1M8 at least) use this type of data format
                  *  offset_q16 = degrees(radians(1<<19)) = 1<<19 = 524288
                  *  offset_q6 = 512
                  *  offset_degrees = +8 degrees
                  * a simplified approximation is: offset_degrees = 8deg - 1376 * (1.0/dist + 96/(dist^3))             */
                } else {
                  static const float extraThingy = 7.5*(1<<16); // (= 491520) 7.5 degrees
                  offsetAngleMean_q16 = extraThingy; // in the SDK they convert this to radians, only to convert back to degrees right after... no thank you
                }
                angle_q6 = ((angle_q6<<10) - offsetAngleMean_q16)>>10; // add offset
                if(angle_q6 < 0) { angle_q6 += (360<<6); } // handle single negative rollover
              }
              if(angle_q6 >= (360<<6)) { angle_q6 -= (360<<6); } // handle single positive rollover
              
              bool startFlag = _incompleteExpressExtendPacket.rotStartFlag() && (((i*3)+j) == 0);
              if(postParseCallback) { postParseCallback(this, dist[j], angle_q6, startFlag, -1); }
            }
          }
          _incompleteExpressExtendCabinBuff = _incompleteExpressExtendPacket.wholeCabin(i); // now that the last cabin is processed, store the current one in its place
        }
      }
/////////////////////////////////////////////////////////////////////////////////////////////////////////express dense///////////////////////////////////////////////////////////////////////////////////
    } else if(scanResponseFormat == RESP_TYPE_SCAN_EXPRESS_DENSE) { // express dense scan data
      if(!_incompleteBytesRemaining) { _incompleteBytesRemaining = sizeof(_incompleteExpressDensePacket); }   uint32_t startReadOffset = sizeof(_incompleteExpressDensePacket) - _incompleteBytesRemaining;
      size_t bytesRead = lidarSerial.readBytes((&_incompleteExpressDensePacket) + startReadOffset, _incompleteBytesRemaining);   _incompleteBytesRemaining -= bytesRead;
      if((_incompleteBytesRemaining == 0) || ((!waitForChecksum) && ((sizeof(_incompleteExpressDensePacket) - _incompleteBytesRemaining) > sizeof(_lidarExpressDataBase<4>)))) { //if the packet is complete or if you don't care about checking data (measurements start after 4byte preamble)
        if(_incompleteBytesRemaining == 0) { // if the whole packet is read
          if(_incompleteExpressDensePacket.calcChecksum() != _incompleteExpressDensePacket.checksum()) {
            #ifdef lidarDebugSerial
              lidarDebugSerial.print("rplidar: handleData() express extend packet checksum error: "); lidarDebugSerial.print(_incompleteExpressDensePacket.calcChecksum()); lidarDebugSerial.print(" != "); lidarDebugSerial.println(_incompleteExpressDensePacket.checksum());  return(-1);
            #endif
            //if(motorHandlerPtr) { //stop motor }
          }
          // updating expressDataAngleDelta doesnt require the whole packet, but i only want to do it once per packet. This delays the function, but since the samplerate is so high, the RPM should change very little between packets
          if(packetCount == 0) { expressDataLastStartAngle = _incompleteExpressDensePacket.startAngle(); packetCount++; 
                                 rotationSpeedTimer = micros();  return(0); } // skip the first packet entirely, just to get the expressDataAngleDelta
          expressDataAngleDelta = _incompleteExpressDensePacket.startAngle() - expressDataLastStartAngle; // update the expressDataAngleDelta
          if(_incompleteExpressDensePacket.startAngle() < expressDataLastStartAngle) { expressDataAngleDelta += (360<<6); rotationCount++; } // angle rollover (only needed in one direction, becuase the lidar always spins in the same direction
          expressDataLastStartAngle = _incompleteExpressDensePacket.startAngle();
          packetCount++;
          rotationSpeedDt = micros() - rotationSpeedTimer;  rotationSpeedTimer = micros(); // for calculating rotation speed (quickly, not accurately)
          //if(motorHandlerPtr) { motorHandlerPtr->RPM_PID(uint32_t angleDiff, uint32_t microsDiff); } // TODO? (this will make motor PID dependent on packet checksum success and therefore potentially inconsistent)
        }
        uint8_t whereToStopParse = (sizeof(_incompleteExpressDensePacket)-_incompleteBytesRemaining-sizeof(_lidarExpressDataBase<4>)) / 2; // CHECK  0 to 40
        uint8_t whereToStartParse = (startReadOffset>sizeof(_lidarExpressDataBase<4>)) ? (startReadOffset-sizeof(_lidarExpressDataBase<4>)) : 0; // CHECK
        for(uint8_t i=whereToStartParse; i<whereToStopParse; i++) {
          uint16_t angle_q6 = _incompleteExpressDensePacket.startAngle() + (expressDataAngleDelta*i)/40; // angle = start + (speed*dt)/40 + extraDelta
          angle_q6 = angle_q6 % (360<<6); //if(angle > (360<<6)) { angle -= (360<<6); } // handle rollover
          bool startFlag = _incompleteExpressDensePacket.rotStartFlag() && (i == 0);
          uint16_t dist = _incompleteExpressDensePacket.dist(i); // express dense dist data is in mm directly (and can therefore theoretically measure things 65.536 meters away), but even the S1 model can only do 40m according to the page 12 of the protocol doc
          if((postParseCallback) && (includeInvalidMeasurements ? true : (dist > 0))) { postParseCallback(this, dist, angle_q6, startFlag, -1); }
        }
      }
    } else {
      #ifdef lidarDebugSerial
        lidarDebugSerial.print("rplidar: handleData() failed, scanResponseFormat unknown: ");lidarDebugSerial.println(scanResponseFormat,HEX); 
      #endif
      return(-1); /*if(motorHandlerPtr) { //stop motor }*/ }
    return(measurementsHandled);
  }

  void stopScan() {
    lidarCMD commandToSend(CMD_STOP, 0);
    _sendCommand(commandToSend); // send command (the STOP command triggers no response)
    //if(motorHandlerPtr) { //stop motor }
  }
  void resetLidar() {
    lidarCMD commandToSend(CMD_RESET, 0);
    _sendCommand(commandToSend); // send command (the STOP command triggers no response)
    //if(motorHandlerPtr) { //stop motor }
    delay(2); // according to page 15 of the protocol documentation, the lidar needs at least 2ms before it accepts another command
  }
  bool connectionCheck() {
    lidarRespDescr resp = _sendRequestNoPayload(CMD_GET_HEALTH);
    if((resp.dataType() == RESP_TYPE_GET_HEALTH) && (resp.responseLength() == sizeof(lidarGetHealthResponse))) {
      lidarSerial.setTimeout(slowSerialTimout);
      lidarGetHealthResponse returnVal;
      size_t bytesRead = lidarSerial.readBytes(&returnVal, sizeof(returnVal));
      return(bytesRead == sizeof(returnVal));
    } else { return(false); }
  }

  uint32_t rawAnglePerMillisecond() { // i strongly recommend implementing your own RPM measurement system if you want accurate data, this is really more of a rough indication
    if(rotationSpeedDt) { // avoid divide by 0
      uint32_t angleDiff = expressDataAngleDelta * 1000;
      return(angleDiff / rotationSpeedDt);
    } else { return(0); }
  }
  float RPM() { // i strongly recommend implementing your own RPM measurement system if you want accurate data, this is really more of a rough indication
    if(rotationSpeedDt) { // avoid divide by 0
      float RPM = expressDataAngleDelta; // angle_q6
      RPM /= rotationSpeedDt; // angle_q6 per micros
      RPM *= (1000000*60)/(360<<6); // angle_q6/micros to RPM
      return(RPM);
    } else { return(0); }
  }
};

#endif
