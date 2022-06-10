
#if defined(ILI9341displayAdafruit) || defined(ILI9341displayTFT_eSPI) //shared TFT values
  #ifdef ILI9341displayAdafruit
    #undef ILI9341displayTFT_eSPI // prefer ILI9341displayAdafruit
  #endif
  //#define UNZOOMVAR 5.0 // (constant) mm per pixel
  float UNZOOMVAR = 5.0; // (variable) mm per pixel
  #define UNZOOMVARpotPin 34 // potentiometer pin for UNZOOMVAR
  
  #define TFT_ROTATION 3
  //#define tft_frame_wait
  #define drawPointCircles 1 //draw circles on the datapoints (as well as lines) with the defined radius
  #define SCREENWIDTH 320
  #define SCREENHEIGHT 240
  #define BG_COLOR 0xC618 //grey
  #define DATA_POINT_COLOR 0xF800 //red  (only used if drawPointCircles is defined)
  #define DATA_LINE_COLOR 0xFFE0 //yellow
  //#define COMPAREDATA_COLOR 0x07E0 //green
  #define CENTER_COLOR 0x001F //blue
  #define TFT_CS   14  // Chip select control pin
  #define TFT_DC   13  // Data Command control pin
  #define SPI_PORT_TO_USE VSPI // can be mapped to any pins, but for max (80MHz) speed, needs default pins: 18,19,23 for VSPI, 14,12,13 for HSPI    (note: this is old code, not sure adafruit_display accepts the change)
  // tie VCC, RESET and LED to 3.3V
  #define SPI_FREQ_TO_USE 20000000 // ILI9341 can do 40MHz confortably (if your wires are worthy!), and 80MHz sometimes (silicon lottery?)
  
  //const int16_t screenMid[2] = {SCREENWIDTH/2, SCREENHEIGHT/2};
  const int16_t screenMid[2] = {SCREENHEIGHT/2, SCREENHEIGHT/2};
  const uint16_t _diagonalPixels = sqrt(screenMid[0]*screenMid[0] + screenMid[1]*screenMid[1]); // used to calculate maxDrawDist (only for ILI9341displayTFT_eSPI)
  int16_t lastPoint[3]; //holds (pixel) pos of last point to draw line from (and the index of that point)

  #define distAngleToPos(angle, dist, output)  output[0]=cos(angle)*(float)dist; output[1]=sin(angle)*(float)dist
  
  #define convertToPixelPoint(floatPos, output) output[0]=(screenMid[0] + ((int16_t)(floatPos[0]/UNZOOMVAR))); output[1]=(uint16_t)(screenMid[1] + ((int16_t)(floatPos[1]/UNZOOMVAR)))
  
//  #elif defined(ZOOMVAR)
//    #define convertToPixelPoint(floatPos, output) output[0]=(screenMid[0] + ((int16_t)(floatPos[0]*ZOOMVAR))); output[1]=(uint16_t)(screenMid[1] + ((int16_t)(floatPos[1]*ZOOMVAR)))
//  #else //no zoomvar set, attempt to display whole range
//    #warning("TBD: determine universal UNZOOMVAR")
//    #define convertToPixelPoint(floatPos, output) output[0]=(screenMid[0] + ((int16_t)floatPos[0])); output[1]=(uint16_t)(screenMid[1] + ((int16_t)floatPos[1]))
//  #endif

  // individual setup things:
  #ifdef ILI9341displayTFT_eSPI
    //this library has cool DMA features, but it's pinout is pretty shit;
    //you have to edit the User_Setup.h file in the library folder to change pinout
    #include <TFT_eSPI.h>
    TFT_eSPI tft = TFT_eSPI();       // Invoke custom library
  
    TFT_eSprite displayBuffer = TFT_eSprite(&tft);
    uint16_t* displayBufferPointer;
  #else // ILI9341displayAdafruit
    #include <SPI.h>
    #include <Adafruit_GFX.h>
    #include <Adafruit_ILI9341.h>
    //SPIClass SPIport = SPIClass(SPI_PORT_TO_USE); //to be implemented, doesnt quite work right now
    //Adafruit_ILI9341 tft = Adafruit_ILI9341(&SPIport, TFT_CS, TFT_DC);
    ////Adafruit_ILI9341 tft = Adafruit_ILI9341(&SPIport, TFT_CS, TFT_DC, TFT_RST);
    Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC); //using V_SPI, pins 18,19,23
    //Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST); //using V_SPI, pins 18,19,23
  #endif

  // the core function
  void drawLidarDataOnILI9341(void *arg) {
    disableCore0WDT(); //screw that watchdog timer
    RPlidar* lidarPointer = (RPlidar*) arg; //retrieve lidar pointer from the passed argument

    #ifdef ILI9341displayTFT_eSPI
      setup_t tftSetup;  tft.getSetup(tftSetup);
      if((tftSetup.pin_tft_mosi != 23) || (tftSetup.pin_tft_clk != 18) || (tftSetup.pin_tft_cs != TFT_CS) || (tftSetup.pin_tft_dc != TFT_DC) || (tftSetup.port != SPI_PORT_TO_USE) || (tftSetup.tft_spi_freq != SPI_FREQ_TO_USE)) 
      { log_e("pinout mismatch, edit User_Setup.h in TFT_eSPI library"); 
        log_e("%d, %d, %d, %d, %d, %u", tftSetup.pin_tft_mosi, tftSetup.pin_tft_clk, tftSetup.pin_tft_cs, tftSetup.pin_tft_dc, tftSetup.port, tftSetup.tft_spi_freq);   }
      tft.begin();
      #ifdef TFT_ROTATION
        tft.setRotation(TFT_ROTATION);
      #endif
      tft.fillScreen(BG_COLOR);
      tft.initDMA();
//      log_v("getFreeHeap before displaybuffer: %d", ESP.getFreeHeap());
      displayBufferPointer = (uint16_t*) displayBuffer.createSprite(SCREENHEIGHT, SCREENHEIGHT);
//      log_v("getFreeHeap after displaybuffer: %d", ESP.getFreeHeap());
      tft.startWrite(); //start writing stuff (and never really stop
    #else // ILI9341displayAdafruit
      tft.begin(SPI_FREQ_TO_USE);
      #ifdef TFT_ROTATION
        tft.setRotation(TFT_ROTATION);
      #endif
      tft.fillScreen(BG_COLOR);
  //    log_v("getFreeHeap before displaybuffer: %d", ESP.getFreeHeap());
      GFXcanvas16 displayBuffer = GFXcanvas16(SCREENHEIGHT, SCREENHEIGHT); //due to ESP32's max single allocation and 240*320*2 being larger than that, we can just do this
  //    log_v("getFreeHeap after displaybuffer: %d", ESP.getFreeHeap());
    #endif
    #if !defined(UNZOOMVAR) && defined(UNZOOMVARpotPin)  // if UNZOOMVAR is a variable (float)
      pinMode(UNZOOMVARpotPin, INPUT);
    #endif
    //setup complete, loop from here

    bool drawLineBool = true;
    uint32_t speedMes[4];
    while(1) {
      #ifndef UNZOOMVAR // if UNZOOMVAR is a variable (float)
        UNZOOMVAR = 1.0 + (analogRead(UNZOOMVARpotPin) / 4096.0) * 200.0;
      #endif
      #ifdef ILI9341displayTFT_eSPI
        uint16_t maxDrawDist = _diagonalPixels * UNZOOMVAR; // the (mm) distance to the corner of the screen
      #endif
      speedMes[0] = micros();
      displayBuffer.fillScreen(BG_COLOR);
      displayBuffer.fillCircle(screenMid[0], screenMid[1], 2, CENTER_COLOR);
      for(uint16_t i=0; i<(dataSetResPerDegree*360); i++) {
        while(lidarDataSet.dataArrayWriting) {} //wait
        //lidarDataSet.dataArrayWriting = true; //for ultimate safety
        uint16_t dist = lidarDataSet.distArray[i];  int8_t quality = lidarDataSet.qualityArray[i];  
        //lidarDataSet.dataArrayWriting = false; //for ultimate safety
        if((dist > 0) 
        #ifdef ILI9341displayTFT_eSPI
          && (dist <= maxDrawDist)   // adafruit's library is better at handling off-screen drawing
        #endif
        ) {
          float angle = ((float)i * PI) / (180*dataSetResPerDegree); // convert to radians (and save yourself one division by incorporating it into the conversion)
          float floatPos[2];  distAngleToPos(angle, dist, floatPos); //calculate position given angle and distance (float floatPos[2])
          int16_t pixelPoint[2];  convertToPixelPoint(floatPos, pixelPoint); //convert to pixel using (UN)ZOOMVAR
          //if(((pixelPoint[0] < 0) || (pixelPoint[0] > SCREENHEIGHT)) || ((pixelPoint[1] < 0) || (pixelPoint[1] > SCREENHEIGHT))) { continue; } // skip any data that isn't inside the display area
          #ifdef ILI9341displayTFT_eSPI
            pixelPoint[0] = constrain(pixelPoint[0], 0, SCREENHEIGHT-1);  pixelPoint[1] = constrain(pixelPoint[1], 0, SCREENHEIGHT-1); // the ILI9341displayTFT_eSPI canvas type can't handle out-of-bounds pixel locations
          #endif
          displayBuffer.drawLine(lastPoint[0], lastPoint[1], pixelPoint[0], pixelPoint[1], DATA_LINE_COLOR);
          lastPoint[0] = pixelPoint[0];  lastPoint[1] = pixelPoint[1];  lastPoint[2] = i; //store last point
          #ifdef drawPointCircles
//            if(quality >= 0) { // if there is actually any quality data
//              uint8_t qualitySixBits = (quality<<2) & 0b00111111; // quality is 6 bits, but only the last 4 bits appear to contain data(?)
//              uint16_t pointColor = (uint16_t)qualitySixBits << 5; // the color is a 16bit value comprised of 5 red, 6 green and 5 blue bits (in that order). This will map the quality between black and green
//              displayBuffer.fillCircle(pixelPoint[0], pixelPoint[1], drawPointCircles, pointColor);
//            } else {
              displayBuffer.fillCircle(pixelPoint[0], pixelPoint[1], drawPointCircles, DATA_POINT_COLOR);
//            }
          #endif
        }
      }
      speedMes[1] = micros();
      #ifdef ILI9341displayTFT_eSPI
        tft.pushImageDMA((SCREENWIDTH-SCREENHEIGHT)/2, 0, SCREENHEIGHT, SCREENHEIGHT, displayBufferPointer);
      #else // ILI9341displayAdafruit
        //tft.drawRGBBitmap(0, 0, displayBuffer.getBuffer(), SCREENWIDTH, SCREENHEIGHT); // draws the square frame off-center
        tft.drawRGBBitmap((SCREENWIDTH-SCREENHEIGHT)/2, 0, displayBuffer.getBuffer(), SCREENHEIGHT, SCREENHEIGHT); // draws the square frame in the middle of the display
      #endif
      speedMes[2] = micros();
      //Serial.print("ILI9431 speedMes: "); Serial.print(speedMes[1]-speedMes[0]); Serial.write(' '); Serial.print(speedMes[2]-speedMes[1]); Serial.write(' '); Serial.println(speedMes[2]-speedMes[3]);
      speedMes[3] = speedMes[2]; // looptime
      #ifdef tft_frame_wait
        uint32_t threeSixtyPeriod=200000; //if(lidarPointer->RPMraw > 0) { threeSixtyPeriod=3840000000UL/lidarPointer->RPMraw; }
        if((micros()-speedMes[0]) < threeSixtyPeriod) { //if drawing the frame took less time than 1 rotation (it should)
          uint32_t oldRotationCount = lidarPointer->rotationCount;
          while(oldRotationCount == lidarPointer->rotationCount) { //while waiting for the 
            delay(1); 
            while(lidarPointer->dataArrayWriting) {} //wait
            //do something fun here
          }
        }
      #endif
      #ifdef ILI9341displayTFT_eSPI
        tft.dmaWait(); // wait until DMA is complete (if it's still going). if tft_frame_wait is defined, this should never take time
      #endif
    }
    #ifdef ILI9341displayTFT_eSPI
      tft.endWrite(); //will never happen, because of while(1) loop
    #endif
  }
#endif // both SPI displays

#ifdef DACdisplay
  #define UNZOOMVAR 200.0 // (constant) mm per pixel
  void drawLidarDataOnDACforXYplot(void *arg) {
    disableCore0WDT(); //screw that watchdog timer
    RPlidar* lidarPointer = (RPlidar*) arg; //retrieve lidar pointer from the passed argument
    //i could've used the global variable, but this is more fun
    pinMode(25, OUTPUT);   pinMode(26, OUTPUT);
    while(1) {
      for(uint16_t i=0; i<(dataSetResPerDegree*360); i++) {
        while(lidarDataSet.dataArrayWriting) {} //wait
        //lidarDataSet.dataArrayWriting = true; //for ultimate safety
        float dist = lidarDataSet.distArray[i];
        //lidarDataSet.dataArrayWriting = false; //for ultimate safety
        //if(dist > 0) { // not needed(?)
        float angle = ((float)i * PI) / (180*dataSetResPerDegree); // convert to radians (and save yourself one division by incorporating it into the conversion)
        float xPoint = cos(angle) * dist;
        float yPoint = sin(angle) * dist;
        dacWrite(25, (uint8_t) 127 + ((int8_t) constrain(xPoint*127.0/UNZOOMVAR, -127, 127)));
        dacWrite(26, (uint8_t) 127 + ((int8_t) constrain(yPoint*127.0/UNZOOMVAR, -127, 127)));
      }
    }
  }
#endif

#ifdef compositedisplay
  #error("compositedisplay is TBD");
//  #define UNZOOMVAR 20.0
//  const int XRES = 320;
//  const int YRES = 200;
//  const int16_t screenMid[2] = {XRES/2, YRES/2};
//  
//  CompositeOutput composite(CompositeOutput::NTSC, XRES * 2, YRES * 2);
//  uint8_t **frame; uint8_t **backbuffer;
//
//  void fillScreen(uint8_t val) {
//    for(int y = 0; y < yres; y++)
//      for(int x = 0; x < xres; x++)
//        backbuffer[y][x] = val;
//  }
//  void frameBufSwitch() {
//    uint8_t **temp = backbuffer;
//    backbuffer = frame;
//    frame = temp;
//  }
//
//  void drawLidarDataOnComposite(void *arg) {
//    disableCore0WDT(); //screw that watchdog timer
//    HLS_LFCD<HardwareSerial>* lidarPointer = (HLS_LFCD<HardwareSerial>*) arg; //retrieve lidar pointer from the passed argument
//    //i could've used the global variable, but this is more fun
//    composite.init();
//    //pinMode(25, OUTPUT);
//    frame = (uint8_t**)malloc(YRES * sizeof(uint8_t*));  backbuffer = (uint8_t**)malloc(yres * sizeof(uint8_t*));
//    for(int y = 0; y < YRES; y++) 
//    { frame[y] = (uint8_t*)malloc(XRES);  backbuffer[y] = (uint8_t*)malloc(xres); }
//    while(1) {
//      for(uint16_t i=0; i<360; i++) {
//        //while(lidarPointer->dataArrayWriting) {} //cant wait, gotta hurry
//        uint16_t dataPoint = lidarPointer->dataArray[i][1]; //only retrieve distance, other parameter ([0]) is wack
//        
//        float dist = dataPoint; //convert uint16_t to float for exact math (frick speed, right)
//        float angle = radians(i);
//        float xPoint = cos(angle) * dist;
//        float yPoint = sin(angle) * dist;
//      }
//      frameBufSwitch();
//
//      composite.sendFrameHalfResolution(&frame);
//    }
//  }
#endif

void rplidar_display_init() {
  #if defined(ILI9341displayAdafruit) || defined(ILI9341displayTFT_eSPI)
    xTaskCreatePinnedToCore(drawLidarDataOnILI9341, "ILI9341plot", 4096, &lidar, 1, NULL, 0); //start drawLidarDataOnILI9341() on core 0 and pass the address of the lidar
  #elif defined(DACdisplay)
    xTaskCreatePinnedToCore(drawLidarDataOnDACforXYplot, "DACplot", 4096, &lidar, 1, NULL, 0); //start drawLidarDataOnDACforXYplot() on core 0 and pass the address of the lidar
//  #elif defined(compositedisplay)
//    xTaskCreatePinnedToCore(drawLidarDataOnComposite, "compositeplot", 4096, &lidar, 1, NULL, 0); //start drawLidarDataOnComposite() on core 0 and pass the address of the lidar
  #else
    #warning("no display method defined")
  #endif
}
