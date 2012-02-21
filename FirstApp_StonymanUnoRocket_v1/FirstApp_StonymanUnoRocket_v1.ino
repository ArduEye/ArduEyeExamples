/* FIRSTAPP_STONYMANUNOROCKET_V1

This is an example "first application" written for the ArduEye system using
a StonymanUnoRocket base shield and a Stonyman image sensor chip mounted on
a Stonyman breakout board. This sketch should be used with the related tutorial
on the www.ardueye.com wiki.

This sketch allows a number of different concepts to be explored, including
operaating a Stonyman image sensor, acquiring an image, compensating for 
fixed pattern noise, tracking lights, and performing both 1D and 2D optical
flow. 

Started Feb 15, 2012
*/

/*
===============================================================================
Copyright (c) 2012 Centeye, Inc. 
All rights reserved.

Redistribution and use in source and binary forms, with or without 
modification, are permitted provided that the following conditions are met:

    Redistributions of source code must retain the above copyright notice, 
    this list of conditions and the following disclaimer.
    
    Redistributions in binary form must reproduce the above copyright notice, 
    this list of conditions and the following disclaimer in the documentation 
    and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY CENTEYE, INC. ``AS IS'' AND ANY EXPRESS OR 
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO 
EVENT SHALL CENTEYE, INC. OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY 
OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are 
those of the authors and should not be interpreted as representing official 
policies, either expressed or implied, of Centeye, Inc.
===============================================================================
*/

/*
Note on image arrays: To save space and speed up operations, we store all images,
whether 1D or 2D, in a 1D array. The pixels are stored row-wise. So suppose
array A holds a 16x16 image. Then values A[0] through A[15] store the first row
of pixel values, values A[16] through A[31] the second row of pixel values, and
so on.
*/

// USING_MEGA: IMPORTANT!!!
// This flag determines whether support for 2D optical flow
// is included in this sketch. With all the other functions
// being demonstrated with this sketch, implementing 2D optical 
// flow requires enough memory that it will overflow a "regular"
// Arduino UNO sized board. If you want to explore 2D optical flow
// and you will be using an Arduino MEGA, then you may set
// this parameter to 1 (one). Otherwise set it to 0 (zero).
//
// Note that you will be able to get 2D optical flow working on 
// an UNO if you remove other variables from the sketch.
#define USING_MEGA 1

//=============================================================================
// INCLUDE FILES. The top four files are part of the ArduEye library and should
// be included in the Arduino "libraries" folder.
#include <SMH1_StonymanHawksbill_v1.h>
#include <CYE_Images_v1.h>
#include <CYE_OpticalFlowOdometry_v1.h>
#include <AEYE_ArduEyeGUI_v1.h>
#include <SPI.h>
#include <EEPROM.h>

//==============================================================================
// GLOBAL VARIABLES

// These two variables define an array of characters used for ASCII
// dumps of images to the Arduino serial monitor. This is a different set
// of ASCII characters than what is in the CYE_Images_v1 library since
// this has fewer distinct characters and accentuates exposed areas
char MY_ASCII_DISP_CHARS[16] = "###@%*=o-,..   ";
char MY_NUM_ASCII_DISP_CHARS = 15;

// WINDOW_SIZE = Size of pinhole image window in pixels (across, high)
#define WINDOWSIZE 16

// NUM_PIXELS = number of pixels in 16x16 window
#define NUMPIXELS (WINDOWSIZE*WINDOWSIZE)

// LINE_IMG_SIZE = size of line image for computing 1D OFs
#define LINE_IMG_SIZE  8

// LINE_IMG_PIXLENGTH = length of pixels for computing 1D images
#define LINE_IMG_PIXLENGTH  8

// SUBWINDOW_SIZE = size of subwindow, for 2D optical flow (MEGA only)
#if USING_MEGA==1
#define SUBWINDOW_SIZE  8
#endif

// SUBWINDOW_NUMPIXLES = number of pixels in the subwindow
#define SUBWINDOW_NUMPIXELS (SUBWINDOW_SIZE*SUBWINDOW_SIZE)

// EEPROM locations for calibration info
#define PINHOLE_LOCATION_EEPROM  0
#define FPN_BASE_EEPROM  10
#define FPN_MASK_EEPROM  20

// dumpType: determine what is dumped to serial terminal every frame
unsigned char dumpType;

// Row and column of pinhole
unsigned char mPinhole,nPinhole; 

// 16x16 Image Array - recall from note above that image arrays are stored
// row-size in a 1D array
short A[NUMPIXELS]; // current 16x16 image. 

// FPN calibration. To save memory we are placing the FPN calibration
// array in C, since the variation between pixels may be expressed with 
// a single byte. Variable Cbase holds an offset value applied to the
// entire FPN array. Thus the FPN mask for pixel i will be the 
// value C[i]+Cbase.
unsigned char C[NUMPIXELS]; // 16x16 FPN calibration image
short Cbase; // FPN calibration image base.

// Line images for 1D optical flow. 
short S1[LINE_IMG_SIZE]; // line image from previous frame
short S2[LINE_IMG_SIZE]; // line image from current frame
char Sorientation; // determines if the line image is vertical or horizontal

// Subwindows: These are 8x8 images used for computing 2D optical flow
#if USING_MEGA==1
short W1[SUBWINDOW_NUMPIXELS]; // previous frame
short W2[SUBWINDOW_NUMPIXELS]; // current frame
#endif

// charbuf: character buffer for generating outputs
char charbuf[30];

// Command inputs - for receiving commands from user via Serial terminal
char command; // command character
int commandArgument; // argument of command

// Sensor mode
char sensormode; // 0=quiet (default), 1=1D optical flow, 2=2D optical flow, 3=lights

// Light thresholds- these thresholds are used for light tracking and determine whether a locally bright
// pixel is identified as a light
short shapeConvexityThreshold; // convexity threshold- pixel must be brigher than neighbors, on average, by this amount
short intensityThreshold; // simple intensity threshold- pixel must be brighter than this

// Light related variables- this code stores a maximum of 10 light locations
#define MAXLIGHTS 10
short lightM[MAXLIGHTS],lightN[MAXLIGHTS]; // light locations. M is for rows and N is for columns
short lightI[MAXLIGHTS]; // light intensity
short lightC[MAXLIGHTS]; // light convexity
char numActiveLights; // number of lights in current frame
   
// Hyperacuity
char useHyperacuity = 0; // Whether or not to use hyperacuity to boost light tracking accuracy by 10

// 1D optical flow values
int OFlinear; // optical flow at current frame
int ODOlinearacc; // accumulated odometry

// 2D optical flow values
#if USING_MEGA==1
float OFx,OFy;
int ODOx,ODOy;
#endif

// GUI object- these are used when the ArduEye is connected to the ArduEyeGUI program 
// (written in Processing)
ArduEyeGUI GUI;
short GUIoffset;
byte bytebuf[20];

// Miscellaneous
char ping = 0;
char counter; // frame counter
char startingup; // set to 1 initially; cleared after first iteration of loop()

//=======================================================================
// FUNCTIONS DEFINED FOR THIS SKETCH

//-----------------------------------------------------------------------
// DISPWHOLESTONYMANCHIP -- Displays the entire Stonyman chip on the screen
// using ASCII rendering. input variable "anain" determines which analog port
// the Stonyman is connected to.
void DispWholeStonymanChip(char anain) {
  unsigned char row,col;
  short val,maximum,minimum;
  
  maximum=0; // initialize
  minimum=20000; // initialize
  
  SMH1_ConfigureAnalogInput(anain); // Make sure analog pin is configured as input	
  
  // FIRST PASS
  // First compute minimum and maximum of chip
  SMH1_SetPtrValSlow(SMH_SYS_ROWSEL,0); // set row = 0
  for (row=0; row<112; ++row) {
    SMH1_SetPtrValSlow(SMH_SYS_COLSEL,0); // set column = 0
    for (col=0; col<112; ++col) {
      // settling delay
      delayMicroseconds(1);
      // get data value
      delayMicroseconds(1);
      val = analogRead(anain);
      // increment column
      SMH1_IncvSlow(1);
      // stats - we wish to exclude ring of outer 10 pixels from stats
      // since they can be substantially different due to edge-of-array
      // electric field effects
      if (row>10 && row<102 && col>10 && col<102) {
        if (val>maximum)
          maximum=val;
        if (val<minimum)
          minimum=val;
      }
    }
    SMH1_SetPtrSlow(SMH_SYS_ROWSEL); // point to row
    SMH1_IncvSlow(1); // increment row
  }
  Serial.println(maximum);
  Serial.println(minimum);
  
  // SECOND PASS
  // Display image
  SMH1_SetPtrValSlow(SMH_SYS_ROWSEL,0); // set row = 0
  for (row=0; row<112; ++row) {
    SMH1_SetPtrValSlow(SMH_SYS_COLSEL,0); // set column = 0
    Serial.print("[");
    for (col=0; col<112; ++col) {
      // settling delay
      delayMicroseconds(1);
      // get data value
      delayMicroseconds(1);
      val = analogRead(anain);
      // increment column
      SMH1_IncvSlow(1);
      val = val - minimum;
      val = val * MY_NUM_ASCII_DISP_CHARS / (maximum-minimum+1);
      if (val<0)
        val=0;
      if (val>MY_NUM_ASCII_DISP_CHARS)
        val = MY_NUM_ASCII_DISP_CHARS;
      Serial.print(MY_ASCII_DISP_CHARS[val]);
    }
    Serial.println("]");
    SMH1_SetPtrSlow(SMH_SYS_ROWSEL); // point to row
    SMH1_IncvSlow(1); // increment row
  }
}

//-----------------------------------------------------------------------
// HyperMax2
// Hyperacuity light position function. b is the intensity of a "brightest" point,
// while a and c are the intensities of points to the left and right. Sets up a
// 2nd order LaGrange polynomial through these points, and computes the maximum
// value of the polynomial. The value thus adjusts the point location by a fraction
// of a pixel. For this function the value returned is in tenths of a pixel, and
// will generally vary from -5 to 5. e.g. -5 means that the actual location is 1/2
// a pixel in the "a" direction, while +5 means that the actual location is 1/2
// a pixel in the "c" direction. 
short HyperMax2(short a, short b, short c) {
  short top = (c-a)*10;
  short bottom = 2*(2*b - (c+a));
  return top/bottom;
}

//-----------------------------------------------------------------------
// GetUserCommand
// Users may send simple commands via the Arduino's serial monitor using
// the format "X#", where "X" is a single character and "#" is an optional
// number. The number "#" may be omitted, or may be one or more digits 
// in size. The extracted command character and number argument are returned
// via pointers.
void GetUserCommand(char *command, int *argument) {
  char cmdbuf[11];
  unsigned char i;
  
  // initialize
  for (i=0; i<11; ++i)
    cmdbuf[i] = 0;
  i = 0;
  // delay to ensure that all stuff is sent through serial port
  delay(100);
  // load cmdbuf
  while (Serial.available() && i<10) {
    cmdbuf[i] = Serial.read();
    i++;
  }
  // clear end of array
  cmdbuf[10]=0;
  // clear rest of buffer
  while (Serial.available())
    ;
  // get command
  *command = cmdbuf[0];
  // get argument
  sscanf(cmdbuf+1,"%d",argument);
}

//-----------------------------------------------------------------------
// dumpCommandList - Dumps a list of commands to the serial monitor to
// help the user out. The delay is used to avoid overflowing the serial
// port. Note that it seems that the Arduino IDE stores these character
// strings in SRAM, so the more extensive the description, the less 
// memory is available for variables. You can free up memory by
// removing this function.
#define DCDL 50
void dumpCommandList() {
  Serial.println("0: quiet"); delay(DCDL);
  Serial.println("1: 1D"); delay(DCDL);
#if USING_MEGA==1
  Serial.println("2: 2D"); delay(DCDL);
#endif
  Serial.println("3: lights"); delay(DCDL);
  Serial.println("a: ASCII win"); delay(DCDL);
  Serial.println("A: ASCII chip"); delay(DCDL);
  Serial.println("b: reset"); delay(DCDL);
  Serial.println("c: calib"); delay(DCDL);
  Serial.println("d#: orient 1/2"); delay(DCDL);
  Serial.println("e: res ODO"); delay(DCDL);
  Serial.println("g#: GUIoffset"); delay(DCDL);
  Serial.println("h#: 1/0 hyp"); delay(DCDL);
  Serial.println("i: ping"); delay(DCDL);
  Serial.println("m#: PH row=#"); delay(DCDL);
  Serial.println("n#: PH col=#"); delay(DCDL);
  Serial.println("o#: outlev=#"); delay(DCDL);
  Serial.println("p: find PH"); delay(DCDL);
  Serial.println("q: calib->EEPROM");
  Serial.println("r: calib<-EEPROM");
  Serial.println("s#: convex th=#"); delay(DCDL);
  Serial.println("t#: intens th=#"); delay(DCDL);
  Serial.println("u: dump"); delay(DCDL);
  Serial.println("z: MATLAB win"); delay(DCDL);  
  Serial.println("Z: MATLAB chip"); delay(DCDL);
  Serial.println("!#: GUI on/off"); delay(DCDL);
}

//------------------------------------------------------------------------
// RESETCHIP - Configures the Arduino to talk to the Stonyman vision chip
// and initializes and turns on the chip. If you unplug the Stonyman chip
// from the Arduino and plug in a new one, you need to call this function
// to reset the chip. 
void resetchip() {
  // This function connects the Arduino to the Stonyman chip
  SMH1_SetupIO(); // Connect Arduino to Stonyman chip
  // These functions initialize the Stonyman chip
  SMH1_ClearAllValsSlow(); // Clear Stonyman chip register values
  SMH1_SetPtrValSlow(SMH_SYS_VREF,50); // VREF bias for 5V
  SMH1_SetPtrValSlow(SMH_SYS_NBIAS,55); // NBIAS bias for 5V
  SMH1_SetPtrValSlow(SMH_SYS_AOBIAS,55); // AOBIAS bias for 5V
  SMH1_SetPtrValSlow(SMH_SYS_CONFIG,16); // Configure for raw pixels (no amplification) and turn chip on
}

//=======================================================================
// ARDUINO SETUP AND LOOP FUNCTIONS

void setup() {
  char w;
  
  startingup=1;
  
  // Initialize the Stonyman chip
  resetchip();
  
  // Begin serial port
  Serial.begin(115200); // you can use the default 9600 bit rate, but it will be ssssslllllooooowwwwwww

  // Arduino digital pins 2 and 3 may be used for debugging and time measurement- you can turn these on or off
  // throughout the program to measure, with an oscilloscope, how long a certain function takes.  Note that
  // digitalWrite() functions take about a microsecond themselves to execute.
  pinMode(2,OUTPUT); // for debugging- digital outputs 2 and 3 may be pulsed etc to take time measurements
  pinMode(3,OUTPUT);

  // Sensormode
  sensormode = 0; // Initially quiet mode
  
  // Initialize thresholds for light tracking
  shapeConvexityThreshold = 7;
  intensityThreshold = 0;
  
  // As an estimate, set initial pinhole location to 55,55
  mPinhole=55;nPinhole=55;
  
  // GUIoffset value- special parameter.
  GUIoffset=40;
  
  // initialize 1D OF and odometry
  ODOlinearacc=0;
  Sorientation=1; // default horizontal pixels
  
  // initialize 2D odometry values
#if USING_MEGA==1
  ODOx=ODOy=0;
#endif
}

void loop() {

  short i,j,k,r,c,m,n;
  
  digitalWrite(3,HIGH);
  
  // PROCESS USER COMMANDS, IF ANY
  if (Serial.available()>0) { // Check Serial buffer for input from user
  
    GetUserCommand(&command,&commandArgument); // get user command and argument
    
    // Biiiiig switch statement to process commands
    switch (command) {
      
      // Put sensor in mode 0 e.g. quiet
      case '0':
        sensormode = 0;
        Serial.println("Mode 0");
        break;
        
      // Put sensor in mode 1 e.g. 1D optical flow
      case '1':
        sensormode = 1;
        Serial.println("Mode 1");
        break;
        
      // Put sensor in mode 2 e.g. 2D optical flow
      case '2':
        sensormode = 2;
        Serial.println("Mode 2");
        break;
        
      // Put sensor in mode 3 e.g. light tracking mode
      case '3':
        sensormode = 3;
        Serial.println("Mode 3");
        break;  
  
      // ASCII dump the 16x16 image A      
      case 'a':
        CYE_ImgShortDumpAsciiSerial(A,WINDOWSIZE,WINDOWSIZE,0,0);
        break;
        
      // ASCII dump the entire Stonyman chip
      case 'A':
        DispWholeStonymanChip(0);
        break;
        
      // Reset the chip - use this command if you plug in a new Stonyman chip w/o power cycling
      case 'b':
        resetchip();
        Serial.println("RES");
        break;
        
      // Grab FPN calibration mask. 
      // Note: this should be performed after the pinhole location is changed, using commands
      // p, m, or n. This should be performed only when the sensor is exposed to a uniform texture
      // e.g. covered with a white sheet of paper or looking at a white wall for example.
      case 'c': // grab calibration mask
        SMH1_GetImageSlow(A,mPinhole-WINDOWSIZE/2,WINDOWSIZE,1,nPinhole-WINDOWSIZE/2,WINDOWSIZE,1,0,0);
        // find minimum
        Cbase = 10000; // e.g. "high"
        for (i=0; i<NUMPIXELS; ++i)
          if (A[i]<Cbase)
            Cbase = A[i];
        // generate calibration mask
        for (i=0; i<NUMPIXELS; ++i)
          C[i] = A[i] - Cbase;
        Serial.println("Cal done");
        break;
        
      // Set orientation of pixels for 1D optical flow computation
      // 1 = horizontal, 2 = vertical
      case 'd': 
        Sorientation = commandArgument;
        if (Sorientation<1 || Sorientation>2)
          Sorientation=1;
        Serial.print("Orient=");
        Serial.println(Sorientation);
        break;
        
      // Reset optical flow odometry values to zero
      case 'e': // reset odometry
        ODOlinearacc = 0;
#if USING_MEGA==1
        ODOx=ODOy=0;
#endif
        break;
        
      // Modify GUI offset value
      case 'g':
        GUIoffset = commandArgument;
        break;
        
      // Hyperacuity for light tracking- turn on (one) or off (zero)
      case 'h':
        if (commandArgument) {
          useHyperacuity=1;
          Serial.println("Hyp on");
        } else {
          useHyperacuity=0;
          Serial.println("Hyp off");
        }          
        break;
        
      // For debugging (legacy)
      case 'i':
        ping = 1;
        break;
        
      // Set pinhole row
      case 'm': // set pinhole row
        mPinhole = commandArgument;
        sprintf(charbuf,"PHrow=%d",mPinhole);
        Serial.println(charbuf);     
        break;   
        
      // Set pinhole column
      case 'n': // set pinhole column
        nPinhole = commandArgument;
        sprintf(charbuf,"PHcol=%d",nPinhole);
        Serial.println(charbuf); 
        break;
        
      // Output dump type- for debugging
      case 'o': // set output dump type
        dumpType = commandArgument;
        Serial.print("outtyp=");
        Serial.println((int)dumpType); 
        break;         
        
      // find pinhole- hold a flashlight or other bright light above sensor and then send
      // this command. Note you will need to recalibrate the sensor afterwards.
      case 'p': 
        Serial.println("Find PH...");
        SMH1_FindMaxSlow(30,50,30,50,0,&mPinhole,&nPinhole);
        sprintf(charbuf,"...PH at %d,%d",mPinhole,nPinhole);
        Serial.println(charbuf);
        delay(2000);
        break;
        
      // Save pinhole location and FPN calibration mask to EEPROM
      case 'q':
        Serial.println("Calib -> EEPROM...");      
        EEPROM.write(PINHOLE_LOCATION_EEPROM,mPinhole);
        EEPROM.write(PINHOLE_LOCATION_EEPROM+1,nPinhole);
        EEPROM.write(FPN_BASE_EEPROM,(Cbase>>8)&0xFF);
        EEPROM.write(FPN_BASE_EEPROM+1,Cbase&0xFF);
        for (i=0; i<NUMPIXELS; ++i)
          EEPROM.write(FPN_MASK_EEPROM+i,C[i]);
        Serial.println("Done"); 
        break;       
       
      // read pinhole location and FPN calibration mask from EEPROM 
      case 'r': 
        Serial.println("Calib <- EEPROM");
        mPinhole = EEPROM.read(PINHOLE_LOCATION_EEPROM);
        nPinhole = EEPROM.read(PINHOLE_LOCATION_EEPROM+1);
        Cbase = EEPROM.read(FPN_BASE_EEPROM) << 8;
        Cbase += EEPROM.read(FPN_BASE_EEPROM+1);
        for (i=0; i<NUMPIXELS; ++i)
          C[i] = EEPROM.read(FPN_MASK_EEPROM+i); 
        Serial.println("Done");     
        break;
        
      // Set shapeConvexityThreshold (for light tracking)
      case 's':
        shapeConvexityThreshold = commandArgument;
        Serial.print("shape thresh = ");
        Serial.println((int)shapeConvexityThreshold); 
        break;     
     
     // Set intensityThreshold (for light tracking)
      case 't': 
        intensityThreshold = commandArgument;
        Serial.print("intens thresh = ");
        Serial.println((int)intensityThreshold); 
        break;         
        
      // Dump 16x16 image A in MATLAB format
      case 'z':
        CYE_ImgShortDumpMatlabSerial(A,WINDOWSIZE,WINDOWSIZE);
        break;
        
      // Dump entire Stonyman chip in MATLAB format
      case 'Z': 
        SMH1_MatlabDumpEntireChip(0,0,0);
        break;
        
      // Turns on or off GUI mode. USE ONLY WHEN CONNECTED TO PROCESSING GUI
      // Otherwise you'll get a lot of gobblygook on the serial monitor...
      case '!':
        if(commandArgument==0) {
          GUI.stop();
          Serial.println("GUI off");
        }
        if(commandArgument==1) {
          GUI.start();
          Serial.println("GUI on");
        }
        break;        
        
      // ? and default- print up command list
      case '?':
      default:
        dumpCommandList();
    }
  }
  digitalWrite(3,LOW);
   
  // ACQUIRE A 16x16 IMAGE FROM THE STONYMAN CHIP  
  digitalWrite(2,HIGH);
  SMH1_GetImageSlow(A,mPinhole-WINDOWSIZE/2,WINDOWSIZE,1,nPinhole-WINDOWSIZE/2,WINDOWSIZE,1,0,0); // about 44ms with 16x16 window
  //SMH1_GetImageFast(A,mPinhole-WINDOWSIZE/2,WINDOWSIZE,1,nPinhole-WINDOWSIZE/2,WINDOWSIZE,1,0,0); // about 30ms with 16x16 window
  // Subtract calibration mask
  for (i=0; i<NUMPIXELS; ++i) {
    A[i] -= Cbase + C[i];
    A[i] = -A[i];
  }
  digitalWrite(2,LOW);
  
  // PERFORM 1D OPTICAL FLOW COMPUTATIONS
  digitalWrite(2,HIGH);
  if (sensormode==1) {
    // Copy S2 to S1
    CYE_ImgShortCopy(S2,S1,8);
    // Extract a 1D image from image A- the inner 8x8 portion of A is converted into a line image
    // using rectangular superpixels, formed by summing the contents of each row (Sorientation=1) 
    // or column (Sorientation=2) of pixels. Store in S1
    CYE_SubwinShort2Dto1D(A,S2,WINDOWSIZE,WINDOWSIZE,5,5,LINE_IMG_SIZE,LINE_IMG_PIXLENGTH,Sorientation);
    // Compute 1D optical flow between S1 and S2 and store result in OFlinear
    CYE_OFO_ShortIIA_1D(S1,S2,8,100,&OFlinear);
    // Add to accumulation
    ODOlinearacc += OFlinear;   
    // if "starting up", then set OFlinear to zero (because it would be meaningless)
    if (startingup)
      OFlinear=0;
    // If dumping for debugging, dump line image values and optical flow
    if (dumpType>=1) {
      for (i=0; i<8; ++i) {
        Serial.print(S2[i]); 
        Serial.print(" ");
      }
      Serial.print(OFlinear); 
      Serial.print(" ");      
    }
    // Dump accumulation
    Serial.println(ODOlinearacc);
  }
  digitalWrite(2,LOW);
  
  // PERFORM 2D OPTICAL FLOW COMPUTATIONS
#if USING_MEGA==1
  if (sensormode==2) {
    // Copy W2 to W1
    CYE_ImgShortCopy(W2,W1,SUBWINDOW_NUMPIXELS);
    // Extract middle 8x8 array of pixels from A and store in W2
    CYE_SubwinShort2D(A,W2,WINDOWSIZE,WINDOWSIZE,5,8,5,8);
    // Compute optical flow between W1 and W2 and store in OFx and OFy
    CYE_OFO_ShortIIA_2D(W1,W2,8,8,&OFx,&OFy);
    // if "starting up" set OFx and OFy to zero
    if (startingup) {
      OFx=OFy=0;
    }
    // Add to accumulations
    ODOx += (int)OFx;
    ODOy += (int)OFy;
    // Print 2D optical flow values and odometries to Serial port
    sprintf(charbuf,"%5d %5d %5d %5d",(int)OFx,(int)OFy,ODOx,ODOy);
    Serial.println(charbuf);
  }
#endif  
  
  // PERFORM LIGHT TRACKING WITHIN ACQUIRED IMAGE A
  if (sensormode==3) {

    // If dumpType>=2 then details of the search will be dumped to serial monitor. Note
    // that this will significantly slow down the program
    numActiveLights=0;
    // pointer to "ego" pixel e.g. pixel point being checked to see if it is a light
    // it is initialized to the second row, second column pixel.
    short *pego = A+WINDOWSIZE+1;
    // pointers to neighbors of "ego" pixel
    short *pup = A+1;
    short *pdown = A+2*WINDOWSIZE+1;
    short *pleft = A+WINDOWSIZE;
    short *pright = A+WINDOWSIZE+2;
    short *pupleft = pup-1;
    short *pupright = pup+1;
    short *pdownleft = pdown-1;
    short *pdownright = pdown+1;
  
    // This loop takes between 1 and 2ms at 16x16 when not dumping information to Serial monitor
    for (r=1; r<WINDOWSIZE-1; ++r) {
      for (c=1; c<WINDOWSIZE-1; ++c) {
        // check to see "ego" is a local maxima. Use of ">" and ">=" as performed below implements arbitration.
        // if two neighboring pixels have the same value, the one above or left is the "winner"
        if (*pego>*pup && *pego>=*pdown && *pego>*pleft && *pego>=*pright && *pego>*pupleft && *pego>*pupright && *pego>=*pdownleft && *pego>=*pdownright) {
          // current pixel is a local maxima. 
          if (dumpType>=2) {
            sprintf(charbuf,"(%d,%d)",r,c);
            Serial.print(charbuf);
          }
          // check to see if current pixel exceeds intensity threshold. If intensityThreshold==0 then we bypass this test
          if (intensityThreshold==0 || *pego>=intensityThreshold) {
            if (dumpType>=2) {
              Serial.print(" b"); Serial.print(*pego);
            }
            // current pixel exceeds intensity threshold. Now check to see if point meets convexity threshold
            short udconvex = 2*(*pego)-*pup-*pdown; // up-down convexity
            short lrconvex = 2*(*pego)-*pleft-*pright; // left-right convexity
            short cvxval = udconvex;
            if (lrconvex<cvxval) // set cvxval to the greater of the convexities (recall that pixels are inverted e.g. high value = dark, low value = bright
              cvxval = lrconvex;
            if (dumpType>=2) {
              Serial.print(" c"); Serial.print(cvxval);
            }
            if (udconvex>=shapeConvexityThreshold && lrconvex>=shapeConvexityThreshold) {
              if (dumpType>=2) {
                Serial.print(" convex");
              }
              // DING DING DING WE HAVE A WINNER!! Record light's location and values
              // Note that we won't store more than MAXLIGHTS lights
              if (numActiveLights<MAXLIGHTS-1) {
                if (useHyperacuity) {
                  short hyperx = HyperMax2(*pleft,*pego,*pright);
                  short hypery = HyperMax2(*pup,*pego,*pdown);
                  lightM[numActiveLights] = 10*r+hypery;
                  lightN[numActiveLights] = 10*c+hyperx;
                } else {
                  lightM[numActiveLights] = r;
                  lightN[numActiveLights] = c;
                }              
                lightI[numActiveLights] = *pego;
                lightC[numActiveLights] = (udconvex+lrconvex)/2;
                numActiveLights++;
              }
            }
          }
          if (dumpType>=2) {
            Serial.println("");
          }
        }
        // advance to next pixel in row by incrementing all pointers
        pego++;pup++;pdown++;pleft++;pright++;pupleft++;pupright++;pdownleft++;pdownright++;
      }
      // advance to next row by double incrementing all pointers (e.g. go to next row and advance one)
      pego+=2;pup+=2;pdown+=2;pleft+=2;pright+=2;pupleft+=2;pupright+=2;pdownleft+=2;pdownright+=2;
    }
    digitalWrite(2,LOW);
  
    // GENERATE OUTPUT
    switch (dumpType) {
      case 0:
        // output coefficients
        for (i=0; i<numActiveLights; ++i) {
          sprintf(charbuf,"  (%3d,%3d)",lightM[i],lightN[i]);
          Serial.print(charbuf);
        }
        if (numActiveLights>=1)
          Serial.println("");
        else
          Serial.println("none");
        break;
    }
  }

  // if GUI is enabled then send image
  // first add GUI offset - this is a bit a hack for the time being (until we can figure out how
  // to get the GUI to read negative numbers!!!)
  for (i=0; i<NUMPIXELS; ++i)
    A[i] += GUIoffset;
  // print image
  GUI.sendImage(WINDOWSIZE,WINDOWSIZE,A,NUMPIXELS);
  
  // if GUI enabled, and 1D OF mode on, then send 1D OF
  if (sensormode==1) {
    // store vector in charbuf
    if (OFlinear>127)
      OFlinear=127;
    if (OFlinear<-127)
      OFlinear=-127;
    charbuf[0]=charbuf[1]=OFlinear;
    if (Sorientation==1) {
      charbuf[0]=0;
    }
    if (Sorientation==2) {
      charbuf[1]=0;
    }
    // send to print
    GUI.sendVectors(1,1,charbuf,1);
  }
  
  // if GUI is enabled, and 2D OF mode on, then send 2D OF
#if USING_MEGA==1  
  if (sensormode==2) {
    if (OFx>127)
      OFx=127;
    if (OFx<-127)
      OFx=-127;
    if (OFy>127)
      OFy=127;
    if (OFy<-127)
      OFy=-127;
    charbuf[0]=-(char)OFx;
    charbuf[1]=-(char)OFy;
    GUI.sendVectors(1,1,charbuf,1);    
  }
#endif
  
  // if GUI enabled, and lights mode is on, then send light locations
  if (sensormode==3) {
    for (i=0; i<numActiveLights; ++i) {
      if (useHyperacuity) {
        bytebuf[2*i]=lightM[i]/10;
        bytebuf[2*i+1]=lightN[i]/10;
      } else {
        bytebuf[2*i]=lightM[i];
        bytebuf[2*i+1]=lightN[i];        
      }
    }
    GUI.sendPoints(WINDOWSIZE,WINDOWSIZE,bytebuf,numActiveLights);
  }
  
  digitalWrite(3,LOW);
  ping = 0;
  startingup = 0;
}
