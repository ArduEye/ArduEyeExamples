
/* ARDUEYE_GUI_EXAMPLE_V1
 
 This sketch features the ArduEye_GUI library, which allows you
 to visualize the output of the ArduEye in the ArduEye 
 Processing GUI.  All code can be found at www.ardueye.com.

 An image is taken using the ArduEyeSMH library and several data 
 sets are sent down to the processing GUI.
 
 Started July 12, 2012
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

//=============================================================================
// INCLUDE FILES. The top two files are part of the ArduEye library and should
// be included in the Arduino "libraries" folder.

#include <ArduEye_SMH.h>  //Stonyman/Hawksbill vision chip library
#include <ArduEye_GUI.h>  //ArduEye processing GUI interface

#include <SPI.h>  //SPI library is needed to use an external ADC
                  //not supported for MEGA 2560

//==============================================================================
// GLOBAL VARIABLES

//mega 2560 can handle 48x48 array and FPN mask
//so we set SKIP_PIXELS=2 to downsample by 2
//and START_PIXELS at row/col 8.  This gives us
//a 48x48 array with superpixels of 2x2.  With the
//start row and col at 8, the 96x96 raw grid is 
//centered on the Stonyman 112x112 raw array.
#if defined(__AVR_ATmega2560__)  
   	#define MAX_ROWS 48
        #define MAX_COLS 48
        #define MAX_PIXELS (MAX_ROWS*MAX_COLS)
        #define SKIP_PIXELS 2
        #define START_PIXEL 8  
//uno can handle 19x19 array and FPN mask
//so we set SKIP_PIXELS=4 to downsample by 4
//and START_PIXELS to 18 to center the resulting
//76x76 raw grid on the Stonyman raw 112x112 array.
#elif defined (__AVR_ATmega8__)||(__AVR_ATmega168__)|  	(__AVR_ATmega168P__)||(__AVR_ATmega328P__)
	#define MAX_ROWS 19    
        #define MAX_COLS 19
        #define MAX_PIXELS (MAX_ROWS*MAX_COLS)
        #define SKIP_PIXELS 4
        #define START_PIXEL 18
#else 
	#  error "Code only supports ATmega 2560 and ATmega 8/168/328"
#endif

// recall from note above that image arrays are stored row-size in a 1D array

short img[MAX_PIXELS];         //1D image array
short row=MAX_ROWS;            //maximum rows allowed by memory
short col=MAX_COLS;            //maximum cols allowed by memory
short skiprow=SKIP_PIXELS;     //pixels to be skipped during readout because of downsampling
short skipcol=SKIP_PIXELS;     //pixels to be skipped during readout because of downsampling 
short sr=START_PIXEL;          //first pixel row to read
short sc=START_PIXEL;          //first pixel col to read

short chipSelect=0;            //which vision chip to read from

// FPN calibration. To save memory we are placing the FPN calibration
// array in an unsigned char, since the variation between pixels may be 
// expressed with a single byte. Variable mask_base holds an offset value 
// applied to the entire FPN array. Thus the FPN mask for pixel i will be the 
// value mask[i]+mask_base.
unsigned char mask[MAX_PIXELS]; // 16x16 FPN calibration image
short mask_base=0; // FPN calibration image base.

// Command inputs - for receiving commands from user via Serial terminal
char command; // command character
int commandArgument; // argument of command

//we find the maximum pixel to display in the GUI
unsigned char row_max;
unsigned char col_max;
byte points[2];  //points array to send down to the GUI

//a set of vectors to send down
char vectors[8];

//default ADC is the Arduino onboard ADC
unsigned char adcType=SMH1_ADCTYPE_ONBOARD;


//=======================================================================
// ARDUINO SETUP AND LOOP FUNCTIONS

void setup() 
{
  // initialize serial port
  Serial.begin(115200); //GUI defaults to this baud rate
  
  //initialize SPI (needed for external ADC
  SPI.begin();
  
  //initialize ArduEye Stonyman
  ArduEyeSMH.begin();
  
  //set the initial binning on the vision chip
  ArduEyeSMH.setBinning(skipcol,skiprow);
}

void loop() 
{

  //process commands from serial (should be performed once every execution of loop())
  processCommands();

  //get an image from the stonyman chip
  ArduEyeSMH.getImage(img,sr,row,skiprow,sc,col,skipcol,adcType,chipSelect);
   
  //find the maximum value.  This actually takes an image a second time, so
  //to speed up this loop you should comment this out
  ArduEyeSMH.findMax(sr,row,skiprow,sc,col,skipcol,adcType,chipSelect,&row_max,&col_max);
    
  //apply an FPNMask to the image.  This needs to be calculated with the "f" command
  //while the vision chip is covered with a white sheet of paper to expose it to 
  //uniform illumination.  Once calculated, it will remove fixed-pattern noise  
  ArduEyeSMH.applyMask(img,row*col,mask,mask_base);
  
  /***********************************************************************************/
  /***********************************************************************************/
  //   GUI EXAMPLE FUNCTIONS
  /***********************************************************************************/
  /***********************************************************************************/
  
  //if GUI is enabled then send image for display
  ArduEyeGUI.sendImage(row,col,img,row*col);
  
  //if GUI is enabled then send max point for display
  points[0]=(byte)row_max;
  points[1]=(byte)col_max;
  ArduEyeGUI.sendPoints(row,col,points,1);
  
  //if GUI is enabled create an array of 2x2 vectors to send to the GUI
  //these vectors should create an X pattern on the GUI display
  vectors[0]=-50;    //vector1 x
  vectors[1]=50;     //vector1 y
  
  vectors[2]=50;     //vector2 x
  vectors[3]=50;     //vector2 y
  
  vectors[4]=-50;    //vector3 x
  vectors[5]=-50;    //vector3 y
  
  vectors[6]=50;     //vector4 x
  vectors[7]=-50;    //vector4 y
  
  //send array of vectors, 2x2 display, 4 vectors
  ArduEyeGUI.sendVectors(2,2,vectors,4);
}


//=======================================================================
// FUNCTIONS DEFINED FOR THIS SKETCH

// the processCommands function reads and responds to commands sent to
// the Arduino over the serial connection.  
void processCommands()
{
  char charbuf[20];
  
  // PROCESS USER COMMANDS, IF ANY
  if (Serial.available()>0) // Check Serial buffer for input from user
  { 

    // get user command and argument
    // this function also checks for the presence of the GUI
    // so you must use this function if you are using the GUI 
    ArduEyeGUI.getCommand(&command,&commandArgument); 

    //switch statement to process commands
    switch (command) {

    //CHANGE ADC TYPE
    case 'a':
      if(commandArgument==0)
      {
       adcType=SMH1_ADCTYPE_ONBOARD;  //arduino onboard
       Serial.println("Onboard ADC");
      }
      if(commandArgument==1)
      {
       adcType=SMH1_ADCTYPE_MCP3201;  //external ADC (168/328 only)
       Serial.println("External ADC (doesn't work with Mega2560)");
      }
      break;

    // calculate FPN mask and apply it to current image
    case 'f': 
      ArduEyeSMH.getImage(img,sr,row,skiprow,sc,col,skipcol,adcType,chipSelect);
      ArduEyeSMH.calcMask(img,row*col,mask,&mask_base);
      Serial.println("FPN Mask done");  
      break;   
     
    //change chip select
    case 's':
      chipSelect=commandArgument;
      sprintf(charbuf,"chip select = %d",chipSelect);
      Serial.println(charbuf);
      break;

    // ? - print up command list
    case '?':
        Serial.println("a: ADC"); 
        Serial.println("f: FPN mask"); 
        Serial.println("s: chip select");
      break;
      
    default:
      break;
    }
  }
}

