
/* ARDUEYE_STONYMAN_EXAMPLE_V1
 
 This sketch features the ArduEye_SMH library, which provides an
 interface between the ArduEye and the Stonyman or Hawksbill 
 vision chips.  It also uses the ArduEye_GUI library to visualize
 the output on the ArduEye Processing GUI.  All code can be found
 at www.ardueye.com.

 An image is taken from a connected Stonyman/Hawksbill vision chip,
 the maximum value of the image is found, and the image plus the
 maximum value pixel are sent down to the GUI for display.

 Several serial commands can change the parameters of the vision chip,
 change the image being taken, and calculate and apply a Fixed-Pattern 
 Noise mask.  Type "?" into the serial terminal or processing GUI for
 a list of commands. 
 
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
  //uniform illumination.  Once calculated, it will produce a better image  
  ArduEyeSMH.applyMask(img,row*col,mask,mask_base);
  
  //if GUI is enabled then send image for display
  ArduEyeGUI.sendImage(row,col,img,row*col);
  
  //if GUI is enabled then send max point for display
  points[0]=(byte)row_max;
  points[1]=(byte)col_max;
  ArduEyeGUI.sendPoints(row,col,points,1);
  
}


//=======================================================================
// FUNCTIONS DEFINED FOR THIS SKETCH

// the fitsMemory function makes sure the image will fit
// in Arduino's memory
unsigned char fitsMemory(short r,short c)
{
 return(((r*c)<=MAX_PIXELS));
}

// the fitsArray function makes sure the image will not
// be larger than the Stonyman 112x112 array size
unsigned char fitsArray(short r,short c,short str,short stc,short skr,short skc)
{
  return( (((str+r*skr)<=112)&&((stc+c*skc)<=112)) );
}

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

    // Reset the chip - use this command if you plug in a new Stonyman chip w/o power cycling
    case 'b':
      //initialize ArduEye Stonyman
      row=MAX_ROWS;            //maximum rows allowed by memory
      col=MAX_COLS;            //maximum cols allowed by memory
      skiprow=SKIP_PIXELS;     //pixels to be skipped during readout because of downsampling
      skipcol=SKIP_PIXELS;     //pixels to be skipped during readout because of downsampling 
      sr=START_PIXEL;          //first pixel row to read
      sc=START_PIXEL;          //first pixel col to read
      chipSelect=0;            //which vision chip to read from
      
      ArduEyeSMH.begin();
      //set the initial binning on the vision chip
      ArduEyeSMH.setBinning(skipcol,skiprow);
      Serial.println("Chip reset");
      break;

    // Set number of columns
    case 'c':   
      if(fitsMemory(row,commandArgument)&&fitsArray(row,commandArgument,sr,sc,skiprow,skipcol))
      {
       col=commandArgument;
       sprintf(charbuf,"cols = %d",commandArgument);
       Serial.println(charbuf);
      } 
      else Serial.println("exceeds memory or 112x112 array size");
      break;
   
   //set starting col
   case 'C':   
      if(fitsMemory(row,col)&&fitsArray(row,col,sr,commandArgument,skiprow,skipcol))
      {
       sc=commandArgument; 
       sprintf(charbuf,"start col = %d",sc);
       Serial.println(charbuf);
      }
      else Serial.println("exceeds memory or 112x112 array size");
      break;
      
    //set amplifier gain
    case 'g':
      ArduEyeSMH.setAmpGain(commandArgument);
      sprintf(charbuf,"Amplifier gain = %d",commandArgument);
      Serial.println(charbuf);
      break;
    
    //set horizontal binning
    case 'h':
      if(fitsMemory(row,col)&&fitsArray(row,col,sr,sc,skipcol,commandArgument))
      {
       skipcol=commandArgument;
       ArduEyeSMH.setBinning(skipcol,skiprow);
       sprintf(charbuf,"Horiz Binning = %d",skipcol);
       Serial.println(charbuf);      
      }
      else Serial.println("exceeds memory or 112x112 array size");
      break;
      
    // calculate FPN mask and apply it to current image
    case 'f': 
      ArduEyeSMH.getImage(img,sr,row,skiprow,sc,col,skipcol,adcType,chipSelect);
      ArduEyeSMH.calcMask(img,row*col,mask,&mask_base);
      Serial.println("FPN Mask done");  
      break;   
      
    //change VREF
    case 'l':
      ArduEyeSMH.setVREF(commandArgument);
      sprintf(charbuf,"VREF = %d",commandArgument);
      Serial.println(charbuf);  
    break;
    
    //print the current array over Serial in Matlab format      
    case 'm':
      ArduEyeSMH.sectionToMatlab(sr,row,skiprow,sc,col,skipcol,adcType,chipSelect);
      break;

    //print the entire chip over Serial in Matlab format
    case 'M':   
      ArduEyeSMH.chipToMatlab(0,adcType,chipSelect);
      break;
      
    //change NBIAS
    case 'n': // set pinhole column
      ArduEyeSMH.setNBIAS(commandArgument);
      sprintf(charbuf,"NBIAS = %d",commandArgument);
      Serial.println(charbuf);  
      break;     

    // Set number of rows
    case 'r':
      if(fitsMemory(commandArgument,col)&&fitsArray(commandArgument,col,sr,sc,skiprow,skipcol))
      {
       row=commandArgument; 
       sprintf(charbuf,"rows = %d",row);
       Serial.println(charbuf);
      }
      else Serial.println("exceeds memory or 112x112 array size");
      break;   
    
    //set starting row
    case 'R':
      if(fitsMemory(row,col)&&fitsArray(row,col,commandArgument,sc,skiprow,skipcol))
      {
       sr=commandArgument; 
       sprintf(charbuf,"start row = %d",sr);
       Serial.println(charbuf);
      }
      else Serial.println("exceeds memory or 112x112 array size");
      break;
     
    //change chip select
    case 's':
      chipSelect=commandArgument;
      sprintf(charbuf,"chip select = %d",chipSelect);
      Serial.println(charbuf);
      break;
    
    //set vertical binning
    case 'v':
      if(fitsMemory(row,col)&&fitsArray(row,col,sr,sc,commandArgument,skipcol))
      {
       skiprow=commandArgument;
       ArduEyeSMH.setBinning(skipcol,skiprow);
       sprintf(charbuf,"Vertical Binning = %d",skiprow);
       Serial.println(charbuf);    
      } 
      else Serial.println("exceeds memory or 112x112 array size");
      break;

    // ? - print up command list
    case '?':
        Serial.println("a: ADC"); 
        Serial.println("b: reset"); 
        Serial.println("c: cols"); 
        Serial.println("C: start col");
        Serial.println("g: amp gain"); 
        Serial.println("h: hor binning"); 
        Serial.println("f: FPN mask"); 
        Serial.println("l: VREF"); 
        Serial.println("m: matlab section"); 
        Serial.println("M: matlab all");
        Serial.println("n: NBIAS");
        Serial.println("r: rows");
        Serial.println("R: start row");
        Serial.println("s: chip select");
        Serial.println("v: vert binning"); 
      break;
      
    default:
      break;
    }
  }
  }

