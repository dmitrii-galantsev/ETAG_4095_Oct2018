
/*
   Data logging sketch for the ETAG RFID Reader
   Version 1.2
   Code by:
   Alexander Moreno
   David Mitchell
   Eli Bridge
   Jay Wilhelm
   Dmitrii Galantsev
   January-2019

   Licenced in the public domain

REQUIREMENTS:
Power supply for the board should come from the USB cable or a 5V battery or DV power source.
A 3.3V CR1025 coin battery should be installed on the back side of the board to maintain the date
and time when the board is disconnected from a primary power source.

FLASH MEMORY STRUCTURE:
The onboard flash memory is divided into pages of 528 bytes each. There are probably several thousand pages.
Page 0 is reserved for RFID tag codes
Page 1 is reserved for parameters and counters (first four bytes are the current backup memory address)
TODO: Page 2 is reserved for RFID id
The rest is for backup memory.

EDITOR:
Please set your tabs to 4 spaces.
On arduino under Linux it's in $HOME/.arduino15/preferences.txt
Set these options:
	-> editor.tabs.expand=false
	-> editor.tabs.size=4

CHANGE LOG:
05-04-18 - Added interrupt driven RFID reader (jaywilhelm)
01-08-19 - Reformatted, Added set-to-compile-time option for rtc (dmitriigalantsev)
 */

// ***********INITIALIZE INCLUDE FILES AND I/O PINS*******************
// #include "ManchesterDecoder.h" // Interrupt driven RFID decoder
#include "SparkFun_RV1805.h"
#// Include "ETAGLowPower.h"
#include <Wire.h>			// Include the standard wire library - used for I2C communication with the clock
#include <SD.h>				// Include the standard SD card library
#include <SPI.h>

// TODO: normalize constant names
// TODO: add a header file
// TODO: remove motor options
// TODO: add logs
// TODO: reduce documentation (there are way too many comments)
// TODO: get rid of unneded global variables, use macros for constants where possible

#define serial SerialUSB	// Designate the USB connection as the primary serial comm port
#define DEMOD_OUT_PIN	30  // (PB03) this is the target pin for the raw RFID data
#define SHD_PINA		8   // (PA06) Setting this pin high activates the primary RFID circuit (only one can be active at a time)
#define SHD_PINB		9   // (PA07) Setting this pin high activates the seconday RFID circuit (only one can be active at a time)
#define MOD_PIN			0   // Not used - defined as zero
#define READY_CLOCK_PIN	0   // Not used - defined as zero
#define SDselect		7   // Chip select for SD card - make this pin low to activate the SD card, also the clock interupt pin
#define csFlash			2	// Chip select for flash memory
#define LED_RFID		31  // Pin to control the LED indicator.
#define interruptPin	7
#define MOTPWM			0	// Motor pulse width
#define MOTL			1	// Motor left
#define MOTR			2	// Motor right
#define mStby			3	// Motor controller standby
#define mSwitch			4	// Motor switch
#define INT1			7

RV1805 rtc;

// ************************* initialize variables******************************
uint32_t tagNo;						// Four least significant hexidecimals in 5 number tag code
uint32_t tagNo2;					// Four least significant hexidecimals in 5 number tag code
uint32_t timeSeconds;				// Stores the past Hour, Minute and Second of a read
uint32_t pastTimeSeconds;			// Stores the past Hour, Minute and Second of a read

// Byte flashArray[528];			// Large array of 528 bytes for writing a full page to onboard flash memory
// Byte tagCount;					// Keeps track of number of stored tags
unsigned int pageAddress;			// Page address for flash memory
unsigned int byteAddress;			// Byte address for flash memory
union flashMem {					// Union variable for constructing instructions to flash memory
	unsigned long flashAddress;
	byte flashAddrByte[4];
};

byte match;								// Used to determine if tags match.
byte RFcircuit = 1;						// Used to determine which RFID circuit is active. 1 = primary circuit, 2 = secondary circuit.
byte pastCircuit;						// Used to keep track of where (which RF circuit) a tag was last read
byte ss, mm, hh, da, mo, yr;			// Byte variables for storing date/time elements
String sss, mms, hhs, das, mos, yrs; 	// String variable for storing date/time text elements
String currentDate;						// USed to get the current date in mm/dd/yyyy format (we're weird in the US)
String currentTime;						// Used to get the time
String currentDateTime;					// Full date and time string
String timeString;						// String for storing the whole date/time line of data
char incomingByte = 0;					// Used for incoming serial data
unsigned int timeIn[12];				// Used for incoming serial data during clock setting
byte menu;								// Keeps track of whether the menu is active.

// Tag reading state variables
byte longPulseDetected = 0;				// A long pulse must first be read from the RFID output to begin reading a tag ID - This variable indicates whether a long pulse has happened
byte pastPulseLong = 0;					// Indicates whether the past pulse was long (1) or short (0).
byte RFIDbitCounter;					// Counts the number of bits that have gone into an RFID tag read
byte RFIDbyteCounter;					// Counts the number of bytes that have gone into an RFID tag read
byte RFIDbytes[11];						// Array of bytes for storing all RFID tag data (ID code and parity bits)
byte OneCounter;						// For counting the number of consecutive 1s in RFID input -- 9 ones signals the beginning of an ID code
unsigned int pulseCount;				// For counting pulses from RFID reader
unsigned int parityFail;				// Indicates if there was a parity mismatch (i.e. a failed read)
byte rParity;							// Temporary storage of parity data.

// Global variable for tag codes
String RFIDstring;					// Stores the TagID as a 10 character string
byte RFIDtagUser = 0;				// Stores the first (most significant) byte of a tag ID (user number)
unsigned long RFIDtagNumber = 0;	// Stores bytes 1 through 4 of a tag ID (user number)
byte RFIDtagArray[5];				// Stores the five individual bytes of a tag ID.

// ********************CONSTANTS (SET UP LOGGING PARAMETERS HERE!!)*******************************
const byte checkTime = 30;			// How long in milliseconds to check to see if a tag is present (Tag is only partially read during this time -- This is just a quick way of detirmining if a tag is present or not
const unsigned int pollTime1 = 200;	// How long in milliseconds to try to read a tag if a tag was initially detected (applies to both RF circuits, but that can be changed)
const unsigned int delayTime = 8;	// Minimim time in seconds between recording the same tag twice in a row (only applies to data logging--other operations are unaffected)
const unsigned long pauseTime = 500;  // CRITICAL - This determines how long in milliseconds to wait between reading attempts. Make this wait time as long as you can and still maintain functionality (more pauseTime = more power saved)

#define SLEEP_H 21								// When to go to sleep at night - hour
#define SLEEP_M 00								// When to go to sleep at night - minute
#define WAKE_H 07								// When to wake up in the morning - hour
#define WAKE_M 00								// When to wake up in the morning - minute
#define SLEEP_INTERVAL 25						// How many seconds to sleep at a time, must be less than 1 min.
#define ON_TIME 1000							// How man MILLISECONDS to stay on between sleep intervals
#define SLEEP_TIME (SLEEP_H * 100 + SLEEP_M)	// Combined hours and minutes for sleep time
#define WAKE_TIME (WAKE_H * 100 + WAKE_M) 		// Combined hours and minutes for wake time

/* The reader will output Serial data for a certain number of read cycles;
 * then it will start using a low power sleep mode during the pauseTime between read attempts.
 * The variable stopCycleCount determines how many read cycles to go
 * through before using the low-power sleep.
 * Once low-power sleep is enabled, the reader will not be able to output
 * serial data (but tag reading and data storage will still work).
 *
 * This may result in arduino not showing up as a serial device.
 * Press upload button in the Arduino IDE and then press the reset button on the board.
 * This should succesfully upload the new code to Arduino.
 */

unsigned int cycleCount = 0;						// Counts read cycles
unsigned int stopCycleCount = 50;					// How many read cycles to maintain serial comminications

byte SDpresent;										// 1 if SD card is detected on startup.
byte doorState = 1;									// Status of the motorized door. 0 = open, 1 = closed.
byte readFlashByte(unsigned long fAddress);
void writeFlashByte(unsigned long fAddress, byte fByte);
unsigned long getFlashAddr();
void writeFlashAddr(unsigned long fAddress);
void setClk();
void dumpMem();
void printDirectory(File dir, int numTabs);

// *******************************SETUP**************************************
void setup() {						// This function sets everything up for logging.
	// Establish startup settings and I/O pins

	Wire.begin();					// Enable I2C communication

	pinMode(LED_RFID, OUTPUT);		// Pin for controlling the on-board LED
	digitalWrite(LED_RFID, HIGH);   // Turn the LED off (LOW turns it on)

	// SD card and flash memory chip select pins - these pins enable SPI communication with these peripherals
	pinMode(SDselect, OUTPUT);		// Chip select pin for SD card must be an output
	pinMode(csFlash, OUTPUT);		// Chip select pin for Flash memory
	digitalWrite(SDselect, HIGH);   // Make both chip selects high (not selected)
	digitalWrite(csFlash, HIGH);
	// GManDecoder1.DisableMonitoring();
	// GManDecoder2.DisableMonitoring();

	// Pins for controlling the RFID circuits
	pinMode(SHD_PINA, OUTPUT);		// Make the primary RFID shutdown pin an output.
	digitalWrite(SHD_PINA, HIGH);   // Turn the primary RFID circuit off (LOW turns on the EM4095)
	pinMode(SHD_PINB, OUTPUT);		// Make the secondary RFID shutdown pin an output.
	digitalWrite(SHD_PINB, HIGH);   // Turn the secondary RFID circuit off (LOW turns on the EM4095)

	// Motor control pins
	pinMode(mStby, OUTPUT);			// Pin that can put the motor controller in low-power standby mode
	digitalWrite(mStby, LOW);		// Motor in standby mode
	pinMode(MOTR, OUTPUT);			// Right motor control pin
	digitalWrite(MOTR, LOW);		// Turn motor off
	pinMode(MOTL, OUTPUT);			// Left motor control pin
	digitalWrite(MOTL, LOW);		// Turn motor off
	pinMode(mSwitch, INPUT_PULLUP); // Motor switch enabled as input with internal pullup resistor

	// Establish a a couple of parameters for low power sleep mode
	SYSCTRL->XOSC32K.bit.RUNSTDBY = 1;					// Set the XOSC32K to run in standby (not sure why this is needed)
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;					// Set sleep mode

	// Try to initiate a serial connection
	serial.begin(9600);

	// Slow flashing LED to create a time window for serial connection.
	blinkLED(5);

	// Start real time clock interfcace and get the current time
	if (rtc.begin() == false) {
		serial.println("Something wrong with clock"); // Try initiation and report if something is wrong
	}
	if (rtc.updateTime() == false) {
		// Updates the time variables from RTC, report if there's a failure
		serial.print("RTC failed");
	}
	serial.println("Clock initialized and set to...."); // Serial message for clock output
	showTime();											// Function that displays the clock time

	// Set up SD card communication
	serial.println("Initializing SD card...");				// Message to user
	if (!SD.begin(SDselect)) {
		serial.println("SD card failed, or not present"); 	// Initiate the SD card function with the pin that activates the card SD card error message
		SDpresent = 0;
	}
	else {
		serial.println("SD card online.");
		SDpresent = 1;
		//			serial.println("SD Contents");  // Show entire contents of SD card Root directory (optional)
		//			File root = SD.open("/");		// Get the contents of the root folder
		//			printDirectory(root, 0);		// Display all of it.
	}
	digitalWrite(SDselect, HIGH); // SD card turned off for now

	// Set up communication with the flash memory
	SPI.begin();														// Enable SPI communication for Flash Memory
	SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0)); 	// Establish SPI parameters - Does this have to be done repeatedly
	unsigned long fAddressEnd2 = getFlashAddr();		// Get flash address
	fAddressEnd2 = getFlashAddr();						// Get flash address; has to be done twice?? why??
	serial.print("checking if Flash Memory is initialized: ");			// It is neccessary to initialize the flash memory on the first startup
	byte byte0 = readFlashByte(0x00000404);				// Read a particular byte from the flash memory
	serial.println(byte0, HEX);							// Display the byte
	if (byte0 == 0xFF) {								// If the byte is 0xFF then the flash memory needs to be initialized
		writeFlashByte(0x00000404, 0xAA);				// Write a different byte to this memory location
		serial.print("Initializing Flash Memory: ");  	// Message
		writeFlashAddr(0x00000800);						// Set initial flash memory address for logging data to page 2, byte address 0
	}
	serial.println("Flash Memory IS initialized."); // Confirm initialization
	getFlashAddr();									// Display the current flash memory data loggin address
	// WriteFlashByte(0x00000404, 0xFF);			// Uncomment to set flash memory initialization flag to 0xFF, which will cause the memory address to be reset on startup

	// Display options menu to user
	menu = 1;
	while (menu == 1) {								// Keep displaying menu until an exit condition happens
		serial.println();
		if (!rtc.updateTime()) {
			serial.print("RTC failed"); 			// Updates the time variables from RTC
		}
		showTime();									// Show the current time
		serial.println("What to do? (input capital letters)");		// Ask the user for instruction and display the options
		serial.println("	C/c = set clock to compile time");
		serial.println("	M/m = set clock manually");
		serial.println("	B/b = display backup memory");
		serial.println("	E/e = erase (reset) backup memory");
		serial.println("	Anything else = start logging");
		unsigned int serDelay = 0;							// If there's no response then eventually move on and just start logging
		while (!serial.available() && serDelay++ < 10000) 	// Wait about 10 seconds for a user response
			delay(1);
		if (!serial.available()) {							// If there is a response then perform the corresponding operation
			menu = 0;
			continue;
		}

		incomingByte = serial.read();	// Get the entry from the user
		switch (incomingByte) {			// Execute whatever option the user selected
			case 'c':
			case 'C':
				if (!rtc.setToCompilerTime()) {
					//PRINT_ERROR("Can't set clock to compiler time");
					Serial.println("Something went wrong setting the time");
				}
				else {
					Serial.println("Clock set successfully");
					//PRINT_LOG("Clock set successfully");
				}
				break;
			case 'm':					// Option to set clock
			case 'M':					// Option to set clock
				inputTime();			// Calls function to get time values from user
				if (!rtc.setTime(0, ss, mm, hh, da, mo, yr, 1)) {	// Attempt to set clock with input values
					Serial.println("Something went wrong setting the time");
				}
				break;
			case 'b':
			case 'B':
				dumpMem();  			// Display backup memory; calls function dumpmem(); break and show menu again
				break;
			case 'e':
			case 'E':
				writeFlashAddr(0x0800); // Reinitialize flash memory by setting memory address to byte 0 of page 2
				serial.print("flash address = ");
				serial.println(getFlashAddr(), BIN);
				break;
			default:
				menu = 0;
				break;
		}
	}

	// Prepare for data logging
	serial.println("Scanning for tags...\n");	// Message to user

}												// End void setup

// ******************************MAIN PROGRAM*******************************

void loop() {  // This is the main function. It loops (repeats) forever.
	if (rtc.updateTime() == false) serial.print("RTC failed at beginning of loop "); // Updates the time variables from RTC

	showTime();
	unsigned int curTime = rtc.getHours() * 100 + rtc.getMinutes();	// Combine hours and minutes into one variable
	if (curTime == SLEEP_TIME) {							// Designated go to sleep time
		digitalWrite(SHD_PINB, HIGH);					// Shut down both RFID readers
		digitalWrite(SHD_PINA, HIGH);
		// Serial.println("seting alarm");
		rtc.setAlarm(00, WAKE_M, WAKE_H, 1, 1);				// Second, minute, hour, date, month
		rtc.enableInterrupt(INTERRUPT_AIE);
		rtc.setAlarmMode(4);
		saveLogSD("SLEEP STARTED");						// Note for the log
		pinMode(SDselect, INPUT);						// Make the SD card pin an input so it can serve as interrupt pin
		lpSleep();										// Call sleep funciton
		// ............//								// Sleep happens here
		blipLED(30);									// Blink indicator - processor reawakened
		rtc.stopTimer();
		if(SDpresent == 1){
			pinMode(SDselect, OUTPUT);					// Chip select pin for SD card must be an output for writing to SD card
			SD.begin(SDselect);
			saveLogSD("SLEEP ENDED");
		}
	}
	// If(cycleCount < stopCycleCount){
	serial.print("Scanning RFID circuit "); 	// Message part 1: Tell the user which circuit is active
	serial.println(RFcircuit);					// Message part 2: show the active RF circuit
	// }

	// Attempt tag read
	if (FastRead(DEMOD_OUT_PIN, RFcircuit, checkTime, pollTime1) == 1) {

		// The following is executed if a tag is detected
		processTag();   // Parse tag data into string and hexidecimal formats
		// Updates the time variables from Real Time Clock
		if (rtc.updateTime() == false) serial.print("RTC failed after tag read ");
		timeSeconds = (rtc.getDate() * 86400) + (rtc.getHours() * 3600) + (rtc.getMinutes() * 60) + rtc.getSeconds();
		// Serial.print("second count is ");
		// Serial.println(timeSeconds, DEC);
		// Serial.print("tag read. old tag is ");  // Optional display -- this will display multiple times as long a tag is present
		// Serial.println(tagNo2, HEX);				// Disply previously read tag ID
		tagNo = RFIDtagNumber;						// Stores the 4 least significant tag ID numbers - good for all tag comparisons I think.
		serial.print(RFIDstring);					// Call a subroutine to display the tag data via serial USB
		serial.print(" detected on antenna ");  	// Message part 1: add a note about which atenna was used
		serial.print(RFcircuit);					// Message part 2: Which antenna
		serial.print(" at ");						// Message part 3
		showTime();									// Message part 4: display the time
		flashLED();									// Flash the LED briefly to indicate a tag read
		if((timeSeconds < pastTimeSeconds + delayTime) & tagNo == tagNo2 & pastCircuit == RFcircuit){ // If everything matches up, the read is a repeat - don't log it.
			serial.println("Repeat read - data not logged.");
		} else {
			if(SDpresent == 1){
				serial.println("Logging tag data");
				logRFID_To_SD(currentDateTime);		// Call function to log data to the SD card
			}
			writeRFID_To_FlashLine();				// Call function to log to backup memory
			pastCircuit = RFcircuit;
			pastTimeSeconds = (rtc.getDate() * 86400) + (rtc.getHours() * 3600) + (rtc.getMinutes() * 60) + rtc.getSeconds();
			tagNo2 = tagNo;							// Set tagNo2 to current tag number for future comparisons // For keeping track of delaytime.
		}
	}

	digitalWrite(SHD_PINB, HIGH);		// Shut down both RFID readers
	digitalWrite(SHD_PINA, HIGH);		// Shut down both RFID readers

	// Delay(pauseTime);				// Pause between polling attempts

	if(cycleCount < stopCycleCount){
		cycleCount++;
		delay(pauseTime);				// Pause between polling attempts
	} else {
		byte pauseInterval = (pauseTime * 64)/1000;
		serial.print("pause interval: ");
		serial.println(pauseInterval, DEC);
		rtc.setRptTimer(pauseInterval, 1); 	// Set timer and use frequency of 64 Hz
		pinMode(SDselect, INPUT);			// Make the SD card pin an input so it can serve as interrupt pin
		rtc.startTimer();					// Start the timer
		lpSleep();							// Call sleep funciton
		// BlipLED(30);						// Blink indicator - processor reawakened
		rtc.stopTimer();
		if(SDpresent == 1){
			pinMode(SDselect, OUTPUT);		// Chip select pin for SD card must be an output for writing to SD card
			SD.begin(SDselect);
		}
	}

	//RFcircuit == 1 ? RFcircuit = 2 : RFcircuit = 1;		// Uncomment this line to alternate between active RF circuits.
}


// *********************SUBROUTINES************************** //
// The Following are all subroutines called by the code above //

byte FastRead(int demodOutPin, byte whichCircuit, byte checkDelay, unsigned int readTime) {
	if (whichCircuit == 1) {
		digitalWrite(SHD_PINA, LOW);			// Turn off primary RFID circuit
		digitalWrite(SHD_PINB, HIGH);
	} else {
		digitalWrite(SHD_PINA, HIGH);			// Turn off primary RFID circuit
		digitalWrite(SHD_PINB, LOW);
	}
	// Serial.println("fast read activated...");
	pinMode(demodOutPin, INPUT);				// Set up the input pin as an input
	rParity = 0;
	parityFail = 0x07FF;  // Start with 11 bits set and clear one for every line-parity check that passes, and clear the last for the column parity check
	pulseCount = 0;
	OneCounter = 0;
	longPulseDetected = 0;
	pastPulseLong = 0;
	RFIDbyteCounter = 0;
	RFIDbitCounter = 4;							// Counts backwards from 4 to zero
	memset(RFIDbytes, 0, sizeof(RFIDbytes));  	// Clear RFID memory space
	unsigned long currentMillis = millis();   	// To determine how long to poll for tags, first get the current value of the built in millisecond clock on the processor
	unsigned long stopMillis = currentMillis + readTime;
	attachInterrupt(digitalPinToInterrupt(demodOutPin), INT_demodOut, CHANGE);

	// Delay(checkTime);
	delay(checkDelay);
	// Serial.print("pulses detected... ");
	// Serial.println(pulseCount, DEC);
	if (pulseCount > (checkDelay - 25)) {		// May want a separate variable for threshold pulse count.
		while (millis() < stopMillis & parityFail != 0) {
			delay(1);
		}
	} else {
		detachInterrupt(digitalPinToInterrupt(demodOutPin));
		digitalWrite(SHD_PINA, HIGH);			// Turn off primary RFID circuit
		digitalWrite(SHD_PINB, HIGH);			// Turn off Secondary RFID circuit
		// Serial.print("nothing read... ");
		return (0);
	}

	detachInterrupt(digitalPinToInterrupt(demodOutPin));
	digitalWrite(SHD_PINA, HIGH);				// Turn off primary RFID circuit
	digitalWrite(SHD_PINB, HIGH);				// Turn off Secondary RFID circuit
	if (parityFail == 0) {
		serial.print("parityOK... ");
		return (1);
	} else {
		serial.print("parity fail... ");
		return (0);
	}

}

void processTag(void)
{
	// Process each byte (could do a loop but.....)
	RFIDtagArray[0] = ((RFIDbytes[0] << 3) & 0xF0) + ((RFIDbytes[1] >> 1) & 0x0F);
	String StringOne = String(RFIDtagArray[0], HEX);
	if(RFIDtagArray[0] < 0x10) {StringOne = String("0" + StringOne);}
	RFIDtagUser = RFIDtagArray[0];

	RFIDtagArray[1] = ((RFIDbytes[2] << 3) & 0xF0) + ((RFIDbytes[3] >> 1) & 0x0F);
	String StringTwo = String(RFIDtagArray[1], HEX);
	if(RFIDtagArray[1] < 0x10) {StringTwo = String("0" + StringTwo);}
	RFIDtagNumber = RFIDtagArray[1] << 24;

	RFIDtagArray[2] = ((RFIDbytes[4] << 3) & 0xF0) + ((RFIDbytes[5] >> 1) & 0x0F);
	String StringThree = String(RFIDtagArray[2], HEX);
	if(RFIDtagArray[2] < 0x10) {StringThree = String("0" + StringThree);}
	RFIDtagNumber = RFIDtagNumber + (RFIDtagArray[2] << 16);

	RFIDtagArray[3] = ((RFIDbytes[6] << 3) & 0xF0) + ((RFIDbytes[7] >> 1) & 0x0F);
	String StringFour = String(RFIDtagArray[3], HEX);
	if(RFIDtagArray[3] < 0x10) {StringFour = String("0" + StringFour);}
	RFIDtagNumber = RFIDtagNumber + (RFIDtagArray[3] << 8);

	RFIDtagArray[4] = ((RFIDbytes[8] << 3) & 0xF0) + ((RFIDbytes[9] >> 1) & 0x0F);
	String StringFive = String(RFIDtagArray[4], HEX);
	if(RFIDtagArray[4] < 0x10) {StringFive = String("0" + StringFive);}
	RFIDtagNumber = RFIDtagNumber + RFIDtagArray[4];
	RFIDstring = String(StringOne + StringTwo + StringThree + StringFour + StringFive);
	RFIDstring.toUpperCase();
}


void INT_demodOut(void)
{
	volatile uint32_t timeNow = micros();		// Store the current microsecond timer value in timeNow
	volatile static uint32_t lastTime = 0;		// Clear this variable
	uint16_t fDiff = timeNow - lastTime;		// Calculate time elapsed since the last execution of this function
	lastTime = timeNow;							// Establish a new value for lastTime
	// Int8_t fTimeClass = ManchesterDecoder::tUnknown;// ??????
	int16_t fVal = digitalRead(DEMOD_OUT_PIN);	// Set fVal to the opposite (!) of the value on the RFID data pin (default is pin 30).
	byte RFbit = 255;							// Set to default, 255, (no bit read)

	if (fDiff > 395 & fDiff < 600) {
		pulseCount++;
		longPulseDetected = 1;
		pastPulseLong = 1;
		RFbit = 200;										// Indicate that successful reading is still going on
		if (OneCounter < 9) {
			fVal == 1 ? OneCounter++ : OneCounter = 0;		// If we have read a 1 add to the one counter. if not clear the one counter
		} else {
			RFbit = fVal;
		}
	}
	if (fDiff < 395 & fDiff > 170) {
		pulseCount++;
		RFbit = 200;											// Indicate that successful reading is still going on
		if (longPulseDetected == 1 && pastPulseLong == 1) {		// Before this input means anything we must have registered one long bit and the last pulse must have been long (or a transition bit)
			if (OneCounter < 9) {								// Only write tag bits when we have read 9 ones.
				fVal == 1 ? OneCounter++ : OneCounter = 0;		// If we have read a 1 add to the one counter. if not clear the one counter
			} else {
				RFbit = fVal;
			}
			pastPulseLong = 0;		// Indicate that the last pulse was short
		} else {
			pastPulseLong = 1;		// Indicate that the last pulse was long.
			// This is not really true, but the second of two consecutive short pulses means the next pulse should indicate a read bit.
		}
	}

	// Now check if RFbit was changed from 255 and if so add to the data compiled in RFIDbytes
	if (RFbit < 100) {
		RFbit == 1 ? bitSet(RFIDbytes[RFIDbyteCounter], RFIDbitCounter) : bitClear(RFIDbytes[RFIDbyteCounter], RFIDbitCounter); // Set or clear the RFID data bit
		if (RFIDbitCounter > 0) {
			rParity = rParity ^ RFbit;   // Calculate running parity bit -- Exclusive or between row parity variable and current RF bit
			RFIDbitCounter--;
		} else {

			if ((RFIDbitCounter == 0) & (RFIDbyteCounter < 10)) {  // Indicates we are at the end of a line - Do a line parity check
				byte tb = RFIDbytes[RFIDbyteCounter];
				rParity = ((tb >> 4) & 1) ^ ((tb >> 3) & 1) ^ ((tb >> 2) & 1) ^ ((tb >> 1) & 1);
				rParity == (tb & 1) ? bitClear(parityFail, RFIDbyteCounter) : bitSet(parityFail, RFIDbyteCounter); // Check parity match and adjust parityFail
				rParity = 0;
				RFIDbyteCounter++;
				RFIDbitCounter = 4;
			}

			if ((RFIDbitCounter == 0) & (RFIDbyteCounter == 10)) {	// Indicates we are on the last bit of an ID code
				// Test all column parity
				byte xorByte = (RFIDbytes[10] & B00011111) >> 1;
				for (byte i = 0; i <= 9; i++) {						// Loop through bytes 1 though 9 (parity row included on first interation - should Xor out to zero
					xorByte = xorByte ^  (RFIDbytes[i] >> 1);
				}
				if (xorByte == 0) {
					bitClear(parityFail, RFIDbyteCounter) ;			// If parity checks out clear the last bit
				}
			}
		}
	}

	if ((RFbit == 255) & (pulseCount != 0)) {						// No pulse detected, clear everything except pulseCount
		rParity = 0;
		parityFail = 0x07FF;
		OneCounter = 0;
		longPulseDetected = 0;
		pastPulseLong = 0;
		RFIDbyteCounter = 0;
		RFIDbitCounter = 4;											// Counts backwards from 4 to zero
		memset(RFIDbytes, 0, sizeof(RFIDbytes));					// Clear RFID memory space
	}
}

void showTime() {
	currentDate = rtc.stringDateUSA(); // Get the current date in mm/dd/yyyy format (we're weird in the US)
	// String currentDate = rtc.stringDate()); // Get the current date in dd/mm/yyyy format (Rest-of-the-world format)
	currentTime = rtc.stringTime();							// Get the time
	currentDateTime = currentDate + " " + currentTime ;
	serial.println(currentDateTime);
}

void flashLED() {
	for (unsigned int n = 0; n < 100 ; n = n + 30) {	// Loop to Flash LED and delay reading after getting a tag
		digitalWrite(LED_RFID, LOW);					// The approximate duration of this loop is determined by the readFreq value
		delay(5);										// It determines how long to wait after a successful read before polling for tags again
		digitalWrite(LED_RFID, HIGH);
		delay(25);
	}
}

void blinkLED(byte repeats) {
	for (int i = 0; i < repeats; i++)	// Flash LED 5 times while waiting for serial connection to come online
	{
		delay(500);						// Pause for 0.5 seconds
		digitalWrite(LED_RFID, LOW);	// Turn the LED on (LOW turns it on)
		delay(500);						// Pause again
		digitalWrite(LED_RFID, HIGH);   // Turn the LED off (HIGH turns it off)
		// Ss = ss + 1;					// Add to counting variable
	}// End while
	digitalWrite(LED_RFID, HIGH);		// Make sure LED is off
}

void blipLED(byte blip) {
	digitalWrite(LED_RFID, LOW);	// Turn the LED on (LOW turns it on)
	delay(blip);					// Leave LED on for a quick flash
	digitalWrite(LED_RFID, HIGH);   // Turn the LED off (HIGH turns it off)
	if(blip == 200) {
		delay(blip);
	}
}


void logRFID_To_SD(String timeString) {
	File dataFile = SD.open("DATA.txt", FILE_WRITE);		// Initialize the SD card and open the file "datalog.txt" or create it if it is not there.
	if (dataFile) {
		/*for (int n = 0; n < 5; n++) {						// Loop to print out the RFID code to the SD card
		  if (tagData[n] < 10) dataFile.print("0");	// Add a leading zero if necessary
		  dataFile.print(tagData[n], HEX);			// Print to the SD card
		  }*/

		dataFile.print(RFIDstring);
		dataFile.print(",");						// Comma for data delineation
		dataFile.print(RFcircuit);					// Log which antenna was active
		dataFile.print(",");						// Comma for data delineation
		dataFile.println(timeString);				// Log the time
		dataFile.close();							// Close the file
		serial.println("saved to SD card.");		// Serial output message to user
	} // Check dataFile is present
	else {
		serial.println("error opening datalog.txt");// Error message if the "datafile.txt" is not present or cannot be created
	}// End check for file
}


void erasePage0() {
	SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
	digitalWrite(SDselect, HIGH);	// Make sure the SD card select is off
	digitalWrite(csFlash, LOW);		// Activate flash chip
	SPI.transfer(0x81);				// Opcode for page erase
	SPI.transfer(0x00);				// First of three address bytes
	SPI.transfer(0x00);				// Second address byte
	SPI.transfer(0x00);				// Third address byte
	digitalWrite(csFlash, HIGH);	// Deactivate flash chip - allow erase to happen
}

void dumpMem() {
	SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
	digitalWrite(SDselect, HIGH);	// Make sure the SD card select is off
	char tagData[16];
	serial.println("Transmitting data from backup memory.");
	getFlashAddr();
	delay(10);
	unsigned long fAddressEnd = getFlashAddr();		// Get flash address
	serial.print("last flash memory address: ");
	serial.println(fAddressEnd, DEC);
	serial.println(fAddressEnd, BIN);
	// FAddressEnd = 0x00001000;
	unsigned long fAddress = 0x00000800;			// First address for stored data
	serial.print("first flash memory: ");
	serial.println(fAddress, DEC);
	serial.println(fAddress, BIN);

	while (fAddress < fAddressEnd) {
		//	if (serial.available())
		//	{
		//			serial.println("User exit");
		//			break;
		//	}
		serial.print("starting loop");
		digitalWrite(csFlash, LOW);					// Activate flash chip
		SPI.transfer(0x03);							// Opcode for low freq read
		SPI.transfer(fAddress >> 16);				// Write most significant byte of Flash address
		SPI.transfer((fAddress >> 8) & 0xFF);		// Second address byte
		SPI.transfer(fAddress & 0xFF);				// Third address byte
		while ((fAddress & 0x000003FF) < 500) {		// Repeat while the byte address is less than 500
			serial.print("from flash memory address: ");
			serial.println(fAddress, BIN);
			for (int n = 0; n < 5; n++) {			// Loop to read in an RFID code from the flash and send it out via serial comm
				tagData[n] = SPI.transfer(0);
				if (tagData[n] < 10) serial.print("0"); // Add a leading zero if necessary
				serial.print(tagData[n], HEX);		// Send out tag datum
			}
			RFcircuit = SPI.transfer(0);			// Read which antenna was active
			mo = SPI.transfer(0);					// Read in date and time
			da = SPI.transfer(0);
			yr = SPI.transfer(0);
			hh = SPI.transfer(0);
			mm = SPI.transfer(0);
			ss = SPI.transfer(0);
			serial.print(",");						// Comma for delineation
			serial.print(RFcircuit);				// Which circuit
			serial.print(",");						// Comma for delineation
			if (mo < 10) {
				serial.print(0);					// Send out date/time bytes; add leading zeros as needed
			}
			serial.print(mo, DEC);
			serial.print("/");
			if (da < 10) {
				serial.print(0);
			}
			serial.print(da, DEC);
			serial.print("/");
			if (yr < 10) {
				serial.print(0);
			}
			serial.print(yr, DEC);
			serial.print(" ");
			if (hh < 10) {
				serial.print(0);					// Send out date/time bytes; add leading zeros as needed
			}
			serial.print(hh, DEC);
			serial.print(":");
			if (mm < 10) {
				serial.print(0);
			}
			serial.print(mm, DEC);
			serial.print(":");
			if (ss < 10) {
				serial.print(0);
			}
			serial.println(ss, DEC);
			fAddress = fAddress + 12;				// Update flash address
			if (fAddress >= fAddressEnd) break;		// Break if we are at the end of the backup data stream
		}
		// When the byte address exceeds 500 the page address needs to be incremented
		fAddress = (fAddress & 0xFFFFC00) + 0x0400; // Set byte address to zero and add 1 to the page address
		digitalWrite(csFlash, HIGH);				// Turn off flash
		if (fAddress >= fAddressEnd) break;			// Break if we are at the end of the backup data stream
		delay(10);									// Wait a bit
	}
}

unsigned long getFlashAddr() {						// Get the address counter for the flash memory from page 1 address 0
	digitalWrite(SDselect, HIGH);					// Make sure the SD card select is off
	digitalWrite(csFlash, LOW);
	SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
	unsigned long fAddress = 0x00030303 ;
	SPI.transfer(0x03);								// Opcode for low freq read
	SPI.transfer(0x00);								// First of three address bytes
	SPI.transfer(0x04);								// 00000100  second address byte - selects page 1
	SPI.transfer(0x00);								// Third address byte selects byte address 0
	// DelayMicroseconds(1000);
	// Serial.println(fAddress, BIN);
	fAddress = SPI.transfer(0) & 0xFF;				// Shift in the address value
	// Serial.println(fAddress, BIN);
	fAddress = (fAddress << 8) + SPI.transfer(0);	// Shift in the address value
	// Serial.println(fAddress, BIN);
	fAddress = (fAddress << 8) + SPI.transfer(0);	// Shift in the address value
	// Serial.println(fAddress, BIN);
	digitalWrite(csFlash, HIGH);					// Deactivate flash chip
	// Serial.print("flash address = ");
	// Serial.println(fAddress, BIN);
	return fAddress;
}

void writeFlashAddr(unsigned long fAddress) {   // Write the address counter for the flash memory on page 1 byte address 0
	SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
	digitalWrite(SDselect, HIGH);				// Make sure the SD card select is off
	digitalWrite(csFlash, LOW);					// Activate flash chip
	SPI.transfer(0x58);							// Opcode for read modify write
	SPI.transfer(0x00);							// First of three address bytes
	SPI.transfer(0x04);							// 000001oo  second address byte - selects page 1
	SPI.transfer(0x00);							// Third address byte selects byte address 0
	SPI.transfer(fAddress >> 16);				// Write most significant byte of Flash address
	SPI.transfer((fAddress >> 8) & 0xFF);		// Second address byte
	SPI.transfer(fAddress & 0xFF);				// Third address byte
	digitalWrite(csFlash, HIGH);				// Deactivate flash chip
	delay(20);
}

void writeRFID_To_FlashLine() {
	unsigned long fAddress = getFlashAddr();	// Get the current flash memory address
	serial.print("transferring to address: ");
	serial.println(fAddress, BIN);
	// Serial.print(" ");
	// DisplayTag();
	digitalWrite(csFlash, LOW);					// Activate flash chip
	SPI.transfer(0x58);							// Opcode for read modify write
	SPI.transfer((fAddress >> 16) & 0xFF);		// Write most significant byte of Flash address
	SPI.transfer((fAddress >> 8) & 0xFF);		// Second address byte
	SPI.transfer(fAddress & 0xFF);				// Third address byte
	for (int n = 0; n < 6; n++) {				// Loop to log the RFID code to the flash
		SPI.transfer(RFIDtagArray[n]);
	}
	SPI.transfer(RFcircuit);					// Log which antenna was active
	SPI.transfer(rtc.getMonth());
	SPI.transfer(rtc.getDate());
	SPI.transfer(rtc.getYear());
	SPI.transfer(rtc.getHours());
	SPI.transfer(rtc.getMinutes());
	SPI.transfer(rtc.getSeconds());
	digitalWrite(csFlash, HIGH);				// Deactivate flash chip
	unsigned int bAddress = fAddress & 0x03FF;	// And with 00000011 11111111 to isolate byte address
	bAddress = bAddress + 12;					// Add 12 to accound for new bytes (5 for RFID, 1 for RFcircuit, and 6 for date/time)
	if (bAddress > 500) {						// Stop writing if beyond byte address 500 (this is kind of wasteful)
		fAddress = (fAddress & 0xFFFFC00) + 0x0400;   // Set byte address to zero and add 1 to the page address
	} else {
		fAddress = (fAddress & 0xFFFFC00) + bAddress; // Just add to the byte address
	}
	// DigitalWrite(csFlash, HIGH); // Deactivate flash chip
	delay(20);
	writeFlashAddr(fAddress);					// Write the updated address to flash.
	serial.println("saved to flash.");			// Serial output message to user
}

char getMode() {
	char fMode = readFlashByte(0x00000403);		// Feeder mode is stored in page 1, byte 3
	if ((fMode != 'A') && (fMode != 'O') && (fMode != 'T')) {
		fMode = 'A';
		writeFlashByte(0x00000403, 'A');
	}
	serial.print("Feeder mode: ");
	serial.println(fMode);
	return fMode;
}

char setMode(char sMode) {
	writeFlashByte(0x00000403, sMode);
	serial.print("Feeder mode set to: ");
	serial.println(sMode);
	return sMode;
}

byte readFlashByte(unsigned long fAddress) {
	digitalWrite(csFlash, LOW);					// Activate flash chip
	SPI.transfer(0x03);							// Opcode for low freq read
	SPI.transfer((fAddress >> 16) & 0xFF);		// First of three address bytes
	SPI.transfer((fAddress >> 8) & 0xFF);		// Second address byte
	SPI.transfer(fAddress & 0xFF);				// Third address byte
	byte fByte = SPI.transfer(0);
	digitalWrite(csFlash, HIGH);				// Deactivate flash chip
	return fByte;
}

void writeFlashByte(unsigned long fAddress, byte fByte) {
	digitalWrite(csFlash, LOW);					// Activate flash chip
	SPI.transfer(0x58);							// Opcode for read modify write
	SPI.transfer((fAddress >> 16) & 0xFF);		// First of three address bytes
	SPI.transfer((fAddress >> 8) & 0xFF);		// Second address byte
	SPI.transfer(fAddress & 0xFF);				// Third address byte
	SPI.transfer(fByte);
	digitalWrite(csFlash, HIGH);				// Deactivate flash chip
	delay(20);
}

void inputTime() {								// Function to set the clock
	serial.println("Enter mmddyyhhmmss");		// Ask for user input
	while (serial.available() == 0) {}			// Wait for 12 characters to accumulate
	for (int n = 0; n < 13; n++) {				// Loop to read all the data from the serial buffer once it is ready
		timeIn[n] = serial.read();				// Read the characters from the buffer into an array of 12 bytes one at a time
	}
	while (serial.available())					// Clear the buffer, in case there were extra characters
	{
		serial.read();							// Read in the date and time data
	}
	// Transform the input into decimal numbers
	mo = ((timeIn[0] - 48) * 10 + (timeIn[1] - 48)); // Convert two ascii characters into a single decimal number
	da = ((timeIn[2] - 48) * 10 + (timeIn[3] - 48)); // Convert two ascii characters into a single decimal number
	yr = ((timeIn[4] - 48) * 10 + (timeIn[5] - 48)); // Convert two ascii characters into a single decimal number
	hh = ((timeIn[6] - 48) * 10 + (timeIn[7] - 48)); // Convert two ascii characters into a single decimal number
	mm = ((timeIn[8] - 48) * 10 + (timeIn[9] - 48)); // Convert two ascii characters into a single decimal number
	ss = ((timeIn[10] - 48) * 10 + (timeIn[11] - 48)); // Convert two ascii characters into a single decimal number
}


void printDirectory(File dir, int numTabs) {
	while (true) {

		File entry =  dir.openNextFile();
		if (! entry) {
			// No more files
			break;
		}
		for (uint8_t i = 0; i < numTabs; i++) {
			serial.print('\t');
		}
		serial.print(entry.name());
		if (entry.isDirectory()) {
			serial.println("/");
			printDirectory(entry, numTabs + 1);
		} else {
			// Files have sizes, directories do not
			serial.print("\t\t");
			serial.println(entry.size(), DEC);
		}
		entry.close();
	}
}


void saveLogSD(String event) { // Save log to log file in SD card
	File dataFile = SD.open("log.txt", FILE_WRITE);	// Initialize the SD card and open the file "datalog.txt" or create it if it is not there.
	if (dataFile) {
		dataFile.print(event);
		dataFile.print(": ");						// Space for data delineation
		if (rtc.updateTime() == false) {
			serial.print("RTC failed");				// Updates the time variables from RTC
		}
		String currentDate = rtc.stringDateUSA();	// Get the current date in mm/dd/yyyy format (we're weird in the US)
		// String currentDate = rtc.stringDate());	// Get the current date in dd/mm/yyyy format (Rest-of-the-world format)
		String currentTime = rtc.stringTime();		// Get the time
		String currentDateTime = currentDate + " " + currentTime ;
		dataFile.println(currentDateTime);			// Log the time
		dataFile.close();							// Close the file
		serial.println("saved log to SD card.");	// Serial output message to user
	} // Check dataFile is present
	else {
		serial.println("error opening log.txt");	// Error message if the "datafile.txt" is not present or cannot be created
	}// End check for file
}

void lpSleep() {
	digitalWrite(MOTR, HIGH) ;						// Must be set high to get low power working - don't know why
	digitalWrite(SHD_PINB, HIGH);					// Shut down both RFID readers
	digitalWrite(SHD_PINA, HIGH);
	attachInterrupt(INT1, ISR, FALLING);
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_EIC) |	// Configure EIC to use GCLK1 which uses XOSC32K
		GCLK_CLKCTRL_GEN_GCLK1   |					// This has to be done after the first call to attachInterrupt()
		GCLK_CLKCTRL_CLKEN;
	USB->DEVICE.CTRLA.reg &= ~USB_CTRLA_ENABLE;		// Disable USB
	SysTick->CTRL  &= ~SysTick_CTRL_ENABLE_Msk;
	__WFI();										// Enter sleep mode
	// ...Sleep...wait for interrupt
	USB->DEVICE.CTRLA.reg |= USB_CTRLA_ENABLE;		// Enable USB
	detachInterrupt(INT1);
	SysTick->CTRL  |= SysTick_CTRL_ENABLE_Msk;
}

void ISR() {}		// Dummy routine - not really needed??

/*
 *
 * WORK IN PROGRESS: BEWARE
 *
 */

#if 0

/* write to flash
 *
 * @param page - page in memory data will be written to
 * @param data - data that will be written to flash
 */
void write_flash_addr(byte page, unsigned long data)
{
	digitalWrite(CS_FLASH, LOW);		// Activate flash chip
	SPI.beginTransaction(SPI_DEFAULT);

	SPI.transfer(READ_MOD_WRITE);
	SPI.transfer(0x00);
	SPI.transfer(page<<2);				// Select page
	SPI.transfer(0x00);					// Select byte address 0

	for (int8_t i = 0x3; i >= 0x0; i--)
		SPI.transfer((data >> (i * 0x8)) & 0xFF);

	digitalWrite(CS_FLASH, HIGH);		// Deactivate flash chip
	SPI.endTransaction();
}

/* read from flash
 * @param page - page in memory data will be written to
 */
unsigned long read_flash_addr(byte page)
{
	unsigned long data = 0;

	digitalWrite(CS_FLASH, LOW);		// Activate flash chip
	SPI.beginTransaction(SPI_DEFAULT);

	SPI.transfer(READ_ARRAY);
	SPI.transfer(0x00);
	SPI.transfer(page<<2);				// Select page
	SPI.transfer(0x00);					// Select byte address 0

	for (int8_t i = 0x3; i >= 0x0; i--)
		data |= SPI.transfer(0x0) << (i * 0x8);

	digitalWrite(CS_FLASH, HIGH);		// Deactivate flash chip
	SPI.endTransaction();

	return data;
}

#endif
