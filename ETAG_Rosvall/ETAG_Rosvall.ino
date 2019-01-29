
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
Page 2 is reserved for RFID id
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
01-09-19 - Added BOARD_ID (dmitriigalantsev)
01-12-19 - Moved constants into the header file (dmitriigalantsev)
01-14-19 - Cleaned up more global variables, data logging is now consistent with gen2 (dmitriigalantsev)
 */

// ***********INITIALIZE INCLUDE FILES AND I/O PINS*******************
#include "SparkFun_RV1805.h"
#include "ETAG_Rosvall.h"
#include <Wire.h>			// Include the standard wire library - used for I2C communication with the clock
#include <SD.h>				// Include the standard SD card library
#include <SPI.h>

// TODO: reduce documentation (there are way too many comments)

RV1805 rtc;

// ************************* initialize variables******************************
uint32_t tagNo;							// Four least significant hexidecimals in 5 number tag code
uint32_t tagNo2;						// Four least significant hexidecimals in 5 number tag code
uint32_t timeSeconds;					// Stores the past Hour, Minute and Second of a read
uint32_t pastTimeSeconds;				// Stores the past Hour, Minute and Second of a read

String currentDate;						// USed to get the current date in mm/dd/yyyy format (we're weird in the US)
String currentDateTime;					// Full date and time string
String currentTime;						// Used to get the time
byte ss, mm, hh, da, mo, yr;			// Byte variables for storing date/time elements
unsigned int timeIn[12];				// Used for incoming serial data during clock setting

// Tag reading state variables
byte OneCounter;						// For counting the number of consecutive 1s in RFID input -- 9 ones signals the beginning of an ID code
byte RFIDbitCounter;					// Counts the number of bits that have gone into an RFID tag read
byte RFIDbyteCounter;					// Counts the number of bytes that have gone into an RFID tag read
byte RFIDbytes[11];						// Array of bytes for storing all RFID tag data (ID code and parity bits)
byte longPulseDetected = 0;				// A long pulse must first be read from the RFID output to begin reading a tag ID - This variable indicates whether a long pulse has happened
byte pastPulseLong = 0;					// Indicates whether the past pulse was long (1) or short (0).
byte rParity;							// Temporary storage of parity data.
unsigned int parityFail;				// Indicates if there was a parity mismatch (i.e. a failed read)
unsigned int pulseCount;				// For counting pulses from RFID reader

// Global variable for tag codes
unsigned long RFIDtagNumber = 0;		// Stores bytes 1 through 4 of a tag ID (user number)

// Board id
unsigned int BOARD_ID = 0;

// Additional debugging info
byte SDpresent;							// 1 if SD card is detected on startup.
unsigned int add_errors = 0;

/* The reader will output Serial data for a certain number of read cycles;
 * then it will start using a low power sleep mode during the PAUSE_TIME between read attempts.
 * The variable stopCycleCount determines how many read cycles to go
 * through before using the low-power sleep.
 * Once low-power sleep is enabled, the reader will not be able to output
 * serial data (but tag reading and data storage will still work).
 *
 * This may result in arduino not showing up as a serial device.
 * Press upload button in the Arduino IDE and then press the reset button on the board.
 * This should succesfully upload the new code to Arduino.
 */

/*******************************SETUP**************************************/
void setup()
{
	// Establish startup settings and I/O pins
	byte temp_id;
	char ch;

	Wire.begin();					// Enable I2C communication

	pinMode(LED_RFID, OUTPUT);		// Pin for controlling the on-board LED
	digitalWrite(LED_RFID, HIGH);   // Turn the LED off (LOW turns it on)

	// SD card and flash memory chip select pins - these pins enable SPI communication with these peripherals
	pinMode(CS_SD, OUTPUT);			// Chip select pin for SD card must be an output
	pinMode(CS_FLASH, OUTPUT);		// Chip select pin for Flash memory
	digitalWrite(CS_SD, HIGH); 	  	// Make both chip selects high (not selected)
	digitalWrite(CS_FLASH, HIGH);

	// Pins for controlling the RFID circuits
	pinMode(SHD_PINA, OUTPUT);		// Make the primary RFID shutdown pin an output.
	digitalWrite(SHD_PINA, HIGH);   // Turn the primary RFID circuit off (LOW turns on the EM4095)

	// Establish a a couple of parameters for low power sleep mode
	SYSCTRL->XOSC32K.bit.RUNSTDBY = 1;					// Set the XOSC32K to run in standby (not sure why this is needed)
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;					// Set sleep mode

	// Try to initiate a serial connection
	serial.begin(9600);

	// Slow flashing LED to create a time window for serial connection.
	blinkLED(5);

	serial.print(LOGO);
	serial.print("Ver: ");
	serial.print(VERSION);
	serial.println(PADDING);

	// Set up SD card communication
	PRINT_LOG("Initializing SD card...");
	if (!SD.begin(CS_SD)) {
		PRINT_WARN("SD card failed or not present"); 	// Initiate the SD card function with the pin that activates the card SD card error message
		SDpresent = 0;
		add_errors++;
	}
	else {
		PRINT_LOG("SD card online.");
		SDpresent = 1;
		//saveLogSD("LOGGING STARTED");
	}

	// Start real time clock interfcace and get the current time
	if (!rtc.begin()) {
		PRINT_ERROR("RTC FAILED TO START"); // Try initiation and report if something is wrong
		saveLogSD("[ERROR] RTC FAILED TO START");
		add_errors++;
	}
	if (!rtc.updateTime()) {
		// Updates the time variables from RTC, report if there's a failure
		PRINT_ERROR("RTC FAILED TO UPDATE");
		saveLogSD("[ERROR] RTC FAILED TO UPDATE");
		add_errors++;
	}
	if (rtc.getYear() == 0) {
		// Check if the year has been set
		PRINT_ERROR("RTC YEAR = 0");
		saveLogSD("[ERROR] RTC YEAR = 0");
		add_errors++;
	}
	PRINT_LOG("Clock is set to: "); 		// Serial message for clock output
	showTime();											// Function that displays the clock time

	// Set up communication with the flash memory
	SPI.begin();														// Enable SPI communication for Flash Memory
	SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0)); 	// Establish SPI parameters - Does this have to be done repeatedly
	unsigned long fAddressEnd2 = getFlashAddr();		// Get flash address
	fAddressEnd2 = getFlashAddr();						// Get flash address; has to be done twice?? why??
	PRINT_LOG("checking if Flash Memory is initialized: ");			// It is neccessary to initialize the flash memory on the first startup
	byte byte0 = readFlashByte(0x00000404);				// Read a particular byte from the flash memory
	serial.println(byte0, HEX);							// Display the byte
	if (byte0 == 0xFF) {								// If the byte is 0xFF then the flash memory needs to be initialized
		writeFlashByte(0x00000404, 0xAA);				// Write a different byte to this memory location
		PRINT_LOG("Initializing Flash Memory: ");		// Message
		writeFlashAddr(0x00000800);						// Set initial flash memory address for logging data to page 2, byte address 0
	}
	PRINT_LOG("Flash Memory IS initialized.");			// Confirm initialization
	getFlashAddr();										// Display the current flash memory data loggin address
	// WriteFlashByte(0x00000404, 0xFF);				// Uncomment to set flash memory initialization flag to 0xFF, which will cause the memory address to be reset on startup

	delay(1000);

	// Set up board ID
	if (readFlashByte(0x000800) == 0x0E)
		BOARD_ID = 0xE00 | readFlashByte(0x000801);
	else {
		PRINT_ERROR("No board ID found in flash!");
		saveLogSD("[ERROR] NO BOARD ID FOUND");
		add_errors++;
	}

	digitalWrite(CS_SD, HIGH); // SD card turned off for now

	// Set time to compiler time if flag is set
	#if defined(SET_TIME_ON_COMPILE) && SET_TIME_ON_COMPILE
		PRINT_LOG("Setting RTC to compiler time");
		set_rtc_to_compiler_time();
	#endif

/*******************************TESTS**************************************/
	run_tests(add_errors);

/*******************************MENU***************************************/
	// Display options menu to user
	byte menu = 1;
	while (menu == 1) {								// Keep displaying menu until an exit condition happens
		serial.println();
		if (!rtc.updateTime()) {
			serial.print("RTC failed"); 			// Updates the time variables from RTC
		}
		showTime();									// Show the current time
		serial.println("What to do?");				// Ask the user for instruction and display the options
		serial.println("	C/c = set clock to compile time");
		serial.println("	M/m = set clock manually");
		serial.println("	I/i = set reader ID");
		serial.println("	P/p = display reader ID");
		serial.println("	B/b = display backup memory");
		serial.println("	E/e = erase (reset) backup memory");
		serial.println("	Anything else = start logging");
		unsigned int serDelay = 0;							// If there's no response then eventually move on and just start logging
		while (!serial.available() && serDelay++ < 15000) 	// Wait about 15 seconds for a user response
			delay(1);
		if (!serial.available()) {							// If there is a response then perform the corresponding operation
			menu = 0;
			continue;
		}

		switch (serial.read()) {
			case 'c':
			case 'C':					// Try to set the clock to compiler time
				set_rtc_to_compiler_time();
				break;
			case 'm':
			case 'M':					// Option to set clock
				inputTime();			// Calls function to get time values from user
				if (!rtc.setTime(0, ss, mm, hh, da, mo, yr, 1)) {	// Attempt to set clock with input values
					PRINT_ERROR("Something went wrong setting the time");
				}
				break;
			case 'i':
			case 'I':
				serial.println("Input new ETAG id 2 digits");
				temp_id = 0;
				for (int i = 0; i < 2; i++) {
					ch = serial.read();
					if (ch == 255) {
						i--;
					} else if (!(ch >= '0' && ch <= '9')) {
						i = 0;
						temp_id = 0;
						PRINT_ERROR("You can only input values 0-9");
					} else {
						temp_id <<= 4;			// shift by 4 to convert to hex
						temp_id |= ch - '0';	// convert char to int
					}
				}
				PRINT_LOG("Your new ID is = ");
				serial.println(temp_id, HEX);
				write_id_to_flash(temp_id);
				BOARD_ID = 0xE00 | temp_id;		// update BOARD_ID
				PRINT_LOG("ID in FLASH (MUST MATCH THE INPUT ID) = ");
				serial.print(readFlashByte(0x000800), HEX);
				serial.println(readFlashByte(0x000801), HEX);
				if (readFlashByte(0x000801) != temp_id)
					PRINT_ERROR("IDs DON'T MATCH!");
				break;
			case 'p':
			case 'P':
				PRINT_LOG("ID in FLASH = ");
				serial.print(readFlashByte(0x000800), HEX);
				serial.println(readFlashByte(0x000801), HEX);
				break;
			case 't':
			case 'T':
				run_tests(add_errors);
				break;
			case 'b':
			case 'B':
				dumpMem();  			// Display backup memory; calls function dumpmem(); break and show menu again
				break;
			case 'e':
			case 'E':
				writeFlashAddr(0x0800); // Reinitialize flash memory by setting memory address to byte 0 of page 2
				PRINT_LOG("flash address = ");
				serial.println(getFlashAddr(), BIN);
				break;
			default:
				menu = 0;
				break;
		}
	}

	// Prepare for data logging
	serial.println("Scanning for tags...\n");	// Message to user


	// Make a file BOARD_IDDATA.txt if it doesn't exist, and add BOARD_ID to the top
	digitalWrite(CS_SD, LOW);
	if (!SD.exists(String(String(BOARD_ID, HEX) + "DATA.txt"))) {
		File dataFile = SD.open(String(String(BOARD_ID, HEX) + "DATA.txt"), FILE_WRITE);
		dataFile.println(String(String(BOARD_ID, HEX) + "DATA"));	// write board id to the file
		dataFile.close();
	}
	// Write message to sd card
	saveLogSD("LOGGING STARTED");
	// Turn off the sd card
	digitalWrite(CS_SD, HIGH);
}

// ******************************MAIN PROGRAM*******************************

/* this is the main function, loops forever */
void loop()
{
	if (!rtc.updateTime()) PRINT_ERROR("RTC failed at beginning of loop");

	showTime();
	byte RFIDtagArray[5];											// Stores the five individual bytes of a tag ID.
	String RFIDstring = "";
	// combined hours and minutes into 1 value
	if (rtc.getHours() * 100 + rtc.getMinutes() == SLEEP_TIME) {
		digitalWrite(SHD_PINA, HIGH);					// Shut down RFID reader
		// Serial.println("seting alarm");
		rtc.setAlarm(0, WAKE_M, WAKE_H, 1, 1);			// Second, minute, hour, date, month
		rtc.enableInterrupt(INTERRUPT_AIE);
		rtc.setAlarmMode(4);
		saveLogSD("SLEEP STARTED");						// Note for the log
		pinMode(CS_SD, INPUT);							// Make the SD card pin an input so it can serve as interrupt pin
		lpSleep();										// Call sleep funciton
		// ............//								// Sleep happens here
		blipLED(30);									// Blink indicator - processor reawakened
		rtc.stopTimer();
		if(SDpresent){
			pinMode(CS_SD, OUTPUT);						// Chip select pin for SD card must be an output for writing to SD card
			SD.begin(CS_SD);
			saveLogSD("SLEEP ENDED");
		}
		//rtc.updateTime();
	}
	serial.println("Scanning RFID circuit "); 	// Message part 1: Tell the user which circuit is active

	// Attempt tag read
	if (FastRead(DEMOD_OUT_PIN, CHECK_TIME, POLL_TIME)) {

		// The following is executed if a tag is detected
		RFIDstring = processTag(RFIDtagArray);   // Parse tag data into string and hexidecimal formats
		// Updates the time variables from Real Time Clock
		if (!rtc.updateTime()) PRINT_ERROR("RTC failed after tag read ");
		timeSeconds = (rtc.getDate() * 86400) + (rtc.getHours() * 3600) + (rtc.getMinutes() * 60) + rtc.getSeconds();
		tagNo = RFIDtagNumber;						// Stores the 4 least significant tag ID numbers - good for all tag comparisons I think.
		serial.print(RFIDstring);					// Call a subroutine to display the tag data via serial USB
		serial.print(" detected ");  	// Message part 1: add a note about which atenna was used
		showTime();									// Message part 2: display the time
		flashLED();									// Flash the LED briefly to indicate a tag read
		if((timeSeconds < pastTimeSeconds + DELAY_TIME) & tagNo == tagNo2){ // If everything matches up, the read is a repeat - don't log it.
			serial.println("Repeat read - data not logged.");
		} else {
			if(SDpresent){
				serial.println("Logging tag data");
				logRFID_To_SD(currentDateTime, RFIDstring);		// Call function to log data to the SD card
			}
			serial.println();
			writeRFID_To_FlashLine(RFIDtagArray);				// Call function to log to backup memory
			pastTimeSeconds = (rtc.getDate() * 86400) + (rtc.getHours() * 3600) + (rtc.getMinutes() * 60) + rtc.getSeconds();
			tagNo2 = tagNo;							// Set tagNo2 to current tag number for future comparisons // For keeping track of delaytime.
		}
	}

	digitalWrite(SHD_PINA, HIGH);		// Shut down RFID

	if (serial) {
		delay(PAUSE_TIME);					// Pause between polling attempts
	} else {
		byte pauseInterval = (PAUSE_TIME * 64)/1000;
		rtc.setRptTimer(pauseInterval, 1);	// Set timer and use frequency of 64 Hz
		pinMode(CS_SD, INPUT);				// Make the SD card pin an input so it can serve as interrupt pin
		rtc.startTimer();					// Start the timer
		lpSleep();							// Call sleep funciton
		rtc.stopTimer();
		if(SDpresent) {
			pinMode(CS_SD, OUTPUT);			// Chip select pin for SD card must be an output for writing to SD card
			SD.begin(CS_SD);
		}
	}
}


// *********************SUBROUTINES************************** //
// The Following are all subroutines called by the code above //

/* attempt to read the RFID tag
 *
 * @param demodOutPin - RFID data pin
 * @param checkDelay  - delay after attaching the interrupt
 * @param readTime    - how long to try and read the tag
 */
byte FastRead(int demodOutPin, byte checkDelay, unsigned int readTime)
{
	digitalWrite(SHD_PINA, LOW);			// Turn on primary RFID circuit
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

	delay(checkDelay);
	if (pulseCount > (checkDelay - 25)) {		// May want a separate variable for threshold pulse count.
		while (millis() < stopMillis & parityFail != 0) {
			delay(1);
		}
	} else {
		detachInterrupt(digitalPinToInterrupt(demodOutPin));
		digitalWrite(SHD_PINA, HIGH);			// Turn off primary RFID circuit
		// Serial.print("nothing read... ");
		return (0);
	}

	detachInterrupt(digitalPinToInterrupt(demodOutPin));
	digitalWrite(SHD_PINA, HIGH);				// Turn off primary RFID circuit
	if (parityFail == 0) {
		serial.println("parityOK... ");
		return (1);
	} else {
		serial.println("parity fail... ");
		return (0);
	}

}

/* convert from byte array to string
 *
 * @param RFIDtagArray - array of RFID bytes to be decoded
 */
String processTag(byte RFIDtagArray[5])
{
	// Process each byte (could do a loop but.....)
	RFIDtagArray[0] = ((RFIDbytes[0] << 3) & 0xF0) + ((RFIDbytes[1] >> 1) & 0x0F);
	String StringOne = String(RFIDtagArray[0], HEX);
	if(RFIDtagArray[0] < 0x10) {StringOne = String("0" + StringOne);}

	RFIDtagArray[1] = ((RFIDbytes[2] << 3) & 0xF0) + ((RFIDbytes[3] >> 1) & 0x0F);
	String StringTwo = String(RFIDtagArray[1], HEX);
	if(RFIDtagArray[1] < 0x10) {StringTwo = String("0" + StringTwo);}
	RFIDtagNumber = RFIDtagArray[1] << 24;

	RFIDtagArray[2] = ((RFIDbytes[4] << 3) & 0xF0) + ((RFIDbytes[5] >> 1) & 0x0F);
	String StringThree = String(RFIDtagArray[2], HEX);
	if(RFIDtagArray[2] < 0x10) {StringThree = String("0" + StringThree);}
	RFIDtagNumber += (RFIDtagArray[2] << 16);

	RFIDtagArray[3] = ((RFIDbytes[6] << 3) & 0xF0) + ((RFIDbytes[7] >> 1) & 0x0F);
	String StringFour = String(RFIDtagArray[3], HEX);
	if(RFIDtagArray[3] < 0x10) {StringFour = String("0" + StringFour);}
	RFIDtagNumber += (RFIDtagArray[3] << 8);

	RFIDtagArray[4] = ((RFIDbytes[8] << 3) & 0xF0) + ((RFIDbytes[9] >> 1) & 0x0F);
	String StringFive = String(RFIDtagArray[4], HEX);
	if(RFIDtagArray[4] < 0x10) {StringFive = String("0" + StringFive);}
	RFIDtagNumber += RFIDtagArray[4];
	String RFIDstring = String(StringOne + StringTwo + StringThree + StringFour + StringFive);
	RFIDstring.toUpperCase();
	return RFIDstring;
}


/* interrupt for decoding the RFID data */
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

/* update date and time strings */
void showTime()
{
	currentDate = String(rtc.stringDateUSA()).substring(0, 6); 				// Get the current date in mm/dd format
	currentDate = currentDate + String(rtc.stringDateUSA()).substring(8); 	// Get yy
	// currentDate = String(rtc.stringDate()).substring(0, 6); 				// Get the current date in dd/mm format
	// currentDate = currentDate + String(rtc.stringDate()).substring(8); 	// Get yy
	currentTime = rtc.stringTime();											// Get the time
	currentDateTime = currentDate + " " + currentTime ;
	serial.println(currentDateTime);
}

/* flash the LED */
void flashLED()
{
	for (unsigned int n = 0; n < 100 ; n = n + 30) {	// Loop to Flash LED and delay reading after getting a tag
		digitalWrite(LED_RFID, LOW);					// The approximate duration of this loop is determined by the readFreq value
		delay(5);										// It determines how long to wait after a successful read before polling for tags again
		digitalWrite(LED_RFID, HIGH);
		delay(25);
	}
}

/* turn on the LED repeats amount of times
 *
 * @param repeats - blink the LED n-amount of times
 */
void blinkLED(byte repeats)
{
	for (int i = 0; i < repeats; i++)	// Flash LED 5 times while waiting for serial connection to come online
	{
		delay(500);						// Pause for 0.5 seconds
		digitalWrite(LED_RFID, LOW);	// Turn the LED on (LOW turns it on)
		delay(500);						// Pause again
		digitalWrite(LED_RFID, HIGH);   // Turn the LED off (HIGH turns it off)
	}
}

/* turn the LED on for blip ms
 *
 * @param blip - amount of ms to keep the LED on for
 */
void blipLED(byte blip)
{
	digitalWrite(LED_RFID, LOW);	// Turn the LED on (LOW turns it on)
	delay(blip);					// Leave LED on for a quick flash
	digitalWrite(LED_RFID, HIGH);   // Turn the LED off (HIGH turns it off)
}


/* log the RFID data to SD card
 *
 * @param timeString - scan time
 * @param RFIDstring - rfid data
 */
void logRFID_To_SD(String timeString, String RFIDstring)
{
	File dataFile = SD.open(String(String(BOARD_ID, HEX) + "DATA.txt"), FILE_WRITE);
	if (dataFile) {
		/*for (int n = 0; n < 5; n++) {				// Loop to print out the RFID code to the SD card
		  if (tagData[n] < 10) dataFile.print("0");	// Add a leading zero if necessary
		  dataFile.print(tagData[n], HEX);			// Print to the SD card
		  }*/

		dataFile.print(RFIDstring);
		dataFile.print(" ");						// Character for data delineation
		dataFile.println(timeString);				// Log the time
		dataFile.close();							// Close the file
		PRINT_LOG("saved to SD card.");				// Serial output message to user
	}
	else {
		// Error message if the "datafile.txt" is not present or cannot be created
		PRINT_ERROR("error opening DATA.txt");
	}
}


/* empty flash page 0 */
void erasePage0()
{
	SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
	digitalWrite(CS_SD, HIGH);	// Make sure the SD card select is off
	digitalWrite(CS_FLASH, LOW);		// Activate flash chip
	SPI.transfer(0x81);				// Opcode for page erase
	SPI.transfer(0x00);				// First of three address bytes
	SPI.transfer(0x00);				// Second address byte
	SPI.transfer(0x00);				// Third address byte
	digitalWrite(CS_FLASH, HIGH);	// Deactivate flash chip - allow erase to happen
}

/* dump some of the flash memory */
void dumpMem()
{
	SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
	digitalWrite(CS_SD, HIGH);	// Make sure the SD card select is off
	char tagData[16];
	serial.println("Transmitting data from backup memory.");
	getFlashAddr();
	delay(10);
	unsigned long fAddressEnd = getFlashAddr();		// Get flash address
	serial.print("last flash memory address: ");
	serial.println(fAddressEnd, DEC);
	serial.println(fAddressEnd, BIN);
	unsigned long fAddress = 0x00000800;			// First address for stored data
	serial.print("first flash memory: ");
	serial.println(fAddress, DEC);
	serial.println(fAddress, BIN);

	while (fAddress < fAddressEnd) {
		serial.print("starting loop");
		digitalWrite(CS_FLASH, LOW);					// Activate flash chip
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
			mo = SPI.transfer(0);					// Read in date and time
			da = SPI.transfer(0);
			yr = SPI.transfer(0);
			hh = SPI.transfer(0);
			mm = SPI.transfer(0);
			ss = SPI.transfer(0);
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
		digitalWrite(CS_FLASH, HIGH);				// Turn off flash
		if (fAddress >= fAddressEnd) break;			// Break if we are at the end of the backup data stream
		delay(10);									// Wait a bit
	}
}

/* get the address counter for the flash memory from page 1 byte 0 */
unsigned long getFlashAddr()
{
	digitalWrite(CS_SD, HIGH);						// Make sure the SD card select is off
	digitalWrite(CS_FLASH, LOW);
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
	digitalWrite(CS_FLASH, HIGH);					// Deactivate flash chip
	// Serial.print("flash address = ");
	// Serial.println(fAddress, BIN);
	return fAddress;
}

/* write the address counter for the flash memory
 *
 * @param fAddress - address that will be written to page 1 byte 0
 */
void writeFlashAddr(unsigned long fAddress)
{
	SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
	digitalWrite(CS_SD, HIGH);					// Make sure the SD card select is off
	digitalWrite(CS_FLASH, LOW);				// Activate flash chip
	SPI.transfer(0x58);							// Opcode for read modify write
	SPI.transfer(0x00);							// First of three address bytes
	SPI.transfer(0x04);							// 00000100  second address byte - selects page 1
	SPI.transfer(0x00);							// Third address byte selects byte address 0
	SPI.transfer(fAddress >> 16);				// Write most significant byte of Flash address
	SPI.transfer((fAddress >> 8) & 0xFF);		// Second address byte
	SPI.transfer(fAddress & 0xFF);				// Third address byte
	digitalWrite(CS_FLASH, HIGH);				// Deactivate flash chip
	delay(20);
}

/* write rfid byte array to flash
 *
 * @param RFIDtagArray - array of RFID bytes
 */
void writeRFID_To_FlashLine(byte RFIDtagArray[5])
{
	unsigned long fAddress = getFlashAddr();	// Get the current flash memory address
	serial.print("transferring to address: ");
	serial.println(fAddress, BIN);
	digitalWrite(CS_FLASH, LOW);				// Activate flash chip
	SPI.transfer(0x58);							// Opcode for read modify write
	SPI.transfer((fAddress >> 16) & 0xFF);		// Write most significant byte of Flash address
	SPI.transfer((fAddress >> 8) & 0xFF);		// Second address byte
	SPI.transfer(fAddress & 0xFF);				// Third address byte
	for (int n = 0; n < 6; n++) {				// Loop to log the RFID code to the flash
		SPI.transfer(RFIDtagArray[n]);
	}
	SPI.transfer(rtc.getMonth());
	SPI.transfer(rtc.getDate());
	SPI.transfer(rtc.getYear());
	SPI.transfer(rtc.getHours());
	SPI.transfer(rtc.getMinutes());
	SPI.transfer(rtc.getSeconds());
	digitalWrite(CS_FLASH, HIGH);				// Deactivate flash chip
	unsigned int bAddress = fAddress & 0x03FF;	// And with 00000011 11111111 to isolate byte address
	bAddress = bAddress + 11;					// Add 12 to accound for new bytes (5 for RFID and 6 for date/time)
	if (bAddress > 500) {						// Stop writing if beyond byte address 500 (this is kind of wasteful)
		fAddress = (fAddress & 0xFFFFC00) + 0x0400;   // Set byte address to zero and add 1 to the page address
	} else {
		fAddress = (fAddress & 0xFFFFC00) + bAddress; // Just add to the byte address
	}
	delay(20);
	writeFlashAddr(fAddress);					// Write the updated address to flash.
	serial.println("saved to flash.");			// Serial output message to user
}

/* read byte from the memory
 *
 * @param fAddress - address to read from
 */
byte readFlashByte(unsigned long fAddress)
{
	digitalWrite(CS_FLASH, LOW);					// Activate flash chip
	SPI.transfer(0x03);							// Opcode for low freq read
	SPI.transfer((fAddress >> 16) & 0xFF);		// First of three address bytes
	SPI.transfer((fAddress >> 8) & 0xFF);		// Second address byte
	SPI.transfer(fAddress & 0xFF);				// Third address byte
	byte fByte = SPI.transfer(0);
	digitalWrite(CS_FLASH, HIGH);				// Deactivate flash chip
	return fByte;
}

/* write byte to flash
 *
 * @param fAddress - memory address
 * @param fByte - byte to be written to fAddress
 */
void writeFlashByte(unsigned long fAddress, byte fByte)
{
	digitalWrite(CS_FLASH, LOW);				// Activate flash chip
	SPI.transfer(0x58);							// Opcode for read modify write
	SPI.transfer((fAddress >> 16) & 0xFF);		// First of three address bytes
	SPI.transfer((fAddress >> 8) & 0xFF);		// Second address byte
	SPI.transfer(fAddress & 0xFF);				// Third address byte
	SPI.transfer(fByte);
	digitalWrite(CS_FLASH, HIGH);				// Deactivate flash chip
	delay(20);
}

/* manually set the clock */
void inputTime()
{
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

/* save log to log file in SD card
 *
 * @param event - string to be written to SD card
 */
void saveLogSD(String event)
{
	File dataFile = SD.open("log.txt", FILE_WRITE);	// Initialize the SD card and open the file "datalog.txt" or create it if it is not there.
	if (dataFile) {
		dataFile.print(event);
		dataFile.print(": ");						// Space for data delineation
		if (!rtc.updateTime()) {
			serial.print("RTC failed");				// Updates the time variables from RTC
		}
		String currentDate = rtc.stringDateUSA();	// Get the current date in mm/dd/yyyy format (we're weird in the US)
		// String currentDate = rtc.stringDate());	// Get the current date in dd/mm/yyyy format (Rest-of-the-world format)
		String currentTime = rtc.stringTime();		// Get the time
		String currentDateTime = currentDate + " " + currentTime ;
		dataFile.print(currentDateTime);			// Log the time
		dataFile.print(" BOARD_ID = ");
		dataFile.println(BOARD_ID, HEX);
		dataFile.close();							// Close the file
		PRINT_LOG("saved log to SD card.");	// Serial output message to user
	} // Check dataFile is present
	else {
		PRINT_ERROR("error opening log.txt");	// Error message if the "datafile.txt" is not present or cannot be created
	}// End check for file
}

/* sleep during night */
void lpSleep()
{
	attachInterrupt(INT1, ISR, FALLING);
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_EIC) |	// Configure EIC to use GCLK1 which uses XOSC32K
		GCLK_CLKCTRL_GEN_GCLK1   |					// This has to be done after the first call to attachInterrupt()
		GCLK_CLKCTRL_CLKEN;
	USB->DEVICE.CTRLA.reg &= ~USB_CTRLA_ENABLE;		// Disable USB
	SysTick->CTRL  &= ~SysTick_CTRL_ENABLE_Msk;
	__WFI();										// Enter sleep mode
	// ...Sleep...wait for interrupt
	detachInterrupt(INT1);
	SysTick->CTRL  |= SysTick_CTRL_ENABLE_Msk;
}

void ISR() {}		// Dummy routine - not really needed??

/* write ID to flash
 *
 * @param id - id of the board
 */
void write_id_to_flash(unsigned int id)
{
	SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
	digitalWrite(CS_SD, HIGH);					// Make sure the SD card select is off
	digitalWrite(CS_FLASH, LOW);				// Activate flash chip
	SPI.transfer(0x58);							// Opcode for read modify write
	SPI.transfer(0x00);							// First of three address bytes
	SPI.transfer(0x08);							// 00001000 second address byte - selects page 2
	SPI.transfer(0x00);							// Third address byte selects byte address 0

	SPI.transfer(0x0E);							// E
	SPI.transfer(id);							// ID number
	digitalWrite(CS_FLASH, HIGH);				// Deactivate flash chip
	delay(20);
}

/* run a number of tests, recommended to run before deployment
 *
 * @param add_errors - additional errors found
 */
void run_tests(unsigned int add_errors)
{
	unsigned int num_of_errors = 0;
	if (num_of_errors + add_errors > 0) {
		PRINT_WARN("[TEST] PRE-SETUP tests failed!");
		PRINT_WARN("[TEST] Errors found: ");
		serial.print("[WARN]    [TEST] ");
		serial.println(num_of_errors + add_errors, DEC);
	} else {
		PRINT_LOG("[TEST] All tests passed!");
	}
}

/* set rtc time */
void set_rtc_to_compiler_time()
{
	if (!rtc.setToCompilerTime()) {
		PRINT_ERROR("Can't set clock to compiler time.\nTry option (M).");
	}
	else {
		PRINT_LOG("Clock set successfully");
	}
}
