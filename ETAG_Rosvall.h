
/*
   Data logging sketch for the ETAG RFID Reader
   Version 1.0
   Code by:
   Dmitrii Galantsev

   Licenced in the public domain

CHANGE LOG:
05-04-18 - Added interrupt driven RFID reader (jaywilhelm)
01-08-19 - Reformatted, Added set-to-compile-time option for rtc (dmitriigalantsev)
01-09-19 - Added BOARD_ID (dmitriigalantsev)
 */

/* Constants (Set up logging parameters here) */
// How long in milliseconds to check to see if a tag is present
// (Tag is only partially read during this time
// This is just a quick way of detirmining if a tag is present or not
#define CHECK_TIME 30
// How long in milliseconds to try to read a tag if a tag was initially
// detected (applies to both RF circuits, but that can be changed)
#define POLL_TIME1 200
// Minimim time in seconds between recording the same tag twice in a row
// (only applies to data logging--other operations are unaffected)
#define DELAY_TIME 8
// CRITICAL - This determines how long in milliseconds to wait between reading attempts.
// Make this wait time as long as you can and still maintain functionality (more PAUSE_TIME = more power saved)
#define PAUSE_TIME 500

/* Constants (sleep) */
#define SLEEP_H 21								// When to go to sleep at night - hour
#define SLEEP_M 0								// When to go to sleep at night - minute
#define WAKE_H 	7								// When to wake up in the morning - hour
#define WAKE_M 	0								// When to wake up in the morning - minute
#define SLEEP_INTERVAL 25						// How many seconds to sleep at a time, must be less than 1 min.
#define ON_TIME 1000							// How many MILLISECONDS to stay on between sleep intervals
#define SLEEP_TIME (SLEEP_H * 100 + SLEEP_M)	// Combined hours and minutes for sleep time
#define WAKE_TIME (WAKE_H * 100 + WAKE_M) 		// Combined hours and minutes for wake time

/* Constants (other) */
#define serial SerialUSB	// Designate the USB connection as the primary serial comm port
#define DEMOD_OUT_PIN	30  // (PB03) this is the target pin for the raw RFID data
#define SHD_PINA		8   // (PA06) Setting this pin high activates the primary RFID circuit (only one can be active at a time)
#define CS_SD			7   // Chip select for SD card - make this pin low to activate the SD card, also the clock interupt pin
#define CS_FLASH		2	// Chip select for flash memory
#define LED_RFID		31  // Pin to control the LED indicator.
#define INT1			7	// Interrupt pin

/* Debug Macros */
#ifndef DISABLE_DEBUG
#define PRINT_LOG(s)	{			\
	SerialUSB.print("[LOG]    ");	\
	SerialUSB.println(F(s));		\
}
#define PRINT_WARN(s)	{			\
	SerialUSB.print("[WARN]   ");	\
	SerialUSB.println(F(s));		\
}
#define PRINT_ERROR(s)	{			\
	SerialUSB.print("[ERROR]  ");	\
	SerialUSB.println(F(s));		\
}

#else

#define PRINT_LOG(s)
#define PRINT_WARN(s)
#define PRINT_ERROR(s)

#endif

/* Function headers */
byte readFlashByte(unsigned long fAddress);
unsigned long getFlashAddr();
void dumpMem();
void setClk();
void writeFlashAddr(unsigned long fAddress);
void writeFlashByte(unsigned long fAddress, byte fByte);
