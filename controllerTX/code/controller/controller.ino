#include <SPI.h>
#include "RF24.h"

// The following define decides whether we have an
// "old style" adapter with a mode switch or a new
// one with a status LED.
//#define OLDADAP

//#define DEBUG
//#define CHECKTXRX
#define DEBUGNOACKBUT
//#define DEBUGNOACKSTAT

// Radio-related things.
// TX Address
const uint64_t rf24_txAddr = 0xC0FFEEBEEF;
const uint8_t rf24_chnl = 100;
const rf24_datarate_e  rf24_rate = RF24_250KBPS;
const rf24_crclength_e  rf24_crc = RF24_CRC_8;
const uint8_t rf24_PA_level = RF24_PA_LOW;
const uint8_t rf24_retries = 1;
const uint8_t rf24_delay = 4;

// Radio. 
// CEPIN: 9
// CSPIN: 10
RF24 radio(9,10);


// N64 related stuff.
// Port defintions. We use D2 for output and D3 for input.
// We do not use the same port to avoid additional time to 
// switch port directions.
// Both are bank D.
// If the values are changed here, some of the ASM code
// needs to be adjusted as well.
#define OUTPORT 2
#define INPORT 3

// Memory or Rumble Mode switch (old adapter).
#define EXTMODEPIN 4

// Status LED pin (new adapter).
#define STATLEDPIN 5

// How often the status should be sent per button reads.
#define STATINTERVAL 2

// Some NOP definitions.
// Yes, this is very pretty. But we do not have code size problems,
// hence using stupid sequences of NOPs makes it nicely timing 
// predictable.
#define NOP1 asm volatile( "nop\n" )
#define NOP5 NOP1; NOP1; NOP1; NOP1; NOP1
#define NOP10 NOP5; NOP5
#define NOP20 NOP10; NOP10
#define NOP40 NOP20; NOP20

// Write low. -> takes 0.125 us
// First set the port to 0 and then change the mode to output.
#define WRITELOW asm (\
  "cbi %0, %1 \n"\
  : : "I" (_SFR_IO_ADDR(PORTD)), "I" (PORTD2) \
); \
asm ( \
  "sbi %0, %1 \n" \
  : : "I" (_SFR_IO_ADDR(DDRD)), "I" (DDD2)\
)

// Write high. -> takes 0.125 us
// First set the mode to input and then nop.
#define WRITEHIGH asm (\
  "cbi %0, %1 \n"\
  : : "I" (_SFR_IO_ADDR(DDRD)), "I" (DDD2) \
); \
asm ( "nop\n" )

// Write an N64 0.
#define WRITE0 WRITELOW;\
NOP40; NOP1; NOP1; NOP1;\
WRITEHIGH;\
NOP10; NOP1; NOP1; NOP1; NOP1

// Write an N64 1.
#define WRITE1 WRITELOW;\
NOP10; NOP1; NOP1;\
WRITEHIGH;\
NOP40; NOP5

#define WRITESTOP WRITELOW;\
NOP10; NOP1; NOP1;\
WRITEHIGH

// Write a short ended 0.
// Useful for inside loops.
#define WRITE0S WRITELOW;\
NOP40; NOP1; NOP1; NOP1;\
WRITEHIGH;\
NOP10

// Write a short ended 1.
// Useful for inside loops.
#define WRITE1S WRITELOW;\
NOP10; NOP1; NOP1;\
WRITEHIGH;\
NOP40

// Polling freqeuncy
#define DEL 10

// Number of bits in a button response.
#define BUTTONLEN 4 * 8 + 1

// Array of button response bits.
int8_t buttonDat[ BUTTONLEN ];

// Compressed array.
uint8_t buttonCmp[ 4 ];

// Flag set high if new buttons to transmit
// are available.
uint8_t newButtonsAvail = 0;

// Debugging code.
#ifdef CHECKTXRX
uint8_t errCnt = 0;
uint8_t readCnt = 0;
#endif

// Read controller buttons.
void readButtons() {
  // Disable interrupts.
  noInterrupts();
  
  uint8_t tries = 0;
  
_readMain:
  // Init array.
  #ifdef DEBUG
  Serial.println( "->Reading buttons." );
  Serial.flush();
  #endif
  for ( uint8_t i = 0; i < BUTTONLEN; ++i ) {
    buttonDat[ i ] = -127;
  }

  // The request is written out by a very straight-forward sequence of 
  // instructions. As there is currently no code-size problem, this 
  // is ok and nicely timing predictable.
  
  // Write 0x01
  WRITE0;
  WRITE0;
  WRITE0;
  WRITE0;
  WRITE0;
  WRITE0;
  WRITE0;
  WRITE1;

  //Stop bit
  WRITESTOP;

  // ASM routine to read controller answer.
  // Measure the time of each signal bit being
  // high and low, then store the difference
  // in an array. Use the overflow flag for
  // timeouts or the end of transmissions.
  asm volatile(
    "ldi r18, 64\n"
    "ldi r19, 64\n"
    "ldi r20, 64\n"
    "ldi r26, lo8(buttonDat)\n"
    "ldi r27, hi8(buttonDat)\n"

    "_INITWAITHIGH:\n"
    "inc r20\n"
    "brmi _TIMEOUT\n"
    "sbic %[bankD], 3\n"
    "rjmp _INITWAITHIGH\n"
    
    "_WAITLOW:\n"
    "inc r18\n"
    "brmi _TIMEOUT\n"
    "sbis %[bankD], 3\n"
    "rjmp _WAITLOW\n"

    "_WAITHIGH:\n"
    "inc r19\n"
    "brmi _TIMEOUT\n"
    "sbic %[bankD], 3\n"
    "rjmp _WAITHIGH\n"

    "sub r18, r19\n"
    "st x+, r18\n"
    "ldi r18, 64\n"
    "ldi r19, 64\n"
    "rjmp _WAITLOW\n"
    
    "_TIMEOUT:\n"
    :
    :[bankD] "I" (_SFR_IO_ADDR(PIND) )
    : "r18", "r19", "r20", "r26", "r27", "memory" );

    // Create the array to be sent over to the console.
    uint8_t tmp = 0;
    uint8_t valid = 1;
    for ( uint8_t i = 0; i < BUTTONLEN-1; ++i ) {
      
     // None should be as large as -127 (init value).
     if ( buttonDat[ i ] == -127 ) {
      // Some missreading.
      valid = 0;
      break;
     }
     uint8_t j = i % 8;

     if ( buttonDat[ i ] <= 0 ) {
      // Was pressed.
      tmp |= ( 1 << ( 7 - j ) );
     }

     // Store.
     if ( j == 7 ) {
      int index = ( ( BUTTONLEN - 2 ) / 8 ) - ( i / 8 );
      buttonCmp[ index ] = tmp;
      tmp = 0;
     }
    }

    // Try twice if something went wrong.
    tries++;
    if ( !valid && tries < 2 )
      goto _readMain;
      

    // If it is a valid reading, set flag.
    newButtonsAvail = valid;
    
    // Re-enable interrupts.
    interrupts();
  
}

// Number of bits in a status response.
#define STATLEN 8 * 3

// An array of the single received bits.
int8_t statRaw[ STATLEN ];

// The actual "compressed" bits to transmit later.
uint8_t statBit[ 3 ];

// And the actual status:
// 0000 0000: MemPack Mode; nothing plugged in.
// 0000 0001: MemPack Mode; something plugged in.
// 0001 0000: Rumble Mode; nothing plugged in.
// 0001 0001: Rumble Mode; something plugged in.
uint8_t stat;

#ifndef OLDADAP
// Mode variable.
// 0 -> MemPack
// 1 -> Local
uint8_t curMode = 0;
// Keep track if mode switch button combo was pressed.
uint8_t modeSwitchPress = 0;
#endif

// Flag.
uint8_t newStatAvail = 0;

void getStatus() {
  uint8_t tries = 0;
  #ifdef DEBUG
  Serial.println( "Trying to get the status." );
  Serial.flush();
  #endif
  
_readMainStat:
  // Init array.
  for ( uint8_t i = 0; i < STATLEN; ++i ) {
    statRaw[ i ] = -127;
  }

  noInterrupts();

  // The request is written out by a very straight-forward sequence of 
  // instructions. As there is currently no code-size problem, this 
  // is ok and nicely timing predictable.
  
  // Write 0x00
  WRITE0;
  WRITE0;
  WRITE0;
  WRITE0;
  WRITE0;
  WRITE0;
  WRITE0;
  WRITE0;

  //Stop bit
  WRITESTOP;

  // ASM routine to read controller answer.
  // Measure the time of each signal bit being
  // high and low, then store the difference
  // in an array. Use the overflow flag for
  // timeouts or the end of transmissions.
  asm volatile(
    "ldi r18, 64\n"
    "ldi r19, 64\n"
    "ldi r20, 64\n"
    "ldi r26, lo8(statRaw)\n"
    "ldi r27, hi8(statRaw)\n"

    "_INITWAITHIGHSTAT:\n"
    "inc r20\n"
    "brmi _TIMEOUTSTAT\n"
    "sbic %[bankD], 3\n"
    "rjmp _INITWAITHIGHSTAT\n"
    
    "_WAITLOWSTAT:\n"
    "inc r18\n"
    "brmi _TIMEOUTSTAT\n"
    "sbis %[bankD], 3\n"
    "rjmp _WAITLOWSTAT\n"

    "_WAITHIGHSTAT:\n"
    "inc r19\n"
    "brmi _TIMEOUTSTAT\n"
    "sbic %[bankD], 3\n"
    "rjmp _WAITHIGHSTAT\n"

    "sub r18, r19\n"
    "st x+, r18\n"
    "ldi r18, 64\n"
    "ldi r19, 64\n"
    "rjmp _WAITLOWSTAT\n"
    
    "_TIMEOUTSTAT:\n"
    :
    :[bankD] "I" (_SFR_IO_ADDR(PIND) )
    : "r18", "r19", "r20", "r26", "r27", "memory" );

    // Create the array to be sent.
    uint8_t extAvail = 0;
    uint8_t valid = 1;
    uint8_t tmp = 0;
    for ( uint8_t i = 0; i < STATLEN; ++i ) {
      
     // None should be as large as -127 (init value).
     if ( statRaw[ i ] == -127 ) {
      // Some missreading.
      valid = 0;
      break;
     }
     uint8_t j = i % 8;

     if ( statRaw[ i ] <= 0 ) {
      // Was pressed.
      tmp |= ( 1 << ( 7 - j ) );
     }

     // Store.
     if ( j == 7 ) {
      int index = i / 8;
      statBit[ index ] = tmp;
      tmp = 0;
     }
    }

    // Try twice.
    tries++;
    if ( !valid && tries < 2 )
      goto _readMainStat;

    // Check if the reading made sense and
    // if something is plugged in.

    #ifdef DEBUG
    Serial.print( "Status read valid: " );
    Serial.println( valid );
    Serial.println( "Status bits:" );
    for ( int i = 0; i < 3; ++i ) {
      Serial.print( statBit[ i ] );
      Serial.print( "  " );
    }
    Serial.println( "\n" );
    Serial.flush();
    #endif
    
    if ( valid == 1 && statBit[ 0 ] == 0b00000101 && statBit[ 1 ] == 0 ) {
      if ( statBit[ 2 ] == 0b00000001 ) {
        // Controller pak plugged in.
        stat = 0x01;
        
      } else if ( statBit[ 2 ] == 0b00000010 || statBit[ 2 ] == 0b00000000 ) {
        // No controller pak plugged in.
        stat = 0x00;
        
      } else {
        valid = 0;
      }

    } else {
      valid = 0;
    }

    if ( valid ) {
      #ifdef OLDADAP
      // Read the mode switch.
      uint8_t mode = digitalRead( EXTMODEPIN );
      // DEBUG
      mode = 1;
      stat |= ( mode << 4 );

      #else
      stat |= ( curMode << 4 );
      #endif
    }

    // If it is a valid reading, set flag.
    newStatAvail = valid;

    interrupts();
}

// Function to turn on or off rumble feature.
void rumble( bool turnOn ) {
  noInterrupts();
  uint8_t datByteSent = 0;
  // First send write command and the address.
  WRITE0;
  WRITE0;
  WRITE0;
  WRITE0;
  
  WRITE0;
  WRITE0;
  WRITE1;
  WRITE1;
  
  
  WRITE1;
  WRITE1;
  WRITE0;
  WRITE0;
  
  WRITE0;
  WRITE0;
  WRITE0;
  WRITE0;
  
  
  WRITE0;
  WRITE0;
  WRITE0;
  WRITE1;
  
  WRITE1;
  WRITE0;
  WRITE1;
  WRITE1S;

  // And now the data.
  if ( turnOn ) {
_TURNON:
  WRITE0;
  WRITE0;
  WRITE0;
  WRITE0;

  WRITE0;
  WRITE0;
  WRITE0;
  WRITE1S;
  if ( ++datByteSent < 32 )
    goto _TURNON;
  } else {
_TURNOFF:
  WRITE0;
  WRITE0;
  WRITE0;
  WRITE0;

  WRITE0;
  WRITE0;
  WRITE0;
  WRITE0S;
  if ( ++datByteSent < 32 )
    goto _TURNOFF;
  }

  // Stop bit.
  WRITESTOP;

  interrupts();
}
// End of N64 stuff.

void setup() {
  // Set up radio.
  radio.begin();
  radio.setPALevel( rf24_PA_level );
  radio.openWritingPipe( rf24_txAddr );
  radio.stopListening();
  radio.setRetries( rf24_delay, rf24_retries );
  radio.setChannel( rf24_chnl );
  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  radio.enableDynamicAck();
  radio.setDataRate( rf24_rate );
  radio.setCRCLength( rf24_crc );
  
  #ifdef OLDADAP
  // Mode swtich pin.
  pinMode( EXTMODEPIN, INPUT_PULLUP );
  #else
  pinMode( STATLEDPIN, OUTPUT );
  digitalWrite( STATLEDPIN, curMode );
  #endif
  
  // Set up serial.
  Serial.begin( 9600 );
  Serial.println( "Started RF24 TX" );
  Serial.flush();

   
  //radio.printDetails();
  //Serial.flush();  
}

// Keep track how many times the main loop is executed.
uint8_t loopCnt = 0;
uint8_t ackData;
uint8_t newAckData = 0;

#ifdef CHECKTXRX
uint8_t txFail = 0;
uint8_t txCnt = 0;
#endif

void loop() {
  // We handle ACK data at the end of the loop.
  
  // Read the buttons.
  #ifdef DEBUG
  readCnt++;
  #endif
  
  readButtons();

  // Check if valid.
  if ( newButtonsAvail ) {
    #ifdef CHECKTXRX
    ++txCnt;
    if ( txCnt > 100 ) {
      Serial.println( "Last Fail Rate:" );
      Serial.println( txFail );
      Serial.flush();
      txCnt = 0;
      txFail = 0;
    }
    #endif    

    #ifdef DEBUG
    unsigned long timer = micros();
    Serial.println( "Sending buttons" );
    Serial.flush()
    #endif
    
    if ( radio.write( buttonCmp, 4 ) ) {
      #ifdef DEBUG
      timer = micros() - timer;
      Serial.print( "Roundtrip: " );
      Serial.print( timer );
      Serial.flush()
      #endif
      if ( radio.available() ) {
        uint8_t siz = radio.getDynamicPayloadSize();
        if ( siz == 1 ) {
          radio.read( &ackData, 1 );
          newAckData = 1;
        } else {
          // Unknown ACK data?
          uint8_t ack[ siz ];
          radio.read( ack, siz );
          #ifdef DEBUG
          Serial.print( " Unknwon ACK: " );
          for ( int i =  0; i < siz; ++i ) {
            Serial.print( ack[ i ] );
          }
          Serial.print( "\n" );
          Serial.flush();
          #endif
        }
      }
    } else {
      #ifdef CHECKTXRX
      ++txFail;
      #endif 
      #ifdef DEBUGNOACKBUT
      Serial.println( "Did not receive ACK for buttons :(" );
      Serial.flush();
      #endif
    }

    #ifndef OLDADAP
    // For the new style adapter, we check here if the mode switch
    // button combo was pressed. The button combo is:
    // L + R + Z + DPAD Up.
    struct {
      uint8_t L : 1;
      uint8_t R : 1;
      uint8_t Z : 1;
      uint8_t Up : 1;
    } curButtons;
    curButtons.Z =  ( buttonCmp[ 3 ] & ( 0b00100000 ) ) >> 5;
    curButtons.Up = ( buttonCmp[ 3 ] & ( 0b00001000 ) ) >> 3;
    curButtons.L =  ( buttonCmp[ 2 ] & ( 0b00100000 ) ) >> 5;
    curButtons.R =  ( buttonCmp[ 2 ] & ( 0b00010000 ) ) >> 4;

    #ifdef DEBUG
    if ( curButtons.Z ) {
      Serial.println( "Z pressed!" );
    }
    if ( curButtons.L ) {
      Serial.println( "L pressed!" );
    }
    if ( curButtons.R ) {
      Serial.println( "R pressed!" );
    }
    if ( curButtons.Up ) {
      Serial.println( "Up pressed!" );
    }
    Serial.flush();
    #endif

    if ( curButtons.Z && curButtons.L && curButtons.R &
         curButtons.Up ) {
      if ( modeSwitchPress == 0 ) {
        // Switch mode.
        curMode ^= 1;
        digitalWrite( STATLEDPIN, curMode );
        modeSwitchPress = 1;
      }
    } else {
      // Check if it was press before.
      if ( modeSwitchPress != 0) {
        // Reset it.
        modeSwitchPress = 0;
      }
    }
    #endif
  } else {
    #ifdef DEBUG
    errCnt++;
    #endif
  }

  // Check if there is new AckData.
  if ( newAckData == 1 ) {
    #ifdef DEBUG
    Serial.print( "New ack data received: " );
    Serial.println( ackData );
    Serial.flush();
    #endif
    
    // Handle it.
    if ( ackData ) {
      rumble( true );
    } else {
      rumble( false );
    }

    newAckData = 0;
  }

  #ifdef DEBUG
  if ( readCnt == 100 ) {
    Serial.print( "Error ratio: " );
    Serial.println( errCnt );
    Serial.flush();
    errCnt = 0;
    readCnt = 0;
  }
  #endif

  // And read status.
  if ( loopCnt == STATINTERVAL ) {
    getStatus();
    loopCnt = 0;
  }

  if ( newStatAvail ) {
    newStatAvail = false;
    
    #ifdef DEBUG
    Serial.println( "Sending status..." );
    Serial.flush();
    #endif

    #ifdef DEBUG
    unsigned long timer = micros();
    #endif
    
    if ( radio.write( &stat, 1 ) ) {
      #ifdef DEBUG
      timer = micros() - timer;
      Serial.print( "Roundtrip: " );
      Serial.print( timer );
      Serial.flush()
      #endif

      if ( radio.available() ) {
        uint8_t siz = radio.getDynamicPayloadSize();
        if ( siz == 1 ) {
          radio.read( &ackData, 1 );
          newAckData = 1;
        } else {
          // Unknown ACK data?
          uint8_t ack[ siz ];
          radio.read( ack, siz );
          #ifdef DEBUG
          Serial.print( " Unknwon ACK: " );
          for ( int i =  0; i < siz; ++i ) {
            Serial.print( ack[ i ] );
          }
          Serial.print( "\n" );
          Serial.flush();
          #endif
        }
      }
    } else {
      #ifdef DEBUGNOACKSTAT
      ++txFail;
      Serial.println( "Did not receive ACK for status :(" );
      Serial.flush();
      #endif
    }
  } else {
    #ifdef DEBUG
    errCnt++;
    #endif
  }
  
  // Check if there is new AckData.
  if ( newAckData == 1 ) {    
    // Handle it.
    if ( ackData ) {
      rumble( true );
    } else {
      rumble( false );
    }

    newAckData = 0;
  }
  

  ++loopCnt;
  delay( DEL );
}
