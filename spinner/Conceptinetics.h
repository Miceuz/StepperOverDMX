/*
  Conceptinetics.h - DMX library for Arduino
  Copyright (c) 2013 W.A. van der Meeren <danny@illogic.nl>.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 3 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

/*
  This code has been tested using the following hardware:

  - Arduino UNO R3 using a CTC-DRA-13-1 ISOLATED DMX-RDM SHIELD 
  - Arduino MEGA2560 R3 using a CTC-DRA-13-1 ISOLATED DMX-RDM SHIELD 
*/


#ifndef CONCEPTINETICS_H_
#define CONCEPTINETICS_H_

#include <Arduino.h>
#include <inttypes.h>


#define DMX_MAX_FRAMESIZE       513     // Startbyte + 512 Slots
#define DMX_MIN_FRAMESIZE       2       // Startbyte + 1 Slot

#define DMX_MAX_FRAMECHANNELS   512     // Maxmim number of channer per frame

#define DMX_STARTCODE_SIZE      1       // Size of startcode in bytes

#define DMX_START_CODE          0x0     // Start code for a DMX frame
#define RDM_START_CODE          0xcc    // Start code for a RDM frame

// Uncomment to enable interbyte gaps
// #define DMX_IBG				    10      // Interbyte gap

// Speed your Arduino is running on in Hz.
#define F_OSC 				16000000UL

// DMX Baudrate, this should be 25000
#define DMX_BAUD_RATE 		250000

// The baudrate used to automaticly generate a break within
// your ISR.. make it lower to generate longer breaks
#define DMX_BREAK_RATE 	 	99900       


namespace dmx 
{
    enum dmxState 
	{
		dmxIdle,
		dmxBreak,
        dmxManualBreak,
		dmxStartByte,
        dmxFrameDetected,
		dmxRecordData,
		dmxData
		};
}

class DMX_FrameBuffer
{
    public:
        //
        // Constructor buffersize = 1-513
        //
        DMX_FrameBuffer     ( uint16_t buffer_size );
        DMX_FrameBuffer     ( DMX_FrameBuffer &buffer );
        ~DMX_FrameBuffer    ( void );

        uint16_t getBufferSize ( void );        

        uint8_t getSlotValue ( uint16_t index );
        void    setSlotValue ( uint16_t index, uint8_t value );
        void    setSlotRange ( uint16_t start, uint16_t end, uint8_t value );
        void    clear ( void );        

        uint8_t &operator[] ( uint16_t index );

    private:

        uint8_t     *m_refcount;
        uint16_t    m_bufferSize;
        uint8_t     *m_buffer;      
};


class DMX_Master
{
    public:
        // Run the DMX master from a pre allocated frame buffer which
        // you have fully under your own control
        DMX_Master ( DMX_FrameBuffer &buffer, int readEnablePin  );
        
        // Run the DMX master by giving a predefined maximum number of
        // channels to support
        DMX_Master ( uint16_t maxChannel, int readEnablePin );

        ~DMX_Master ( void );
    
        void enable  ( void );              // Start transmitting
        void disable ( void );              // Stop transmitting

        // Get reference to the internal framebuffer
        DMX_FrameBuffer &getBuffer ( void );

        // Update channel values
        void setChannelValue ( uint16_t channel, uint8_t value );
        void setChannelRange ( uint16_t start, uint16_t end, uint8_t value );

    public:
        //
        // Manual control over the break period
        //
        void setAutoBreakMode ( void );     // Generated from ISR
        void setManualBreakMode ( void );   // Generate manually

        uint8_t autoBreakEnabled ( void );

        // We are waiting for a manual break to be generated 
        uint8_t waitingBreak ( void );
        
        // Generate break and start transmission of frame
        void breakAndContinue ();


    protected:
        void setStartCode ( uint8_t value ); 


    private:
        DMX_FrameBuffer m_frameBuffer;
        int             m_re_pin;
        uint8_t         m_autoBreak;
};


class DMX_Slave
{
    public:
        DMX_Slave ( DMX_FrameBuffer &buffer, int readEnablePin = -1 );

        DMX_Slave ( uint16_t nrChannels, int readEnablePin = -1 );

        ~DMX_Slave ( void );

        void enable     ( void );           // Enable receiver
        void disable    ( void );           // Disable receiver

 
        // Get reference to the internal framebuffer
        DMX_FrameBuffer &getBuffer ( void );

        uint8_t  getChannelValue ( uint16_t channel );

        uint16_t getStartAddress ( void );
        void     setStartAddress ( uint16_t );

        // Register on receive complete callback in case
        // of time critical applications
        void onReceiveComplete ( void (*func)(void) );


    protected:
        // Startcode is default set to record dmx frames but
        // can be used using this function.
        // This is intended for future use
        void setStartCode ( uint8_t value = DMX_START_CODE ); 

    public:        
        static void (*event_onFrameReceived)(void);


    private:
        DMX_FrameBuffer m_frameBuffer;
        int             m_re_pin;
        uint16_t        m_startAddress;     // Slave start address
};


#endif /* CONCEPTINETICS_H_ */
