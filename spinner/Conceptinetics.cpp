/*
  Conceptinetics.cpp - DMX library for Arduino
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


#include "pins_arduino.h"
#include "Conceptinetics.h"

#include <inttypes.h>
#include <stdlib.h>

#include <avr/interrupt.h>
#include <avr/io.h>

#include <util/delay.h>

#if defined (USART__TXC_vect)
  #define USART_TX USART__TXC_vect
#elif defined(USART_TX_vect)
  #define USART_TX  USART_TX_vect
#elif defined(USART0_TX_vect)
  #define USART_TX USART0_TX_vect
#endif 

#if defined (USART__RXC_vect)
  #define USART_RX USART__RXC_vect
#elif defined(USART_RX_vect)
  #define USART_RX  USART_RX_vect
#elif defined(USART0_RX_vect)
  #define USART_RX USART0_RX_vect
#endif 


#if defined UDR
  #define DMX_UDR UDR
#elif defined UDR0
  #define DMX_UDR UDR0
#endif


#if defined(UBRRH) && defined(UBRRL)
  #define DMX_UBRRH UBRRH
  #define DMX_UBRRL UBRRL
#elif defined(UBRR0H) && defined(UBRR0L)
  #define DMX_UBRRH UBRR0H
  #define DMX_UBRRL UBRR0L
#endif

#if defined(UCSRA)
  #define DMX_UCSRA UCSRA
#elif defined(UCSR0A)
  #define DMX_UCSRA UCSR0A
#endif

#if defined(FE)
  #define DMX_FE FE
#elif defined(FE0)
  #define DMX_FE FE0
#endif


DMX_FrameBuffer::DMX_FrameBuffer ( uint16_t buffer_size )
{
    m_refcount = (uint8_t*) malloc ( sizeof ( uint8_t ) );

    if ( buffer_size >= DMX_MIN_FRAMESIZE && buffer_size <= DMX_MAX_FRAMESIZE )
    {
        m_buffer = (uint8_t*) malloc ( buffer_size );
        if ( m_buffer != NULL )
        {
            memset ( (void *)m_buffer, 0x0, buffer_size );
            m_bufferSize = buffer_size;
        }
        else 
            m_buffer = 0x0;
    }
    else
        m_bufferSize = 0x0;

    *m_refcount++;
}

DMX_FrameBuffer::DMX_FrameBuffer ( DMX_FrameBuffer &buffer )
{
    // Copy references and make sure the parent object does not dispose our
    // buffer when deleted and we are still active
    this->m_refcount = buffer.m_refcount;
    (*this->m_refcount)++;
    
    this->m_buffer = buffer.m_buffer;
    this->m_bufferSize = buffer.m_bufferSize;
}

DMX_FrameBuffer::~DMX_FrameBuffer ( void )
{
    // If we are the last object using the
    // allocated buffer then free it together
    // with the refcounter
    if ( --(*m_refcount) == 0 )
    {
        if ( m_buffer )
            free ( m_buffer );

        free ( m_refcount );
    }
}

uint16_t DMX_FrameBuffer::getBufferSize ( void )
{
    return m_bufferSize;
}        


uint8_t DMX_FrameBuffer::getSlotValue ( uint16_t index )
{
    if (index < m_bufferSize)
        return m_buffer[index];
    else
        return 0x0;
}


void DMX_FrameBuffer::setSlotValue ( uint16_t index, uint8_t value )
{
    if ( index < m_bufferSize )
        m_buffer[index] = value;
}


void DMX_FrameBuffer::setSlotRange ( uint16_t start, uint16_t end, uint8_t value )
{
    if ( start < m_bufferSize && end < m_bufferSize && start < end )
        memset ( (void *) &m_buffer[start], value, end-start );
}

void DMX_FrameBuffer::clear ( void )
{
    memset ( (void *) m_buffer, 0x0, m_bufferSize );
}        

uint8_t &DMX_FrameBuffer::operator[] ( uint16_t index )
{
    return m_buffer[index];
}


DMX_Master      *__dmx_master;
DMX_Slave       *__dmx_slave;
dmx::dmxState   __dmx_masterState;                      // TX ISR state
dmx::dmxState   __dmx_slaveState;                       // RX ISR state


DMX_Master::DMX_Master ( DMX_FrameBuffer &buffer, int readEnablePin )
: m_frameBuffer ( buffer ), 
  m_re_pin ( readEnablePin ),
  m_autoBreak ( 1 )                                     // Autobreak generation is default on
{
	pinMode ( m_re_pin, OUTPUT );		            	// Set direction to output	
	digitalWrite ( m_re_pin, LOW ); 			        // Set shield into read (slave) mode 
    setStartCode ( DMX_START_CODE );    
}

DMX_Master::DMX_Master ( uint16_t maxChannel, int readEnablePin )
: m_frameBuffer ( maxChannel + DMX_STARTCODE_SIZE ), 
  m_re_pin ( readEnablePin ),
  m_autoBreak ( 1 )                                     // Autobreak generation is default on
{
	pinMode ( m_re_pin, OUTPUT );		            	// Set direction to output	
	digitalWrite ( m_re_pin, LOW ); 			        // Set shield into read (slave) mode    
    setStartCode ( DMX_START_CODE );
}

DMX_Master::~DMX_Master ( void )
{
    disable ();                                         // Stop sending
    __dmx_master = NULL;
}

DMX_FrameBuffer &DMX_Master::getBuffer ( void )
{
    return m_frameBuffer;                               // Return reference to frame buffer
}

void DMX_Master::setStartCode ( uint8_t value )
{
    m_frameBuffer[0] = value;                           // Set the first byte in our frame buffer
}

void DMX_Master::setChannelValue ( uint16_t channel, uint8_t value )
{
    if ( channel > 0 )                                  // Prevent overwriting the start code
        m_frameBuffer.setSlotValue ( channel, value );
}

void DMX_Master::setChannelRange ( uint16_t start, uint16_t end, uint8_t value )
{
    if ( start > 0 )                                    // Prevent overwriting the start code
        m_frameBuffer.setSlotRange ( start, end, value );
}


void DMX_Master::enable  ( void )
{
	DMX_UBRRH = (unsigned char)(((F_CPU + DMX_BAUD_RATE * 8L) / (DMX_BAUD_RATE * 16L) - 1)>>8);
	DMX_UBRRL = (unsigned char) ((F_CPU + DMX_BAUD_RATE * 8L) / (DMX_BAUD_RATE * 16L) - 1);			 
	
    // Prepare before kicking off ISR
	DMX_UDR             = 0x0;       
    __dmx_master        = this;                 

    if ( m_autoBreak )
    {
    	__dmx_masterState   = dmx::dmxBreak; 
    #if defined(UCSRC) && defined(UCSRB)
	    UCSRC |= (1<<UMSEL)|(3<<UCSZ0)|(1<<USBS);
	    UCSRB |= (1<<TXEN) |(1<<TXCIE);								
    #elif defined(UCSR0C) && defined(UCSR0B)
	    UCSR0C |= (3<<UCSZ00)|(1<<USBS0); 
	    UCSR0B |= (1<<TXEN0) |(1<<TXCIE0);
    #endif 
    }
    else
    {
        __dmx_masterState = dmx::dmxManualBreak;
    }

    // Set Shield into transmit mode (DMX Master)
	digitalWrite ( m_re_pin, HIGH );    
}

void DMX_Master::disable ( void )
{
#if defined(UCSRB)
	UCSRB &= ~(1<<TXCIE);						        // Disable TX interupt
#elif defined(UCSR0B)
	UCSR0B &= ~(1<<TXCIE0);						        // Disable TX interupt
#endif

	digitalWrite ( m_re_pin, LOW );                     // Set transeiver into read mode (release bus)
    __dmx_masterState = dmx::dmxIdle;
    __dmx_master = NULL;                                // No active master
}

void    DMX_Master::setAutoBreakMode ( void ) { m_autoBreak = 1; }
void    DMX_Master::setManualBreakMode ( void ) { m_autoBreak = 0; }
uint8_t DMX_Master::autoBreakEnabled ( void ) { return m_autoBreak; }


uint8_t DMX_Master::waitingBreak ( void )
{
    return ( __dmx_masterState == dmx::dmxManualBreak );
}
        
void DMX_Master::breakAndContinue ()
{
    // Only execute if we are the controlling master object
    if ( __dmx_master == this && __dmx_masterState == dmx::dmxManualBreak )
    {
        pinMode ( 1, OUTPUT );
        digitalWrite ( 1, LOW );                    // Begin BREAK                               

        _delay_us ( 100 );

        // Turn TX Pin into Logic HIGH
        digitalWrite ( 1, HIGH );                   // END BREAK

        __dmx_masterState = dmx::dmxStartByte;
    
        // TX Enable
        #if defined(UCSRC) && defined(UCSRB)
	        UCSRB |= (1<<TXEN);								
        #elif defined(UCSR0C) && defined(UCSR0B)
	        UCSR0B |= (1<<TXEN0);
        #endif 

        _delay_us ( 12 );                           // MAB 12µSec
        
        // TX Interupt enable
        #if defined(UCSRC) && defined(UCSRB)
	        UCSRB |= (1<<TXCIE);								
        #elif defined(UCSR0C) && defined(UCSR0B)
	        UCSR0B |= (1<<TXCIE0);
        #endif 
    }
}


void (*DMX_Slave::event_onFrameReceived)(void);


DMX_Slave::DMX_Slave ( DMX_FrameBuffer &buffer, int readEnablePin )
: m_frameBuffer ( buffer ), 
  m_re_pin ( readEnablePin ),
  m_startAddress ( 1 )
{
    // Pin 0 and 1 is actually not possible either
    // since those are the rx and tx pins
    if ( m_re_pin >= 0 )
    {
        pinMode ( m_re_pin, OUTPUT );		            	// Set direction to output	
	    digitalWrite ( m_re_pin, LOW ); 			        // Set shield into read (slave) mode  
    }

    setStartCode ( DMX_START_CODE );                        // listen for DMX frames
}

DMX_Slave::DMX_Slave ( uint16_t nrChannels, int readEnablePin )
: m_frameBuffer ( nrChannels + 1 ), 
  m_re_pin ( readEnablePin ),
  m_startAddress ( 1 )
{
    // Pin 0 and 1 is actually not possible either
    // since those are the rx and tx pins
    if ( m_re_pin >= 0 )
    {
        pinMode ( m_re_pin, OUTPUT );		            	// Set direction to output	
	    digitalWrite ( m_re_pin, LOW ); 			        // Set shield into read (slave) mode  
    }

    setStartCode ( DMX_START_CODE );                        // listen for DMX frames
}

DMX_Slave::~DMX_Slave ( void )
{
    disable ();
    __dmx_slave = NULL;
}

void DMX_Slave::enable ( void )
{
	DMX_UBRRH = (unsigned char)(((F_CPU + DMX_BAUD_RATE * 8L) / (DMX_BAUD_RATE * 16L) - 1)>>8);
	DMX_UBRRL = (unsigned char) ((F_CPU + DMX_BAUD_RATE * 8L) / (DMX_BAUD_RATE * 16L) - 1);			 
	
    // Prepare before kicking off ISR
	DMX_UDR             = 0x0;
	__dmx_slaveState    = dmx::dmxIdle;        
    __dmx_slave         = this;

    if ( m_re_pin >= 0 )
	    digitalWrite ( m_re_pin, LOW ); 			        // Set shield into read (slave) mode  

#if defined(UCSRC) && defined(UCSRB)
	UCSRC |= (1<<UMSEL)|(3<<UCSZ0)|(1<<USBS);
	UCSRB |= (1<<RXEN) |(1<<RXCIE);								
#elif defined(UCSR0C) && defined(UCSR0B)
	UCSR0C |= (3<<UCSZ00)|(1<<USBS0);                       
	UCSR0B |= (1<<RXEN0) |(1<<RXCIE0);                      // Enable receiver and rx interrupt
#endif 


}

void DMX_Slave::disable ( void )
{
#if defined(UCSRB)
	UCSRB &= ~(1<<RXCIE);						// Disable RX interupt
#elif defined(UCSR0B)
	UCSR0B &= ~(1<<RXCIE0);						// Disable RX interupt
#endif    
}

void DMX_Slave::setStartCode ( uint8_t value )
{
    m_frameBuffer[0] = value;                           // Set the first byte in our frame buffer
}

DMX_FrameBuffer &DMX_Slave::getBuffer ( void )
{
    return m_frameBuffer;
}

uint8_t DMX_Slave::getChannelValue ( uint16_t channel )
{
    return m_frameBuffer.getSlotValue ( channel );
}


uint16_t DMX_Slave::getStartAddress ( void )
{
    return m_startAddress;
}

void DMX_Slave::setStartAddress ( uint16_t addr )
{
    m_startAddress = addr;
}

void DMX_Slave::onReceiveComplete ( void (*func)(void) )
{
    event_onFrameReceived = func;
}

//
// TX UART (DMX Transmission ISR)
//
ISR (USART_TX)
{
	static uint16_t			current_slot;

	switch ( __dmx_masterState )
	{
	case dmx::dmxBreak:

		DMX_UBRRH = (unsigned char)(((F_CPU + DMX_BREAK_RATE * 8L) / (DMX_BREAK_RATE * 16L) - 1)>>8);
        DMX_UBRRL = (unsigned char) ((F_CPU + DMX_BREAK_RATE * 8L) / (DMX_BREAK_RATE * 16L) - 1);
        __dmx_masterState = dmx::dmxStartByte;

        DMX_UDR = 0x0;
        
		break;


	case dmx::dmxStartByte:
		DMX_UBRRH = (unsigned char)(((F_CPU + DMX_BAUD_RATE * 8L) / (DMX_BAUD_RATE * 16L) - 1)>>8);
		DMX_UBRRL = (unsigned char) ((F_CPU + DMX_BAUD_RATE * 8L) / (DMX_BAUD_RATE * 16L) - 1);						
	
        current_slot = 0;	
        DMX_UDR = __dmx_master->getBuffer()[ current_slot++ ];
		__dmx_masterState = dmx::dmxData;
		break;
		
	case dmx::dmxData:
        // NOTE: we always send full frames of 513 bytes, this will bring us 
        // close to 40 frames / sec with no interbyte gabs
        #ifdef DMX_IBG
            _delay_us (DMX_IBG);
        #endif

		DMX_UDR = __dmx_master->getBuffer().getSlotValue( current_slot++ );
			
		// Send 512 channels
		if ( current_slot >= DMX_MAX_FRAMESIZE )
        {
		    if ( __dmx_master->autoBreakEnabled () )
            {
                __dmx_masterState = dmx::dmxBreak;
            }
            else
            {
                #if defined(UCSRB)
                UCSRB &= ~(1<<TXEN);
	            UCSRB &= ~(1<<TXCIE);						        // Disable TX interupt
                #elif defined(UCSR0B)
                UCSR0B &= ~(1<<TXEN0);
	            UCSR0B &= ~(1<<TXCIE0);						        // Disable TX interupt
                #endif
            
                __dmx_masterState = dmx::dmxManualBreak;            // Waiting for a manual break
            }
        
	    }
        
		break;
	}
}



//
// RX UART (DMX Reception ISR)
//
ISR (USART_RX)
{
    static uint16_t current_slot;

    uint8_t usart_state    = DMX_UCSRA;
    uint8_t usart_data     = DMX_UDR;

    //
    // Check for framing error and reset if found
    // A framing most likely* indicate a break in our ocasion
    //
    if ( usart_state & (1<<DMX_FE) )
	{
	    DMX_UCSRA &= ~(1<<DMX_FE);
        __dmx_slaveState = dmx::dmxBreak;
        return;
    }

    switch ( __dmx_slaveState )
    {
        case dmx::dmxBreak:
            if ( usart_data == __dmx_slave->getBuffer()[0] )
            {
                current_slot = __dmx_slave->getStartAddress();
                __dmx_slaveState = dmx::dmxFrameDetected;
            }
            else
            {
                __dmx_slaveState = dmx::dmxIdle;
            }
        //
        // DMX Frame detected
        //
        case dmx::dmxFrameDetected:
            if (--current_slot == 0)
                __dmx_slaveState = dmx::dmxRecordData;
            break;
       
        //
        // Start address found.. start recording channel values
        //
        case dmx::dmxRecordData:
            if ( current_slot++ < __dmx_slave->getBuffer().getBufferSize() )
                __dmx_slave->getBuffer().setSlotValue ( current_slot, usart_data );
            else
            {
                // If a onFrameReceived callback is register...
                if (__dmx_slave->event_onFrameReceived)
                    __dmx_slave->event_onFrameReceived();
                
                __dmx_slaveState = dmx::dmxIdle;
            }
            break;

    }
    
}

