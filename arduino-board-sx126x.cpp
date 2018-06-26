/*!
 * \file      sx1261dvk1bas-board.c
 *
 * \brief     Target board SX1261DVK1BAS shield driver implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
#include <stdlib.h>
#include "board.h"
#include "radio.h"
#include "sx126x-board.h"

#include "Arduino.h"

#ifdef __AVR__
  #include <util/delay.h>
#endif

//#include "board-config.h"
#include <SPI.h>

/*!
 * Defines the time required for the TCXO to wakeup [ms].
 */
#define BOARD_TCXO_WAKEUP_TIME                      0

/*!
 * Board MCU pins definitions
 */
#define RADIO_RESET                                 1                       

#define RADIO_MOSI                                  11
#define RADIO_MISO                                  12
#define RADIO_SCLK                                  13
#define RADIO_NSS                                   10
#define RADIO_BUSY                                  6
                                
#define RADIO_DIO_1                                 3
#define RADIO_DIO_2                                 
#define RADIO_DIO_3                                 7                            
                                

#define RADIO_DEVICE_SEL                            A1

/*!
 * Antenna switch GPIO pins objects
 */
//Gpio_t AntPow;
//Gpio_t DeviceSel;

void SX126xIoInit( void )
{
    pinMode(RADIO_NSS, OUTPUT);
    pinMode(RADIO_BUSY, INPUT);
    pinMode(RADIO_DIO_1, INPUT);
    
    
    /*
    GpioInit( &SX126x.Spi.Nss, RADIO_NSS, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );
    GpioInit( &SX126x.BUSY, RADIO_BUSY, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &SX126x.DIO1, RADIO_DIO_1, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &DeviceSel, RADIO_DEVICE_SEL, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    */
}

void SX126xIoIrqInit( DioIrqHandler dioIrq )
{
    attachInterrupt(digitalPinToInterrupt(RADIO_DIO_1), dioIrq, RISING );
    //GpioSetInterrupt( &SX126x.DIO1, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, dioIrq ); 
}

void SX126xIoDeInit( void )
{
    // GpioInit( &SX126x.Spi.Nss, RADIO_NSS, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_PULL_UP, 1 );
    // GpioInit( &SX126x.BUSY, RADIO_BUSY, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    // GpioInit( &SX126x.DIO1, RADIO_DIO_1, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
}

uint32_t SX126xGetBoardTcxoWakeupTime( void )
{
    return BOARD_TCXO_WAKEUP_TIME;
}

void SX126xReset( void )
{
    delay( 10 );
    pinMode(RADIO_RESET,OUTPUT);
    //GpioInit( &SX126x.Reset, RADIO_RESET, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    delay( 20 );
    pinMode(RADIO_RESET,INPUT);
    //GpioInit( &SX126x.Reset, RADIO_RESET, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 ); // internal pull-up
    delay( 10 );
}

void SX126xWaitOnBusy( void )
{
    while(digitalRead(RADIO_BUSY) == HIGH);
    //while( GpioRead( &SX126x.BUSY ) == 1 );
}

void SX126xWakeup( void )
{
    BoardDisableIrq( );

    digitalWrite(RADIO_NSS,LOW); 
    //GpioWrite( &SX126x.Spi.Nss, 0 );

    SPI.transfer(RADIO_GET_STATUS);
    SPI.transfer(0x00);
    // SpiInOut( &SX126x.Spi, RADIO_GET_STATUS );
    // SpiInOut( &SX126x.Spi, 0x00 );

    digitalWrite(RADIO_NSS,HIGH); 
    // GpioWrite( &SX126x.Spi.Nss, 1 );

    // Wait for chip to be ready.
    SX126xWaitOnBusy( );

    BoardEnableIrq( );
}

void SX126xWriteCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
    SX126xCheckDeviceReady( );

    digitalWrite(RADIO_NSS,LOW); 
    //GpioWrite( &SX126x.Spi.Nss, 0 );

    SPI.transfer((uint8_t)command);
    //SpiInOut( &SX126x.Spi, ( uint8_t )command );

    for( uint16_t i = 0; i < size; i++ )
    {
        SPI.transfer(buffer[i]);
        //SpiInOut( &SX126x.Spi, buffer[i] );
    }

    digitalWrite(RADIO_NSS,HIGH); 
    //GpioWrite( &SX126x.Spi.Nss, 1 );

    if( command != RADIO_SET_SLEEP )
    {
        SX126xWaitOnBusy( );
    }
}

void SX126xReadCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
    SX126xCheckDeviceReady( );

    digitalWrite(RADIO_NSS,LOW); 
    //GpioWrite( &SX126x.Spi.Nss, 0 );

    SPI.transfer((uint8_t)command);
    SPI.transfer(0x00);
    //SpiInOut( &SX126x.Spi, ( uint8_t )command );
    //SpiInOut( &SX126x.Spi, 0x00 );
    for( uint16_t i = 0; i < size; i++ )
    {
        buffer[i] = SPI.transfer(0x00);
        //Serial.println(buffer[0],BIN);
        //buffer[i] = SpiInOut( &SX126x.Spi, 0 );
    }
    digitalWrite(RADIO_NSS,LOW); 
    //GpioWrite( &SX126x.Spi.Nss, 1 );

    SX126xWaitOnBusy( );
}

void SX126xWriteRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
    SX126xCheckDeviceReady( );

    digitalWrite(RADIO_NSS,LOW);
    //GpioWrite( &SX126x.Spi.Nss, 0 );
    
    SPI.transfer(RADIO_WRITE_REGISTER);
    SPI.transfer(( address & 0xFF00 ) >> 8);
    SPI.transfer(address & 0x00FF);
    // SpiInOut( &SX126x.Spi, RADIO_WRITE_REGISTER );
    // SpiInOut( &SX126x.Spi, ( address & 0xFF00 ) >> 8 );
    // SpiInOut( &SX126x.Spi, address & 0x00FF );
    
    for( uint16_t i = 0; i < size; i++ )
    {
        SPI.transfer(buffer[i]);
        //SpiInOut( &SX126x.Spi, buffer[i] );
    }

    digitalWrite(RADIO_NSS,HIGH);
    //GpioWrite( &SX126x.Spi.Nss, 1 );

    SX126xWaitOnBusy( );
}

void SX126xWriteRegister( uint16_t address, uint8_t value )
{
    SX126xWriteRegisters( address, &value, 1 );
}

void SX126xReadRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
    SX126xCheckDeviceReady( );

    digitalWrite(RADIO_NSS,LOW);
    //GpioWrite( &SX126x.Spi.Nss, 0 );

    SPI.transfer(RADIO_READ_REGISTER);
    SPI.transfer(( address & 0xFF00 ) >> 8);
    SPI.transfer(address & 0x00FF);
    SPI.transfer(0x00);
    // SpiInOut( &SX126x.Spi, RADIO_READ_REGISTER );
    // SpiInOut( &SX126x.Spi, ( address & 0xFF00 ) >> 8 );
    // SpiInOut( &SX126x.Spi, address & 0x00FF );
    // SpiInOut( &SX126x.Spi, 0 );
    for( uint16_t i = 0; i < size; i++ )
    {
        buffer[i] = SPI.transfer(0x00);
        //buffer[i] = SpiInOut( &SX126x.Spi, 0 );
    }

    digitalWrite(RADIO_NSS,HIGH);
    //GpioWrite( &SX126x.Spi.Nss, 1 );

    SX126xWaitOnBusy( );
}

uint8_t SX126xReadRegister( uint16_t address )
{
    uint8_t data;
    SX126xReadRegisters( address, &data, 1 );
    return data;
}

void SX126xWriteBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    SX126xCheckDeviceReady( );

    digitalWrite(RADIO_NSS,LOW);
    //GpioWrite( &SX126x.Spi.Nss, 0 );

    SPI.transfer(RADIO_WRITE_BUFFER);
    SPI.transfer(offset);
    // SpiInOut( &SX126x.Spi, RADIO_WRITE_BUFFER );
    // SpiInOut( &SX126x.Spi, offset );
    for( uint16_t i = 0; i < size; i++ )
    {
        SPI.transfer(buffer[i]);
        // SpiInOut( &SX126x.Spi, buffer[i] );
    }
    digitalWrite(RADIO_NSS,HIGH);
    //GpioWrite( &SX126x.Spi.Nss, 1 );

    SX126xWaitOnBusy( );
}

void SX126xReadBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    SX126xCheckDeviceReady( );

    digitalWrite(RADIO_NSS,LOW);
    //GpioWrite( &SX126x.Spi.Nss, 0 );

    SPI.transfer(RADIO_READ_BUFFER);
    SPI.transfer(offset);
    SPI.transfer(0x00);
    // SpiInOut( &SX126x.Spi, RADIO_READ_BUFFER );
    // SpiInOut( &SX126x.Spi, offset );
    // SpiInOut( &SX126x.Spi, 0 );
    for( uint16_t i = 0; i < size; i++ )
    {
        buffer[i] = SPI.transfer(0x00);
        // buffer[i] = SpiInOut( &SX126x.Spi, 0 );
    }
    digitalWrite(RADIO_NSS,HIGH);
    // GpioWrite( &SX126x.Spi.Nss, 1 );

    SX126xWaitOnBusy( );
}

void SX126xSetRfTxPower( int8_t power )
{
    SX126xSetTxParams( power, RADIO_RAMP_40_US );
}

uint8_t SX126xGetPaSelect( uint32_t channel )
{
//     if( GpioRead( &DeviceSel ) == 1 )
//     {
//         return SX1261;
//     }
//     else
//     {
//         return SX1262;
//     }
  return SX1261; 
}

void SX126xAntSwOn( void )
{
  pinMode(RADIO_DEVICE_SEL , OUTPUT);
  digitalWrite(RADIO_DEVICE_SEL, HIGH);
    // GpioInit( &AntPow, RADIO_ANT_SWITCH_POWER, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );
}

void SX126xAntSwOff( void )
{
  pinMode(RADIO_DEVICE_SEL, INPUT);
    // GpioInit( &AntPow, RADIO_ANT_SWITCH_POWER, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
}

bool SX126xCheckRfFrequency( uint32_t frequency )
{
    // Implement check. Currently all frequencies are supported
    return true;
}

void BoardDisableIrq( void )
{
  noInterrupts();
}

void BoardEnableIrq( void )
{
  interrupts();
}

