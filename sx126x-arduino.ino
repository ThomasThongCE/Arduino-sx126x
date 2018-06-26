#include "radio.h"
#include "sx126x-board.h"
#include "sx126x.h"
#include <SPI.h>

#define LED 9
//#define Receive

#define USE_MODEM_LORA

#define RF_FREQUENCY                                868000000 // Hz

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
//  1: 250 kHz,
//  2: 500 kHz,
//  3: Reserved]
#define LORA_SPREADING_FACTOR                       12        // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
//  2: 4/6,
//  3: 4/7,
//  4: 4/8]
#define LORA_SYMBOL_TIMEOUT                         5         // Symbols
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

#define TX_OUTPUT_POWER                             20
static RadioEvents_t RadioEvents;
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );

void OnTxDone(void);
uint8_t packet[] = {'h','e','l','l','o'};

 RadioStatus_t status;
  uint16_t Interrupt;

void setup() {
  // put your setup code here, to run once:
  pinMode(LED,OUTPUT);
  
  SPI.begin();
  SX126xIoInit();
  Serial.begin(115200);
  Serial.println("Begin");
  status = SX126xGetStatus();
  Serial.print("Begin Status : ");
  Serial.println(status.Value,BIN);
  
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.TxDone = OnTxDone;

  Radio.Init( &RadioEvents );
  Serial.println("Init events");

  Radio.SetChannel( RF_FREQUENCY );
  Serial.println("Set rf frequency");

#ifdef Receive
  Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                     LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                     LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                     0, true, 0, 0, LORA_IQ_INVERSION_ON, true );
  Serial.println("Set Rx config");
#else
  Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000000 );
  Serial.println("Set Tx config");
#endif
  
delay(3000);

  status = SX126xGetStatus();
  Serial.print("Status : ");
  Serial.println(status.Value,BIN);

  Interrupt = SX126xGetIrqStatus();
  Serial.print("interrupt status : ");
  Serial.println(Interrupt,BIN);

  #ifdef Receive
  Radio.Rx( 0 ); // Continuous Rx
  Serial.println("RX");
#else
  Radio.Send(packet,5);
  Serial.println("Sent");
  #endif
  
  status = SX126xGetStatus();
  Serial.print("Status : ");
  Serial.println(status.Value,BIN);

  delay(500);
  status = SX126xGetStatus();
  Serial.print("Status : ");
  Serial.println(status.Value,BIN);
//  Interrupt = SX126xGetIrqStatus();
//  Serial.print("interrupt status : ");
//  Serial.println(Interrupt,BIN);
//while(1)
//{
//  delay(500);
//  status = SX126xGetStatus();
//  Serial.print("Status : ");
//  Serial.println(status.Value,BIN);
//}

}

void loop() {
  // put your main code here, to run repeatedly:
//  delay (3000);
//  status = SX126xGetStatus();
//  Serial.print("Status : ");
//  Serial.println(status.Value,BIN);
//
//  Interrupt = SX126xGetIrqStatus();
//  Serial.print("interrupt status : ");
//  Serial.println(Interrupt,BIN);
//SX126xClearIrqStatus( IRQ_RADIO_ALL );
//Serial.print("RSSI : ");
  //Serial.println(SX126xGetRssiInst());
//SX126xClearIrqStatus( IRQ_RADIO_ALL );
  digitalWrite(LED,HIGH);
    delay(2000);
    digitalWrite(LED,LOW);
    delay(2000);
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
  digitalWrite(LED,HIGH);
  Serial.println("rx finish");

  Serial.print("packet : ");
  for (int i = 0; i < size; ++i)
    Serial.print(payload[i]);

  Serial.println();
  Serial.print("RSSI : ");
  Serial.println(rssi);
  
  Serial.print("SNR : ");
  Serial.println(snr);
  
  
  digitalWrite(LED,LOW);
}

void OnTxDone(void)
{
  digitalWrite(LED,HIGH);
  delay(200);
  Serial.println("tx finish");
  Radio.Send(packet,5);
  Serial.println("Sent");
  digitalWrite(LED,LOW);
}
