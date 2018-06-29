#include "radio.h"
#include "sx126x-board.h"
#include "sx126x.h"
#include <SPI.h>

#define LED 9
//#define Receive

#define USE_MODEM_LORA

#define RF_FREQUENCY                                868000000 // Hz

#define LORA_BANDWIDTH                              2         // [0: 125 kHz,
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

#define TX_OUTPUT_POWER                             12
static RadioEvents_t RadioEvents;
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );
void OnTxDone(void);
void OnTxTimeout(void);
uint8_t packet[] = {'h','e','l','l','o'};

 RadioStatus_t status;
  uint16_t Interrupt;
  RadioError_t error;

void setup() {
  // put your setup code here, to run once:
  pinMode(LED,OUTPUT);
  
  SPI.begin();
  SX126xIoInit();
  Serial.begin(115200);
  Serial.println("Begin");
  
  SX126xCalibrateImage(868000000);
  CalibrationParams_t calibParam;

  calibParam.Value = 0x7F;
  SX126xCalibrate( calibParam );
  error = SX126xGetDeviceErrors();
  Serial.print("error code : ");
  Serial.println(error.Value,BIN);

  
  status = SX126xGetStatus();
  Serial.print("Begin Status : ");
  Serial.println(status.Value,BIN);
  
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;

  Radio.Init( &RadioEvents );
  Serial.println("Init events");

  Interrupt = SX126xGetIrqStatus();
  Serial.print("interrupt status : ");
  Serial.println(Interrupt,BIN);
  
  Radio.SetChannel( RF_FREQUENCY );
  Serial.println("Set rf frequency");

  Interrupt = SX126xGetIrqStatus();
  Serial.print("interrupt status : ");
  Serial.println(Interrupt,BIN);

  Radio.SetPublicNetwork(true);
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

  Interrupt = SX126xGetIrqStatus();
  Serial.print("interrupt status : ");
  Serial.println(Interrupt,BIN);
}

void loop() {

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
  Serial.println("send again");
  Serial.println("Sent");
  digitalWrite(LED,LOW);
}

void OnTxTimeout()
{
  Serial.println("tx timeout");
}


