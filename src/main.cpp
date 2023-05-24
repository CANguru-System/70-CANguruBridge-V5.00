#include <Arduino.h>
#include "CANguruDefs.h"
#include "CANguru.h"
#include <ESP32CAN.h>
#include <CAN_config.h>
#include <CAN_const.h>
#include "MOD-LCD.h"
#include <espnow.h>
#include <ETH.h>
#include <Adafruit_GFX.h>
#include <ESPAsyncWebServer.h>

// buffer for receiving and sending data
uint8_t M_PATTERN[] = {0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

const uint8_t maxPackets = 30;
bool bLokDiscovery;

bool locofileread;
bool initialDataAlreadySent;
byte locid;

canguruETHClient telnetClient;

//
void printCANFrame(uint8_t *buffer, CMD dir)
{
  // Diese Prozedur könnte mehrere Male aufgerufen werden;
  // deshalb wird die Erhöhung begrenzt
  if (buffer[Framelng] <= 0x08)
    buffer[Framelng] += 0x0F;
  sendOutGW(buffer, dir);
  return;
}

// sendet den CAN-Frame buffer über den CAN-Bus an die Gleisbox
void proc2CAN(uint8_t *buffer, CMD dir)
{
  twai_message_t Message2Send;
  // CAN uses (network) big endian format
  // Maerklin TCP/UDP Format: always 13 (CAN_FRAME_SIZE) bytes
  //   byte 0 - 3  CAN ID
  //   byte 4      DLC
  //   byte 5 - 12 CAN data
  //

  memset(&Message2Send, 0, CAN_FRAME_SIZE);
  Message2Send.rtr = CAN_MSG_FLAG_RTR_;
  Message2Send.ss = CAN_MSG_FLAG_SS_;
  Message2Send.extd = CAN_MSG_FLAG_EXTD_;
  memcpy(&Message2Send.identifier, buffer, 4);
  Message2Send.identifier = ntohl(Message2Send.identifier);
  // Anzahl Datenbytes
  Message2Send.data_length_code = buffer[4];
  // Datenbytes
  if (Message2Send.data_length_code > 0)
    memcpy(&Message2Send.data, &buffer[5], Message2Send.data_length_code);
  ESP32Can.CANWriteFrame(&Message2Send);
  printCANFrame(buffer, dir);
}

void setup_can_driver()
{
  ESP32CAN_status_t error = ESP32Can.CANInit(GPIO_NUM_5, GPIO_NUM_35, ESP32CAN_SPEED_250KBPS);

  if (error == ESP32CAN_OK)
  {
    displayLCD("CAN is running!");
  }
  else
  {
    displayLCD("Starting CAN failed!");
    while (1)
      delay(10);
  }
}

void printMSG(uint8_t no)
{
  uint8_t MSG[] = {0x00, no, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  sendOutGW(MSG, MSGfromBridge);
}

// ein CAN-Frame wird erzeugt, der Parameter noFrame gibt an, welche Daten
// in den Frame zu kopieren sind
void produceFrame(patterns noFrame)
{
  memset(M_PATTERN, 0x0, CAN_FRAME_SIZE);
  M_PATTERN[2] = 0x03;
  M_PATTERN[3] = 0x01;
  switch (noFrame)
  {
  case M_GLEISBOX_MAGIC_START_SEQUENCE:
    M_PATTERN[1] = 0x36;
    M_PATTERN[2] = 0x03;
    M_PATTERN[3] = 0x01;
    M_PATTERN[4] = 0x05;
    M_PATTERN[9] = 0x11;
    break;
  case M_GLEISBOX_ALL_PROTO_ENABLE:
    M_PATTERN[2] = 0x03;
    M_PATTERN[3] = 0x01;
    M_PATTERN[4] = 0x06;
    M_PATTERN[9] = 0x08;
    M_PATTERN[10] = 0x07;
    break;
  case M_GO:
    M_PATTERN[4] = 0x05;
    M_PATTERN[9] = 0x01;
    break;
  case M_STOP:
    M_PATTERN[1] = 0x00;
    M_PATTERN[4] = 0x05;
    M_PATTERN[9] = 0x00;
    break;
  case M_BIND:
    M_PATTERN[1] = 0x04;
    M_PATTERN[4] = 0x06;
    M_PATTERN[9] = 0x00;
    break;
  case M_VERIFY:
    M_PATTERN[1] = 0x06;
    M_PATTERN[4] = 0x06;
    M_PATTERN[9] = 0x00;
    break;
  case M_FUNCTION:
    M_PATTERN[1] = 0x0C;
    M_PATTERN[4] = 0x06;
    M_PATTERN[7] = 0x04;
    break;
  case M_CAN_PING:
    M_PATTERN[1] = 0x30;
    M_PATTERN[2] = 0x47;
    M_PATTERN[3] = 0x11;
    break;
  case M_PING_RESPONSE:
    M_PATTERN[0] = 0x00;
    M_PATTERN[1] = 0x30;
    M_PATTERN[2] = 0x00;
    M_PATTERN[3] = 0x00;
    M_PATTERN[4] = 0x00;
    break;
  case M_CAN_PING_CS2:
    M_PATTERN[1] = 0x31;
    M_PATTERN[2] = 0x47;
    M_PATTERN[3] = 0x11;
    M_PATTERN[4] = 0x08;
    M_PATTERN[9] = 0x03;
    M_PATTERN[10] = 0x08;
    M_PATTERN[11] = 0xFF;
    M_PATTERN[12] = 0xFF;
    break;
  case M_CAN_PING_CS2_1:
    M_PATTERN[1] = 0x31;
    M_PATTERN[2] = 0x63;
    M_PATTERN[3] = 0x4A;
    M_PATTERN[4] = 0x08;
    M_PATTERN[9] = 0x04;
    M_PATTERN[10] = 0x02;
    M_PATTERN[11] = 0xFF;
    M_PATTERN[12] = 0xF0;
    break;
  case M_CAN_PING_CS2_2:
    M_PATTERN[1] = 0x31;
    M_PATTERN[2] = 0x63;
    M_PATTERN[3] = 0x4B;
    M_PATTERN[4] = 0x08;
    M_PATTERN[9] = 0x03;
    M_PATTERN[10] = 0x44;
    break;
  case M_READCONFIG:
    M_PATTERN[1] = ReadConfig;
    M_PATTERN[4] = 0x07;
    M_PATTERN[7] = 0x40;
    break;
  case M_STARTCONFIG:
    M_PATTERN[1] = MfxProc_R;
    M_PATTERN[4] = 0x02;
    M_PATTERN[5] = 0x01;
    break;
  case M_FINISHCONFIG:
    M_PATTERN[1] = MfxProc_R;
    M_PATTERN[4] = 0x01;
    M_PATTERN[5] = 0x00;
    break;
  case M_DOCOMPRESS:
    M_PATTERN[1] = DoCompress;
    M_PATTERN[4] = 0x00;
    break;
  case M_DONOTCOMPRESS:
    M_PATTERN[1] = DoNotCompress;
    M_PATTERN[4] = 0x00;
    break;
  case M_GETCONFIG1:
    M_PATTERN[1] = LoadCS2Data;
    M_PATTERN[4] = 0x08;
    M_PATTERN[5] = 0x6C;
    M_PATTERN[6] = 0x6F;
    M_PATTERN[7] = 0x6B;
    M_PATTERN[8] = 0x73;
    break;
  case M_GETCONFIG2:
    M_PATTERN[1] = LoadCS2Data_R;
    M_PATTERN[4] = 0x08;
    M_PATTERN[5] = 0x6C;
    M_PATTERN[6] = 0x6F;
    M_PATTERN[7] = 0x6B;
    M_PATTERN[8] = 0x73;
    break;
  case M_SIGNAL:
    M_PATTERN[1] = 0x50;
    M_PATTERN[4] = 0x01;
    break;
  case M_CNTLOKBUFFER:
    M_PATTERN[1] = sendCntLokBuffer;
    break;
  case M_SENDLOKBUFFER:
    M_PATTERN[1] = sendLokBuffer;
    M_PATTERN[4] = 0x01;
    break;
  }
}

#include <utils.h>

void sendOutGW(uint8_t *buffer, CMD cmd)
{
  //%
/*  buffer[0] = cmd;
  UdpOUTGW.beginPacket(ipGateway, localPortoutGW);
  UdpOUTGW.write(buffer, CAN_FRAME_SIZE);
  UdpOUTGW.endPacket();
  buffer[0] = 0x00;*/
}

void msgStartScanning()
{
//  telnetClient.printTelnet(true, "Start scanning for slaves ... ", 0);
}

// sendet einen CAN-Frame an den Teilnehmer und gibt ihn anschließend aus
void sendOutTCP(uint8_t *buffer)
{
//  writeTCP(buffer, CAN_FRAME_SIZE);
  //  printCANFrame(buffer, toTCP);
}

void sendOutTCPfromCAN(uint8_t *buffer)
{
//  writeTCP(buffer, CAN_FRAME_SIZE);
  //  printCANFrame(buffer, fromCAN2TCP);
  //  print_can_frame(3, buffer);
}

void sendOutUDP(uint8_t *buffer)
{
/*  UdpOUTSYS.beginPacket(ipGateway, localPortoutSYS);
  UdpOUTSYS.write(buffer, CAN_FRAME_SIZE);
  UdpOUTSYS.endPacket();
  printCANFrame(buffer, toUDP);*/
}

void sendOutUDPfromCAN(uint8_t *buffer)
{
/*  UdpOUTSYS.beginPacket(ipGateway, localPortoutSYS);
  UdpOUTSYS.write(buffer, CAN_FRAME_SIZE);
  UdpOUTSYS.endPacket();*/
}

void sendOutClnt(uint8_t *buffer, CMD dir)
{
 /* log_e("sendOutClnt: %X", buffer[0x01]);
  switch (buffer[0x01])
  {
  case CONFIG_Status:
    send2OneClient(buffer);
    break;
  case SYS_CMD:
    switch (buffer[0x09])
    {
    case SYS_STAT:
    case RESET_MEM:
    case START_OTA:
      send2OneClient(buffer);
      break;
    default:
      send2AllClients(buffer);
      break;
    }
    break;
  // send to all
  default:
    send2AllClients(buffer);
    break;
  }
  printCANFrame(buffer, dir);*/
}

void setup() {
#if defined ARDUINO_ESP32_EVB
  delay(350);
#endif
  Serial.begin(bdrMonitor);
  //  Serial.setDebugOutput(true);
  Serial.println("\r\n\r\nC A N g u r u - B r i d g e - " + CgVersionnmbr);
  drawCircle = false;
  // das Display wird initalisiert
  initDisplayLCD28();
  // noch nicht nach Slaves scannen
  set_time4Scanning(false);
  // der Timer für das Blinken wird initialisiert
  stillAliveBlinkSetup();
  // start the CAN bus at 250 kbps
  setup_can_driver();
  // ESPNow wird initialisiert
  // Variablen werden auf Anfangswerte gesetzt und die Routinen für das Senden
  // und Empfangen über ESNOW werden registriert
  espInit();
  // die Routine für die Statusmeldungen des WiFi wird registriert 
  wifi_event_id_t inet_evt_hnd = WiFi.onEvent(iNetEvtCB);
  // das gleiche mit ETHERNET
  ETH.begin();
  // das Zugprogramm (WDP) wurde noch nicht erkannt
  set_SYSseen(false);
  
  // Variablen werden gesetzt
  bLokDiscovery = false;
  locofileread = false;
  initialDataAlreadySent = false;
  locid = 1;
  // start the telnetClient
  telnetClient.telnetInit();
}

// Meldung, dass SYS gestartet werden kann
void goSYS()
{
  printMSG(StartTrainApplication);
  drawCircle = true;
}

void loop() {
  // put your main code here, to run repeatedly:
}
