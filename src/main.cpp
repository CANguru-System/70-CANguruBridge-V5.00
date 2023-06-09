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

enum enum_canguruStatus
{
  /*00*/ waiting4server,
  /*01*/ startTelnetServer,
  /*02*/ startGleisbox,
  /*03*/ startScanning,
  /*04*/ stopScanning,
  /*05*/ wait4slaves,
  /*06*/ wait4lokbuffer,
  /*07*/ wait4ping,
  nextStep
};
enum_canguruStatus canguruStatus;

// buffer for receiving and sending data
uint8_t M_PATTERN[] = {0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t lastmfxUID[] = {0x00, 0x00, 0x00, 0x00};

uint8_t cntLoks;
struct LokBufferType
{
  uint8_t lastmfxUID[4];
  uint8_t adr;
};
LokBufferType *LokBuffer = NULL;

const uint8_t maxPackets = 30;
bool bLokDiscovery;

bool locofileread;
bool initialDataAlreadySent;
byte locid;
bool scanningFinished;
bool allLoksAreReported;

canguruETHClient telnetClient;

// Forward declarations
//
void sendToServer(uint8_t *buffer, CMD cmd);
void sendToWDPfromCAN(uint8_t *buffer);
//

void printCANFrame(uint8_t *buffer, CMD dir)
{
  // Diese Prozedur könnte mehrere Male aufgerufen werden;
  // deshalb wird die Erhöhung begrenzt
  if (buffer[Framelng] <= 0x08)
    buffer[Framelng] += 0x0F;
  sendToServer(buffer, dir);
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
    displayLCD("CAN started!");
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
  sendToServer(MSG, MSGfromBridge);
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

// sendet den Frame, auf den der Zeiger buffer zeigt, über ESPNow
// an alle Slaves
void send2AllClients(uint8_t *buffer)
{
  uint8_t slaveCnt = get_slaveCnt();
  for (uint8_t s = 0; s < slaveCnt; s++)
  {
    sendTheData(s, buffer, CAN_FRAME_SIZE);
  }
}

// wenn aus der UID des Frames ein Slave identifizierbar ist, wird der
// Frame nur an diesen Slave geschickt, ansonsten an alle
void send2OneClient(uint8_t *buffer)
{
  uint8_t ss = matchUID(buffer);
  if (ss == 0xFF)
    send2AllClients(buffer);
  else
    sendTheData(ss, buffer, CAN_FRAME_SIZE);
}

void sendOutClnt(uint8_t *buffer, CMD dir)
{
  log_e("sendOutClnt: %X", buffer[0x01]);
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
  printCANFrame(buffer, dir);
}

// prüft, ob es Slaves gibt und sendet den CAN-Frame buffer dann an die Slaves
void proc2Clnts(uint8_t *buffer, CMD dir)
{
  // to Client
  if (get_slaveCnt() > 0)
  {
    sendOutClnt(buffer, dir);
  }
}

void sendToServer(uint8_t *buffer, CMD cmd)
{
  //%
  buffer[0] = cmd;
  UDPToServer.beginPacket(ipGateway, localPortToServer);
  UDPToServer.write(buffer, CAN_FRAME_SIZE);
  UDPToServer.endPacket();
  buffer[0] = 0x00;
}

void msgStartScanning()
{
  telnetClient.printTelnet(true, "Start scanning for slaves ... ", 0);
}
/*
void writeTCP(uint8_t *tcb, uint16_t strlng)
{
  if (TCPclient)
  {
    TCPclient.write(tcb, strlng);
    delay(2);
  }
}

// sendet einen CAN-Frame an den Teilnehmer und gibt ihn anschließend aus
void sendOutTCP(uint8_t *buffer)
{
  writeTCP(buffer, CAN_FRAME_SIZE);
  //  printCANFrame(buffer, toTCP);
}

void sendOutTCPfromCAN(uint8_t *buffer)
{
  writeTCP(buffer, CAN_FRAME_SIZE);
//    printCANFrame(buffer, fromCAN2TCP);
//    print_can_frame(3, buffer);
}*/

// sendet CAN-Frames vom  CAN (Gleisbox) zum SYS
void proc_fromCAN2WDPandServer()
{
  twai_message_t MessageReceived;
  if (ESP32Can.CANReadFrame(&MessageReceived) == ESP32CAN_OK)
  {
    // read a packet from CAN
    uint8_t UDPbuffer[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    MessageReceived.identifier &= CAN_EFF_MASK;
    MessageReceived.identifier = htonl(MessageReceived.identifier);
    memcpy(UDPbuffer, &MessageReceived.identifier, 4);
    UDPbuffer[4] = MessageReceived.data_length_code;
    memcpy(&UDPbuffer[5], MessageReceived.data, MessageReceived.data_length_code);
    // now dispatch
    log_e("proc_fromCAN2WDPandServer: %X", UDPbuffer[0x01]);
    switch (UDPbuffer[0x01])
    {
    case PING: // PING
      sendToServer(UDPbuffer, fromCAN);
      break;
    case PING_R: // PING
                 // wenn alle slaves zu Beginn des Programmes gezählt werden, ist damit die Gleisbox auch dabei
      incSlavesAreReadyToZero();
      sendToServer(UDPbuffer, fromCAN);
      sendToWDPfromCAN(UDPbuffer);
      //      sendOutTCPfromCAN(UDPbuffer);
      delay(100);
      if (UDPbuffer[12] == DEVTYPE_GB && get_slaveCnt() == 0)
      {
        printMSG(NoSlaves);
        displayLCD(" -- No Slaves!");
        goSYS();
      }
      break;
      /*    case LokDiscovery_R:
            // mfxdiscovery war erfolgreich
            if (UDPbuffer[4] == 0x05)
            {
              bindANDverify(UDPbuffer);
              // an gateway den anfang melden
            }
            break;
          case MFXVerify:
            bLokDiscovery = true;
            produceFrame(M_STARTCONFIG);
            // LocID
            M_PATTERN[6] = UDPbuffer[10];
            // MFX-UID
            memcpy(&M_PATTERN[7], lastmfxUID, 4);
            // to Gateway
            sendToServer(M_PATTERN, fromCAN);
            cvIndex = readConfig(0);
            break;
          case ReadConfig_R:
            // Rückmeldungen von config
            sendToServer(UDPbuffer, fromCAN);
            sendToWDPfromCAN(UDPbuffer);
      //      sendOutTCPfromCAN(UDPbuffer);
            if ((UDPbuffer[10] == 0x03) && (bLokDiscovery == true))
            {
              if (UDPbuffer[11] == 0x00)
              {
                // das war das letzte Zeichen
                // an gateway den schluss melden
                bLokDiscovery = false;
                // verwendete locid, damit stellt der Server fest, ob
                // die erkannte Lok neu oder bereits bekannt war
                M_PATTERN[6] = locid;
                produceFrame(M_FINISHCONFIG);
                // to Gateway
                sendToServer(M_PATTERN, fromCAN);
              }
              else
              {
                // to Gateway
                cvIndex = readConfig(cvIndex);
              }
            }
            break;*/
    // Magnetartikel schalten
    case SWITCH_ACC_R:
      /*
    Der Schaltbefehl vom Steuerungsprogramm wurde an die Decoder, aber auch
    direkt an die Gleisbox gesendet. Also antwortet auch die Gleisbox und bestätigt
    damit den Befehl, obwohl möglicherweise gar kein Decoder angeschlossen ist. Deshalb wird
    mit diesem Konstrukt die Antwort der Gleisbox unterdrückt.
    Die Bestätigungsmeldung kommt ausschließlich vom Decoder und wird unter espnow.cpp bearbeitet.

    Wird momentan nicht umgesetzt, da ältere WinDigi-Pet-Versionen die Rückmeldung von der Gleisbox erwarten!
    */
      sendToServer(UDPbuffer, fromCAN);
      sendToWDPfromCAN(UDPbuffer);
      //      sendOutTCPfromCAN(UDPbuffer);

      break;
    case WriteConfig_R:
      sendToServer(UDPbuffer, fromCAN);
      sendToWDPfromCAN(UDPbuffer);
      //      sendOutTCPfromCAN(UDPbuffer);
      break;
    default:
      sendToWDPfromCAN(UDPbuffer);
      //      sendOutTCPfromCAN(UDPbuffer);
      break;
    }
  }
}

//////////////// Ausgaberoutinen

void sendToWDP(uint8_t *buffer)
{
  UDPToWDP.beginPacket(ipGateway, localPortToWDP);
  UDPToWDP.write(buffer, CAN_FRAME_SIZE);
  UDPToWDP.endPacket();
  printCANFrame(buffer, toUDP);
}

void sendToWDPfromCAN(uint8_t *buffer)
{
  UDPToWDP.beginPacket(ipGateway, localPortToWDP);
  UDPToWDP.write(buffer, CAN_FRAME_SIZE);
  UDPToWDP.endPacket();
}

//////////////// Empfangsroutinen

// Behandlung der Kommandos, die der CANguru-Server aussendet
void proc_fromServer2CANandClnt()
{
  uint8_t Lokno;
  uint8_t UDPbuffer[CAN_FRAME_SIZE]; // buffer to hold incoming packet,
  int packetSize = UDPFromServer.parsePacket();
  // if there's data available, read a packet
  if (packetSize)
  {
    // read the packet into packetBufffer
    UDPFromServer.read(UDPbuffer, CAN_FRAME_SIZE);
    log_e("fromGW2CANandClnt: %X", UDPbuffer[0x01]);
    printCANFrame(UDPbuffer, fromServer);
    // send received data via usb and CAN
    switch (UDPbuffer[0x1])
    {
    case SYS_CMD:
    case 0x36:
      proc2CAN(UDPbuffer, fromGW2CAN);
      proc2Clnts(UDPbuffer, fromGW2Clnt);
      break;
    case 0x02:
    case 0x04:
    case 0x06:
    case SEND_IP:
    case CONFIG_Status:
      proc2Clnts(UDPbuffer, fromGW2Clnt);
      break;
    case ReadConfig:
    case WriteConfig:
      proc2CAN(UDPbuffer, fromGW2CAN);
      break;
    case MfxProc:
      // received next locid
      locid = UDPbuffer[0x05];
      produceFrame(M_SIGNAL);
      sendToServer(M_PATTERN, fromCAN);
      break;
    case MfxProc_R:
      /*      // there is a new file lokomotive.cs2 to send
            produceFrame(M_SIGNAL);
            sendToServer(M_PATTERN, fromCAN);
            receiveLocFile(0, false);*/
      break;
    // PING
    case PING:
      log_e("M_CAN_PING");
      produceFrame(M_CAN_PING);
      proc2CAN(M_PATTERN, fromGW2CAN);
      break;
    case sendCntLokBuffer_R:
      // Anzahl der Loks, die der Server kennt, werden gemeldet
      log_e("sendCntLokBuffer_R %x", UDPbuffer[0x05]);
      cntLoks = UDPbuffer[0x05];
      // die Anzahl an gemeldeten Loks im Server ausgeben
      char cntBuffer[25];
      sprintf(cntBuffer, "%d Lok(s) im System", cntLoks);
      telnetClient.printTelnet(true, cntBuffer, 0);
      if (cntLoks > 0)
      {
        // erste Lok abrufen
        // evtl. alte Zuweisung löschen
        if (LokBuffer != NULL)
        {
          delete[] LokBuffer; // Free memory allocated for the buffer array.
          LokBuffer = NULL;   // Be sure the deallocated memory isn't used.
        }
        // Speicher für alle Loks reservieren
        if (LokBuffer == NULL)
          LokBuffer = new LokBufferType[cntLoks]; // Allocate cntLoks LokBufferType and save ptr in buffer
        produceFrame(M_SENDLOKBUFFER);
        M_PATTERN[5] = 0;
        sendToServer(M_PATTERN, toServer);
      }
      break;
    case sendLokBuffer_R:
      Lokno = UDPbuffer[0x05];
      log_e("sendLokBuffer_R: %x", Lokno);
      if (LokBuffer != NULL)
      {
        //        saveFrame(UDPbuffer);
        LokBuffer[Lokno].adr = UDPbuffer[0x06];
        for (uint8_t b = 0; b < 4; b++)
        {
          LokBuffer[Lokno].lastmfxUID[b] = UDPbuffer[0x07 + b];
        }
        Lokno++;
        allLoksAreReported = cntLoks == Lokno;
        if (cntLoks > Lokno)
        {
          // nächste Lok abrufen
          produceFrame(M_SENDLOKBUFFER);
          M_PATTERN[5] = Lokno;
          sendToServer(M_PATTERN, toServer);
          // das war die letzte Lok
          // der Anmeldeprozess kann weitergehen
        }
      }
      break;
    case restartBridge:
      proc2Clnts(UDPbuffer, fromGW2Clnt);
      ESP.restart();
      break;
    }
  }
}

// sendet CAN-Frames vom SYS zum CAN (Gleisbox)
void proc_fromWDP2CAN()
{
  uint8_t UDPbuffer[CAN_FRAME_SIZE]; // buffer to hold incoming packet
  uint8_t M_PING_RESPONSEx[] = {0x00, 0x30, 0x00, 0x00, 0x00};
  int packetSize = UDPFromWDP.parsePacket();
  // if there's data available, read a packet
  if (packetSize)
  {
    // read the packet into packetBufffer
    UDPFromWDP.read(UDPbuffer, CAN_FRAME_SIZE);
    // send received data via usb and CAN
    if (UDPbuffer[0x01] == SYS_CMD && UDPbuffer[0x09] == SYS_GO)
      set_SYSseen(true);
    proc2CAN(UDPbuffer, fromServer2CAN);
    log_e("fromServer2CAN: %X", UDPbuffer[0x01]);
    switch (UDPbuffer[0x01])
    {
    case PING:
      produceFrame(M_CAN_PING_CS2); //% M_CAN_PING_CS2_2
      sendToWDP(M_PATTERN);
      produceFrame(M_CAN_PING_CS2_2);
      sendToWDP(M_PATTERN);
      set_SYSseen(true);
      break;
    case PING_R:
      if ((UDPbuffer[11] == 0xEE) && (UDPbuffer[12] == 0xEE))
      {
        proc2CAN(UDPbuffer, fromServer2CAN);
        delay(100);
        memcpy(UDPbuffer, M_PING_RESPONSEx, 5);
        sendToWDP(UDPbuffer);
        //        proc2CAN(UDPbuffer, fromServer2CAN);
        delay(100);
      }
      produceFrame(M_CAN_PING_CS2_1);
      sendToWDP(M_PATTERN);
      produceFrame(M_CAN_PING_CS2_2);
      sendToWDP(M_PATTERN);
      break;
    case Lok_Speed:
    case Lok_Direction:
    case Lok_Function:
      // send received data via wifi to clients
      proc2Clnts(UDPbuffer, fromServer2CAN);
      break;
    case SWITCH_ACC:
      // send received data via wifi to clients
      proc2Clnts(UDPbuffer, toClnt);
      break;
    case S88_Polling:
      UDPbuffer[0x01]++;
      UDPbuffer[0x04] = 7;
      sendToWDPfromCAN(UDPbuffer);
      break;
    case S88_EVENT:
      UDPbuffer[0x01]++;
      UDPbuffer[0x09] = 0x00; // is free
      UDPbuffer[0x04] = 8;
      sendToWDPfromCAN(UDPbuffer);
      break;
    }
  }
}

void setup()
{
#if defined ARDUINO_ESP32_EVB
  delay(350);
#endif
  Serial.begin(bdrMonitor);
  //  Serial.setDebugOutput(true);
  Serial.println("\r\n\r\nC A N g u r u - B r i d g e - " + CgVersionnmbr);
  drawCircle = false;
  // das Display wird initalisiert
  initDisplayLCD28();
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
  canguruStatus = waiting4server;
  scanningFinished = false;
  allLoksAreReported = false;
  setallSlavesAreReadyToZero();
  // start the telnetClient
  telnetClient.telnetInit();
  // This initializes udp and transfer buffer
  if (UDPFromWDP.begin(localPortFromWDP) == 0)
  {
    displayLCD("ERROR From WDP");
  }
  if (UDPFromServer.begin(localPortFromServer) == 0)
  {
    displayLCD("ERROR To WDP");
  }
  if (UDPToServer.begin(localPortToServer) == 0)
  {
    displayLCD("ERROR To Server");
  }
  if (UDPToWDP.begin(localPortToWDP) == 0)
  {
    displayLCD("ERROR From Server");
  }
  // start the TCP-server
  TCPINSYS.begin();
  //
}

// Meldung, dass SYS gestartet werden kann
void goSYS()
{
  printMSG(StartTrainApplication);
  drawCircle = true;
}

// die Routin antwortet auf die Anfrage des CANguru-Servers mit CMD 0x88;
// damit erhält er die IP-Adresse der CANguru-Bridge und kann dann
// damit eine Verbindung aufbauen
bool proc_wait4Server()
{
  bool result;
  uint8_t UDPbuffer[CAN_FRAME_SIZE]; // buffer to hold incoming packet,
  yield();
  result = false;
  int packetSize = UDPFromServer.parsePacket();
  // if there's data available, read a packet
  if (packetSize)
  {
    // read the packet into packetBuffer
    UDPFromServer.read(UDPbuffer, CAN_FRAME_SIZE);
    // send received data via ETHERNET
    switch (UDPbuffer[0x1])
    {
    case CALL4CONNECT:
      setServerStatus(true);
      ipGateway = telnetClient.setipBroadcast(UDPFromServer.remoteIP());
      UDPbuffer[0x1]++;
      sendToServer(UDPbuffer, toServer);
      result = true;
      break;
    }
  }
  return result;
}

void proc_start_lokBuffer()
{
  //  log_e("proc_start_lokBuffer");
  produceFrame(M_CNTLOKBUFFER);
  sendToServer(M_PATTERN, toServer);
}

// damit wird die Gleisbox zum Leben erweckt
void send_start_60113_frames()
{
  produceFrame(M_GLEISBOX_MAGIC_START_SEQUENCE);
  proc2CAN(M_PATTERN, toCAN);
  produceFrame(M_GLEISBOX_ALL_PROTO_ENABLE);
  proc2CAN(M_PATTERN, toCAN);
}

void loop()
{
  // die folgenden Routinen werden ständig aufgerufen
  stillAliveBlinking();
  switch (canguruStatus)
  {
  case waiting4server:
    log_e("waiting4server");
    if (proc_wait4Server() == true)
      canguruStatus = startTelnetServer;
    break;
  case startTelnetServer:
    log_e("startTelnetServer");
    if (telnetClient.startTelnetSrv())
      canguruStatus = startGleisbox;
    break;
  case startGleisbox:
    log_e("startGleisbox");
    delay(5);
    send_start_60113_frames();
    // erstes PING soll schnell kommen
    secs = wait_for_ping;
    canguruStatus = startScanning;
    break;
  case startScanning:
    log_e("startScanning");
    Scan4Slaves();
    canguruStatus = stopScanning;
    break;
  case stopScanning:
    if (scanningFinished == true)
    {
      log_e("stopScanning");
      registerSlaves();
      //      sendOutTCP(M_PATTERN);
      canguruStatus = wait4lokbuffer;
    }
    break;
  case wait4lokbuffer:
    log_e("wait4lokbuffer");
    proc_start_lokBuffer();
    canguruStatus = wait4ping;
    break;
  case wait4ping:
    log_e("wait4ping");
    proc_fromServer2CANandClnt();
    if (allLoksAreReported == true)
    {
      delay(200);
      produceFrame(M_CAN_PING);
      proc2CAN(M_PATTERN, toCAN);
      proc2Clnts(M_PATTERN, toClnt);
      canguruStatus = wait4slaves;
    }
    break;
  case wait4slaves:
    log_e("wait4slaves");
    proc_fromCAN2WDPandServer();
    if (getallSlavesAreReady() == (get_slaveCnt() + 1))
      canguruStatus = nextStep;
    break;
  case nextStep:
    //  log_e("nextStep");
    proc_fromServer2CANandClnt();
    break;
  default:
    break;
  }
}
