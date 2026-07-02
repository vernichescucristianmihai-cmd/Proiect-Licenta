#include <SPI.h>
#include <mcp_can.h>

#define CAN_CS 10

MCP_CAN CAN(CAN_CS);

bool dtcActive = true;

void setup() {
  Serial.begin(115200);
  delay(1000);

  while (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("Arduino ECU: MCP2515 init failed");
    delay(500);
  }

  CAN.setMode(MCP_NORMAL);

  Serial.println("Arduino Jade Uno+ ECU Simulator pornit");
  Serial.println("Astept cereri OBD2 pe ID 0x7DF...");
}

void loop() {
  if (CAN.checkReceive() == CAN_MSGAVAIL) {
    unsigned long id = 0;
    byte len = 0;
    byte buf[8];

    CAN.readMsgBuf(&id, &len, buf);
    id = id & 0x7FF;

    printFrame("RX", id, len, buf);

    if (id == 0x7DF && len >= 2) {
      byte mode = buf[1];

      if (mode == 0x01 && len >= 3) {
        handleMode01(buf[2]);
      } 
      else if (mode == 0x03) {
        sendDTCs();
      } 
      else if (mode == 0x04) {
        clearDTCs();
      } 
      else if (mode == 0x09 && len >= 3) {
        handleMode09(buf[2]);
      }
    }
  }
}

void printFrame(const char* label, unsigned long id, byte len, byte* buf) {
  Serial.print(label);
  Serial.print(" ID: 0x");
  Serial.print(id, HEX);
  Serial.print(" DATA: ");

  for (byte i = 0; i < len; i++) {
    if (buf[i] < 0x10) Serial.print("0");
    Serial.print(buf[i], HEX);
    Serial.print(" ");
  }

  Serial.println();
}

void sendFrame(byte data[8]) {
  CAN.sendMsgBuf(0x7E8, 0, 8, data);
  printFrame("TX", 0x7E8, 8, data);
}

void handleMode01(byte pid) {
  switch (pid) {

    case 0x00: {
      // PIDs suportate 01-20
      // Suportă: 01, 05, 0C, 0D, 0F, 11
      byte resp[8] = {0x06, 0x41, 0x00, 0x88, 0x18, 0x80, 0x00, 0x00};
      sendFrame(resp);
      break;
    }

    case 0x01: {
      // Monitor status
      // bit 7 = MIL aprins, restul = numar DTC
      byte milDtc = dtcActive ? 0x81 : 0x00;
      byte resp[8] = {0x06, 0x41, 0x01, milDtc, 0x07, 0x65, 0x04, 0x00};
      sendFrame(resp);
      break;
    }

    case 0x05: {
      // Temperatura lichid racire = A - 40
      // 90 C => A = 130
      byte resp[8] = {0x03, 0x41, 0x05, 130, 0, 0, 0, 0};
      sendFrame(resp);
      break;
    }

    case 0x0C: {
      // RPM = ((A * 256) + B) / 4
      // 850 RPM => 3400 = 0x0D48
      byte resp[8] = {0x04, 0x41, 0x0C, 0x0D, 0x48, 0, 0, 0};
      sendFrame(resp);
      break;
    }

    case 0x0D: {
      // Viteza km/h = A
      byte resp[8] = {0x03, 0x41, 0x0D, 0x00, 0, 0, 0, 0};
      sendFrame(resp);
      break;
    }

    case 0x0F: {
      // Temperatura aer admisie = A - 40
      // 25 C => A = 65
      byte resp[8] = {0x03, 0x41, 0x0F, 65, 0, 0, 0, 0};
      sendFrame(resp);
      break;
    }

    case 0x11: {
      // Pozitie acceleratie = A * 100 / 255
      // 46 => aprox 18%
      byte resp[8] = {0x03, 0x41, 0x11, 46, 0, 0, 0, 0};
      sendFrame(resp);
      break;
    }

    default:
      Serial.print("PID Mode 01 nesuportat: 0x");
      Serial.println(pid, HEX);
      break;
  }
}

void sendDTCs() {
  if (dtcActive) {
    // Coduri simulate:
    // P0100 = 01 00
    // P0110 = 01 10
    // P0300 = 03 00
    byte resp[8] = {0x06, 0x43, 0x01, 0x00, 0x01, 0x10, 0x03, 0x00};
    sendFrame(resp);
  } else {
    byte resp[8] = {0x02, 0x43, 0x00, 0, 0, 0, 0, 0};
    sendFrame(resp);
  }
}

void clearDTCs() {
  dtcActive = false;

  byte resp[8] = {0x01, 0x44, 0, 0, 0, 0, 0, 0};
  sendFrame(resp);

  Serial.println("DTC-urile simulate au fost sterse.");
}

void handleMode09(byte pid) {
  if (pid == 0x00) {
    // PID 0x02 suportat
    byte resp[8] = {0x06, 0x49, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00};
    sendFrame(resp);
  }

  else if (pid == 0x02) {
    // VIN simplificat, un singur frame
    // Pentru VIN complet trebuie ISO-TP multi-frame
    byte resp[8] = {0x07, 0x49, 0x02, 0x01, 'T', 'E', 'S', 'T'};
    sendFrame(resp);
  }

  else {
    Serial.print("PID Mode 09 nesuportat: 0x");
    Serial.println(pid, HEX);
  }
}