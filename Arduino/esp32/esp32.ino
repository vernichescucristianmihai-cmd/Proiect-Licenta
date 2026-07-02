#include <SPI.h>
#include <mcp_can.h>

#define CAN_CS 5

MCP_CAN CAN(CAN_CS);

void setup() {
  Serial.begin(115200);
  delay(1000);

  SPI.begin(18, 19, 23, CAN_CS);

  while (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("ESP32 Tester: MCP2515 init failed");
    delay(500);
  }

  CAN.setMode(MCP_NORMAL);

  Serial.println();
  Serial.println("ESP32 OBD2 Tester pornit");
  printMenu();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();

    if (c == '1') readLiveData();
    else if (c == '2') readDTCs();
    else if (c == '3') clearDTCs();
    else if (c == '4') readVIN();
    else if (c == 'm' || c == 'M') printMenu();
  }

  readAndPrintRaw();
}

void printMenu() {
  Serial.println("Comenzi:");
  Serial.println("1 = citire live data");
  Serial.println("2 = citire coduri eroare DTC");
  Serial.println("3 = stergere coduri eroare DTC");
  Serial.println("4 = citire VIN simplificat");
  Serial.println("m = meniu");
  Serial.println();
}

void sendOBD(byte mode, byte pid) {
  byte req[8] = {0x02, mode, pid, 0, 0, 0, 0, 0};

  byte result = CAN.sendMsgBuf(0x7DF, 0, 8, req);

  if (result == CAN_OK) {
    Serial.print("TX 0x7DF: ");
    printData(8, req);
  } else {
    Serial.println("TX failed");
  }
}

void sendOBDModeOnly(byte mode) {
  byte req[8] = {0x01, mode, 0, 0, 0, 0, 0, 0};

  byte result = CAN.sendMsgBuf(0x7DF, 0, 8, req);

  if (result == CAN_OK) {
    Serial.print("TX 0x7DF: ");
    printData(8, req);
  } else {
    Serial.println("TX failed");
  }
}

bool waitResponse(byte expectedMode, byte expectedPid, byte* response, unsigned long timeoutMs) {
  unsigned long start = millis();

  while (millis() - start < timeoutMs) {
    if (CAN.checkReceive() == CAN_MSGAVAIL) {
      unsigned long id = 0;
      byte len = 0;
      byte buf[8];

      CAN.readMsgBuf(&id, &len, buf);
      id = id & 0x7FF;

      Serial.print("RX 0x");
      Serial.print(id, HEX);
      Serial.print(": ");
      printData(len, buf);

      if (id == 0x7E8 && len >= 3) {
        if (expectedPid == 0xFF) {
          if (buf[1] == expectedMode) {
            copyFrame(buf, response);
            return true;
          }
        } else {
          if (buf[1] == expectedMode && buf[2] == expectedPid) {
            copyFrame(buf, response);
            return true;
          }
        }
      }
    }
  }

  Serial.println("Timeout - nu am primit raspuns");
  return false;
}

void copyFrame(byte* src, byte* dst) {
  for (byte i = 0; i < 8; i++) {
    dst[i] = src[i];
  }
}

void readLiveData() {
  byte r[8];

  Serial.println();
  Serial.println("=== LIVE DATA ===");

  sendOBD(0x01, 0x0C);
  if (waitResponse(0x41, 0x0C, r, 1000)) {
    int rpm = ((r[3] * 256) + r[4]) / 4;
    Serial.print("RPM: ");
    Serial.println(rpm);
  }

  sendOBD(0x01, 0x0D);
  if (waitResponse(0x41, 0x0D, r, 1000)) {
    int speed = r[3];
    Serial.print("Viteza: ");
    Serial.print(speed);
    Serial.println(" km/h");
  }

  sendOBD(0x01, 0x05);
  if (waitResponse(0x41, 0x05, r, 1000)) {
    int coolant = r[3] - 40;
    Serial.print("Temperatura lichid racire: ");
    Serial.print(coolant);
    Serial.println(" C");
  }

  sendOBD(0x01, 0x0F);
  if (waitResponse(0x41, 0x0F, r, 1000)) {
    int intake = r[3] - 40;
    Serial.print("Temperatura aer admisie: ");
    Serial.print(intake);
    Serial.println(" C");
  }

  sendOBD(0x01, 0x11);
  if (waitResponse(0x41, 0x11, r, 1000)) {
    float throttle = r[3] * 100.0 / 255.0;
    Serial.print("Pozitie acceleratie: ");
    Serial.print(throttle);
    Serial.println(" %");
  }

  Serial.println("=== FINAL LIVE DATA ===");
  Serial.println();
}

void readDTCs() {
  byte r[8];

  Serial.println();
  Serial.println("=== CITIRE DTC ===");

  sendOBDModeOnly(0x03);

  if (waitResponse(0x43, 0xFF, r, 1000)) {
    decodeDTC(r[2], r[3]);
    decodeDTC(r[4], r[5]);
    decodeDTC(r[6], r[7]);
  }

  Serial.println("=== FINAL DTC ===");
  Serial.println();
}

void clearDTCs() {
  byte r[8];

  Serial.println();
  Serial.println("=== STERGERE DTC ===");

  sendOBDModeOnly(0x04);

  if (waitResponse(0x44, 0xFF, r, 1000)) {
    Serial.println("DTC sterse cu succes pe ECU simulator.");
  }

  Serial.println();
}

void readVIN() {
  byte r[8];

  Serial.println();
  Serial.println("=== CITIRE VIN ===");

  sendOBD(0x09, 0x02);

  if (waitResponse(0x49, 0x02, r, 1000)) {
    Serial.print("VIN simplificat: ");
    for (byte i = 4; i < 8; i++) {
      Serial.print((char)r[i]);
    }
    Serial.println();
  }

  Serial.println();
}

void decodeDTC(byte A, byte B) {
  if (A == 0x00 && B == 0x00) return;

  char type;

  switch ((A & 0xC0) >> 6) {
    case 0: type = 'P'; break;
    case 1: type = 'C'; break;
    case 2: type = 'B'; break;
    case 3: type = 'U'; break;
  }

  int d1 = (A & 0x30) >> 4;
  int d2 = A & 0x0F;
  int d3 = (B & 0xF0) >> 4;
  int d4 = B & 0x0F;

  Serial.print("DTC: ");
  Serial.print(type);
  Serial.print(d1);
  Serial.print(d2, HEX);
  Serial.print(d3, HEX);
  Serial.println(d4, HEX);
}

void readAndPrintRaw() {
  if (CAN.checkReceive() == CAN_MSGAVAIL) {
    unsigned long id = 0;
    byte len = 0;
    byte buf[8];

    CAN.readMsgBuf(&id, &len, buf);
    id = id & 0x7FF;

    Serial.print("RX RAW 0x");
    Serial.print(id, HEX);
    Serial.print(": ");
    printData(len, buf);
  }
}

void printData(byte len, byte* buf) {
  for (byte i = 0; i < len; i++) {
    if (buf[i] < 0x10) Serial.print("0");
    Serial.print(buf[i], HEX);
    Serial.print(" ");
  }

  Serial.println();
}