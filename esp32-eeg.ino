/**
 * Connect to EEG directly using ESP32
 * Using pin in Bluetooth is NOT possible in the latest ESP32 library
 * https://github.com/espressif/arduino-esp32/tree/master/libraries/BluetoothSerial
 * Downgrade to version 2.x in Boards Manager
 */

#include "BluetoothSerial.h"

enum EEG_CODE: byte {
  SYNC = 0xAA,
  EXCODE = 0x55,
  SIGNAL = 0x02,
  ATTENTION = 0x04,
  MEDITATION = 0x05,
  BLINK = 0x16,
  RAW_WAVE = 0x80,
  EEG_POWER = 0x83
};

BluetoothSerial SerialBT;

String myName = "ESP32-BT-EEG-PRO-MAX";                     // this device's name
const char *pin = "1234";                                   // pin for bluetooth connection
String MACadd = "04:23:10:20:02:B0";                        // eeg bluetooth mac address
uint8_t address[6]  = {0x04, 0x23, 0x10, 0x20, 0x02, 0xB0}; // in hex

byte generatedChecksum = 0;
byte checksum = 0;
int payloadLength = 0;
byte payload[169] = {};

struct EEGData {
  int signal;
  int attention;
  int meditation;
  int delta;
  int theta;
  int lowAlpha;
  int highAlpha;
  int lowBeta;
  int highBeta;
  int lowGamma;
  int midGamma;
};

EEGData data;

void connectEeg() {
  bool connected;

  Serial.print("Connecting to slave BT device with MAC "); Serial.println(MACadd);
  connected = SerialBT.connect(address);

  if(connected) {
    Serial.println("Connected Successfully!");
  } else {
    while(!SerialBT.connected(3000)) {
      Serial.println("Failed to connect.");
    }
  }
}

void setup() {
  Serial.begin(115200);
  SerialBT.begin(myName, true);
  Serial.printf("The device \"%s\" started in master mode!\n", myName.c_str());

  SerialBT.setPin(pin);
  Serial.println("Using PIN");

  connectEeg();
}

byte readByte() {
  while(!SerialBT.available()) {
    if (!SerialBT.connected()) connectEeg();
  }

  return (int) SerialBT.read();
}

void loop() {
  if (readByte() != SYNC) return;
  if (readByte() != SYNC) return;

  payloadLength = readByte();

  if (payloadLength > 169) return;

  generatedChecksum = 0;
  for (int i = 0; i < payloadLength; i++) {
    int byte = readByte();
    payload[i] = byte;
      generatedChecksum += byte;
  }

  checksum = readByte();
  generatedChecksum = ~(generatedChecksum & 0xFF) & 0xFF;

  if (checksum != generatedChecksum) return;

  int parsed = 0;

  while (parsed < payloadLength) {
    while (payload[parsed] == EXCODE) parsed++;

    byte code = payload[parsed++];
    int dataLength = 1;

    if (code & 0x80) dataLength = payload[parsed++];

    switch (code) {
      case SIGNAL:
        data.signal = payload[parsed];
        break;
      case ATTENTION:
        data.attention = payload[parsed];
        break;
      case MEDITATION:
        data.meditation = payload[parsed];
        break;
      case EEG_POWER:
        data.delta = (payload[parsed+0] & 0xFF << 16) | (payload[parsed+1] & 0xFF << 8) | payload[parsed+2];
        data.theta = (payload[parsed+3] & 0xFF << 16) | (payload[parsed+4] & 0xFF << 8) | payload[parsed+5];
        data.lowAlpha = (payload[parsed+6] & 0xFF << 16) | (payload[parsed+7] & 0xFF << 8) | payload[parsed+8];
        data.highAlpha = (payload[parsed+9] & 0xFF << 16) | (payload[parsed+10] & 0xFF << 8) | payload[parsed+11];
        data.lowBeta = (payload[parsed+12] & 0xFF << 16) | (payload[parsed+13] & 0xFF << 8) | payload[parsed+14];
        data.highBeta = (payload[parsed+15] & 0xFF << 16) | (payload[parsed+16] & 0xFF << 8) | payload[parsed+17];
        data.lowGamma = (payload[parsed+18] & 0xFF << 16) | (payload[parsed+19] & 0xFF << 8) | payload[parsed+20];
        data.midGamma = (payload[parsed+21] & 0xFF << 16) | (payload[parsed+22] & 0xFF << 8) | payload[parsed+23];
        break;
    }

    parsed += dataLength;
  }

  // Serial.print(" Sig: "); Serial.print(data.signal);
  // Serial.print(" Att: "); Serial.print(data.attention);
  // Serial.print(" Med: "); Serial.print(data.meditation);
  // Serial.print(" Del: "); Serial.print(data.delta);
  // Serial.print(" The: "); Serial.print(data.theta);
  // Serial.print(" LoA: "); Serial.print(data.lowAlpha);
  // Serial.print(" HiA: "); Serial.print(data.highAlpha);
  // Serial.print(" LoB: "); Serial.print(data.lowBeta);
  // Serial.print(" HiB: "); Serial.print(data.highBeta);
  // Serial.print(" LoG: "); Serial.print(data.lowGamma);
  // Serial.print(" MiG: "); Serial.print(data.midGamma);
  // Serial.println();
  Serial.printf(
    "Sig:%d,Att:%d,Med:%d,Del:%d,The:%d,LoA:%d,HiA:%d,LoB:%d,HiB:%d,LoG:%d,MiG:%d\n",
    data.signal, data.attention, data.meditation, data.delta, data.theta, data.lowAlpha, data.highAlpha, data.lowBeta, data.highBeta, data.lowGamma, data.midGamma
  );
}
