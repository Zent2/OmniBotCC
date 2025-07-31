#include <Wire.h>

#define AS5600_ADDR       0x36

// Registros
#define ZMCO_REG          0x00
#define ZPOS_REG_MSB      0x01
#define ZPOS_REG_LSB      0x02
#define MANG_REG_MSB      0x05  
#define MANG_REG_LSB      0x06
#define CONF_REG_MSB      0x07
#define CONF_REG_LSB      0x08
#define STATUS_REG        0x0B
#define RAW_ANGLE_MSB     0x0C
#define RAW_ANGLE_LSB     0x0D
#define BURN_REG          0xFF

// Comandos de BURN según la hoja de datos
#define BURN_SETTING_CMD  0x40
#define BURN_ANGLE_CMD    0x80

// Función para leer un registro 8‑bit
uint8_t readReg8(uint8_t reg) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(AS5600_ADDR, (uint8_t)1);
  return Wire.read();
}

// Función para leer un registro 12‑bit (2 bytes)
uint16_t readReg16(uint8_t msbReg) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(msbReg);
  Wire.endTransmission();
  Wire.requestFrom(AS5600_ADDR, (uint8_t)2);
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  return ((uint16_t)hi << 8) | lo;
}

// Función para escribir un registro 8‑bit
void writeReg8(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// Función para escribir un registro 12‑bit (2 bytes)
void writeReg16(uint8_t msbReg, uint16_t value) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(msbReg);
  Wire.write((uint8_t)(value >> 8));
  Wire.write((uint8_t)(value & 0xFF));
  Wire.endTransmission();
}

// Función de confirmación Serial Y/N
bool confirm(const char *msg) {
  //Serial.print(msg);
  //Serial.print(" (Y/N): ");
  while (!Serial.available()) { delay(10); }
  char c = toupper(Serial.read());
  Serial.println(c);
  return (c == 'Y');
}

bool waitForYes(const char* msg) {
  Serial.print(msg);
  Serial.print(" (Y/N): ");
  while (true) {
    if (confirm(msg)) return true;
    Serial.println("   Esperando confirmación para continuar...");
    delay(1000);
  }
}


void setup() {
  Serial.begin(115200);
  Wire.begin();  
  delay(100);

  Serial.println("\n=== Configuración AS5600 paso a paso ===");


  // 2) Leer STATUS y comprobar MD
  waitForYes("2) Leer STATUS_REG");
  //if (!confirm("2) Leer STATUS_REG")) while(1);
  uint8_t status = readReg8(STATUS_REG);
  bool md = status & (1 << 5);
  Serial.print("   MD (magnet detect): ");
  Serial.println(md ? "OK" : "NO DETECTADO");
  if (!md) {
    Serial.println("   Coloca un imán adecuado y confirma nuevamente.");
    while (!confirm("   ¿Seguir?")) {}
  }

  // 3) Leer ZMCO
  waitForYes("3) Leer ZMCO (veces BURN_ANGLE)");
  //if (!confirm("3) Leer ZMCO (veces BURN_ANGLE)")) while(1);
  uint8_t zmco = readReg8(ZMCO_REG) & 0x03;
  Serial.print("   ZMCO = ");
  Serial.println(zmco);

  // 4) Leer/Modificar/Escribir CONF
  waitForYes("4) Leer CONF, aplicar ajustes y reescribir");
  //if (!confirm("4) Leer CONF, aplicar ajustes y reescribir")) while(1);
  uint16_t conf = readReg16(CONF_REG_MSB);
  // WD=0, FTH=000, SF=11, PWMF=00, OUTS=00, HYST=00, PM=00
  // Bits: [7]=WD, [6:4]=FTH, [3:2]=SF, [1:0]=PWMF
  //       [11:10]=OUTS, [9:8]=HYST, [1:0]=PM (en LSB)
  conf &= ~(1<<7);                   // WD=0
  conf &= ~(0b111 << 4);             // FTH=000
  conf = (conf & ~(0b11 << 2)) | (0b11 << 2); // SF=11
  conf &= ~(0b11);                   // PWMF=00
  // OUTS,HYST,PM ya a 0
  writeReg16(CONF_REG_MSB, conf);
  Serial.println("   CONF reescrito");

  // 5) Escribir MANG = 0x0FFF
  waitForYes("5) Fijar MANG a 0x0FFF");
  //if (!confirm("5) Fijar MANG a 0x0FFF")) while(1);
  writeReg16(MANG_REG_MSB, 0x0FFF);
  Serial.println("   MANG escrito");

  // 6) BURN_SETTING
  waitForYes("6) BURN_SETTING (0x40)");
  //if (!confirm("6) BURN_SETTING (0x40)")) while(1);
  writeReg8(BURN_REG, BURN_SETTING_CMD);
  Serial.println("   BURN_SETTING enviado");

  // 7) Verificar secuencia 0x01,0x11,0x10
  waitForYes("7) Verificar: enviar 0x01,0x11,0x10 a BURN");
  //if (!confirm("7) Verificar: enviar 0x01,0x11,0x10 a BURN")) while(1);
  writeReg8(BURN_REG, 0x01);
  writeReg8(BURN_REG, 0x11);
  writeReg8(BURN_REG, 0x10);
  Serial.println("   Secuencia enviada");

  // 8) Leer MANG y CONF para comprobar
  waitForYes("8) Leer de nuevo MANG y CONF");
  //if (!confirm("8) Leer de nuevo MANG y CONF")) while(1);
  uint16_t mang_check = readReg16(MANG_REG_MSB);
  uint16_t conf_check = readReg16(CONF_REG_MSB);
  Serial.print("   MANG = 0x"); Serial.println(mang_check, HEX);
  Serial.print("   CONF = 0x"); Serial.println(conf_check, HEX);

  // 9) Escribir ZPOS = RAW_ANGLE
  waitForYes("9) Leer RAW_ANGLE y fijar ZPOS");
  //if (!confirm("9) Leer RAW_ANGLE y fijar ZPOS")) while(1);
  uint16_t raw = readReg16(RAW_ANGLE_MSB) & 0x0FFF;
  writeReg16(ZPOS_REG_MSB, raw);
  Serial.print("   ZPOS fijado a 0x"); Serial.println(raw, HEX);

  // 10) BURN_ANGLE
  waitForYes("10) BURN_ANGLE (0x80)");
  //if (!confirm("10) BURN_ANGLE (0x80)")) while(1);
  writeReg8(BURN_REG, BURN_ANGLE_CMD);
  Serial.println("   BURN_ANGLE enviado");

  // 11) Verificar secuencia 0x01,0x11,0x10
  waitForYes("11) Verificar: enviar 0x01,0x11,0x10 a BURN");
  //if (!confirm("11) Verificar: enviar 0x01,0x11,0x10 a BURN")) while(1);
  writeReg8(BURN_REG, 0x01);
  writeReg8(BURN_REG, 0x11);
  writeReg8(BURN_REG, 0x10);
  Serial.println("   Secuencia enviada");

  // 12) Leer ZPOS
  waitForYes("12) Leer ZPOS para comprobar");
  //if (!confirm("12) Leer ZPOS para comprobar")) while(1);
  uint16_t zpos_check = readReg16(ZPOS_REG_MSB);
  Serial.print("   ZPOS = 0x"); Serial.println(zpos_check, HEX);

  // 13) Leer ZMCO final
  waitForYes("13) Leer ZMCO final");
  //if (!confirm("13) Leer ZMCO final")) while(1);
  uint8_t zmco_final = readReg8(ZMCO_REG) & 0x03;
  Serial.print("   ZMCO final = ");
  Serial.println(zmco_final);

  Serial.println("\n=== Configuración COMPLETA ===");
}

void loop() {
  // Nada que hacer en loop
}
