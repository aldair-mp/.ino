// Copyright 2023 RT Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "FS.h"
#include "SPIFFS.h"
#include "device.h"
#include "map_manager.h"
#include "mytypedef.h"
#include "parameter.h"

signed char g_mode;
short g_battery_value;
t_sensor g_sen_r, g_sen_l, g_sen_fr, g_sen_fl;
t_control g_con_wall;

volatile double g_accel;
double g_max_speed, g_min_speed;
volatile double g_speed;
volatile bool g_motor_move;
MapManager g_map_control;

// Estas son librerías necesarias para la comunicacion
#include <WiFi.h>//Conectarse a una red WiFi
#include <WiFiClient.h>//Establecer conexiones TCP/IP
#include <WiFiUdp.h>//Establecer comunicación usando el protocolo UDP
/*const char* ssid = "USER";//Usuario del WiFi
const char* password = "CONTRASEÑA";//contraseña delWiFi
WiFiUDP udp;//Se crea un objeto UDP que permite enviar paquetes de datos por la red usando el protocolo UDP
const int udpPort = 12345;//el puerto al que enviarás los datos
const char* pcIp = "192.168.137.55";//direccion ip del computador*/

const char* ssid = "MORA P";
const char* password = "2025mora";
WiFiUDP udp;
const int udpPort = 12345;
const char* pcIp = "192.168.1.41";

void connectToWiFi() {
  WiFi.begin(ssid, password);
  Serial.println("🔄 Conectando a WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ Conectado a WiFi: " + WiFi.localIP().toString());
}

void sendSensorTask(void* pvParameters) {
  while (1) {
    char buffer[128];
    snprintf(buffer, sizeof(buffer),
             "R:%d FR:%d FL:%d L:%d V:%dmV\n",
             g_sen_r.value,
             g_sen_fr.value,
             g_sen_fl.value,
             g_sen_l.value,
             g_battery_value);

    udp.beginPacket(pcIp, udpPort);
    udp.write((uint8_t*)buffer, strlen(buffer));
    udp.endPacket();

    delay(200);  // Enviar cada 200 ms (5 Hz)
  }
}

void sendMap() {
  char buffer[256];

  for (int y = MAZESIZE_Y - 1; y >= 0; y--) {
    buffer[0] = '\0';
    for (int x = 0; x < MAZESIZE_X; x++) {
      // Obtener valor de pared como 0 o 1
      int n = (g_map_control.getWallData(x, y, north) != 0) ? 1 : 0;
      int e = (g_map_control.getWallData(x, y, east)  != 0) ? 1 : 0;
      int s = (g_map_control.getWallData(x, y, south) != 0) ? 1 : 0;
      int w = (g_map_control.getWallData(x, y, west)  != 0) ? 1 : 0;

      char cell[8];
      snprintf(cell, sizeof(cell), "%d%d%d%d ", n, e, s, w);
      strcat(buffer, cell);
    }
    udp.beginPacket(pcIp, udpPort);
    udp.write((uint8_t*)buffer, strlen(buffer));
    udp.endPacket();
    delay(50);  // Espera entre filas para no saturar
  }

  udp.beginPacket(pcIp, udpPort);
  udp.print("END\n");  // Marca de fin
  udp.endPacket();
}
void sendStepsMap() {
  char buffer[256];
  for (int y = MAZESIZE_Y - 1; y >= 0; y--) {
    buffer[0] = '\0';
    for (int x = 0; x < MAZESIZE_X; x++) {
      char cell[8];
      snprintf(cell, sizeof(cell), "%04d ", g_map_control.getStepValue(x, y));
      strcat(buffer, cell);
    }
    udp.beginPacket(pcIp, udpPort);
    udp.write((uint8_t*)buffer, strlen(buffer));
    udp.endPacket();
    delay(50);
  }

  udp.beginPacket(pcIp, udpPort);
  udp.print("ENDSTEP\n");  // Marca final
  udp.endPacket();
}

void setup()
{
  Serial.begin(115200);
  connectToWiFi();
  udp.begin(udpPort);  // Inicializa UDP

  xTaskCreatePinnedToCore(
    sendSensorTask,   // función de tarea
    "SensorUDPTask",  // nombre
    4096,             // stack size
    NULL,             // parámetros
    1,                // prioridad
    NULL,             // handle de tarea
    1                 // core (1 para no bloquear lógica principal)
  );

  if (!SPIFFS.begin(true)) {
    Serial.println("❌ Error al montar SPIFFS");
  } else {
    Serial.println("✅ SPIFFS montado correctamente");
  }

  initAll();
  disableBuzzer();
  g_mode = 1;
}

void loop()
{
  setLED(g_mode);
  switch (getSW()) {
    case SW_LM:
      g_mode = decButton(g_mode, 1, 15);
      break;
    case SW_RM:
      g_mode = incButton(g_mode, 15, 1);
      break;
    case SW_CM:
      okButton();
      execByMode(g_mode);
      break;
  }
  delay(1);
}

void execByMode(int mode)
{
  enableMotor();
  delay(1000);

  switch (mode) {
    case 1:
      searchLefthand();
      break;
    case 2:  // 足立法
      g_map_control.positionInit();
      searchAdachi(g_map_control.getGoalX(), g_map_control.getGoalY());
      rotate(right, 2);
      g_map_control.nextDir(right);
      g_map_control.nextDir(right);
      goalAppeal();
      searchAdachi(0, 0);
      rotate(right, 2);
      g_map_control.nextDir(right);
      g_map_control.nextDir(right);
      mapWrite();
      sendMap();  // Enviar mapa tras Adachi
      sendStepsMap();

      break;
    case 3:  // 最短走行
      copyMap();
      g_map_control.positionInit();
      fastRun(g_map_control.getGoalX(), g_map_control.getGoalY());
      rotate(right, 2);
      g_map_control.nextDir(right);
      g_map_control.nextDir(right);
      goalAppeal();
      break;
    case 4:
      // Puedes llamar sendMap() aquí si quieres enviar mapa también en modo 4
      break;
    case 15:
      disableMotor();
      adjustMenu();
      break;
    default:
      break;
  }

  disableMotor();
}

