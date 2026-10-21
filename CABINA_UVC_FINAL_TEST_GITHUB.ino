/*
 * Proyecto: Cabina UV-C Automatizada con Monitoreo IoT y Control Visual RGB
 * Plataforma: ESP32 (Espressif Systems)
 * Área de Impacto: Laboratorio Químico - Programa CISA (UNI)
 * Cumplimiento Normativo: ISO 15858:2016 & IEC 62471:2006 (RG3)
 * Versión: uvc_safe_2026_final_test
 */

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <ArduinoJson.h>

// ==========================================
// 1. CONFIGURACIÓN RED INALÁMBRICA (WIFI)
// ==========================================
const char* ssids[]     = { "Wokwi-GUEST", "S24 Ultra J", "T_ELEC", "Galaxy A36 BJHL" };
const char* passwords[] = { "", "12345678", "Taller$2025", "12345678" };
const int wifiCount     = sizeof(ssids) / sizeof(ssids[0]);

// ==========================================
// 2. CONFIGURACIÓN BROKER MQTT (HIVEMQ CLOUD)
// ==========================================
const char* mqtt_server = "a6d2614b7fc249b4a46aa71f4a7268af.s1.eu.hivemq.cloud";
const int mqtt_port     = 8883;
const char* mqtt_user   = "cabina_uvc";
const char* mqtt_pass   = "Cabina2026!";

const char* topic_tele  = "/cabina_uvc/tele";
const char* topic_cmd   = "/cabina_uvc/cmd";
const char* topic_alert = "/cabina_uvc/alert";

// ==========================================
// 3. DISTRIBUCIÓN DE PINES (GPIO LAYOUT RGB)
// ==========================================
// Actuadores Visuales y Sonoros (Salidas)
const int RGB_RED_PIN       = 2;   // Canal Rojo LED RGB (Resistencia 220 Ohm)
const int RGB_GREEN_PIN     = 4;   // Canal Verde LED RGB (Resistencia 220 Ohm)
const int RGB_BLUE_PIN      = 5;   // Canal Azul LED RGB (Resistencia 220 Ohm)
const int RELAY_PIN         = 18;  // Control de potencia lámpara UV-C
const int BUZZER_PIN        = 19;  // Alarma sonora de seguridad

// Sensores (Fallo Seguro)
const int REED_SWITCH_PIN   = 15;  // Enclavamiento magnético de puerta

// Datos I2C (Hardware Nativo)
const int I2C_SDA           = 21;  
const int I2C_SCL           = 22;  

const bool RELAY_ACTIVE_HIGH = true;

// Ajuste según el tipo de LED RGB (true = Cátodo Común, false = Ánodo Común)
const bool RGB_CATEDO_COMUN  = true; 

// ==========================================
// 4. TEMPORIZADORES Y CONSTANTES DE DISEÑO
// ==========================================
const int MAX_CYCLE_S              = 10000;
const int MAX_DIGITS               = 5;
const int RSSI_LOW_LIMIT           = -80;

const unsigned long TELEMETRY_INTERVAL_MS = 2000;
const unsigned long LCD_REFRESH_MS        = 500;
const unsigned long WIFI_RETRY_MS         = 10000;
const unsigned long MQTT_RETRY_MS         = 5000;
const unsigned long MQTT_CMD_GRACE_MS     = 8000;
const unsigned long START_COOLDOWN_MS     = 1500;
const unsigned long KEY_COOLDOWN_MS       = 250;
const unsigned long STANDBY_TIMEOUT_MS    = 300000; // 5 minutos sin interacción
const unsigned long RESTART_CONFIRM_MS    = 5000;   // Ventana para confirmar reinicio total

// ==========================================
// 5. MÁQUINA DE ESTADOS FINITOS (FSM)
// ==========================================
const char* ST_IDLE       = "IDLE";
const char* ST_RUNNING    = "RUNNING";
const char* ST_PAUSED     = "PAUSED";
const char* ST_DONE       = "DONE";
const char* ST_ALARM      = "ALARM";
const char* ST_ERROR_WIFI = "ERROR_WIFI";
const char* ST_ERROR_MQTT = "ERROR_MQTT";

String estadoSistema    = ST_IDLE;
String estadoPuerta     = "CLOSED";
String estadoRelay      = "OFF";
String wifiStatus       = "DISCONNECTED";
String mqttStatus       = "DISCONNECTED";
String faultCode        = "NONE";
String faultDetail      = "Sistema operativo";
String lastCmd          = "BOOT";
String lastSource       = "system";

// Variables de Tiempo de Ejecución
unsigned long bootMs               = 0;
unsigned long tiempoInicio         = 0;
unsigned long duracionCiclo        = 0;
unsigned long ultimoEnvioTelemetria = 0;
unsigned long ultimoRefreshLCD     = 0;
unsigned long ultimoIntentoWiFi    = 0;
unsigned long ultimoIntentoMQTT    = 0;
unsigned long ultimoToggleBuzzer   = 0;
unsigned long ultimoStartMs        = 0;
unsigned long ultimaTeclaMs        = 0;
unsigned long ultimaActividadMs    = 0;
unsigned long reinicioArmadoMs     = 0;
unsigned long msgCounter           = 0;
unsigned long ciclosEjecutados     = 0;

int cycle_s            = 0;
int remaining_s        = 0;
int tiempoConfirmado_s = 0;

String keypadBuffer    = "";
char ultimaTecla       = 0;

bool cicloActivo         = false;
bool cicloPausado        = false;
bool requiereNuevoTiempo = true;
bool lcdDirty            = true;
bool buzzerState         = false;
bool modoStandby         = false;
bool reinicioArmado      = false;

// ==========================================
// 6. INSTANCIACIÓN DE PERIFÉRICOS
// ==========================================
WiFiClientSecure espClient;
PubSubClient client(espClient);
LiquidCrystal_I2C lcd(0x27, 16, 2);

const byte ROWS = 4;
const byte COLS = 4;
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
byte rowPins[ROWS] = {26, 25, 33, 32};
byte colPins[COLS] = {13, 12, 14, 27};

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// ==========================================
// 7. FUNCIONES DE CONTROL CROMÁTICO (RGB)
// ==========================================
void fijarColorRGB(bool red, bool green, bool blue) {
  digitalWrite(RGB_RED_PIN,   RGB_CATEDO_COMUN ? red   : !red);
  digitalWrite(RGB_GREEN_PIN, RGB_CATEDO_COMUN ? green : !green);
  digitalWrite(RGB_BLUE_PIN,  RGB_CATEDO_COMUN ? blue  : !blue);
}

void actualizarIndicadoresVisuales() {
  // En standby de interfaz se apagan indicadores no críticos.
  if (modoStandby && !esEstado(ST_ALARM) && !esEstado(ST_RUNNING)) {
    fijarColorRGB(false, false, false);
    return;
  }

  // 1. Alerta por Apertura de Puerta (Rojo Fijo)
  if (esEstado(ST_ALARM)) {
    fijarColorRGB(true, false, false); 
    return;
  }

  // 2. CICLO DE DESINFECCIÓN ACTIVO: Rojo Fijo (Peligro UV-C)
  if (esEstado(ST_RUNNING) || cicloActivo) {
    fijarColorRGB(true, false, false);
    return;
  }

  // 3. Ciclo Concluido Exitosamente: Verde Seguro
  if (esEstado(ST_DONE)) {
    fijarColorRGB(false, true, false);
    return;
  }

  // 4. Estado de Reposo o Pausa: Apagado
  fijarColorRGB(false, false, false);
}

// ==========================================
// 8. FUNCIONES DE MONITOREO Y CONTROL LOCAL
// ==========================================
bool esEstado(const char* estado) { return estadoSistema == estado; }
bool tiempoValido(int segundos) { return segundos > 0 && segundos <= MAX_CYCLE_S; }
bool puertaAbierta() { return digitalRead(REED_SWITCH_PIN) == HIGH; }
void leerPuerta() { estadoPuerta = puertaAbierta() ? "OPEN" : "CLOSED"; }
void marcarLCD() { lcdDirty = true; }

void registrarCmd(String cmd, String source) {
  lastCmd = cmd;
  lastSource = source;
}

void despertarInterfaz() {
  if (modoStandby) {
    modoStandby = false;
    lcd.backlight();
    marcarLCD();
  }
}

void registrarActividad(String source = "local") {
  bool estabaEnStandby = modoStandby;
  ultimaActividadMs = millis();
  despertarInterfaz();
  if (estabaEnStandby) registrarCmd("WAKE", source);
}

void lcdPrintLine(byte row, String text) {
  lcd.setCursor(0, row);
  if (text.length() > 16) text = text.substring(0, 16);
  lcd.print(text);
  for (int i = text.length(); i < 16; i++) lcd.print(" ");
}

void actualizarLCD(bool force = false) {
  if (modoStandby) return;
  if (!force && !lcdDirty && millis() - ultimoRefreshLCD < LCD_REFRESH_MS) return;
  ultimoRefreshLCD = millis();
  lcdDirty = false;

  if (esEstado(ST_ERROR_WIFI)) {
    lcdPrintLine(0, "SIN RED WIFI");
    lcdPrintLine(1, "Reconectando...");
  } else if (esEstado(ST_ERROR_MQTT)) {
    lcdPrintLine(0, "SIN ENLACE MQTT");
    lcdPrintLine(1, "Reconectando...");
  } else if (faultCode == "RSSI_LOW") {
    lcdPrintLine(0, "SENAL WIFI DEBIL");
    lcdPrintLine(1, String(WiFi.RSSI()) + " dBm");
  } else if (esEstado(ST_ALARM)) {
    lcdPrintLine(0, "ALERTA UV-C");
    lcdPrintLine(1, "CIERRE LA PUERTA");
  } else if (esEstado(ST_RUNNING)) {
    lcdPrintLine(0, "UV-C ENCENDIDO");
    lcdPrintLine(1, "Restan " + String(remaining_s) + " s");
  } else if (esEstado(ST_PAUSED)) {
    lcdPrintLine(0, "PAUSA " + String(remaining_s) + " s");
    lcdPrintLine(1, "A=Seguir #=Borr");
  } else if (reinicioArmado) {
    lcdPrintLine(0, "REINICIO TOTAL");
    lcdPrintLine(1, "Pulse D de nuevo");
  } else if (keypadBuffer.length() > 0) {
    lcdPrintLine(0, "Tiempo: " + keypadBuffer + " s");
    lcdPrintLine(1, "*=OK #=Borrar");
  } else if (esEstado(ST_DONE)) {
    lcdPrintLine(0, "CICLO FINALIZADO");
    lcdPrintLine(1, "Ciclos: " + String(ciclosEjecutados));
  } else if (tiempoConfirmado_s > 0) {
    lcdPrintLine(0, "Tiempo: " + String(tiempoConfirmado_s) + " s");
    lcdPrintLine(1, "A=Iniciar #=Borr");
  } else {
    lcdPrintLine(0, "CABINA UV-C");
    lcdPrintLine(1, "Ingrese tiempo");
  }
}

void relayOn() {
  digitalWrite(RELAY_PIN, RELAY_ACTIVE_HIGH ? HIGH : LOW);
  estadoRelay = "ON";
}

void relayOff() {
  digitalWrite(RELAY_PIN, RELAY_ACTIVE_HIGH ? LOW : HIGH);
  estadoRelay = "OFF";
}

void setFault(String code, String detail, String systemState) {
  despertarInterfaz();
  ultimaActividadMs = millis();
  faultCode = code;
  faultDetail = detail;
  estadoSistema = systemState;
  marcarLCD();
}

void apagarUVCSeguro() {
  cicloActivo = false;
  relayOff();
}

void abortarPorComunicacion(String code, String detail, String state, String source) {
  apagarUVCSeguro();
  cicloPausado = false;
  requiereNuevoTiempo = true;
  cycle_s = 0; remaining_s = 0; duracionCiclo = 0; tiempoConfirmado_s = 0;
  keypadBuffer = "";
  registrarCmd(code, source);
  setFault(code, detail, state);
}

void entrarIdleSeguro(String cmd, String source, bool borrarTiempo) {
  apagarUVCSeguro();
  cicloPausado = false;
  if (borrarTiempo) {
    requiereNuevoTiempo = true;
    keypadBuffer = ""; tiempoConfirmado_s = 0; cycle_s = 0; remaining_s = 0; duracionCiclo = 0;
  }
  estadoSistema = ST_IDLE;
  faultCode = "NONE"; faultDetail = "Sistema operativo";
  registrarCmd(cmd, source);
  marcarLCD();
}

void actualizarBuzzer() {
  if (esEstado(ST_ALARM)) {
    digitalWrite(BUZZER_PIN, HIGH);
    return;
  }
  if (esEstado(ST_ERROR_WIFI) || esEstado(ST_ERROR_MQTT)) {
    if (millis() - ultimoToggleBuzzer >= 1000) {
      ultimoToggleBuzzer = millis();
      buzzerState = !buzzerState;
      digitalWrite(BUZZER_PIN, buzzerState ? HIGH : LOW);
    }
    return;
  }
  digitalWrite(BUZZER_PIN, LOW);
}

// ==========================================
// 9. CAPA DE TELEMETRÍA (JSON)
// ==========================================
void publicarTelemetria() {
  StaticJsonDocument<768> doc;
  doc["device_id"]    = "cabina_uvc_01";
  doc["fw_version"]   = "uvc_safe_2026_final_test";
  doc["msg_id"]       = ++msgCounter;
  doc["state"]        = estadoSistema;
  doc["door"]         = estadoPuerta;
  doc["relay_uvc"]    = estadoRelay;
  doc["cycle_s"]      = cycle_s;
  doc["remaining_s"]  = remaining_s;
  doc["confirmed_s"]  = tiempoConfirmado_s;
  doc["cycle_count"]  = ciclosEjecutados;
  doc["rssi"]         = (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : 0;
  doc["wifi_status"]  = wifiStatus;
  doc["mqtt_status"]  = mqttStatus;
  doc["fault_code"]   = faultCode;
  doc["fault_detail"] = faultDetail;
  doc["last_cmd"]     = lastCmd;
  doc["source"]       = lastSource;
  doc["free_heap"]    = ESP.getFreeHeap();
  doc["uptime_s"]     = millis() / 1000UL;
  doc["standby"]      = modoStandby;
  doc["ip_address"]   = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : "0.0.0.0";
  doc["ts"]           = millis();

  char buffer[768];
  serializeJson(doc, buffer);
  if (client.connected()) client.publish(topic_tele, buffer);
}

void publicarAlerta(String tipo, String severidad, String detalle) {
  StaticJsonDocument<128> doc;
  doc["type"] = tipo; doc["severity"] = severidad; doc["detail"] = detalle;
  char buffer[128];
  serializeJson(doc, buffer);
  if (client.connected()) client.publish(topic_alert, buffer);
}

void publicarAck(String cmd, bool ok, String detalle) {
  StaticJsonDocument<128> doc;
  doc["type"] = "ACK"; doc["cmd"] = cmd; doc["ok"] = ok; doc["detail"] = detalle;
  char buffer[128];
  serializeJson(doc, buffer);
  if (client.connected()) client.publish(topic_alert, buffer);
}

void gestionarStandby() {
  bool estadoSeguroParaStandby =
      !cicloActivo && estadoRelay == "OFF" &&
      (esEstado(ST_IDLE) || esEstado(ST_DONE));

  if (!estadoSeguroParaStandby || modoStandby) return;

  if (millis() - ultimaActividadMs >= STANDBY_TIMEOUT_MS) {
    modoStandby = true;
    registrarCmd("STANDBY", "system");
    fijarColorRGB(false, false, false);
    digitalWrite(BUZZER_PIN, LOW);
    lcd.clear();
    lcd.noBacklight();
    if (client.connected()) publicarTelemetria();
  }
}

void cancelarConfirmacionReinicio() {
  if (reinicioArmado && millis() - reinicioArmadoMs > RESTART_CONFIRM_MS) {
    reinicioArmado = false;
    registrarCmd("RESTART_CANCELLED", "system");
    marcarLCD();
  }
}

void reinicioForzado(String source) {
  registrarActividad(source);

  if (!reinicioArmado) {
    apagarUVCSeguro();
    cicloPausado = false;
    requiereNuevoTiempo = true;
    cycle_s = 0;
    remaining_s = 0;
    tiempoConfirmado_s = 0;
    duracionCiclo = 0;
    keypadBuffer = "";
    estadoSistema = ST_IDLE;
    faultCode = "NONE";
    faultDetail = "Sistema operativo";
    reinicioArmado = true;
    reinicioArmadoMs = millis();
    registrarCmd("RESTART_ARMED", source);
    actualizarLCD(true);
    publicarAck("FORCE_RESTART", false, "Confirmar con D nuevamente");
    publicarTelemetria();
    return;
  }

  if (millis() - reinicioArmadoMs <= RESTART_CONFIRM_MS) {
    apagarUVCSeguro();
    digitalWrite(BUZZER_PIN, LOW);
    fijarColorRGB(false, false, false);
    registrarCmd("FORCE_RESTART", source);
    publicarAlerta("SYSTEM_RESTART", "MEDIUM", "Reinicio total solicitado por operador");
    publicarAck("FORCE_RESTART", true, "Reiniciando ESP32");
    publicarTelemetria();
    lcd.backlight();
    lcdPrintLine(0, "REINICIANDO...");
    lcdPrintLine(1, "Espere");
    delay(500);
    ESP.restart();
  }
}

// ==========================================
// 10. GESTIÓN FIABLE DE TRANSICIONES DE CICLO
// ==========================================
bool precheckStart(String source) {
  leerPuerta();
  if (estadoPuerta == "OPEN") {
    apagarUVCSeguro();
    setFault("DOOR_OPEN", "Puerta abierta", ST_ALARM);
    registrarCmd("START_REJECTED", source);
    publicarAlerta("DOOR_OPEN", "HIGH", "Fallo de enclavamiento fisico");
    return false;
  }
  return true;
}

void iniciarCiclo(int segundos, String source) {
  if (cicloActivo || esEstado(ST_RUNNING) || (millis() - ultimoStartMs < START_COOLDOWN_MS)) return;

  int segundosAUsar = esEstado(ST_PAUSED) ? remaining_s : segundos;

  if (requiereNuevoTiempo && !esEstado(ST_PAUSED) && !tiempoValido(segundosAUsar)) {
    entrarIdleSeguro("START_REJECTED", source, false);
    return;
  }

  if (!precheckStart(source)) return;

  ultimoStartMs = millis();
  cycle_s = segundosAUsar; remaining_s = segundosAUsar;
  duracionCiclo = segundosAUsar * 1000UL; tiempoInicio = millis();
  cicloActivo = true; cicloPausado = false; requiereNuevoTiempo = false;

  if (tiempoConfirmado_s <= 0) tiempoConfirmado_s = segundosAUsar;

  estadoSistema = ST_RUNNING;
  faultCode = "NONE"; faultDetail = "Sistema operativo";
  registrarCmd("START", source);
  relayOn();
  publicarAck("START", true, "Ciclo activo");
  marcarLCD();
}

void pausarCiclo(String motivo, String source) {
  if (!cicloActivo && !esEstado(ST_RUNNING)) return;
  apagarUVCSeguro();
  cicloPausado = remaining_s > 0;
  duracionCiclo = 0; requiereNuevoTiempo = false;
  estadoSistema = ST_PAUSED;
  registrarCmd(motivo, source);
  publicarTelemetria();
  marcarLCD();
}

void finalizarCiclo(String source) {
  apagarUVCSeguro();
  ultimaActividadMs = millis();
  cicloPausado = false; requiereNuevoTiempo = true;
  ciclosEjecutados++;
  cycle_s = 0; remaining_s = 0; tiempoConfirmado_s = 0; duracionCiclo = 0; keypadBuffer = "";
  estadoSistema = ST_DONE;
  registrarCmd("DONE", source);
  publicarTelemetria();
  marcarLCD();
}

void manejarPuertaAbierta() {
  if (estadoPuerta != "OPEN") return;
  
  if (cicloActivo || estadoRelay == "ON" || esEstado(ST_RUNNING)) {
    apagarUVCSeguro();
    cicloPausado = remaining_s > 0; requiereNuevoTiempo = false; duracionCiclo = 0;
    setFault("DOOR_OPEN", "Puerta abierta", ST_ALARM);
    registrarCmd("DOOR_OPEN", "reed_switch");
    publicarAlerta("DOOR_OPEN", "HIGH", "Apertura forzada: ciclo en pausa");
    publicarTelemetria();
    actualizarLCD(true);
  }
}

void manejarPuertaCerrada() {
  if (!(estadoPuerta == "CLOSED" && esEstado(ST_ALARM) && faultCode == "DOOR_OPEN")) return;
  relayOff();
  digitalWrite(BUZZER_PIN, LOW); buzzerState = false;
  faultCode = "NONE"; faultDetail = "Sistema operativo";
  registrarCmd("DOOR_CLOSED", "reed_switch");

  estadoSistema = (cicloPausado && remaining_s > 0) ? ST_PAUSED : ST_IDLE;
  requiereNuevoTiempo = !cicloPausado;

  publicarTelemetria();
  actualizarLCD(true);
}

// ==========================================
// 11. GESTIÓN DE ENTRADAS (KEYPAD / OPERACIONES)
// ==========================================
bool teclaPermitida(char key) {
  if (esEstado(ST_RUNNING)) return key == 'B' || key == 'D';
  if (esEstado(ST_PAUSED))  return key == 'A' || key == '#' || key == 'D';
  if (esEstado(ST_ALARM))   return key == 'D';
  if (esEstado(ST_DONE))    return (key >= '0' && key <= '9') || key == '#' || key == 'D';
  if (esEstado(ST_ERROR_WIFI) || esEstado(ST_ERROR_MQTT)) return key == 'D';
  return true;
}

bool teclaRepetida(char key) {
  if (key != ultimaTecla) {
    ultimaTecla = key; ultimaTeclaMs = millis();
    return false;
  }
  if (millis() - ultimaTeclaMs < KEY_COOLDOWN_MS) return true;
  ultimaTeclaMs = millis();
  return false;
}

void confirmarTiempo(String source) {
  int valor = keypadBuffer.toInt();
  if (!tiempoValido(valor)) return;
  
  tiempoConfirmado_s = valor; cycle_s = valor; remaining_s = valor;
  keypadBuffer = ""; requiereNuevoTiempo = false; cicloPausado = false;
  estadoSistema = ST_IDLE;
  registrarCmd("ENTER", source);
  marcarLCD();
}

void procesarTecla(char key, String source) {
  registrarActividad(source);
  if (teclaRepetida(key) || !teclaPermitida(key)) return;

  if (key >= '0' && key <= '9') {
    if (esEstado(ST_DONE)) { estadoSistema = ST_IDLE; }
    if (keypadBuffer.length() < MAX_DIGITS) keypadBuffer += key;
    marcarLCD();
    return;
  }
  if (key == '#') { entrarIdleSeguro("CLEAR", source, true); return; }
  if (key == '*') { confirmarTiempo(source); return; }
  if (key == 'A') { iniciarCiclo(tiempoConfirmado_s, source); return; }
  if (key == 'B') { pausarCiclo("STOP", source); return; }
  if (key == 'C') { entrarIdleSeguro("RESET", source, false); remaining_s = tiempoConfirmado_s; return; }
  if (key == 'D') { reinicioForzado(source); return; }
}

// ==========================================
// 12. CALLBACK DE COMANDOS (MQTT)
// ==========================================
void callback(char* topic, byte* payload, unsigned int length) {
  String mensaje = "";
  for (unsigned int i = 0; i < length; i++) mensaje += (char)payload[i];

  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, mensaje)) return;

  String cmd    = doc["cmd"] | "";
  String source = doc["source"] | "dashboard";
  int tiempo    = doc["cycle_s"] | 0;
  cmd.toUpperCase();

  if (millis() - bootMs < MQTT_CMD_GRACE_MS) return;
  registrarActividad(source);

  if (cmd == "KEY") {
    const char* keyText = doc["key"] | "";
    if (strlen(keyText) > 0) procesarTecla(keyText[0], source);
    return;
  }
  if (cmd == "START")      iniciarCiclo(tiempo, source);
  else if (cmd == "STOP")  pausarCiclo("STOP", source);
  else if (cmd == "RESET") { if (!esEstado(ST_RUNNING)) entrarIdleSeguro("RESET", source, false); }
  else if (cmd == "SAFE_RESET" || cmd == "FORCE_RESTART") reinicioForzado(source);
}

// ==========================================
// 13. CAPAS DE CONECTIVIDAD (WIFI/MQTT)
// ==========================================
bool conectarWiFi() {
  if (WiFi.status() == WL_CONNECTED) { wifiStatus = "CONNECTED"; return true; }
  if (millis() - ultimoIntentoWiFi < WIFI_RETRY_MS) return false;

  ultimoIntentoWiFi = millis();
  wifiStatus = "SEARCHING";

  if (millis() - bootMs > 3000) abortarPorComunicacion("WIFI_LOST", "Modo local activo", ST_ERROR_WIFI, "system");
  actualizarLCD(true);

  for (int i = 0; i < wifiCount; i++) {
    WiFi.disconnect(true);
    delay(500);
    WiFi.begin(ssids[i], passwords[i]);

    int intentos = 0;
    while (WiFi.status() != WL_CONNECTED && intentos < 6) {
      delay(400);
      intentos++;
    }
    if (WiFi.status() == WL_CONNECTED) {
      wifiStatus = "CONNECTED";
      if (!cicloActivo && !esEstado(ST_ALARM)) { estadoSistema = ST_IDLE; faultCode = "NONE"; }
      actualizarLCD(true);
      return true;
    }
  }
  wifiStatus = "DISCONNECTED";
  return false;
}

bool reconnect() {
  if (client.connected()) { mqttStatus = "CONNECTED"; return true; }
  if (WiFi.status() != WL_CONNECTED || (millis() - ultimoIntentoMQTT < MQTT_RETRY_MS)) return false;

  ultimoIntentoMQTT = millis();
  mqttStatus = "CONNECTING";
  actualizarLCD(true);

  String clientId = "ESP32_CISA_" + String(random(0xffff), HEX);
  if (client.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
    mqttStatus = "CONNECTED";
    client.subscribe(topic_cmd, 1);
    if (!cicloActivo && !esEstado(ST_ALARM)) { estadoSistema = ST_IDLE; faultCode = "NONE"; }
    publicarTelemetria();
    actualizarLCD(true);
    return true;
  }
  mqttStatus = "DISCONNECTED";
  return false;
}

// ==========================================
// 14. INICIALIZACIÓN DEL ENTORNO (SETUP)
// ==========================================
void setup() {
  bootMs = millis();
  Serial.begin(115200);

  pinMode(RGB_RED_PIN, OUTPUT);
  pinMode(RGB_GREEN_PIN, OUTPUT);
  pinMode(RGB_BLUE_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(REED_SWITCH_PIN, INPUT_PULLUP);

  relayOff();
  digitalWrite(BUZZER_PIN, LOW);

  Wire.begin(I2C_SDA, I2C_SCL);
  lcd.init();
  lcd.backlight();
  lcdPrintLine(0, "Cabina UV-C IoT");
  lcdPrintLine(1, "CISA - UNI");

  // Secuencia de inicio: Parpadeo Rojo/Verde
  unsigned long arranqueMs = millis();
  bool toggleArranque = false;
  while (millis() - arranqueMs < 4000) {
    toggleArranque = !toggleArranque;
    fijarColorRGB(toggleArranque, !toggleArranque, false);
    delay(250); 
  }

  espClient.setInsecure();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  client.setBufferSize(768);

  conectarWiFi();
  leerPuerta();
  ultimaActividadMs = millis();
  actualizarLCD(true);
}

// ==========================================
// 15. BUCLE DE EJECUCIÓN CONTINUA (LOOP)
// ==========================================
void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    wifiStatus = "DISCONNECTED"; mqttStatus = "DISCONNECTED";
    if (millis() - bootMs > 3000 && !esEstado(ST_ERROR_WIFI)) {
      abortarPorComunicacion("WIFI_LOST", "Fallo de Red", ST_ERROR_WIFI, "system");
    }
    conectarWiFi();
  } else {
    wifiStatus = "CONNECTED";
    if (!client.connected()) {
      mqttStatus = "DISCONNECTED";
      if (millis() - bootMs > 3000 && !esEstado(ST_ERROR_MQTT)) {
        abortarPorComunicacion("MQTT_LOST", "Fallo de Broker", ST_ERROR_MQTT, "system");
      }
      reconnect();
    } else {
      mqttStatus = "CONNECTED";
      client.loop();
    }
  }

  // Capa Crítica de Control de Puerta (Enclavamiento <50ms)
  leerPuerta();
  manejarPuertaAbierta();
  manejarPuertaCerrada();

  // Control de temporización del ciclo activo
  if (cicloActivo) {
    unsigned long transcurrido = millis() - tiempoInicio;
    if (transcurrido >= duracionCiclo) {
      remaining_s = 0;
      finalizarCiclo("system");
    } else {
      int nuevoRestante = cycle_s - (transcurrido / 1000);
      if (nuevoRestante != remaining_s) {
        remaining_s = nuevoRestante;
        marcarLCD();
      }
    }
  }

  // Lectura de teclado local
  char key = keypad.getKey();
  if (key != NO_KEY) {
    procesarTecla(key, "local_keypad");
  }

  // Gestión de espera y confirmación de reinicio
  cancelarConfirmacionReinicio();
  gestionarStandby();

  // Actualización de estado visual e indicadores
  actualizarLCD();
  actualizarBuzzer();
  actualizarIndicadoresVisuales();

  // Envío periódico de telemetría por MQTT
  if (client.connected() && (millis() - ultimoEnvioTelemetria >= TELEMETRY_INTERVAL_MS)) {
    publicarTelemetria();
    ultimoEnvioTelemetria = millis();
  }
}