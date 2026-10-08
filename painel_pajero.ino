/*
 * PAINEL OBD-II - PAJERO TR4 2007 (GASOLINA)
 * Placa: ESP32-2432S028R ("CYD") - display 2,8" 240x320 + touch resistivo
 * Adaptador: ELM327 Bluetooth Classic
 *
 * Paginas:
 *   0 = VIAGEM      (distancia, tempos, medias, ritmo, consumo, litros, carga media)
 *   1 = OFF-ROAD    (barras de carga e temperatura, RPM, bateria, extras)
 *
 * Toque curto : troca de pagina
 * Toque longo : (2 s) zera a viagem (pagina 0) ou os picos da sessao (pagina 1)
 *
 * Arquitetura:
 *   - Tarefa FreeRTOS "obd" (core 0): Bluetooth, comandos AT, consultas PID.
 *   - loop() (core 1): touch, calculos da viagem, desenho, gravacao na NVS.
 *   Assim, uma reconexao Bluetooth lenta nunca congela a tela.
 *
 * Bibliotecas: TFT_eSPI, XPT2046_Touchscreen (Paul Stoffregen),
 *              BluetoothSerial, Preferences (as duas ultimas vem com o core ESP32)
 * Particao:    "Huge APP" (Bluetooth Classic ocupa muita flash)
 */

#include <TFT_eSPI.h>
#include <SPI.h>
#include <XPT2046_Touchscreen.h>
#include <BluetoothSerial.h>
#include <Preferences.h>
#include <math.h>
#include "tipos.h"   // tipos em arquivo separado (evita bug dos prototipos automaticos da Arduino IDE)

// =====================================================================
//  CONFIGURACAO (ajuste aqui)
// =====================================================================

// MAC do adaptador ELM327
uint8_t mac_elm[6] = {0xAA, 0xBB, 0xCC, 0x11, 0x22, 0x33};

// --- Tanque -----------------------------------------------------------
const float TANQUE_L               = 53.0f;
const float TANQUE_ALERTA_AMARELO  = 0.70f;   // 70% usado (~37 L)
const float TANQUE_ALERTA_VERMELHO = 0.85f;   // 85% usado (~45 L, sobram ~8 L)

// --- Estimativa de consumo (so usada se o carro NAO tiver MAF, PID 0110)
// Metodo "speed-density": calcula o ar pela pressao do coletor (MAP).
const float MOTOR_CILINDRADA_L  = 2.0f;    // litros
const float EFIC_VOLUMETRICA    = 0.80f;   // chute inicial; calibre pelo consumo real
const float AFR                 = 14.7f;   // relacao ar/combustivel (gasolina pura)
const float DENSIDADE_G_L       = 745.0f;  // g/L da gasolina
const float FATOR_CALIB_CONSUMO = 1.00f;   // multiplicador final (calibre apos abastecer)

// --- Carga do motor (%) ---
const int CARGA_AMARELO  = 71;   // >= 71 amarelo
const int CARGA_VERMELHO = 85;   // >= 85 vermelho

// --- Temperatura do liquido (C) ---
const int TEMP_FRIO     = 70;    // abaixo: azul (motor ainda aquecendo)
const int TEMP_AMARELO  = 100;   // >= 100 amarelo
const int TEMP_VERMELHO = 105;   // >= 105 vermelho + LED + aviso
const int TEMP_MIN_BARRA = 40;   // escala da barra
const int TEMP_MAX_BARRA = 120;

// --- RPM ---
const int RPM_AMARELO  = 4500;
const int RPM_VERMELHO = 5500;

// --- Viagem ---
const float VEL_MOVIMENTO_KMH = 3.0f;       // acima disso conta como "em movimento"
const uint32_t SALVAR_A_CADA_MS = 30000;    // NVS
const uint32_t TOQUE_CURTO_MAX_MS = 700;
const uint32_t TOQUE_LONGO_MS     = 2000;
const float TAU_CONSUMO_S = 10.0f;          // suavizacao do consumo instantaneo
const uint8_t JANELA_RITMO_S = 30;          // media movel do ritmo (segundos)

// =====================================================================
//  HARDWARE
// =====================================================================
#define T_CLK  25
#define T_MISO 39
#define T_MOSI 32
#define T_CS   33
#define LED_R   4      // LED RGB da placa (logica invertida: LOW = acende)
#define LED_G  16
#define LED_B  17

TFT_eSPI tft = TFT_eSPI();
SPIClass touchSPI(VSPI);
XPT2046_Touchscreen ts(T_CS);
BluetoothSerial SerialBT;
Preferences prefs;

// Cores
const uint16_t COR_ROT   = 0x8410;  // cinza dos rotulos
const uint16_t COR_GRADE = 0x2104;  // cinza escuro das linhas
const uint16_t COR_AZUL  = 0x04FF;  // azul claro (motor frio)

// =====================================================================
//  ESTADO COMPARTILHADO ENTRE AS TAREFAS
// =====================================================================
volatile EstadoLink estadoLink = LINK_DESCONECTADO;
volatile uint8_t paginaAtual = 0;

struct Dados {
  volatile float vel = 0, rpm = 0, carga = 0, temp = 0, acel = 0, iat = 0;
  volatile float mapKpa = 0, maf = 0, volt = 0;
  // millis() da ultima leitura valida (0 = nunca)
  volatile uint32_t tVel = 0, tRpm = 0, tCarga = 0, tTemp = 0, tAcel = 0, tIat = 0;
  volatile uint32_t tMap = 0, tMaf = 0, tVolt = 0;
} d;

inline bool fresco(uint32_t t, uint32_t idade = 4000) {
  return t != 0 && (millis() - t) < idade;
}

// =====================================================================
//  OBD / ELM327  (roda na tarefa "obd")
// =====================================================================


// A ORDEM deve ser a mesma do enum QId
Consulta consultas[Q_N] = {
  /* Q_VEL   */ {"010D", 0x0D,  500, false, true, 0, 0},
  /* Q_RPM   */ {"010C", 0x0C,  500, false, true, 0, 0},
  /* Q_CARGA */ {"0104", 0x04,  800, false, true, 0, 0},
  /* Q_MAF   */ {"0110", 0x10,  800, false, true, 0, 0},
  /* Q_MAP   */ {"010B", 0x0B,  800, false, true, 0, 0},
  /* Q_TEMP  */ {"0105", 0x05, 2500, false, true, 0, 0},
  /* Q_IAT   */ {"010F", 0x0F, 5000, false, true, 0, 0},
  /* Q_ACEL  */ {"0111", 0x11, 1000, true,  true, 0, 0},
  /* Q_VOLT  */ {"ATRV", 0x00, 2000, false, true, 0, 0},
};

bool pidSup[256];
char resp[192];

static inline void espera(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }

// Envia um comando e espera o prompt '>'. Guarda a resposta em buf
// (sem espacos nem '\n'; '\r' fica como separador de linhas).
bool enviaEEspera(const char* cmd, char* buf, size_t tam, uint32_t timeoutMs) {
  while (SerialBT.available()) SerialBT.read();   // descarta lixo antigo
  char linha[16];
  snprintf(linha, sizeof(linha), "%s\r", cmd);
  SerialBT.print(linha);

  size_t n = 0;
  buf[0] = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    if (!SerialBT.connected()) return false;
    while (SerialBT.available()) {
      char c = SerialBT.read();
      if (c == '>') { buf[n] = 0; return true; }
      if (c == ' ' || c == '\n') continue;
      if (n < tam - 1) buf[n++] = c;
    }
    espera(1);
  }
  buf[n] = 0;
  return false;
}

int hexv(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

// Procura "41<pid>" na resposta e le n bytes hexadecimais depois dele.
bool extrai(const char* r, uint8_t pid, uint8_t* out, uint8_t n) {
  char marca[5];
  snprintf(marca, sizeof(marca), "41%02X", pid);
  const char* p = strstr(r, marca);
  if (!p) return false;
  p += 4;
  for (uint8_t i = 0; i < n; i++) {
    if (!p[0] || !p[1]) return false;
    int h = hexv(p[0]), l = hexv(p[1]);
    if (h < 0 || l < 0) return false;
    out[i] = (uint8_t)(h * 16 + l);
    p += 2;
  }
  return true;
}

void marcaSuporte(uint8_t base, const uint8_t* b) {
  for (int i = 0; i < 32; i++) {
    if (b[i / 8] & (0x80 >> (i % 8))) pidSup[base + 1 + i] = true;
  }
}

bool bitmapOk = false;

// Pergunta ao carro quais PIDs ele suporta (0100 / 0120 / 0140).
bool descobrePids() {
  uint8_t b[4];
  // O primeiro comando dispara a busca de protocolo (ISO 9141/KWP pode levar ~10 s)
  if (!enviaEEspera("0100", resp, sizeof(resp), 30000)) return false;
  if (!extrai(resp, 0x00, b, 4)) return false;

  memset(pidSup, 0, sizeof(pidSup));
  marcaSuporte(0x00, b);
  if (pidSup[0x20] && enviaEEspera("0120", resp, sizeof(resp), 4000) && extrai(resp, 0x20, b, 4))
    marcaSuporte(0x20, b);
  if (pidSup[0x40] && enviaEEspera("0140", resp, sizeof(resp), 4000) && extrai(resp, 0x40, b, 4))
    marcaSuporte(0x40, b);

  bitmapOk = true;
  Serial.print("PIDs suportados:");
  for (int i = 1; i < 0x61; i++) if (pidSup[i]) Serial.printf(" %02X", i);
  Serial.println();
  return true;
}

void aplicaSuporte() {
  for (uint8_t i = 0; i < Q_N; i++) {
    Consulta& q = consultas[i];
    q.ativo  = (q.pid == 0) || !bitmapOk || pidSup[q.pid];
    q.falhas = 0;
    q.ultima = 0;
  }
  // Se o carro tem MAF, nao precisamos do MAP para estimar o consumo
  if (bitmapOk && consultas[Q_MAF].ativo) consultas[Q_MAP].ativo = false;
}

bool configuraELM() {
  estadoLink = LINK_CONFIGURANDO;
  const char* cmds[] = {"ATZ", "ATE0", "ATL0", "ATS0", "ATH0", "ATSP0"};
  for (uint8_t i = 0; i < 6; i++) {
    uint32_t to = (i == 0) ? 5000 : 2500;
    if (!enviaEEspera(cmds[i], resp, sizeof(resp), to)) return false;
    if (i == 0) espera(200);
  }

  // Espera a ECU responder (ignicao ligada)
  estadoLink = LINK_SEM_ECU;
  while (SerialBT.connected()) {
    if (descobrePids()) break;
    espera(2000);
  }
  if (!SerialBT.connected()) return false;

  aplicaSuporte();
  estadoLink = LINK_ONLINE;
  return true;
}

// Escolhe a consulta mais "atrasada" em relacao ao intervalo desejado.
int proximaConsulta() {
  uint32_t agora = millis();
  int melhor = -1;
  float melhorScore = 1.0f;
  for (uint8_t i = 0; i < Q_N; i++) {
    Consulta& q = consultas[i];
    if (!q.ativo) continue;
    if (q.soPagina2 && paginaAtual != 1) continue;
    float score = (float)(agora - q.ultima) / (float)q.intervalo;
    if (score >= melhorScore) { melhorScore = score; melhor = i; }
  }
  return melhor;
}

bool executaConsulta(QId id) {
  Consulta& q = consultas[id];
  if (!enviaEEspera(q.cmd, resp, sizeof(resp), 3000)) return false;

  uint8_t b[2];
  uint32_t t = millis();
  switch (id) {
    case Q_VOLT: {
      float v = atof(resp);                      // "12.6V"
      if (v < 3.0f || v > 30.0f) return false;
      d.volt = v; d.tVolt = t; return true;
    }
    case Q_VEL:
      if (!extrai(resp, q.pid, b, 1)) return false;
      d.vel = b[0]; d.tVel = t; return true;
    case Q_RPM:
      if (!extrai(resp, q.pid, b, 2)) return false;
      d.rpm = (b[0] * 256.0f + b[1]) / 4.0f; d.tRpm = t; return true;
    case Q_CARGA:
      if (!extrai(resp, q.pid, b, 1)) return false;
      d.carga = b[0] * 100.0f / 255.0f; d.tCarga = t; return true;
    case Q_MAF:
      if (!extrai(resp, q.pid, b, 2)) return false;
      d.maf = (b[0] * 256.0f + b[1]) / 100.0f; d.tMaf = t; return true;
    case Q_MAP:
      if (!extrai(resp, q.pid, b, 1)) return false;
      d.mapKpa = b[0]; d.tMap = t; return true;
    case Q_TEMP:
      if (!extrai(resp, q.pid, b, 1)) return false;
      d.temp = (float)b[0] - 40.0f; d.tTemp = t; return true;
    case Q_IAT:
      if (!extrai(resp, q.pid, b, 1)) return false;
      d.iat = (float)b[0] - 40.0f; d.tIat = t; return true;
    case Q_ACEL:
      if (!extrai(resp, q.pid, b, 1)) return false;
      d.acel = b[0] * 100.0f / 255.0f; d.tAcel = t; return true;
    default: return false;
  }
}

void cicloConsultas() {
  uint8_t falhasSeguidas = 0;
  uint32_t tUltimoOk = millis();

  while (SerialBT.connected()) {
    int i = proximaConsulta();
    if (i < 0) { espera(5); continue; }

    consultas[i].ultima = millis();
    if (executaConsulta((QId)i)) {
      consultas[i].falhas = 0;
      falhasSeguidas = 0;
      tUltimoOk = millis();
      estadoLink = LINK_ONLINE;
    } else {
      consultas[i].falhas++;
      falhasSeguidas++;
      // PID que falha sempre enquanto os outros funcionam = nao suportado de fato
      if (consultas[i].falhas >= 8 && (millis() - tUltimoOk) < 3000) {
        consultas[i].ativo = false;
        Serial.printf("Consulta %s desativada (sem resposta)\n", consultas[i].cmd);
      }
      // Tudo falhando: ignicao desligada, ECU fora, cabo solto...
      if (falhasSeguidas >= 12) {
        estadoLink = LINK_SEM_ECU;
        while (SerialBT.connected() && !descobrePids()) espera(2000);
        if (!SerialBT.connected()) return;
        aplicaSuporte();
        falhasSeguidas = 0;
        tUltimoOk = millis();
        estadoLink = LINK_ONLINE;
      }
    }
  }
}

void tarefaOBD(void* arg) {
  for (;;) {
    if (!SerialBT.connected()) {
      estadoLink = LINK_CONECTANDO;
      if (!SerialBT.connect(mac_elm)) {
        estadoLink = LINK_DESCONECTADO;
        espera(4000);
        continue;
      }
    }
    if (!configuraELM()) {
      SerialBT.disconnect();
      estadoLink = LINK_DESCONECTADO;
      espera(3000);
      continue;
    }
    cicloConsultas();
    // Saiu do ciclo = perdeu o Bluetooth. Marca e tenta de novo.
    estadoLink = LINK_DESCONECTADO;
    SerialBT.disconnect();
    espera(2000);
  }
}

// =====================================================================
//  VIAGEM (calculos e persistencia) - roda no loop()
// =====================================================================
const uint32_t VERSAO_DADOS = 1;

struct Viagem {
  uint32_t versao;
  double distKm;
  double tTotal;     // s, motor ligado
  double tMov;       // s, v >= VEL_MOVIMENTO_KMH
  double litros;
  double cargaInt;   // integral da carga (% * s)
  double tCarga;     // s com carga valida
};
Viagem v;
bool sujo = false;
uint32_t tUltSalvar = 0;
uint32_t tInteg = 0;

// media movel da velocidade (ritmo instantaneo)
float velBuf[JANELA_RITMO_S];
uint8_t velIdx = 0, velN = 0;
uint32_t tAmostraVel = 0;

// consumo instantaneo (EMA)
float lhEma = 0, velEma = 0;
bool emaInit = false;

// picos da sessao (so RAM)
float picoTemp = -999, picoCarga = -1;

void salvaViagem() {
  prefs.putBytes("viagem", &v, sizeof(v));
  sujo = false;
}

void carregaViagem() {
  memset(&v, 0, sizeof(v));
  v.versao = VERSAO_DADOS;
  if (prefs.getBytesLength("viagem") == sizeof(Viagem)) {
    Viagem tmp;
    prefs.getBytes("viagem", &tmp, sizeof(tmp));
    if (tmp.versao == VERSAO_DADOS) v = tmp;
  }
}

void resetViagem() {
  memset(&v, 0, sizeof(v));
  v.versao = VERSAO_DADOS;
  velN = 0; velIdx = 0;
  emaInit = false;
  salvaViagem();
}

bool combustivelDisponivel() {
  return consultas[Q_MAF].ativo || consultas[Q_MAP].ativo;
}

// Vazao de combustivel em L/h (NAN se nao der para calcular)
float fluxoLh() {
  float maf = NAN;   // g/s de ar
  if (consultas[Q_MAF].ativo && fresco(d.tMaf)) {
    maf = d.maf;
  } else if (consultas[Q_MAP].ativo && fresco(d.tMap) && fresco(d.tRpm)) {
    float iat = fresco(d.tIat, 15000) ? d.iat : 30.0f;
    float tK = iat + 273.15f;
    // massa de ar por ciclo (mol) * 28,97 g/mol * ciclos por segundo (RPM/120)
    maf = (d.rpm / 120.0f) * (d.mapKpa * MOTOR_CILINDRADA_L * EFIC_VOLUMETRICA) / (8.314f * tK) * 28.97f;
  }
  if (isnan(maf)) return NAN;
  float lh = maf * 3600.0f / (AFR * DENSIDADE_G_L) * FATOR_CALIB_CONSUMO;
  if (lh < 0) lh = 0;
  if (lh > 80) lh = 80;
  return lh;
}

void integraViagem(uint32_t agora) {
  double dt = (agora - tInteg) / 1000.0;
  tInteg = agora;
  if (dt <= 0 || dt > 1.0) return;

  bool motor = fresco(d.tRpm) && d.rpm > 300;
  if (!motor) return;

  v.tTotal += dt;
  bool velOk = fresco(d.tVel);
  float vel = velOk ? d.vel : 0.0f;

  if (velOk) {
    v.distKm += vel * dt / 3600.0;
    if (vel >= VEL_MOVIMENTO_KMH) v.tMov += dt;
  }
  if (fresco(d.tCarga)) {
    v.cargaInt += d.carga * dt;
    v.tCarga += dt;
  }

  float lh = fluxoLh();
  if (!isnan(lh)) {
    v.litros += lh * dt / 3600.0;
    if (!emaInit) { lhEma = lh; velEma = vel; emaInit = true; }
    else {
      float a = (float)dt / (TAU_CONSUMO_S + (float)dt);
      lhEma  += a * (lh - lhEma);
      velEma += a * (vel - velEma);
    }
  }

  // amostra a velocidade 1x por segundo para a media movel do ritmo
  if (agora - tAmostraVel >= 1000) {
    tAmostraVel = agora;
    velBuf[velIdx] = vel;
    velIdx = (velIdx + 1) % JANELA_RITMO_S;
    if (velN < JANELA_RITMO_S) velN++;
  }

  if (fresco(d.tTemp) && d.temp > picoTemp) picoTemp = d.temp;
  if (fresco(d.tCarga) && d.carga > picoCarga) picoCarga = d.carga;

  sujo = true;
}

float velMediaMovel() {
  if (velN == 0) return 0;
  float s = 0;
  for (uint8_t i = 0; i < velN; i++) s += velBuf[i];
  return s / velN;
}

// =====================================================================
//  UI - helpers
// =====================================================================

void desenhaRotulo(const Celula& c) {
  if (!c.rotulo) return;
  tft.setTextDatum(TL_DATUM);
  tft.setTextPadding(0);
  tft.setTextColor(COR_ROT, TFT_BLACK);
  tft.drawString(c.rotulo, c.x, c.y, c.fonteRot);
}

void setCelula(Celula& c, const char* txt, uint16_t cor) {
  if (cor == c.ultCor && strcmp(txt, c.ult) == 0) return;
  tft.setTextDatum(c.datum);
  tft.setTextColor(cor, TFT_BLACK);
  tft.setTextPadding(c.w);
  tft.drawString(txt, c.x, c.y + c.yVal, c.fonteVal);
  strncpy(c.ult, txt, sizeof(c.ult) - 1);
  c.ult[sizeof(c.ult) - 1] = 0;
  c.ultCor = cor;
}

void limpaCache(Celula& c) { c.ult[0] = 0; c.ultCor = 0; }

// ---- Pagina 1 (viagem): grade 2 colunas x 5 linhas ----
// campos: x, y, w, yVal, fonteVal, rotulo, fonteRot, ult, ultCor, datum
Celula p1[10] = {
  {  4,  20, 152, 16, 4, "DISTANCIA km",    2, "", 0, 0},
  {164,  20, 152, 16, 4, "TEMPO TOTAL",     2, "", 0, 0},
  {  4,  64, 152, 16, 4, "TEMPO MOVIM.",    2, "", 0, 0},
  {164,  64, 152, 16, 4, "VEL MEDIA km/h",  2, "", 0, 0},
  {  4, 108, 152, 16, 4, "RITMO MED /10km", 2, "", 0, 0},
  {164, 108, 152, 16, 4, "RITMO INS /10km", 2, "", 0, 0},
  {  4, 152, 152, 16, 4, "CONSUMO INS km/L",2, "", 0, 0},
  {164, 152, 152, 16, 4, "CONSUMO MED km/L",2, "", 0, 0},
  {  4, 196, 152, 16, 4, "LITROS USADOS",   2, "", 0, 0},
  {164, 196, 152, 16, 4, "CARGA MEDIA %",   2, "", 0, 0},
};

// ---- Pagina 2 (off-road) ----
Celula p2Rpm  = {120, 20, 196, 18, 7, "ROTACAO RPM", 2, "", 0, 0};
Celula p2Bat  = {120, 92, 196, 18, 7, "BATERIA V",   2, "", 0, 0};
Celula p2Vel  = {120, 164,  96, 10, 4, "VELOCIDADE km/h", 1, "", 0, 0};
Celula p2Acel = {220, 164,  96, 10, 4, "ACELERADOR %",    1, "", 0, 0};
Celula p2Iat  = {120, 202,  96, 10, 4, "AR ADMISSAO C",   1, "", 0, 0};
Celula p2Pico = {220, 202,  96, 10, 4, "PICO TEMP C",     1, "", 0, 0};
// valores sob as barras (centralizados)
Celula p2CargaV = {29, 204, 54, 0, 4, nullptr, 2, "", 0, TC_DATUM};
Celula p2TempV  = {87, 204, 54, 0, 4, nullptr, 2, "", 0, TC_DATUM};

Barra barCarga = {  2, 38, 54, 160, 0.70f, 0.85f, -1, 0};
Barra barTemp  = { 60, 38, 54, 160,
                   (float)(TEMP_AMARELO  - TEMP_MIN_BARRA) / (TEMP_MAX_BARRA - TEMP_MIN_BARRA),
                   (float)(TEMP_VERMELHO - TEMP_MIN_BARRA) / (TEMP_MAX_BARRA - TEMP_MIN_BARRA),
                   -1, 0};

void desenhaMoldura(const Barra& b) {
  tft.drawRect(b.x, b.y, b.w, b.h, COR_ROT);
}

void atualizaBarra(Barra& b, float frac, uint16_t cor) {
  if (frac < 0) frac = 0;
  if (frac > 1) frac = 1;
  int16_t ih  = b.h - 2;
  int16_t alt = (int16_t)(frac * ih + 0.5f);
  if (alt == b.ultAlt && cor == b.ultCor) return;

  int16_t yInt = b.y + 1;
  tft.fillRect(b.x + 1, yInt, b.w - 2, ih - alt, TFT_BLACK);
  tft.fillRect(b.x + 1, yInt + ih - alt, b.w - 2, alt, cor);
  // marcas dos limites
  int16_t y1 = yInt + ih - (int16_t)(b.t1 * ih);
  int16_t y2 = yInt + ih - (int16_t)(b.t2 * ih);
  tft.drawFastHLine(b.x + 1, y1, b.w - 2, TFT_WHITE);
  tft.drawFastHLine(b.x + 1, y2, b.w - 2, TFT_WHITE);
  b.ultAlt = alt;
  b.ultCor = cor;
}

// ---- cores por faixa ----
uint16_t corCarga(int pct) {
  if (pct >= CARGA_VERMELHO) return TFT_RED;
  if (pct >= CARGA_AMARELO)  return TFT_YELLOW;
  return TFT_GREEN;
}

uint16_t corTemp(float t) {
  if (t >= TEMP_VERMELHO) return TFT_RED;
  if (t >= TEMP_AMARELO)  return TFT_YELLOW;
  if (t <  TEMP_FRIO)     return COR_AZUL;
  return TFT_GREEN;
}

uint16_t corTensao(float volt, bool motorLigado) {
  if (motorLigado) {
    if (volt < 12.2f || volt > 15.0f) return TFT_RED;
    if (volt < 13.2f) return TFT_YELLOW;     // alternador carregando pouco
    return TFT_GREEN;
  }
  if (volt < 12.0f) return TFT_RED;
  if (volt < 12.4f) return TFT_YELLOW;
  return TFT_GREEN;
}

uint16_t corLitros(float litros) {
  if (litros >= TANQUE_L * TANQUE_ALERTA_VERMELHO) return TFT_RED;
  if (litros >= TANQUE_L * TANQUE_ALERTA_AMARELO)  return TFT_YELLOW;
  return TFT_WHITE;
}

// ---- formatacao ----
void fmtTempo(char* buf, size_t n, double seg) {
  uint32_t s = (uint32_t)seg;
  snprintf(buf, n, "%u:%02u:%02u", (unsigned)(s / 3600), (unsigned)((s / 60) % 60), (unsigned)(s % 60));
}

// ritmo em min/10 km no formato m:ss, a partir da velocidade (km/h)
void fmtRitmo(char* buf, size_t n, float kmh) {
  if (kmh < 2.0f) { snprintf(buf, n, "--:--"); return; }
  uint32_t tot = (uint32_t)(600.0f / kmh * 60.0f + 0.5f);
  if (tot > 99 * 60 + 59) { snprintf(buf, n, "--:--"); return; }
  snprintf(buf, n, "%u:%02u", (unsigned)(tot / 60), (unsigned)(tot % 60));
}

// =====================================================================
//  UI - cabecalho, paginas, status
// =====================================================================
uint8_t bootFase = 0;             // 0 = conectando, 1 = lista de PIDs, 2 = paginas
uint32_t tBoot = 0;
int8_t statusMostrado = -1;

EstadoLink headerEstado = LINK_DESCONECTADO;
bool headerAlarme = false;
bool alarmeTemp = false;
char msgTxt[24] = "";
uint32_t msgAte = 0;
bool msgVisivel = false;
bool headerForcar = true;

void setMsg(const char* t) {
  strncpy(msgTxt, t, sizeof(msgTxt) - 1);
  msgTxt[sizeof(msgTxt) - 1] = 0;
  msgAte = millis() + 2000;
  headerForcar = true;
}

uint16_t corLink(EstadoLink e) {
  switch (e) {
    case LINK_ONLINE:       return TFT_GREEN;
    case LINK_DESCONECTADO: return TFT_RED;
    default:                return TFT_ORANGE;
  }
}

void desenhaHeader() {
  tft.fillRect(0, 0, 320, 18, TFT_BLACK);
  tft.setTextDatum(TL_DATUM);
  tft.setTextPadding(0);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.drawString(paginaAtual == 0 ? "VIAGEM" : "OFF-ROAD", 4, 1, 2);

  msgVisivel = (msgTxt[0] && millis() < msgAte);
  if (alarmeTemp) {
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawString("TEMP ALTA!", 110, 1, 2);
  } else if (msgVisivel) {
    tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    tft.drawString(msgTxt, 110, 1, 2);
  }

  tft.setTextColor(COR_ROT, TFT_BLACK);
  tft.drawString(paginaAtual == 0 ? "1/2" : "2/2", 268, 1, 2);
  tft.fillCircle(308, 9, 6, corLink(estadoLink));

  headerEstado = estadoLink;
  headerAlarme = alarmeTemp;
  headerForcar = false;
}

void atualizaHeader() {
  bool msgDeveriaEstar = (msgTxt[0] && millis() < msgAte);
  if (headerForcar || headerEstado != estadoLink || headerAlarme != alarmeTemp ||
      msgDeveriaEstar != msgVisivel) {
    desenhaHeader();
  }
}

void desenhaPagina() {
  tft.fillScreen(TFT_BLACK);
  desenhaHeader();

  if (paginaAtual == 0) {
    for (uint8_t i = 0; i < 10; i++) { limpaCache(p1[i]); desenhaRotulo(p1[i]); }
    tft.drawFastVLine(160, 18, 222, COR_GRADE);
    for (uint8_t r = 1; r < 5; r++) tft.drawFastHLine(0, 18 + r * 44 - 2, 320, COR_GRADE);
  } else {
    Celula* cels[] = {&p2Rpm, &p2Bat, &p2Vel, &p2Acel, &p2Iat, &p2Pico, &p2CargaV, &p2TempV};
    for (uint8_t i = 0; i < 8; i++) { limpaCache(*cels[i]); desenhaRotulo(*cels[i]); }
    tft.setTextDatum(TC_DATUM);
    tft.setTextPadding(0);
    tft.setTextColor(COR_ROT, TFT_BLACK);
    tft.drawString("CARGA", 29, 20, 2);
    tft.drawString("TEMP", 87, 20, 2);
    barCarga.ultAlt = -1; barTemp.ultAlt = -1;
    desenhaMoldura(barCarga);
    desenhaMoldura(barTemp);
  }
}

void mudaPagina(uint8_t n) {
  paginaAtual = n;
  desenhaPagina();
}

void desenhaStatus() {
  int8_t e = (int8_t)estadoLink;
  if (e == statusMostrado) return;
  statusMostrado = e;

  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(TL_DATUM);
  tft.setTextPadding(0);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.drawString("PAINEL PAJERO TR4", 10, 10, 4);

  const char* l1; const char* l2 = ""; uint16_t cor = TFT_ORANGE;
  switch (estadoLink) {
    case LINK_DESCONECTADO: l1 = "Bluetooth desconectado"; l2 = "Tentando novamente..."; cor = TFT_RED; break;
    case LINK_CONECTANDO:   l1 = "Conectando ao ELM327..."; break;
    case LINK_CONFIGURANDO: l1 = "Configurando adaptador..."; break;
    case LINK_SEM_ECU:      l1 = "Aguardando a ECU"; l2 = "Ligue a ignicao"; break;
    default:                l1 = "Pronto"; cor = TFT_GREEN; break;
  }
  tft.setTextColor(cor, TFT_BLACK);
  tft.drawString(l1, 10, 70, 4);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(l2, 10, 105, 4);

  char s[40];
  snprintf(s, sizeof(s), "Viagem salva: %.1f km", v.distKm);
  tft.setTextColor(COR_ROT, TFT_BLACK);
  tft.drawString(s, 10, 170, 2);
  tft.drawString("Toque para abrir os paineis", 10, 200, 2);
}

void desenhaPids() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(TL_DATUM);
  tft.setTextPadding(0);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.drawString("PIDs suportados", 10, 6, 4);

  struct ItemPid { const char* nome; QId id; };
  const ItemPid itens[] = {
    {"Velocidade (010D)", Q_VEL}, {"Rotacao (010C)", Q_RPM}, {"Carga motor (0104)", Q_CARGA},
    {"Temp. liquido (0105)", Q_TEMP}, {"Ar admissao (010F)", Q_IAT}, {"Acelerador (0111)", Q_ACEL},
  };
  int y = 40;
  for (uint8_t i = 0; i < 6; i++) {
    bool ok = consultas[itens[i].id].ativo;
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString(itens[i].nome, 10, y, 2);
    tft.setTextColor(ok ? TFT_GREEN : TFT_RED, TFT_BLACK);
    tft.drawString(ok ? "OK" : "NAO", 250, y, 2);
    y += 20;
  }
  const char* fm = consultas[Q_MAF].ativo ? "MAF (medido)"
                 : consultas[Q_MAP].ativo ? "MAP (estimado)" : "INDISPONIVEL";
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("Consumo", 10, y, 2);
  tft.setTextColor(combustivelDisponivel() ? TFT_GREEN : TFT_RED, TFT_BLACK);
  tft.drawString(fm, 170, y, 2);

  char s[40];
  snprintf(s, sizeof(s), "Viagem retomada: %.1f km", v.distKm);
  tft.setTextColor(COR_ROT, TFT_BLACK);
  tft.drawString(s, 10, 195, 2);
  tft.drawString("Toque para continuar", 10, 215, 2);
}

// ---- atualizacao das paginas ----
void atualizaPagina1() {
  char s[20];

  snprintf(s, sizeof(s), "%.1f", v.distKm);
  setCelula(p1[0], s, TFT_WHITE);

  fmtTempo(s, sizeof(s), v.tTotal);
  setCelula(p1[1], s, TFT_WHITE);

  fmtTempo(s, sizeof(s), v.tMov);
  setCelula(p1[2], s, TFT_WHITE);

  float vm = (v.tTotal > 10.0) ? (float)(v.distKm / (v.tTotal / 3600.0)) : 0.0f;
  snprintf(s, sizeof(s), "%.1f", vm);
  setCelula(p1[3], s, TFT_WHITE);

  fmtRitmo(s, sizeof(s), vm);
  setCelula(p1[4], s, TFT_WHITE);

  fmtRitmo(s, sizeof(s), velMediaMovel());
  setCelula(p1[5], s, TFT_WHITE);

  // consumo instantaneo / medio
  if (!combustivelDisponivel()) {
    snprintf(s, sizeof(s), "N/D");
    setCelula(p1[6], s, COR_ROT);
    setCelula(p1[7], s, COR_ROT);
    setCelula(p1[8], s, COR_ROT);
  } else {
    if (emaInit && lhEma > 0.3f && velEma >= VEL_MOVIMENTO_KMH) {
      float kml = velEma / lhEma;
      if (kml > 99.9f) kml = 99.9f;
      snprintf(s, sizeof(s), "%.1f", kml);
    } else {
      snprintf(s, sizeof(s), "--");
    }
    setCelula(p1[6], s, TFT_WHITE);

    if (v.litros > 0.3 && v.distKm > 0.1) snprintf(s, sizeof(s), "%.1f", (float)(v.distKm / v.litros));
    else snprintf(s, sizeof(s), "--");
    setCelula(p1[7], s, TFT_WHITE);

    snprintf(s, sizeof(s), "%.1f", (float)v.litros);
    setCelula(p1[8], s, corLitros((float)v.litros));
  }

  // carga media do trajeto
  if (v.tCarga > 5.0) {
    int pct = (int)lroundf((float)(v.cargaInt / v.tCarga));
    snprintf(s, sizeof(s), "%d", pct);
    setCelula(p1[9], s, corCarga(pct));
  } else {
    snprintf(s, sizeof(s), "--");
    setCelula(p1[9], s, TFT_WHITE);
  }
}

void atualizaPagina2() {
  char s[16];
  bool okRpm = fresco(d.tRpm);
  bool motor = okRpm && d.rpm > 300;

  // RPM (destaque)
  uint16_t cor = TFT_WHITE;
  if (okRpm) {
    snprintf(s, sizeof(s), "%d", (int)(d.rpm + 0.5f));
    if (d.rpm >= RPM_VERMELHO) cor = TFT_RED;
    else if (d.rpm >= RPM_AMARELO) cor = TFT_YELLOW;
  } else snprintf(s, sizeof(s), "--");
  setCelula(p2Rpm, s, cor);

  // Bateria (destaque)
  if (fresco(d.tVolt, 8000)) {
    snprintf(s, sizeof(s), "%.1f", d.volt);
    setCelula(p2Bat, s, corTensao(d.volt, motor));
  } else {
    snprintf(s, sizeof(s), "--");
    setCelula(p2Bat, s, TFT_WHITE);
  }

  // Barra de carga
  if (fresco(d.tCarga)) {
    int pct = (int)lroundf(d.carga);
    uint16_t c = corCarga(pct);
    atualizaBarra(barCarga, pct / 100.0f, c);
    snprintf(s, sizeof(s), "%d%%", pct);
    setCelula(p2CargaV, s, c);
  } else {
    atualizaBarra(barCarga, 0, TFT_DARKGREY);
    snprintf(s, sizeof(s), "--");
    setCelula(p2CargaV, s, TFT_WHITE);
  }

  // Barra de temperatura
  if (fresco(d.tTemp, 8000)) {
    uint16_t c = corTemp(d.temp);
    float frac = (d.temp - TEMP_MIN_BARRA) / (float)(TEMP_MAX_BARRA - TEMP_MIN_BARRA);
    atualizaBarra(barTemp, frac, c);
    snprintf(s, sizeof(s), "%d", (int)lroundf(d.temp));
    setCelula(p2TempV, s, c);
  } else {
    atualizaBarra(barTemp, 0, TFT_DARKGREY);
    snprintf(s, sizeof(s), "--");
    setCelula(p2TempV, s, TFT_WHITE);
  }

  // Secundarios
  if (fresco(d.tVel)) snprintf(s, sizeof(s), "%d", (int)(d.vel + 0.5f)); else snprintf(s, sizeof(s), "--");
  setCelula(p2Vel, s, TFT_WHITE);

  if (fresco(d.tAcel, 6000)) snprintf(s, sizeof(s), "%d", (int)lroundf(d.acel)); else snprintf(s, sizeof(s), "--");
  setCelula(p2Acel, s, TFT_WHITE);

  if (fresco(d.tIat, 15000)) snprintf(s, sizeof(s), "%d", (int)lroundf(d.iat)); else snprintf(s, sizeof(s), "--");
  setCelula(p2Iat, s, TFT_WHITE);

  if (picoTemp > -100) snprintf(s, sizeof(s), "%d", (int)lroundf(picoTemp)); else snprintf(s, sizeof(s), "--");
  setCelula(p2Pico, s, picoTemp > -100 ? corTemp(picoTemp) : TFT_WHITE);
}

// ---- alarme (temperatura) ----
void atualizaAlarme(uint32_t agora) {
  alarmeTemp = fresco(d.tTemp, 8000) && d.temp >= TEMP_VERMELHO;
  bool piscaOn = alarmeTemp && ((agora / 400) % 2 == 0);
  digitalWrite(LED_R, piscaOn ? LOW : HIGH);
}

// =====================================================================
//  TOQUE
// =====================================================================
void aoToqueCurto() {
  if (bootFase < 2) {
    bootFase = 2;
    mudaPagina(0);
    return;
  }
  mudaPagina((paginaAtual + 1) % 2);
}

void aoToqueLongo() {
  if (bootFase < 2) return;
  if (paginaAtual == 0) {
    resetViagem();
    setMsg("VIAGEM ZERADA");
  } else {
    picoTemp = -999;
    picoCarga = -1;
    limpaCache(p2Pico);
    setMsg("PICOS ZERADOS");
  }
}

void desenhaProgressoToque(float frac) {
  if (bootFase < 2) return;
  if (frac < 0) frac = 0;
  if (frac > 1) frac = 1;
  tft.fillRect(100, 0, 160, 18, TFT_BLACK);
  tft.drawRect(104, 2, 152, 14, TFT_ORANGE);
  tft.fillRect(106, 4, (int16_t)(148 * frac), 10, TFT_ORANGE);
}

void tratarToque(uint32_t agora) {
  static bool pressionado = false, longoFeito = false;
  static uint32_t tIni = 0, tUltToque = 0;

  bool t = ts.touched();
  if (t) tUltToque = agora;
  bool solto = !t && (agora - tUltToque > 80);   // debounce do touch resistivo

  if (t && !pressionado) {
    pressionado = true;
    longoFeito = false;
    tIni = agora;
  } else if (pressionado && !solto) {
    uint32_t dur = agora - tIni;
    if (!longoFeito) {
      if (dur >= TOQUE_LONGO_MS) {
        longoFeito = true;
        headerForcar = true;
        aoToqueLongo();
      } else if (dur > TOQUE_CURTO_MAX_MS) {
        desenhaProgressoToque((float)(dur - TOQUE_CURTO_MAX_MS) / (TOQUE_LONGO_MS - TOQUE_CURTO_MAX_MS));
      }
    }
  } else if (pressionado && solto) {
    uint32_t dur = tUltToque - tIni;
    pressionado = false;
    if (!longoFeito) {
      headerForcar = true;                       // apaga a barra de progresso
      if (dur <= TOQUE_CURTO_MAX_MS) aoToqueCurto();
    }
  }
}

// =====================================================================
//  SETUP / LOOP
// =====================================================================
void setup() {
  Serial.begin(115200);

  pinMode(LED_R, OUTPUT); pinMode(LED_G, OUTPUT); pinMode(LED_B, OUTPUT);
  digitalWrite(LED_R, HIGH); digitalWrite(LED_G, HIGH); digitalWrite(LED_B, HIGH);

  tft.init();
  tft.setRotation(1);
  tft.invertDisplay(true);
  tft.fillScreen(TFT_BLACK);

  touchSPI.begin(T_CLK, T_MISO, T_MOSI, T_CS);
  ts.begin(touchSPI);
  ts.setRotation(1);

  prefs.begin("painel", false);
  carregaViagem();
  tInteg = millis();
  tUltSalvar = millis();

  SerialBT.begin("ESP32_Painel", true);   // master
  xTaskCreatePinnedToCore(tarefaOBD, "obd", 8192, NULL, 1, NULL, 0);

  desenhaStatus();
}

void loop() {
  uint32_t agora = millis();
  tratarToque(agora);
  atualizaAlarme(agora);

  if (bootFase == 0) {
    desenhaStatus();
    if (estadoLink == LINK_ONLINE) {
      bootFase = 1;
      tBoot = agora;
      desenhaPids();
    }
    delay(20);
    return;
  }

  if (bootFase == 1) {
    if (agora - tBoot > 5000) { bootFase = 2; mudaPagina(0); }
    delay(20);
    return;
  }

  // ---- fase 2: operacao normal ----
  integraViagem(agora);

  if (agora - tUltSalvar >= SALVAR_A_CADA_MS) {
    tUltSalvar = agora;
    if (sujo) salvaViagem();
  }

  atualizaHeader();
  if (paginaAtual == 0) atualizaPagina1();
  else                  atualizaPagina2();

  delay(20);
}
