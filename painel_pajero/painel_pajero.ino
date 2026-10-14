/*
 * PAINEL OBD-II - PAJERO TR4 2007 (GASOLINA)
 * Placa: ESP32-2432S028R ("CYD") - display 2,8" 240x320 + touch resistivo
 * Adaptador: ELM327 Bluetooth Classic
 *
 * Paginas:
 *   0 = VIAGEM      (distancia, tempos, medias, ritmo, consumo, litros, carga media)
 *   1 = OFF-ROAD    (barras de carga e temperatura, RPM, bateria, mistura, extras)
 *
 * Toque curto : troca de pagina
 * Toque longo : (2 s) zera a viagem (pagina 0) ou os picos da sessao (pagina 1)
 *
 * Cores: todas as faixas usam degrade verde -> amarelo -> vermelho.
 *        Os pontos de cada degrade ficam nas constantes logo abaixo.
 *
 * Arquitetura:
 *   - Tarefa FreeRTOS "obd" (core 0): Bluetooth, comandos AT, consultas PID.
 *   - loop() (core 1): touch, calculos da viagem, desenho, gravacao na NVS,
 *     registro CSV no cartao SD.
 *   Assim, uma reconexao Bluetooth lenta nunca congela a tela.
 *
 * Registro CSV (cartao microSD, FAT32): /viagem_0001.csv, 0002... (1 linha/s).
 *   Indicador "SD" no cabecalho: verde = gravando, vermelho = sem cartao.
 *   Para analisar: tire o cartao e envie os .csv.
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
#include <FS.h>
#include <SD.h>
#include <math.h>
#include "tipos.h"   // tipos em arquivo separado (evita bug dos prototipos automaticos da Arduino IDE)
#include "config_pessoal.h"   // SEUS valores: MAC do ELM327 e calibracao do consumo

// =====================================================================
//  CONFIGURACAO (ajuste aqui)
// =====================================================================

// MAC do adaptador ELM327, AFR, densidade e fator de consumo: veja config_pessoal.h

// --- Tanque -----------------------------------------------------------
// Degrade dos litros usados: verde ate 50%, amarelo em 70%, vermelho a partir de 85%
const float TANQUE_L               = 53.0f;
const float TANQUE_VERDE_ATE       = 0.50f;   // ~26,5 L
const float TANQUE_ALERTA_AMARELO  = 0.70f;   // ~37 L
const float TANQUE_ALERTA_VERMELHO = 0.85f;   // ~45 L (sobram ~8 L)

// --- Estimativa de consumo (so usada se o carro NAO tiver MAF, PID 0110)
// Metodo "speed-density": calcula o ar pela pressao do coletor (MAP).
const float MOTOR_CILINDRADA_L  = 2.0f;    // litros
const float EFIC_VOLUMETRICA    = 0.80f;   // chute inicial; calibre pelo consumo real
// AFR, DENSIDADE_G_L e FATOR_CALIB_CONSUMO: veja config_pessoal.h

// --- Consumo (km/L): verde acima de 7,5 / amarelo em 6,5 / vermelho a 5,5 ou menos
const float CONSUMO_VERDE    = 7.5f;
const float CONSUMO_AMARELO  = 6.5f;
const float CONSUMO_VERMELHO = 5.5f;

// --- Carga do motor (%) ---
const float CARGA_VERDE    = 70.0f;   // ate aqui: verde
const float CARGA_AMARELO  = 77.5f;   // amarelo puro
const float CARGA_VERMELHO = 85.0f;   // a partir daqui: vermelho

// --- Temperatura do liquido (C) ---
const float TEMP_FRIO     = 70.0f;    // abaixo: azul (motor ainda aquecendo)
const float TEMP_VERDE    = 95.0f;    // ate aqui: verde
const float TEMP_AMARELO  = 102.0f;   // amarelo puro
const float TEMP_VERMELHO = 108.0f;   // a partir daqui: vermelho
const float TEMP_ALARME   = 108.0f;   // "TEMP ALTA!" no topo + LED piscando
const float TEMP_MIN_BARRA = 40.0f;   // escala da barra
const float TEMP_MAX_BARRA = 120.0f;

// --- RPM ---
const float RPM_VERDE    = 3500.0f;
const float RPM_AMARELO  = 4500.0f;
const float RPM_VERMELHO = 5500.0f;

// --- Ajuste de mistura (soma curto + longo prazo, em %, valor absoluto) ---
const float TRIM_VERDE    = 5.0f;
const float TRIM_AMARELO  = 10.0f;
const float TRIM_VERMELHO = 15.0f;

// --- Suavizacao da pagina 2 (constante de tempo, em segundos; maior = mais suave/lento) ---
const float TAU_UI_RAPIDO_S = 0.30f;   // RPM, velocidade, carga, acelerador
const float TAU_UI_LENTO_S  = 0.80f;   // temperaturas, bateria, mistura

// --- Viagem ---
const float VEL_MOVIMENTO_KMH = 3.0f;       // acima disso conta como "em movimento"
const uint32_t SALVAR_A_CADA_MS = 30000;    // NVS
const uint32_t TOQUE_CURTO_MAX_MS = 700;
const uint32_t TOQUE_LONGO_MS     = 2000;
const float TAU_CONSUMO_S = 10.0f;          // suavizacao do consumo instantaneo
const uint8_t JANELA_RITMO_S = 30;          // media movel do ritmo (segundos)

// --- Registro em cartao SD (CSV) ---
const bool     LOG_ATIVO_CONFIG = true;     // false = nunca mexe no cartao
const uint32_t LOG_INTERVALO_MS = 1000;     // uma linha por segundo (o ELM327 nao entrega mais rapido)
const uint32_t LOG_FLUSH_MS     = 5000;     // grava no cartao a cada 5 s (se desligar do nada, perde no maximo isso)
const uint32_t LOG_RETENTAR_MS  = 30000;    // sem cartao: tenta montar de novo a cada 30 s

// --- Bluetooth: conexao robusta ---
const uint8_t  BT_TENTATIVAS_ATE_REINICIAR   = 4;      // falhas seguidas ate o painel reiniciar sozinho
const uint8_t  BT_MAX_REINICIOS_SEGUIDOS     = 5;      // depois disso so continua tentando (sem reiniciar)
const uint32_t BT_ESPERA_ENTRE_TENTATIVAS_MS = 2000;
const uint32_t BT_ESPERA_SEM_REINICIO_MS     = 15000;
const uint8_t  ELM_SEM_PROMPT_MAX            = 5;      // consultas mudas seguidas ate reconectar

// =====================================================================
//  HARDWARE
// =====================================================================
#define T_CLK  25
#define T_MISO 39
#define T_MOSI 32
#define T_CS   33
#define SD_CS   5      // slot microSD da placa
#define SD_SCK 18
#define SD_MISO 19
#define SD_MOSI 23
#define LED_R   4      // LED RGB da placa (logica invertida: LOW = acende)
#define LED_G  16
#define LED_B  17

TFT_eSPI tft = TFT_eSPI();
// O touch e o cartao SD usam o MESMO barramento (VSPI), cada um com seus pinos.
// Antes de falar com um deles, o codigo religa o barramento nos pinos certos
// (usaBusTouch / usaBusSD). Tudo roda no loop(), entao nunca ha acesso simultaneo.
SPIClass barramentoSPI(VSPI);
XPT2046_Touchscreen ts(T_CS);
BluetoothSerial SerialBT;
Preferences prefs;

const uint8_t BUS_NENHUM = 0, BUS_TOUCH = 1, BUS_SD = 2;
uint8_t busAtual = BUS_NENHUM;

void usaBusTouch() {
  if (busAtual == BUS_TOUCH) return;
  barramentoSPI.end();
  barramentoSPI.begin(T_CLK, T_MISO, T_MOSI, T_CS);
  busAtual = BUS_TOUCH;
}

void usaBusSD() {
  if (busAtual == BUS_SD) return;
  barramentoSPI.end();
  barramentoSPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  busAtual = BUS_SD;
}

// Cores
const uint16_t COR_ROT   = 0x8410;  // cinza dos rotulos
const uint16_t COR_GRADE = 0x2104;  // cinza escuro das linhas
const uint16_t COR_AZUL  = 0x04FF;  // azul claro (motor frio)

// =====================================================================
//  ESTADO COMPARTILHADO ENTRE AS TAREFAS
// =====================================================================
volatile EstadoLink estadoLink = LINK_DESCONECTADO;
volatile uint8_t paginaAtual = 0;
volatile uint8_t tentativaBT = 0;        // numero da tentativa de conexao (mostrado na tela)
volatile bool pedidoReinicio = false;    // a tarefa OBD pede; o loop() salva a viagem e reinicia
volatile bool podeReiniciar = true;      // false depois de muitos reinicios seguidos
volatile bool semPromptUltimo = false;   // true = o ELM327 nem devolveu o ">" na ultima consulta
volatile bool logPronto = false;         // true = cartao SD montado e registrando (aceleracao e mistura passam a ser lidas sempre)

void setMsg(const char* t);              // definida mais abaixo (mensagem temporaria no cabecalho)

struct Dados {
  volatile float vel = 0, rpm = 0, carga = 0, temp = 0, acel = 0, iat = 0;
  volatile float mapKpa = 0, maf = 0, volt = 0, stft = 0, ltft = 0;
  // millis() da ultima leitura valida (0 = nunca)
  volatile uint32_t tVel = 0, tRpm = 0, tCarga = 0, tTemp = 0, tAcel = 0, tIat = 0;
  volatile uint32_t tMap = 0, tMaf = 0, tVolt = 0, tStft = 0, tLtft = 0;
} d;

inline bool fresco(uint32_t t, uint32_t idade = 4000) {
  return t != 0 && (millis() - t) < idade;
}

// =====================================================================
//  OBD / ELM327  (roda na tarefa "obd")
// =====================================================================

// A ORDEM deve ser a mesma do enum QId (tipos.h)
Consulta consultas[Q_N] = {
  /* Q_VEL   */ {"010D", 0x0D,  500, false, true, 0, 0},
  /* Q_RPM   */ {"010C", 0x0C,  500, false, true, 0, 0},
  /* Q_CARGA */ {"0104", 0x04,  800, false, true, 0, 0},
  /* Q_MAF   */ {"0110", 0x10,  800, false, true, 0, 0},
  /* Q_MAP   */ {"010B", 0x0B,  800, false, true, 0, 0},
  /* Q_TEMP  */ {"0105", 0x05, 2500, false, true, 0, 0},
  /* Q_IAT   */ {"010F", 0x0F, 5000, false, true, 0, 0},
  /* Q_ACEL  */ {"0111", 0x11, 1000, true,  true, 0, 0},
  /* Q_STFT  */ {"0106", 0x06, 3000, true,  true, 0, 0},
  /* Q_LTFT  */ {"0107", 0x07, 3000, true,  true, 0, 0},
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
      if (c == '>') { buf[n] = 0; semPromptUltimo = false; return true; }
      if (c == ' ' || c == '\n') continue;
      if (n < tam - 1) buf[n++] = c;
    }
    espera(1);
  }
  buf[n] = 0;
  semPromptUltimo = true;
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
// Retorno:  1 = ok
//           0 = o ELM327 respondeu, mas a ECU nao (ignicao desligada?)
//          -1 = o ELM327 nem respondeu (travado ou fora de alcance)
int8_t descobrePids() {
  uint8_t b[4];
  // O primeiro comando dispara a busca de protocolo (ISO 9141/KWP pode levar ~10 s)
  if (!enviaEEspera("0100", resp, sizeof(resp), 30000)) return -1;
  if (!extrai(resp, 0x00, b, 4)) return 0;

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
  return 1;
}

// Espera a ECU responder (ignicao ligada).
// false = o link caiu ou o ELM327 travou (precisa reconectar).
bool aguardaEcu() {
  estadoLink = LINK_SEM_ECU;
  uint8_t semPrompt = 0;
  while (SerialBT.connected()) {
    int8_t r = descobrePids();
    if (r == 1) return true;
    if (r < 0) { if (++semPrompt >= 2) return false; }
    else       { semPrompt = 0; }
    espera(2000);
  }
  return false;
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

bool configCompleta = true;   // true = precisa de ATZ + descoberta de PIDs (1a vez, ou apos travamento)

// Comando AT com ate 3 tentativas
bool enviaAT(const char* cmd, uint32_t timeoutMs) {
  for (uint8_t t = 0; t < 3; t++) {
    if (enviaEEspera(cmd, resp, sizeof(resp), timeoutMs)) return true;
    if (!SerialBT.connected()) return false;
    espera(300);
  }
  return false;
}

// Configura o ELM327. Na reconexao (configCompleta == false) pula o ATZ e a
// descoberta de PIDs: o ELM327 mantem o protocolo e a lista de PIDs ja e conhecida,
// entao a volta dos dados leva segundos em vez de dezenas de segundos.
bool configuraELM() {
  estadoLink = LINK_CONFIGURANDO;
  if (configCompleta) {
    if (!enviaAT("ATZ", 5000)) return false;
    espera(200);
  }
  const char* cmds[] = {"ATE0", "ATL0", "ATS0", "ATH0"};
  for (uint8_t i = 0; i < 4; i++) {
    if (!enviaAT(cmds[i], 2500)) return false;
  }

  if (configCompleta) {
    if (!enviaAT("ATSP0", 2500)) return false;
    if (!aguardaEcu()) return false;
    aplicaSuporte();
    configCompleta = false;
    estadoLink = LINK_ONLINE;
  } else {
    aplicaSuporte();   // zera os temporizadores; o estado vira ONLINE na 1a leitura valida
  }
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
    if (q.soPagina2 && paginaAtual != 1 && !logPronto) continue;   // registrando: le tudo sempre
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
    case Q_STFT:   // ajuste de mistura curto prazo: (A-128)*100/128 %
      if (!extrai(resp, q.pid, b, 1)) return false;
      d.stft = ((float)b[0] - 128.0f) * 100.0f / 128.0f; d.tStft = t; return true;
    case Q_LTFT:   // ajuste de mistura longo prazo
      if (!extrai(resp, q.pid, b, 1)) return false;
      d.ltft = ((float)b[0] - 128.0f) * 100.0f / 128.0f; d.tLtft = t; return true;
    default: return false;
  }
}

// Roda enquanto o link estiver bom. Retorna quando o Bluetooth cai ou quando o
// ELM327 para de responder (nesse caso marca configCompleta = true).
void cicloConsultas() {
  uint8_t falhasSeguidas = 0;
  uint8_t semPromptSeguidos = 0;
  uint32_t tUltimoOk = millis();

  while (SerialBT.connected()) {
    int i = proximaConsulta();
    if (i < 0) { espera(5); continue; }

    consultas[i].ultima = millis();
    if (executaConsulta((QId)i)) {
      consultas[i].falhas = 0;
      falhasSeguidas = 0;
      semPromptSeguidos = 0;
      tUltimoOk = millis();
      estadoLink = LINK_ONLINE;
    } else {
      consultas[i].falhas++;
      falhasSeguidas++;

      // ELM327 mudo (nem o '>' volta): travou ou saiu do alcance
      if (semPromptUltimo) semPromptSeguidos++; else semPromptSeguidos = 0;
      if (semPromptSeguidos >= ELM_SEM_PROMPT_MAX) {
        Serial.println("ELM327 sem resposta: reconectando");
        configCompleta = true;
        return;
      }

      // PID que falha sempre enquanto os outros funcionam = nao suportado de fato
      if (consultas[i].falhas >= 8 && (millis() - tUltimoOk) < 3000) {
        consultas[i].ativo = false;
        Serial.printf("Consulta %s desativada (sem resposta)\n", consultas[i].cmd);
      }

      // Tudo falhando, mas o ELM327 responde: ignicao desligada, ECU fora, cabo solto...
      if (falhasSeguidas >= 12) {
        if (!aguardaEcu()) { configCompleta = true; return; }
        aplicaSuporte();
        falhasSeguidas = 0;
        semPromptSeguidos = 0;
        tUltimoOk = millis();
        estadoLink = LINK_ONLINE;
      }
    }
  }
}

// Maneiras de abrir a conexao, tentadas em rodizio quando uma falha.
// (canal 0 = descobrir o canal por busca de servico; papel 0 = master, 1 = slave)
// A 1a tenta o canal serial 1 direto, que costuma ser o dos ELM327 e dispensa a
// busca de servico (etapa lenta e que falha com clones).
const EstrategiaBT ESTRATEGIAS[] = {
  {1, 0},
  {0, 1},
  {1, 1},
  {0, 0},
};
const uint8_t N_ESTRATEGIAS = sizeof(ESTRATEGIAS) / sizeof(ESTRATEGIAS[0]);

bool conectaBT(uint32_t tentativa) {
  const EstrategiaBT& e = ESTRATEGIAS[tentativa % N_ESTRATEGIAS];
  Serial.printf("BT tentativa %u: canal %u, papel %s\n",
                (unsigned)(tentativa + 1), (unsigned)e.canal, e.papel ? "slave" : "master");
  SerialBT.disconnect();     // limpa qualquer estado da tentativa anterior
  espera(300);
  bool ok = SerialBT.connect(mac_elm, e.canal, ESP_SPP_SEC_NONE, (esp_spp_role_t)e.papel);
  if (ok) ok = SerialBT.connected();
  Serial.println(ok ? "BT conectado" : "BT falhou");
  return ok;
}

void tarefaOBD(void* arg) {
  espera(1500);                     // deixa o ELM327 terminar de ligar
  uint16_t falhas = 0;
  uint32_t tentativaTotal = 0;

  for (;;) {
    bool ok = SerialBT.connected();
    if (!ok) {
      estadoLink = LINK_CONECTANDO;
      tentativaBT = (falhas + 1 > 99) ? 99 : (uint8_t)(falhas + 1);
      ok = conectaBT(tentativaTotal++);
    }
    if (ok) ok = configuraELM();

    if (ok) {
      falhas = 0;
      tentativaBT = 0;
      cicloConsultas();             // so volta quando o link cai ou o ELM327 trava
      estadoLink = LINK_DESCONECTADO;
      SerialBT.disconnect();
      espera(500);
      continue;                     // reconecta na hora, em modo rapido
    }

    // Falhou: limpa e tenta de novo (ou pede reinicio do painel)
    SerialBT.disconnect();
    configCompleta = true;
    estadoLink = LINK_DESCONECTADO;
    falhas++;
    if (podeReiniciar && falhas >= BT_TENTATIVAS_ATE_REINICIAR) {
      pedidoReinicio = true;        // o loop() salva a viagem e reinicia
      espera(60000);
    }
    espera(podeReiniciar ? BT_ESPERA_ENTRE_TENTATIVAS_MS : BT_ESPERA_SEM_REINICIO_MS);
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
//  REGISTRO EM CARTAO SD (CSV) - roda no loop()
// =====================================================================
// Um arquivo por partida a frio: /viagem_0001.csv, /viagem_0002.csv ...
// (o reinicio automatico para reconectar o Bluetooth continua no MESMO arquivo;
//  a coluna "boot" diz a qual partida cada linha pertence).
// Uma linha por segundo com o motor ligado. Campo vazio = sem leitura recente.
// Linhas de evento (coluna "evento"): RESET_VIAGEM, RESET_PICOS, LINK_PERDIDO,
// LINK_OK, REINICIO_AUTO, REINICIO_MANUAL.
const char LOG_CABECALHO[] =
  "boot,t_ms,vel_kmh,rpm,carga_pct,tps_pct,maf_gs,temp_c,iat_c,"
  "stft_pct,ltft_pct,bat_v,lh_calc,km_viagem,litros_calc,evento\n";

File logArq;
bool sdMontado = false;
bool logAberto = false;
uint32_t bootN = 0;          // numero da partida (aumenta a cada reinicio do painel)
uint32_t logSeq = 0;         // numero do arquivo
char logNome[24] = "";
char logBuf[1800];           // linhas aguardando gravacao
size_t logBufN = 0;
uint32_t tLogUlt = 0, tLogFlush = 0, tLogTentou = 0;
bool linkOkAnterior = false;

// 0 = registro desligado, 1 = cartao ok, 2 = sem cartao / erro
uint8_t estadoSd() {
  if (!LOG_ATIVO_CONFIG) return 0;
  return logPronto ? 1 : 2;
}

bool logMonta() {
  if (sdMontado) return true;
  usaBusSD();
  sdMontado = SD.begin(SD_CS, barramentoSPI, 4000000);
  busAtual = BUS_SD;     // SD.begin() pode religar o barramento; os pinos padrao do VSPI sao os do cartao
  if (sdMontado && SD.cardType() == CARD_NONE) { SD.end(); sdMontado = false; }
  Serial.println(sdMontado ? "Cartao SD montado" : "Cartao SD nao encontrado");
  return sdMontado;
}

// Cartao retirado, cheio ou com defeito: fecha tudo e tenta de novo mais tarde.
void logFalha(const char* motivo) {
  Serial.printf("Registro SD: %s\n", motivo);
  if (logAberto) { logArq.close(); logAberto = false; }
  if (sdMontado) { SD.end(); sdMontado = false; }
  logPronto = false;
  logBufN = 0;
  tLogTentou = millis();
  setMsg("SD: ERRO");
}

bool logAbreArquivo() {
  if (logAberto) return true;
  if (!logPronto) return false;
  usaBusSD();
  logArq = SD.open(logNome, FILE_APPEND);
  if (!logArq) { logFalha("nao abriu o arquivo"); return false; }
  if (logArq.size() == 0) logArq.print(LOG_CABECALHO);
  logAberto = true;
  Serial.printf("Registro SD: gravando em %s\n", logNome);
  return true;
}

// Grava o que esta na memoria no cartao.
void logDescarrega() {
  if (!logAberto || logBufN == 0) return;
  usaBusSD();
  size_t n = logArq.write((const uint8_t*)logBuf, logBufN);
  logArq.flush();
  if (n != logBufN) { logFalha("falha ao gravar"); return; }
  logBufN = 0;
  tLogFlush = millis();
}

// Acrescenta ",valor" (ou "," vazio) a linha em montagem.
static void addNum(char*& p, char* fim, bool ok, const char* fmt, float val) {
  if (p < fim) *p++ = ',';
  if (ok && p < fim) {
    int n = snprintf(p, fim - p, fmt, (double)val);
    if (n > 0) p += n;
    if (p > fim) p = fim;
  }
}

void logLinha(uint32_t agora, const char* evento) {
  if (!logAbreArquivo()) return;
  if (sizeof(logBuf) - logBufN < 260) {
    logDescarrega();
    if (!logAberto) return;
  }
  char* p = logBuf + logBufN;
  char* fim = logBuf + sizeof(logBuf) - 2;

  p += snprintf(p, fim - p, "%u,%lu", (unsigned)bootN, (unsigned long)agora);
  addNum(p, fim, fresco(d.tVel),                                 "%.0f", d.vel);
  addNum(p, fim, fresco(d.tRpm),                                 "%.0f", d.rpm);
  addNum(p, fim, fresco(d.tCarga),                               "%.1f", d.carga);
  addNum(p, fim, fresco(d.tAcel, 6000),                          "%.1f", d.acel);
  addNum(p, fim, consultas[Q_MAF].ativo && fresco(d.tMaf),       "%.2f", d.maf);
  addNum(p, fim, fresco(d.tTemp, 8000),                          "%.0f", d.temp);
  addNum(p, fim, fresco(d.tIat, 15000),                          "%.0f", d.iat);
  addNum(p, fim, fresco(d.tStft, 10000),                         "%.1f", d.stft);
  addNum(p, fim, fresco(d.tLtft, 10000),                         "%.1f", d.ltft);
  addNum(p, fim, fresco(d.tVolt, 8000),                          "%.1f", d.volt);
  float lh = fluxoLh();
  addNum(p, fim, !isnan(lh),                                     "%.2f", lh);
  addNum(p, fim, true,                                           "%.3f", (float)v.distKm);
  addNum(p, fim, true,                                           "%.3f", (float)v.litros);
  if (p < fim) *p++ = ',';
  while (evento && *evento && p < fim) *p++ = *evento++;
  *p++ = '\n';
  logBufN = p - logBuf;
}

// Evento marcado na hora (grava e descarrega no cartao imediatamente).
void registraEvento(const char* ev) {
  if (!LOG_ATIVO_CONFIG || !logPronto) return;
  logLinha(millis(), ev);
  logDescarrega();
}

void atualizaRegistro(uint32_t agora) {
  if (!LOG_ATIVO_CONFIG) return;

  if (!logPronto) {                       // sem cartao: tenta de novo de tempos em tempos
    if (agora - tLogTentou >= LOG_RETENTAR_MS) {
      tLogTentou = agora;
      if (logMonta()) { logPronto = true; setMsg("SD: OK"); }
    }
    return;
  }

  // perda e volta do link ficam marcadas (so durante uma viagem ja registrada)
  bool online = (estadoLink == LINK_ONLINE);
  if (online != linkOkAnterior) {
    linkOkAnterior = online;
    if (logAberto) logLinha(agora, online ? "LINK_OK" : "LINK_PERDIDO");
  }

  bool motor = fresco(d.tRpm) && d.rpm > 300;
  if (motor && agora - tLogUlt >= LOG_INTERVALO_MS) {
    tLogUlt = agora;
    logLinha(agora, "");
  }
  if (logAberto && logBufN > 0 && agora - tLogFlush >= LOG_FLUSH_MS) logDescarrega();
}

// =====================================================================
//  CORES (degrade verde -> amarelo -> vermelho)
// =====================================================================

// x0 = valor em que a cor ainda e verde, x1 = valor do amarelo puro,
// x2 = valor em que fica vermelho. Funciona nos dois sentidos
// (x0 < x1 < x2 para "quanto maior pior"; x0 > x1 > x2 para "quanto menor pior").
uint16_t corGradiente(float val, float x0, float x1, float x2) {
  float r, g;
  float u = (val - x0) / (x1 - x0);
  if (u <= 0.0f) {
    r = 0; g = 255;
  } else if (u < 1.0f) {
    r = 255.0f * u; g = 255;
  } else {
    float w = (val - x1) / (x2 - x1);
    if (w >= 1.0f) { r = 255; g = 0; }
    else           { r = 255; g = 255.0f * (1.0f - w); }
  }
  return tft.color565((uint8_t)r, (uint8_t)g, 0);
}

uint16_t corCarga(float pct) { return corGradiente(pct, CARGA_VERDE, CARGA_AMARELO, CARGA_VERMELHO); }

uint16_t corTemp(float t) {
  if (t < TEMP_FRIO) return COR_AZUL;
  return corGradiente(t, TEMP_VERDE, TEMP_AMARELO, TEMP_VERMELHO);
}

uint16_t corRpm(float rpm) { return corGradiente(rpm, RPM_VERDE, RPM_AMARELO, RPM_VERMELHO); }

uint16_t corConsumo(float kml) { return corGradiente(kml, CONSUMO_VERDE, CONSUMO_AMARELO, CONSUMO_VERMELHO); }

uint16_t corTrim(float somaPct) {
  return corGradiente(fabsf(somaPct), TRIM_VERDE, TRIM_AMARELO, TRIM_VERMELHO);
}

uint16_t corLitros(float litros) {
  return corGradiente(litros, TANQUE_L * TANQUE_VERDE_ATE,
                      TANQUE_L * TANQUE_ALERTA_AMARELO, TANQUE_L * TANQUE_ALERTA_VERMELHO);
}

// Tensao: faixa boa no meio; degrade para os dois lados (baixa e alta demais)
uint16_t corTensao(float volt, bool motorLigado) {
  if (motorLigado) {
    if (volt < 14.0f) return corGradiente(volt, 13.2f, 12.7f, 12.2f);   // alternador fraco
    return corGradiente(volt, 14.8f, 15.1f, 15.5f);                     // sobretensao
  }
  return corGradiente(volt, 12.5f, 12.1f, 11.7f);                       // bateria descarregando
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
  {  4,  20, 152, 16, 4, "DISTANCIA km",     2, "", 0, 0},   // 0
  {164,  20, 152, 16, 4, "TEMPO TOTAL",      2, "", 0, 0},   // 1
  {  4,  64, 152, 16, 4, "VEL MEDIA km/h",   2, "", 0, 0},   // 2
  {164,  64, 152, 16, 4, "TEMPO MOVIM.",     2, "", 0, 0},   // 3
  {  4, 108, 152, 16, 4, "RITMO MED /10km",  2, "", 0, 0},   // 4
  {164, 108, 152, 16, 4, "RITMO INS /10km",  2, "", 0, 0},   // 5
  {  4, 152, 152, 16, 4, "CONSUMO MED km/L", 2, "", 0, 0},   // 6
  {164, 152, 152, 16, 4, "CONSUMO INS km/L", 2, "", 0, 0},   // 7
  {  4, 196, 152, 16, 4, "LITROS USADOS",    2, "", 0, 0},   // 8
  {164, 196, 152, 16, 4, "CARGA MEDIA %",    2, "", 0, 0},   // 9
};

// ---- Pagina 2 (off-road): barras a esquerda + grade 2x4 de dados iguais ----
Celula p2Rpm   = {120,  20, 96, 18, 4, "RPM",         2, "", 0, 0};
Celula p2Bat   = {220,  20, 96, 18, 4, "BATERIA V",   2, "", 0, 0};
Celula p2Vel   = {120,  75, 96, 18, 4, "VELOC km/h",  2, "", 0, 0};
Celula p2Acel  = {220,  75, 96, 18, 4, "ACELER. %",   2, "", 0, 0};
Celula p2Iat   = {120, 130, 96, 18, 4, "AR ADM. C",   2, "", 0, 0};
Celula p2Pico  = {220, 130, 96, 18, 4, "PICO TEMP",   2, "", 0, 0};
Celula p2Trim  = {120, 185, 96, 18, 4, "MISTURA %",   2, "", 0, 0};
Celula p2TrimD = {220, 185, 96, 20, 2, "CURTO/LONGO", 2, "", 0, 0};
// valores sob as barras (centralizados)
Celula p2CargaV = { 29, 204, 54, 0, 4, nullptr, 2, "", 0, TC_DATUM};
Celula p2TempV  = { 87, 204, 54, 0, 4, nullptr, 2, "", 0, TC_DATUM};

Barra barCarga = {  2, 38, 54, 160, 0.0f, 100.0f, corCarga, 0};
Barra barTemp  = { 60, 38, 54, 160, TEMP_MIN_BARRA, TEMP_MAX_BARRA, corTemp, 0};

void desenhaMoldura(const Barra& b) {
  tft.drawRect(b.x, b.y, b.w, b.h, COR_ROT);
}

// Barra com degrade fixo: cada linha tem a cor do valor que representa.
// Desenha/apaga so as linhas que mudaram (sem piscar).
void atualizaBarra(Barra& b, float frac) {
  if (frac < 0) frac = 0;
  if (frac > 1) frac = 1;
  int16_t ih  = b.h - 2;
  int16_t alt = (int16_t)(frac * ih + 0.5f);
  if (alt == b.ultAlt) return;

  int16_t yBase = b.y + 1 + ih;     // linha logo abaixo da ultima linha util
  if (alt > b.ultAlt) {
    for (int16_t i = b.ultAlt; i < alt; i++) {
      float val = b.vmin + ((float)i + 0.5f) / (float)ih * (b.vmax - b.vmin);
      tft.drawFastHLine(b.x + 1, yBase - 1 - i, b.w - 2, b.corFn(val));
    }
  } else {
    tft.fillRect(b.x + 1, yBase - b.ultAlt, b.w - 2, b.ultAlt - alt, TFT_BLACK);
  }
  b.ultAlt = alt;
}

// ---- suavizacao ----
Suave sRpm, sBat, sVel, sAcel, sIat, sStft, sLtft, sCarga, sTemp;

float suaviza(Suave& s, float alvo, float dt, float tau) {
  if (!s.ok) { s.v = alvo; s.ok = true; }
  else       { s.v += (alvo - s.v) * (1.0f - expf(-dt / tau)); }
  return s.v;
}

void resetSuaves() {
  Suave* todos[] = {&sRpm, &sBat, &sVel, &sAcel, &sIat, &sStft, &sLtft, &sCarga, &sTemp};
  for (uint8_t i = 0; i < 9; i++) todos[i]->ok = false;
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
uint8_t headerSd = 255;
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
  } else if (bootFase >= 2 && estadoLink != LINK_ONLINE) {
    tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    tft.drawString(estadoLink == LINK_SEM_ECU ? "SEM ECU" : "SEM LINK", 110, 1, 2);
  }

  uint8_t sd = estadoSd();                 // verde = gravando no cartao, vermelho = sem cartao
  if (sd) {
    tft.setTextColor(sd == 1 ? TFT_GREEN : TFT_RED, TFT_BLACK);
    tft.drawString("SD", 236, 1, 2);
  }

  tft.setTextColor(COR_ROT, TFT_BLACK);
  tft.drawString(paginaAtual == 0 ? "1/2" : "2/2", 268, 1, 2);
  tft.fillCircle(308, 9, 6, corLink(estadoLink));

  headerEstado = estadoLink;
  headerAlarme = alarmeTemp;
  headerSd = sd;
  headerForcar = false;
}

void atualizaHeader() {
  bool msgDeveriaEstar = (msgTxt[0] && millis() < msgAte);
  if (headerForcar || headerEstado != estadoLink || headerAlarme != alarmeTemp ||
      headerSd != estadoSd() || msgDeveriaEstar != msgVisivel) {
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
    Celula* cels[] = {&p2Rpm, &p2Bat, &p2Vel, &p2Acel, &p2Iat, &p2Pico,
                      &p2Trim, &p2TrimD, &p2CargaV, &p2TempV};
    for (uint8_t i = 0; i < 10; i++) { limpaCache(*cels[i]); desenhaRotulo(*cels[i]); }
    tft.setTextDatum(TC_DATUM);
    tft.setTextPadding(0);
    tft.setTextColor(COR_ROT, TFT_BLACK);
    tft.drawString("CARGA", 29, 20, 2);
    tft.drawString("TEMP", 87, 20, 2);
    barCarga.ultAlt = 0; barTemp.ultAlt = 0;     // tela acabou de ser apagada
    desenhaMoldura(barCarga);
    desenhaMoldura(barTemp);
    resetSuaves();
  }
}

void mudaPagina(uint8_t n) {
  paginaAtual = n;
  desenhaPagina();
}

int8_t tentativaMostrada = -1;

void desenhaStatus() {
  int8_t e = (int8_t)estadoLink;
  int8_t tent = (int8_t)tentativaBT;
  if (e == statusMostrado && tent == tentativaMostrada) return;
  statusMostrado = e;
  tentativaMostrada = tent;

  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(TL_DATUM);
  tft.setTextPadding(0);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.drawString("PAINEL PAJERO TR4", 10, 10, 4);

  char tentTxt[32] = "";
  if (tentativaBT > 0) {
    if (podeReiniciar) snprintf(tentTxt, sizeof(tentTxt), "Tentativa %u de %u",
                                (unsigned)tentativaBT, (unsigned)BT_TENTATIVAS_ATE_REINICIAR);
    else               snprintf(tentTxt, sizeof(tentTxt), "Tentativa %u", (unsigned)tentativaBT);
  }
  const char* l1; const char* l2 = ""; uint16_t cor = TFT_ORANGE;
  switch (estadoLink) {
    case LINK_DESCONECTADO: l1 = "Sem conexao Bluetooth"; l2 = tentTxt[0] ? tentTxt : "Tentando novamente..."; cor = TFT_RED; break;
    case LINK_CONECTANDO:   l1 = "Conectando ao ELM327..."; l2 = tentTxt; break;
    case LINK_CONFIGURANDO: l1 = "Configurando adaptador..."; break;
    case LINK_SEM_ECU:      l1 = "Aguardando a ECU"; l2 = "Ligue a ignicao"; break;
    default:                l1 = "Pronto"; cor = TFT_GREEN; break;
  }
  tft.setTextColor(cor, TFT_BLACK);
  tft.drawString(l1, 10, 70, 4);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(l2, 10, 105, 4);

  uint8_t sd = estadoSd();
  tft.setTextColor(sd == 1 ? TFT_GREEN : (sd == 2 ? TFT_RED : COR_ROT), TFT_BLACK);
  tft.drawString(sd == 1 ? "Cartao SD: OK (registrando)"
               : sd == 2 ? "Cartao SD: nao encontrado"
                         : "Registro em cartao: desligado", 10, 145, 2);

  char s[40];
  snprintf(s, sizeof(s), "Viagem salva: %.1f km", v.distKm);
  tft.setTextColor(COR_ROT, TFT_BLACK);
  tft.drawString(s, 10, 170, 2);
  tft.drawString("Toque para abrir os paineis", 10, 200, 2);
  tft.drawString("Toque 2 s: reiniciar e reconectar", 10, 218, 2);
}

void desenhaPids() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(TL_DATUM);
  tft.setTextPadding(0);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.drawString("PIDs suportados", 10, 4, 4);

  struct ItemPid { const char* nome; QId id; };
  const ItemPid itens[] = {
    {"Velocidade (010D)", Q_VEL}, {"Rotacao (010C)", Q_RPM}, {"Carga motor (0104)", Q_CARGA},
    {"Temp. liquido (0105)", Q_TEMP}, {"Ar admissao (010F)", Q_IAT}, {"Acelerador (0111)", Q_ACEL},
    {"Ajuste mistura (0106/07)", Q_STFT},
  };
  int y = 32;
  for (uint8_t i = 0; i < 7; i++) {
    bool ok = consultas[itens[i].id].ativo;
    if (itens[i].id == Q_STFT) ok = ok && consultas[Q_LTFT].ativo;
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString(itens[i].nome, 10, y, 2);
    tft.setTextColor(ok ? TFT_GREEN : TFT_RED, TFT_BLACK);
    tft.drawString(ok ? "OK" : "NAO", 250, y, 2);
    y += 19;
  }
  const char* fm = consultas[Q_MAF].ativo ? "MAF (medido)"
                 : consultas[Q_MAP].ativo ? "MAP (estimado)" : "INDISPONIVEL";
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("Consumo", 10, y, 2);
  tft.setTextColor(combustivelDisponivel() ? TFT_GREEN : TFT_RED, TFT_BLACK);
  tft.drawString(fm, 170, y, 2);

  uint8_t sd = estadoSd();
  char s[40];
  if (sd == 1)      snprintf(s, sizeof(s), "Registro: %s", logNome + 1);
  else if (sd == 2) snprintf(s, sizeof(s), "Registro: sem cartao SD");
  else              snprintf(s, sizeof(s), "Registro em cartao: desligado");
  tft.setTextColor(sd == 1 ? TFT_GREEN : (sd == 2 ? TFT_RED : COR_ROT), TFT_BLACK);
  tft.drawString(s, 10, 178, 2);

  snprintf(s, sizeof(s), "Viagem retomada: %.1f km", v.distKm);
  tft.setTextColor(COR_ROT, TFT_BLACK);
  tft.drawString(s, 10, 197, 2);
  tft.drawString("Toque para continuar", 10, 216, 2);
}

// ---- atualizacao das paginas ----
void atualizaPagina1() {
  char s[20];

  // linha 1: distancia | tempo total
  snprintf(s, sizeof(s), "%.1f", v.distKm);
  setCelula(p1[0], s, TFT_WHITE);

  fmtTempo(s, sizeof(s), v.tTotal);
  setCelula(p1[1], s, TFT_WHITE);

  // linha 2: velocidade media (sobre o tempo total) | tempo em movimento
  float vm = (v.tTotal > 10.0) ? (float)(v.distKm / (v.tTotal / 3600.0)) : 0.0f;
  snprintf(s, sizeof(s), "%.1f", vm);
  setCelula(p1[2], s, TFT_WHITE);

  fmtTempo(s, sizeof(s), v.tMov);
  setCelula(p1[3], s, TFT_WHITE);

  // linha 3: ritmo medio | ritmo instantaneo (media movel)
  fmtRitmo(s, sizeof(s), vm);
  setCelula(p1[4], s, TFT_WHITE);

  fmtRitmo(s, sizeof(s), velMediaMovel());
  setCelula(p1[5], s, TFT_WHITE);

  // linha 4: consumo medio | consumo instantaneo (cores em degrade)
  if (!combustivelDisponivel()) {
    snprintf(s, sizeof(s), "N/D");
    setCelula(p1[6], s, COR_ROT);
    setCelula(p1[7], s, COR_ROT);
    setCelula(p1[8], s, COR_ROT);
  } else {
    if (v.litros > 0.3 && v.distKm > 0.1) {
      float med = (float)(v.distKm / v.litros);
      if (med > 99.9f) med = 99.9f;
      snprintf(s, sizeof(s), "%.1f", med);
      setCelula(p1[6], s, corConsumo(med));
    } else {
      snprintf(s, sizeof(s), "--");
      setCelula(p1[6], s, TFT_WHITE);
    }

    if (emaInit && lhEma > 0.3f && velEma >= VEL_MOVIMENTO_KMH) {
      float kml = velEma / lhEma;
      if (kml > 99.9f) kml = 99.9f;
      snprintf(s, sizeof(s), "%.1f", kml);
      setCelula(p1[7], s, corConsumo(kml));
    } else {
      snprintf(s, sizeof(s), "--");
      setCelula(p1[7], s, TFT_WHITE);
    }

    // linha 5 (esquerda): litros usados
    snprintf(s, sizeof(s), "%.1f", (float)v.litros);
    setCelula(p1[8], s, corLitros((float)v.litros));
  }

  // linha 5 (direita): carga media do trajeto
  if (v.tCarga > 5.0) {
    float media = (float)(v.cargaInt / v.tCarga);
    snprintf(s, sizeof(s), "%d", (int)lroundf(media));
    setCelula(p1[9], s, corCarga(media));
  } else {
    snprintf(s, sizeof(s), "--");
    setCelula(p1[9], s, TFT_WHITE);
  }
}

void atualizaPagina2(float dt) {
  char s[16];
  bool okRpm = fresco(d.tRpm);
  bool motor = okRpm && d.rpm > 300;

  // RPM
  if (okRpm) {
    float r = suaviza(sRpm, d.rpm, dt, TAU_UI_RAPIDO_S);
    snprintf(s, sizeof(s), "%d", (int)(r + 0.5f));
    setCelula(p2Rpm, s, corRpm(r));
  } else {
    sRpm.ok = false;
    setCelula(p2Rpm, "--", TFT_WHITE);
  }

  // Bateria
  if (fresco(d.tVolt, 8000)) {
    float b = suaviza(sBat, d.volt, dt, TAU_UI_LENTO_S);
    snprintf(s, sizeof(s), "%.1f", b);
    setCelula(p2Bat, s, corTensao(b, motor));
  } else {
    sBat.ok = false;
    setCelula(p2Bat, "--", TFT_WHITE);
  }

  // Barra de carga
  if (fresco(d.tCarga)) {
    float c = suaviza(sCarga, d.carga, dt, TAU_UI_RAPIDO_S);
    atualizaBarra(barCarga, c / 100.0f);
    snprintf(s, sizeof(s), "%d%%", (int)lroundf(c));
    setCelula(p2CargaV, s, corCarga(c));
  } else {
    sCarga.ok = false;
    atualizaBarra(barCarga, 0);
    setCelula(p2CargaV, "--", TFT_WHITE);
  }

  // Barra de temperatura
  if (fresco(d.tTemp, 8000)) {
    float t = suaviza(sTemp, d.temp, dt, TAU_UI_LENTO_S);
    atualizaBarra(barTemp, (t - TEMP_MIN_BARRA) / (TEMP_MAX_BARRA - TEMP_MIN_BARRA));
    snprintf(s, sizeof(s), "%d", (int)lroundf(t));
    setCelula(p2TempV, s, corTemp(t));
  } else {
    sTemp.ok = false;
    atualizaBarra(barTemp, 0);
    setCelula(p2TempV, "--", TFT_WHITE);
  }

  // Velocidade
  if (fresco(d.tVel)) {
    float vv = suaviza(sVel, d.vel, dt, TAU_UI_RAPIDO_S);
    snprintf(s, sizeof(s), "%d", (int)(vv + 0.5f));
    setCelula(p2Vel, s, TFT_WHITE);
  } else {
    sVel.ok = false;
    setCelula(p2Vel, "--", TFT_WHITE);
  }

  // Acelerador
  if (fresco(d.tAcel, 6000)) {
    float a = suaviza(sAcel, d.acel, dt, TAU_UI_RAPIDO_S);
    snprintf(s, sizeof(s), "%d", (int)lroundf(a));
    setCelula(p2Acel, s, TFT_WHITE);
  } else {
    sAcel.ok = false;
    setCelula(p2Acel, "--", TFT_WHITE);
  }

  // Temperatura do ar de admissao
  if (fresco(d.tIat, 15000)) {
    float i = suaviza(sIat, d.iat, dt, TAU_UI_LENTO_S);
    snprintf(s, sizeof(s), "%d", (int)lroundf(i));
    setCelula(p2Iat, s, TFT_WHITE);
  } else {
    sIat.ok = false;
    setCelula(p2Iat, "--", TFT_WHITE);
  }

  // Pico de temperatura da sessao
  if (picoTemp > -100) {
    snprintf(s, sizeof(s), "%d", (int)lroundf(picoTemp));
    setCelula(p2Pico, s, corTemp(picoTemp));
  } else {
    setCelula(p2Pico, "--", TFT_WHITE);
  }

  // Ajuste de mistura: soma (curto + longo) e, ao lado, cada um
  if (fresco(d.tStft, 10000) && fresco(d.tLtft, 10000)) {
    float cu = suaviza(sStft, d.stft, dt, TAU_UI_LENTO_S);
    float lo = suaviza(sLtft, d.ltft, dt, TAU_UI_LENTO_S);
    float soma = cu + lo;
    snprintf(s, sizeof(s), "%+.1f", soma);
    setCelula(p2Trim, s, corTrim(soma));
    snprintf(s, sizeof(s), "%+.1f/%+.1f", cu, lo);
    setCelula(p2TrimD, s, COR_ROT);
  } else {
    sStft.ok = false; sLtft.ok = false;
    setCelula(p2Trim, "--", TFT_WHITE);
    setCelula(p2TrimD, "--", COR_ROT);
  }
}

// ---- alarme (temperatura) ----
void atualizaAlarme(uint32_t agora) {
  alarmeTemp = fresco(d.tTemp, 8000) && d.temp >= TEMP_ALARME;
  bool piscaOn = alarmeTemp && ((agora / 400) % 2 == 0);
  digitalWrite(LED_R, piscaOn ? LOW : HIGH);

  // LED azul piscando = sem Bluetooth durante a operacao (os dados nao estao sendo medidos)
  bool semLink = bootFase >= 2 && estadoLink != LINK_ONLINE && estadoLink != LINK_SEM_ECU;
  digitalWrite(LED_B, (semLink && ((agora / 500) % 2 == 0)) ? LOW : HIGH);
}

// =====================================================================
//  TOQUE
// =====================================================================
uint8_t reiniciosSeguidos = 0;

// Salva a viagem e reinicia o ESP32 (equivale a desligar e ligar o painel).
// automatico = true conta como reinicio automatico (limitado); false = pedido por toque.
void reiniciaPainel(bool automatico) {
  registraEvento(automatico ? "REINICIO_AUTO" : "REINICIO_MANUAL");
  if (sujo) salvaViagem();
  prefs.putUChar("rst", automatico ? (uint8_t)(reiniciosSeguidos + 1) : 0);
  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(TL_DATUM);
  tft.setTextPadding(0);
  tft.setTextColor(TFT_ORANGE, TFT_BLACK);
  tft.drawString("Reiniciando para", 10, 90, 4);
  tft.drawString("reconectar...", 10, 125, 4);
  delay(800);
  ESP.restart();
}

void aoToqueCurto() {
  if (bootFase < 2) {
    bootFase = 2;
    mudaPagina(0);
    return;
  }
  mudaPagina((paginaAtual + 1) % 2);
}

void aoToqueLongo() {
  if (bootFase < 2) {
    if (estadoLink != LINK_ONLINE) reiniciaPainel(false);   // reinicio manual na tela de conexao
    return;
  }
  if (paginaAtual == 0) {
    registraEvento("RESET_VIAGEM");   // a linha leva os totais finais (km e litros calculados) antes de zerar
    resetViagem();
    setMsg("VIAGEM ZERADA");
  } else {
    registraEvento("RESET_PICOS");
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

  usaBusTouch();                  // o cartao SD pode ter deixado o barramento nos pinos dele
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

  prefs.begin("painel", false);
  if (esp_reset_reason() != ESP_RST_SW) prefs.putUChar("rst", 0);   // partida a frio zera o contador
  reiniciosSeguidos = prefs.getUChar("rst", 0);
  podeReiniciar = reiniciosSeguidos < BT_MAX_REINICIOS_SEGUIDOS;
  carregaViagem();

  // Registro em cartao: um arquivo novo a cada partida a frio; o reinicio automatico
  // (para reconectar o Bluetooth) continua no mesmo arquivo.
  bootN = prefs.getUInt("bootn", 0) + 1;
  prefs.putUInt("bootn", bootN);
  logSeq = prefs.getUInt("logseq", 0);
  if (esp_reset_reason() != ESP_RST_SW || logSeq == 0) {
    logSeq++;
    prefs.putUInt("logseq", logSeq);
  }
  snprintf(logNome, sizeof(logNome), "/viagem_%04u.csv", (unsigned)logSeq);
  if (LOG_ATIVO_CONFIG && logMonta()) logPronto = true;
  tLogTentou = millis();

  usaBusTouch();                  // touch e SD dividem o VSPI: volta aos pinos do touch
  ts.begin(barramentoSPI);
  ts.setRotation(1);
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
  atualizaRegistro(agora);

  if (pedidoReinicio) reiniciaPainel(true);      // a conexao nao se recuperou sozinha
  if (estadoLink == LINK_ONLINE && reiniciosSeguidos > 0) {
    reiniciosSeguidos = 0;                       // conectou: zera o contador
    prefs.putUChar("rst", 0);
    podeReiniciar = true;
  }

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

  // intervalo entre quadros (para a suavizacao da pagina 2)
  static uint32_t tQuadroAnt = 0;
  float dt = (agora - tQuadroAnt) / 1000.0f;
  tQuadroAnt = agora;
  if (dt > 0.25f) dt = 0.25f;

  atualizaHeader();
  if (paginaAtual == 0) atualizaPagina1();
  else                  atualizaPagina2(dt);

  delay(20);
}
