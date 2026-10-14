#pragma once
#include <Arduino.h>

// Tipos usados em parametros de funcoes do sketch.
// Ficam aqui porque a Arduino IDE gera prototipos automaticos ANTES das
// definicoes do .ino, o que quebra funcoes que recebem esses tipos.

enum EstadoLink : uint8_t {
  LINK_DESCONECTADO, LINK_CONECTANDO, LINK_CONFIGURANDO, LINK_SEM_ECU, LINK_ONLINE
};

enum QId : uint8_t {
  Q_VEL, Q_RPM, Q_CARGA, Q_MAF, Q_MAP, Q_TEMP, Q_IAT, Q_ACEL,
  Q_STFT, Q_LTFT,      // ajuste de mistura: curto prazo (06) e longo prazo (07)
  Q_VOLT, Q_N
};

struct Consulta {
  const char* cmd;
  uint8_t  pid;        // 0 = comando AT (nao depende de suporte do carro)
  uint16_t intervalo;  // ms desejado entre leituras
  bool     soPagina2;  // so consulta quando a pagina off-road esta aberta
  bool     ativo;
  uint8_t  falhas;
  uint32_t ultima;
};

struct Celula {
  int16_t x, y, w;
  int16_t yVal;          // deslocamento do valor em relacao a y
  uint8_t fonteVal;
  const char* rotulo;    // nullptr = sem rotulo
  uint8_t fonteRot;
  char ult[16];          // ultimo texto desenhado
  uint16_t ultCor;
  uint8_t datum;         // alinhamento do valor (0 = TL_DATUM)
};

// Barra vertical com degrade: cada linha da barra tem a cor correspondente
// ao valor que ela representa (corFn recebe o valor e devolve a cor RGB565).
struct Barra {
  int16_t x, y, w, h;
  float vmin, vmax;              // escala da barra
  uint16_t (*corFn)(float);
  int16_t ultAlt;                // altura (em pixels) atualmente desenhada
};

// Valor suavizado para exibicao (animacao dos numeros e das barras).
struct Suave {
  float v;
  bool  ok;                      // false = proxima leitura entra direto, sem animar
};

// Maneira de abrir a conexao Bluetooth (tentadas em rodizio quando uma falha)
struct EstrategiaBT {
  uint8_t canal;   // 0 = descobrir o canal por busca de servico; >0 = usa esse canal direto
  uint8_t papel;   // 0 = master, 1 = slave
};
