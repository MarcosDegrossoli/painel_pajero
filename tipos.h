#pragma once
#include <Arduino.h>

// Tipos usados em parametros de funcoes do sketch.
// Ficam aqui porque a Arduino IDE gera prototipos automaticos ANTES das
// definicoes do .ino, o que quebra funcoes que recebem esses tipos.

enum EstadoLink : uint8_t {
  LINK_DESCONECTADO, LINK_CONECTANDO, LINK_CONFIGURANDO, LINK_SEM_ECU, LINK_ONLINE
};

enum QId : uint8_t { Q_VEL, Q_RPM, Q_CARGA, Q_MAF, Q_MAP, Q_TEMP, Q_IAT, Q_ACEL, Q_VOLT, Q_N };

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

struct Barra {
  int16_t x, y, w, h;
  float t1, t2;          // marcas (fracao 0..1) dos limites de cor
  int16_t ultAlt;
  uint16_t ultCor;
};
