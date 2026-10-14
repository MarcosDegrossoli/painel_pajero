#pragma once
#include <Arduino.h>

// =====================================================================
//  SEUS VALORES  -  edite este arquivo, nao o .ino
//  Assim, quando chegar uma versao nova do .ino, voce nao precisa
//  refazer estes ajustes.
// =====================================================================

// MAC do adaptador ELM327 (o que aparece como "OBDII"), 6 bytes em hexadecimal.
// Troque os valores abaixo pelo MAC do seu adaptador.
uint8_t mac_elm[6] = {0xAA, 0xBB, 0xCC, 0x11, 0x22, 0x33};

// --- Calibracao do consumo de combustivel ---
const float AFR                 = 13.1f;    // relacao ar/combustivel (gasolina pura = 14.7)
const float DENSIDADE_G_L       = 755.0f;   // g/L da gasolina
const float FATOR_CALIB_CONSUMO = 1.00f;    // multiplicador final (calibre apos abastecer)
