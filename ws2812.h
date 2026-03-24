#pragma once
#include <stdint.h>

// Hardware-Konstanten für WS2812
#define NUM_LEDS 8
#define USE_BRIGHTNESS 0 // Optional: globale Helligkeit (kostet Rechenzeit)

// Timer-Werte für 0 und 1 (bei ARR = 104)
#define WS2812_BIT_0  30
#define WS2812_BIT_1  70

// Struktur für eine Farbe
typedef struct {
    uint8_t R;
    uint8_t G;
    uint8_t B;
} RGB_Color_t;

// Das "Lager" im RAM (hier speichern wir den Zustand)
extern RGB_Color_t LED_Lager[NUM_LEDS];

// Funktionen
void WS2812_Init(void);
void WS2812_SetLED(int index, uint8_t r, uint8_t g, uint8_t b);
void WS2812_Update(void); // Schickt die Daten an die LEDs