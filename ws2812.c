#include "ws2812.h"
#include "bsp.h" // Damit wir Zugriff auf htim2 haben

// Interner Speicher für die Farben (Das "Lager")
RGB_Color_t LED_Lager[NUM_LEDS];

/* --- KONFIGURATION --- */
// Werte für den Timer (ARR = 104 bei 84MHz)
#define PWM_HI  70  // Logisch '1' (Langer Puls)
#define PWM_LO  30  // Logisch '0' (Kurzer Puls)
#define RESET_SLOTS 60 // Anzahl der Nullen für den Reset (>50µs)

// Puffer: 8 LEDs * 24 Bit (GRB) + Reset-Pause am Ende
// WICHTIG: uint32_t nutzen, damit der DMA 32-Bit-Wörter zum Timer schiebt!
#define DMA_BUFFER_SIZE (NUM_LEDS * 24) + RESET_SLOTS
uint32_t dma_buffer[DMA_BUFFER_SIZE];

// Ampel: 1 = DMA sendet gerade, 0 = Bereit
volatile uint8_t ws2812_sending = 0;

/* --- FUNKTIONEN --- */

/**
 * @brief  Initialisiert den LED-Streifen.
 *
 * Warum brauchen wir das?
 * Nach dem Einschalten ("Power On") wissen die WS2812-Chips nicht,
 * was sie tun sollen. Manchmal leuchten sie zufällig blau oder grün auf,
 * je nach elektrischem Rauschen.
 *
 * Diese Funktion:
 * 1. Löscht unseren internen Farbspeicher (setzt alles auf Schwarz/0).
 * 2. Schickt dieses "Alles Aus"-Signal sofort an die LEDs.
 * Damit startet das System immer in einem sauberen, dunklen Zustand.
 */
void WS2812_Init(void) {
    // Alle LEDs auf Schwarz (0,0,0) setzen und einmal senden
    for(int i=0; i<NUM_LEDS; i++) {
        WS2812_SetLED(i, 0, 0, 0);
    }
    WS2812_Update();
}

/**
 * @brief  Setzt die Wunschfarbe für eine bestimmte LED im Speicher.
 *
 * @param  index: Nummer der LED (0 bis 7 auf dem Baseboard).
 * @param  r, g, b: Farbwerte von 0 (Aus) bis 255 (Vollgas).
 *
 * WICHTIG:
 * Diese Funktion ändert noch NICHTS am Licht!
 * Sie schreibt nur in unser "Notizbuch" (das Array LED_Lager), wie
 * wir die LEDs gerne hätten.
 *
 * Erst wenn wir WS2812_Update() aufrufen, wird das Notizbuch
 * an die Hardware gesendet. Das nennt man "Double Buffering".
 * Vorteil: Wir können ein komplettes Bild in Ruhe berechnen und dann
 * schlagartig ("atomar") anzeigen, ohne dass es flackert.
 */
void WS2812_SetLED(int index, uint8_t r, uint8_t g, uint8_t b) {
    if(index >= 0 && index < NUM_LEDS) {
        LED_Lager[index].R = r;
        LED_Lager[index].G = g;
        LED_Lager[index].B = b;
    }
}

/**
 * @brief  Wandelt Farben in Timer-Pulse um und startet den DMA.
 *
 * Hier passiert die Magie der ULP3-Hardware-Ansteuerung:
 * 1. Wir nehmen die RGB-Werte aus dem Speicher.
 * 2. Wir wandeln sie in die seltsame Sprache der WS2812 um:
 * - Ein Bit dauert immer 1,25 µs.
 * - Eine "0" ist ein kurzer Puls (Wert 30 im Timer).
 * - Eine "1" ist ein langer Puls (Wert 70 im Timer).
 * 3. Wir füllen einen riesigen Puffer (dma_buffer) mit diesen 30ern und 70ern.
 * 4. Wir geben dem DMA den Startbefehl.
 *
 * PROTOKOLL-BESONDERHEIT:
 * Die LEDs erwarten die Farben nicht in der Reihenfolge R-G-B,
 * sondern G-R-B (Grün zuerst!). Das müssen wir hier tauschen.
 */
void WS2812_Update(void) {
    if(ws2812_sending) return; // Warten, falls noch gesendet wird

    uint32_t buffer_index = 0;

    for(int i = 0; i < NUM_LEDS; i++) {
        // WS2812 erwartet die Reihenfolge GRB (Grün, Rot, Blau)
        uint8_t color_bytes[3] = {LED_Lager[i].G, LED_Lager[i].R, LED_Lager[i].B};

        // Schleife über die 3 Farben (Byte für Byte)
        for(int c = 0; c < 3; c++) {
            uint8_t byte = color_bytes[c];
            
            // Schleife über 8 Bits (MSB first: Bit 7 bis 0)
            for(int bit = 7; bit >= 0; bit--) {
                if(byte & (1 << bit)) {
                    dma_buffer[buffer_index] = WS2812_BIT_1; // Bit ist 1 -> Lang (70)
                } else {
                    dma_buffer[buffer_index] = WS2812_BIT_0; // Bit ist 0 -> Kurz (30)
                }
                buffer_index++;
            }
        }
    }

    // Den "Reset-Code" aktiv senden
    // Wir hängen 50 Nullen an. Da eine '0' ca. 1.25µs dauert, 
    // sind 50 Stück > 60µs. Das ist perfekt für den Reset (>50µs).
    for(int i = 0; i < 50; i++) {
        dma_buffer[buffer_index] = 0;
        buffer_index++;
    }

    // Senden starten
    ws2812_sending = 1;
    
    // Timer 2, Kanal 3 feuert los. Der DMA füttert ihn aus dma_buffer
    HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_3, dma_buffer, buffer_index);
}

/**
 * @brief  Wird vom DMA aufgerufen, wenn alle Daten gesendet wurden.
 *
 * WARUM IST DAS WICHTIG?
 * Der DMA schiebt Daten in den Timer. Wenn der Puffer leer ist, hört der
 * DMA zwar auf, aber der TIMER läuft einfach weiter!
 * Er würde immer wieder den letzten Wert (z.B. eine 70) wiederholen und
 * damit unendlich viele "Einsen" an die LEDs schicken.
 *
 * Deshalb müssen wir hier den Timer sofort stoppen (`Stop_DMA`).
 * Dadurch fällt die Datenleitung auf 0V (Low) zurück.
 * Das ist perfekt, denn die WS2812 brauchen >50µs Pause ("Reset Code"),
 * um zu wissen: "Das Bild ist fertig, jetzt anzeigen!".
 * Die 50 µs Pause entsteht automatisch, weil unser Programm
 * jetzt wieder normal weiterläuft und wir erst nach langer Zeit
 * wieder ein Update schicken.
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
    // NEU: Wir prüfen nur die Instanz. Da wir TIM2 nur für LEDs nutzen, ist das sicher.
    if(htim->Instance == TIM2) {
        HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_3);
        ws2812_sending = 0;
    }
}

// Hilfsfunktion: Wandelt Paket-ID in RGB-Werte laut Tabelle um
void get_color_for_id(uint8_t id, uint8_t* r, uint8_t* g, uint8_t* b)
{
    switch(id) {
        case 0:  *r=0;   *g=0;   *b=0;   break; // Leer = Aus

        // Tabelle aus dem Screenshot:
        case 1:  *r=255; *g=255; *b=255; break; // 1: White
        case 2:  *r=255; *g=0;   *b=0;   break; // 2: Red
        case 3:  *r=0;   *g=255; *b=0;   break; // 3: Green
        case 4:  *r=0;   *g=0;   *b=255; break; // 4: Blue
        case 5:  *r=0;   *g=255; *b=255; break; // 5: Cyan
        case 6:  *r=255; *g=0;   *b=255; break; // 6: Magenta
        case 7:  *r=255; *g=255; *b=0;   break; // 7: Yellow
        case 8:  *r=191; *g=128; *b=64;  break; // 8: Brown
        case 9:  *r=191; *g=255; *b=0;   break; // 9: Lime
        case 10: *r=128; *g=128; *b=0;   break; // 10: Olive
        case 11: *r=255; *g=128; *b=0;   break; // 11: Orange
        case 12: *r=255; *g=191; *b=191; break; // 12: Pink
        case 13: *r=191; *g=0;   *b=64;  break; // 13: Purple
        case 14: *r=0;   *g=128; *b=128; break; // 14: Teal
        case 15: *r=128; *g=0;   *b=128; break; // 15: Violet
        case 16: *r=224; *g=176; *b=255; break; // 16: Mauve

        // Fallback für unbekannte IDs (weiß gedimmt)
        default: *r=50;  *g=50;  *b=50;  break; 
    }
}