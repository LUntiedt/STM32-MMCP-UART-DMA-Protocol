#pragma once 

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <string.h>

extern uint8_t myAddress; // Nummer des Boards

/* DEFINES */
#define MMCP_MASTER_ADDRESS 0
#define MMCP_VERSION 7
#define L7_PDU_size 9
#define L7_SDU_size 8
#define L7_PCI_size 1
#define L3_PDU_size 13
#define L3_SDU_size 9
#define L3_PCI_size 4
#define L2_PDU_size 14
#define L2_SDU_size 13
#define L2_PCI_size 1
#define L1_PDU_size 16
#define L1_SDU_size 14
#define L1_PCI_size 2


enum states
{
    processing,
    awaiting,
    received,
    sent,
    failure
};

// --- FEHLERCODES (Global definiert) ---
#define MMCP_SUCCESS 0
#define MMCP_ERR_EXISTS 1
#define MMCP_ERR_FULL 2
#define MMCP_ERR_NOT_AVAIL 3
#define MMCP_ERR_UNKNOWN_PARTNER 4

// --- PARTNER IDs (Himmelsrichtungen) ---
#define PARTNER_NONE  0
#define PARTNER_NORTH 1
#define PARTNER_EAST  26
#define PARTNER_SOUTH 2
#define PARTNER_WEST  24

// Flags für Nachbarn
extern volatile bool signal_north;
extern volatile bool signal_east;
extern volatile bool signal_south;
extern volatile bool signal_west;

// Neue Globals für die Paket-Logik, extern damit sie in anderen Dateien genutzt werden können
extern uint8_t last_error_code;  // Damit main den Fehler anzeigen kann
extern uint8_t paketiB;          // Welches Paket bearbeiten wir?
extern uint8_t active_partner_id; // Mit welchem Nachbarn reden wir? (Für ApNr 42/44)
extern uint8_t target_partner_id; // An wen leiten wir weiter? (0 = behalten), ist für ApNr 43
volatile extern uint8_t lager[6];       // Lagerplätze (0 = leer)
extern uint8_t current_state;    // 0=Processing, 1=Awaiting, 2=Received, 3=Sent, 4=Failure

// bsp.h
extern volatile bool frame_received_flag; // Nur ein "Wegweiser"

// Zähler für Button-Presses
extern uint8_t cnt_button_press;

// --- NEUE EVENT FLAGS FÜR SA/RT (Kontrollflüsse) ---
extern volatile bool flag_ap42_create;  // Create Befehl (Partner 0)
extern volatile bool flag_ap42_await;   // Await Befehl (Partner != 0)
extern volatile bool flag_ap43_fwd;     // Forwarding Befehl
extern volatile bool flag_ap44_pass;    // Pass On Befehl
extern volatile bool flag_ap44_deliver; // Deliver Befehl
extern volatile bool flag_ap50_was_read; // AP50 wurde gelesen

// variablen und handles
extern UART_HandleTypeDef huart1; // UART-Handle für USART1
extern UART_HandleTypeDef huart2; // UART-Handle für USART2

// Für die WS2812 Handles (müssen global erreichbar sein)
extern TIM_HandleTypeDef htim2;
extern DMA_HandleTypeDef hdma_tim2_ch3;

// --- NEUE VARIABLEN FÜR MMCP ---
// --- ÄNDERUNG: WICHTIG! Nur extern davor schreiben! ---
extern uint8_t L1_PDU_IT_BUFFER[L1_PDU_size]; 
extern uint8_t L1_PDU[L1_PDU_size];
extern uint8_t L1_SDU[L1_SDU_size];
extern uint8_t L1_PCI[L1_PCI_size];

extern uint8_t L2_PDU[L2_PDU_size];
extern uint8_t L2_SDU[L2_SDU_size];
extern uint8_t L2_PCI[L2_PCI_size];

extern uint8_t L3_PDU[L3_PDU_size];
extern uint8_t L3_SDU[L3_SDU_size];
extern uint8_t L3_PCI[L3_PCI_size];

extern uint8_t L7_PDU[L7_PDU_size];
extern uint8_t L7_SDU[L7_SDU_size];
extern uint8_t L7_PCI[L7_PCI_size];



// Funktionsprototypen
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void Error_Handler(void);
void MX_DMA_Init(void);
void MX_TIM2_Init(void);

// Hilfsfunktionen für die LED-Farben
void get_color_for_id(uint8_t id, uint8_t* r, uint8_t* g, uint8_t* b);

uint8_t lager_check(uint8_t pkg_id);
uint8_t lager_add(uint8_t pkg_id);

// Funktion zur Berechnung der L2-CRC
uint8_t l2_calculate_crc(uint8_t* data, int laenge);

// Receive Funktionen
void L1_receive(uint8_t L1_PDU[]);
void L2_receive(uint8_t L2_PDU[]);
void L3_receive(uint8_t L3_PDU[]);
void L7_receive(uint8_t L7_PDU[]);

// Send Funktionen
void L7_send(uint8_t ApNr,uint8_t L7_SDU[]);
void L3_send(uint8_t L3_SDU[]);
void L2_send(uint8_t L2_SDU[]);
void L1_send(uint8_t L1_SDU[]);


