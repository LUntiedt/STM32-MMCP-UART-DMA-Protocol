#include "stm32f4xx_hal.h"
#include <stdio.h>
#include "bsp.h"
#include "ws2812.h"

/* --- GLOBALE VARIABLEN --- */

uint8_t myAddress = 0x19; // Nummer des Boards

// --- FLAGS (Signale von Interrupts an die main-loop) ---
volatile bool flag_ap42_create = false;
volatile bool flag_ap42_await = false;
volatile bool flag_ap43_fwd = false;
volatile bool flag_ap44_pass = false;
volatile bool flag_ap44_deliver = false;
volatile bool flag_ap50_was_read = false;     // Handshake-Flag für AP50 (Status gelesen)
volatile bool frame_received_flag = false;    // Signal: Ein neuer Frame liegt bereit
bool richtiger_partner_hat_gedrueckt = false; // Signal: GPIO-Interrupt (Nachbar)

// --- ZUSTANDS-VARIABLEN (MMCP) ---
uint8_t last_error_code = 0;     // Speichert den letzten Fehler für AP50
uint8_t paketiB = 0;             // Aktuelles Paket in Bearbeitung
uint8_t active_partner_id = 0;   // Kommunikationspartner (z.B. Sender bei Await)
uint8_t target_partner_id = 0;   // Zielpartner (z.B. Empfänger bei Forward/Pass)
volatile uint8_t lager[6] = {0}; // Lagerplätze (0 = leer)
uint8_t current_state = 0;       // Zustandsautomat: 0=Proc, 1=Await, 2=Recv, 3=Sent, 4=Fail

// --- PERIPHERIE HANDLES ---
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim2_ch3;

// --- DATENPUFFER (OSI-Schichten) ---
uint8_t L1_PDU_IT_BUFFER[L1_PDU_size]; // Interrupt-Puffer
uint8_t L1_PDU[L1_PDU_size];
uint8_t L1_SDU[L1_SDU_size];
uint8_t L1_PCI[L1_PCI_size];

uint8_t L2_PDU[L2_PDU_size];
uint8_t L2_SDU[L2_SDU_size];
uint8_t L2_PCI[L2_PCI_size];

uint8_t L3_PDU[L3_PDU_size];
uint8_t L3_SDU[L3_SDU_size];
uint8_t L3_PCI[L3_PCI_size];

uint8_t L7_PDU[L7_PDU_size];
uint8_t L7_SDU[L7_SDU_size];
uint8_t L7_PCI[L7_PCI_size];

// --- TIMER FÜR ZUSTANDSVERZÖGERUNGEN ---
// Nötig, damit langsame Testskripte (Python) den Status "Processing" erfassen können,
// bevor auf "Sent" gewechselt wird.
uint32_t send_timer_start = 0;    // Für PassOn / Forward
uint32_t deliver_timer_start = 0; // Für Deliver (Löschen)

/* --- HILFSFUNKTIONEN --- */

// Visualisierung: Mappt Lagerbestand und Status auf die WS2812 LEDs
void update_leds(void)
{
    uint8_t r, g, b;

    // 1. Lagerbestand anzeigen (LEDs 1-6)
    for (int i = 0; i < 6; i++)
    {
        get_color_for_id(lager[i], &r, &g, &b);
        WS2812_SetLED(i + 1, r / 2, g / 2, b / 2);
    }

    // 2. Status anzeigen (LED 0 = Eingang, LED 7 = Ausgang)
    WS2812_SetLED(0, 0, 0, 0); 
    WS2812_SetLED(7, 0, 0, 0); 

    // Eingang aktiv (Awaiting/Received)
    if (current_state == 1 || current_state == 2) 
    {
        if (paketiB != 0)
        {
            get_color_for_id(paketiB, &r, &g, &b);
            WS2812_SetLED(0, r, g, b); 
        }
        else
        {
            WS2812_SetLED(0, 20, 20, 20); // Weiß gedimmt (Warten auf Unbekannt)
        }
    }
    // Ausgang aktiv (Processing/Sent)
    else if (current_state == 0 || current_state == 3) 
    {
        if (paketiB != 0)
        {
            get_color_for_id(paketiB, &r, &g, &b);
            WS2812_SetLED(7, r, g, b); 
        }
    }
    // Fehlerzustand
    else if (current_state == 4) 
    {
        WS2812_SetLED(0, 255, 0, 0); // Rot
        WS2812_SetLED(7, 255, 0, 0); // Rot
    }

    WS2812_Update();
}

// Erzeugt einen 1ms Impuls auf dem SV-Pin des angegebenen Nachbarn
void send_signal_to_neighbor(uint8_t partner_id)
{
    GPIO_TypeDef *port = NULL;
    uint16_t pin = 0;

    switch (partner_id)
    {
    case PARTNER_NORTH: // PB0
        port = GPIOB;
        pin = GPIO_PIN_0;
        break;
    case PARTNER_EAST: // PA7
        port = GPIOA;
        pin = GPIO_PIN_7;
        break;
    case PARTNER_SOUTH: // PA6
        port = GPIOA;
        pin = GPIO_PIN_6;
        break;
    case PARTNER_WEST: // PB9
        port = GPIOB;
        pin = GPIO_PIN_9;
        break;
    default:
        return; // Ungültig
    }

    if (port != NULL)
    {
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);   
        HAL_Delay(1);                                 
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET); 
    }
}

// Validiert die Partner-ID (Muss eine der 4 Himmelsrichtungen sein)
bool is_valid_partner(uint8_t id)
{
    return (id == PARTNER_NORTH || id == PARTNER_EAST ||
            id == PARTNER_SOUTH || id == PARTNER_WEST);
}

/* --- MAIN PROGRAMM --- */
int main(void)
{
    // --- Initialisierung ---
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();        
    MX_USART1_UART_Init(); 
    MX_USART2_UART_Init(); 
    MX_DMA_Init();         
    MX_TIM2_Init();        
    WS2812_Init();         

    static uint32_t led_timer = 0;

    // UART Flags bereinigen (Fehlerflags löschen)
    __HAL_UART_CLEAR_OREFLAG(&huart2);
    __HAL_UART_CLEAR_NEFLAG(&huart2);
    __HAL_UART_CLEAR_FEFLAG(&huart2);
    __HAL_UART_CLEAR_PEFLAG(&huart2);

    // Dummy-Read falls Daten im Register sind
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE))
    {
        volatile uint8_t dummy = huart2.Instance->DR;
        (void)dummy; 
    }

    // Starten des interrupt-basierten Empfangs ("Angel auswerfen")
    HAL_UART_Receive_IT(&huart2, L1_PDU_IT_BUFFER, L1_PDU_size);

    while (1)
    {
        // --- 1. MMCP Empfangsverarbeitung ---
        if (frame_received_flag)
        {
            frame_received_flag = false; 
            L1_receive(L1_PDU); // Startet die Kapselung L1 -> L7
        }

        // --- 2. Zustandsautomat (State Machine) ---
        switch (current_state)
        {
        // =========================================================
        // ZUSTAND 0: PROCESSING
        // =========================================================
        case processing:

            // --- Ereignis: AP42 CREATE (Partner == 0) ---
            if (flag_ap42_create)
            {
                flag_ap42_create = false;

                // Create ist intern -> Paket direkt ins Lager
                uint8_t res = lager_add(paketiB);
                
                if (res == MMCP_SUCCESS)
                {
                    flag_ap50_was_read = false;
                    current_state = awaiting;
                }
                else
                {
                    last_error_code = res;
                    flag_ap50_was_read = false; 
                    current_state = failure;
                }
            }

            // --- Ereignis: AP42 AWAIT (Partner != 0) ---
            else if (flag_ap42_await)
            {
                flag_ap42_await = false;

                // Partner Validierung
                if (!is_valid_partner(active_partner_id))
                {
                    last_error_code = MMCP_ERR_UNKNOWN_PARTNER;
                    flag_ap50_was_read = false;
                    current_state = failure;
                }
                else
                {
                    // Partner OK -> Platz im Lager prüfen
                    uint8_t res = lager_check(paketiB); 
                    if (res == MMCP_SUCCESS)
                    {
                        // Flags resetten für sauberen Empfang
                        signal_north = false;
                        signal_east = false;
                        signal_south = false;
                        signal_west = false;

                        flag_ap50_was_read = false;
                        current_state = awaiting;
                    }
                    else
                    {
                        last_error_code = res;
                        flag_ap50_was_read = false; 
                        current_state = failure;
                    }
                }
            }

            // --- Ereignis: AP43 FORWARD ---
            else if (flag_ap43_fwd)
            {
                flag_ap43_fwd = false;

                // Validierung BEIDER Partner (Von und Zu)
                if (!is_valid_partner(active_partner_id) || !is_valid_partner(target_partner_id))
                {
                    last_error_code = MMCP_ERR_UNKNOWN_PARTNER;
                    flag_ap50_was_read = false;
                    current_state = failure;
                }
                else
                {
                    // Reset Signale
                    signal_north = false;
                    signal_east = false;
                    signal_south = false;
                    signal_west = false;

                    flag_ap50_was_read = false; 
                    current_state = awaiting;
                }
            }

            // --- Ereignis: AP44 PASS ON (Senden an Nachbarn) ---
            else if (flag_ap44_pass)
            {
                flag_ap44_pass = false;

                // Validierung Ziel-Partner
                if (!is_valid_partner(target_partner_id))
                {
                    last_error_code = MMCP_ERR_UNKNOWN_PARTNER;
                    flag_ap50_was_read = false;
                    current_state = failure;
                }
                else
                {
                    // Paket aus dem Lager holen
                    int found = -1;
                    for (int i = 0; i < 6; i++)
                    {
                        if (lager[i] == paketiB)
                            found = i;
                    }

                    if (found != -1)
                    {
                        lager[found] = 0; // Entfernen
                        // Senden passiert unten im Timer-Block
                    }
                    else
                    {
                        last_error_code = MMCP_ERR_NOT_AVAIL;
                        flag_ap50_was_read = false; 
                        current_state = failure;
                    }
                }
            }

            // --- Ereignis: AP44 DELIVER (Hier behalten/löschen) ---
            else if (flag_ap44_deliver)
            {
                flag_ap44_deliver = false;
                
                int found = -1;
                for (int i = 0; i < 6; i++)
                {
                    if (lager[i] == paketiB)
                        found = i;
                }

                if (found != -1)
                {
                    lager[found] = 0; // Paket entfernen
                    
                    // Timer starten statt Status sofort zu wechseln
                    // Damit das Testskript den Zustand "Processing" erkennen kann.
                    deliver_timer_start = HAL_GetTick();
                    
                    flag_ap50_was_read = false;
                    target_partner_id = PARTNER_NONE;
                }
                else
                {
                    last_error_code = MMCP_ERR_NOT_AVAIL;
                    flag_ap50_was_read = false; 
                    current_state = failure;
                }
            }

            // --- TIMER LOGIK (Für verzögerte Zustandswechsel) ---

            // 1. Deliver Timer: Warten nach dem Löschen
            if (deliver_timer_start > 0)
            {
                // 1500ms warten für Test-Synchronisation
                if ((HAL_GetTick() - deliver_timer_start) > 1500)
                {
                    deliver_timer_start = 0;    
                    flag_ap50_was_read = false; 
                    current_state = sent;       
                }
            }

            // 2. Sende Timer: Physisches Senden simulieren
            if (paketiB != 0 && target_partner_id != PARTNER_NONE)
            {
                if (send_timer_start == 0)
                    send_timer_start = HAL_GetTick();

                // 1500ms Dauer für Sendevorgang
                if ((HAL_GetTick() - send_timer_start) > 1500)
                {
                    send_signal_to_neighbor(target_partner_id); 

                    flag_ap50_was_read = false; 
                    current_state = sent;

                    target_partner_id = PARTNER_NONE;
                    send_timer_start = 0;
                }
            }
            break;

        // =========================================================
        // ZUSTAND 1: AWAITING
        // =========================================================
        case awaiting:
            // Pfad A: Create (Interne Erstellung)
            if (active_partner_id == PARTNER_NONE)
            {
                if (flag_ap50_was_read) // Handshake erfolgt
                {
                    flag_ap50_was_read = false; 
                    current_state = received;
                }
            }
            // Pfad B: Echtes Warten auf Nachbarn
            else
            {
                // Status-Flag hier ignorieren, wir warten auf GPIO
                if (flag_ap50_was_read) flag_ap50_was_read = false;

                // Simulation: Button
                if (cnt_button_press > 0)
                {
                    richtiger_partner_hat_gedrueckt = true;
                    cnt_button_press = 0;
                }
                
                // Echte GPIOs prüfen, kommt später, wenn wir echten Signal-Pin haben
                if ((active_partner_id == PARTNER_NORTH && signal_north) ||
                    (active_partner_id == PARTNER_EAST && signal_east) ||
                    (active_partner_id == PARTNER_SOUTH && signal_south) ||
                    (active_partner_id == PARTNER_WEST && signal_west))
                {
                    richtiger_partner_hat_gedrueckt = true;
                }

                if (richtiger_partner_hat_gedrueckt)
                {
                    // Reset Signale
                    signal_north = false;
                    signal_east = false;
                    signal_south = false;
                    signal_west = false;
                    richtiger_partner_hat_gedrueckt = false;

                    // Wenn es kein Forwarding ist -> Einlagern
                    if (target_partner_id == PARTNER_NONE)
                    {
                        lager_add(paketiB);
                    }

                    flag_ap50_was_read = false;
                    current_state = received;
                }
            }
            break;

        // =========================================================
        // ZUSTAND 2: RECEIVED
        // =========================================================
        case received:
            // Warten auf Handshake (AP50)
            if (flag_ap50_was_read)
            {
                flag_ap50_was_read = false; 

                if (target_partner_id != PARTNER_NONE)
                    current_state = processing; // Forwarding -> weiter
                else {
                    paketiB = 0;
                    current_state = processing; // Fertig
                }
            }
            break;

        // =========================================================
        // ZUSTAND 3: SENT
        // =========================================================
        case sent:
            // Warten auf Handshake (AP50)
            if (flag_ap50_was_read)
            {
                flag_ap50_was_read = false; 
                paketiB = 0;                
                current_state = processing;
            }
            break;

        // =========================================================
        // ZUSTAND 4: FAILURE
        // =========================================================
        case failure:
            // Fehler muss quittiert werden (AP50)
            if (flag_ap50_was_read)
            {
                flag_ap50_was_read = false;
                last_error_code = 0;
                paketiB = 0;
                current_state = processing;
            }
            break;

        default:
            current_state = processing;
            break;
        }

        // --- 3. LED Update (Non-Blocking, 20Hz) ---
        if ((HAL_GetTick() - led_timer) > 50)
        {
            update_leds();
            led_timer = HAL_GetTick(); 
        }
    }
}