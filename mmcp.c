/**
 * @brief  Der Unterschied: Warten (Blocking) vs. Melden (Interrupt)
 *
 * HAL_UART_Receive() -- "Warten" (BLOCKING / Synchron)
 * ----------------------------------------------------------------
 * - Das Programm FRIERT AN DIESER STELLE EIN.
 * - Es wartet, bis alle 16 Bytes empfangen wurden.
 * - Blockiert die main_loop.
 * - Analogie: Man starrt auf die Angel, bis ein Fisch anbeißt,
 * und kann nichts anderes tun.
 *
 *
 * HAL_UART_Receive_IT() -- "Melden" (NON-BLOCKING / Asynchron)
 * ----------------------------------------------------------------
 * - Das Programm gibt der Hardware nur einen "Auftrag" und LÄUFT SOFORT WEITER.
 * - Die Hardware sammelt die Bytes im Hintergrund (per DMA/Interrupts).
 * - Wenn fertig, wird die CPU per Callback ("Bissanzeiger") informiert.
 * - Analogie: Man wirft die Angel aus, liest ein Buch (main_loop)
 * und reagiert nur, wenn der Bissanzeiger klingelt (Callback).
 */

/*
 * =========================================================================
 * WICHTIGE DESIGN-ANMERKUNG (Race Condition / Puffer-Problem):
 * =========================================================================
 * * AKTUELLES DESIGN:
 * Wir verwenden einen einzigen globalen Puffer (z.B. L1_PDU) und ein
 * einziges Flag (frame_received_flag).
 *
 * ERKANNTES PROBLEM:
 * Dieses Design ist anfällig für Datenverlust bei hoher Sendefrequenz.
 * * ABLAUF DES FEHLERS (Beispiel):
 * 1. Frame 1 kommt an. Interrupt schreibt in L1_PDU. Flag wird 'true'.
 * 2. main()-Schleife sieht Flag, setzt es auf 'false' und beginnt
 * die Verarbeitung (l1receive() kopiert Frame 1 sicher weg,
 * l2receive()...l7receive() dauern eine Weile).
 * 3. *Währenddessen* kommt Frame 2 an. Interrupt schreibt Frame 2 in L1_PDU
 * (überschreibt alte Daten von Frame 1, was ok ist). Flag wird 'true'.
 * 4. *Währenddessen* kommt Frame 3 an (bevor main() fertig ist).
 * Interrupt schreibt Frame 3 in L1_PDU und überschreibt damit Frame 2.
 * Das Flag (war schon 'true') bleibt 'true'.
 * 5. main() wird mit Frame 1 fertig, startet neu, sieht das Flag (von
 * Frame 2 & 3) und verarbeitet Frame 3.
 *
 * ERGEBNIS: Frame 2 wurde empfangen, aber von Frame 3 überschrieben,
 * bevor die main()-Schleife ihn abholen konnte. Er ist verloren.
 *
 * PROFESSIONELLE LÖSUNG (außerhalb des Scopes dieser Übung):
 * Man würde einen interrupt-sicheren Ringpuffer (FIFO) implementieren.
 * Der Interrupt (Produzent) legt neue Frames in den Puffer, solange Platz
 * ist. Die main()-Schleife (Konsument) holt sie in ihrem eigenen Tempo
 * nacheinander ab.
 *
 * FAZIT FÜR DIESE ÜBUNG:
 * Wir akzeptieren diese Einschränkung (Frame-Verlust bei hoher Last),
 * da der Fokus der Aufgabe auf der Implementierung des
 * OSI-Schichtenmodells (Kapselung/Dekapselung) liegt.
 * =========================================================================
 */

#include "stm32f4xx_hal.h"
#include <stdio.h>
#include "bsp.h"
#include "ws2812.h" // Für LED-Steuerung

// Dient als "Ampel": true = ROT (Sender beschäftigt), false = GRÜN (Sender frei)
volatile bool g_uart_tx_busy = false;
// Zähler für Taster
uint8_t cnt_button_press = 0;
// volatile ist wichtig, damit der Compiler weiß,
// dass sich die Variable jederzeit (im Interrupt) ändern kann.
volatile uint32_t g_last_press_time = 0;

// Flags für Nachbarn
volatile bool signal_north = false;
volatile bool signal_east = false;
volatile bool signal_south = false;
volatile bool signal_west = false;

// Implementierung Check (Prüft nur)
uint8_t lager_check(uint8_t pkg_id)
{
  // 1. Prüfen ob ID schon existiert
  for (int i = 0; i < 6; i++)
  {
    if (lager[i] == pkg_id)
      return MMCP_ERR_EXISTS;
  }
  // 2. Prüfen ob Lager voll
  int count = 0;
  for (int i = 0; i < 6; i++)
  {
    if (lager[i] != 0)
      count++;
  }
  if (count >= 6)
    return MMCP_ERR_FULL;

  return MMCP_SUCCESS;
}

// Implementierung Add (Fügt hinzu)
uint8_t lager_add(uint8_t pkg_id)
{
  // Erst prüfen (Wiederverwendung der Logik!)
  uint8_t check = lager_check(pkg_id);
  if (check != MMCP_SUCCESS)
    return check;

  // Dann hinzufügen
  for (int i = 0; i < 6; i++)
  {
    if (lager[i] == 0)
    {
      lager[i] = pkg_id;
      return MMCP_SUCCESS;
    }
  }
  return MMCP_ERR_FULL; // Fallback
}

/**
 * @brief  Wird aufgerufen, wenn UART-Fehler auftreten (Overrun, Noise, Frame Error).
 * WICHTIG: Ohne diese Funktion bleibt der UART nach einem Fehler einfach stehen!
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    // 1. Fehler-Flags löschen (passiert oft schon im Handler, aber sicher ist sicher)
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);

    // 2. Empfang sofort wieder neu starten!
    HAL_UART_Receive_IT(huart, L1_PDU_IT_BUFFER, L1_PDU_size);
  }
}

// Callback-Funktionen für Interrupts
/**
 * @brief  EXTI Line detection callback. Hier implementieren wir die Logik.
 * Diese Funktion wird von HAL_GPIO_EXTI_IRQHandler() aufgerufen.
 * ist als "weak" deklariert, d.h. wir überschreiben die Standard-Implementierung.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_13)
  {
    // 1. Hol die aktuelle Systemzeit (in Millisekunden)
    uint32_t current_time = HAL_GetTick();

    // 2. Prüfe, ob seit dem LETZTEN gültigen Klick
    //    genug Zeit vergangen ist (z.B. 200ms)
    if (current_time - g_last_press_time > 200)
    {
      // 3. Ja, es ist ein "echter" Klick, kein Prellen.
      cnt_button_press++;

      // 4. Merke dir die Zeit DIESES Klicks für die nächste Prüfung.
      g_last_press_time = current_time;
    }
  }
  // else:
  // Das war nur ein "Prell-Interrupt" innerhalb der 200ms.
  // Ignoriere ihn einfach.

  // --- NEU: Nachbar-Signale erfassen ---
  // Wir setzen nur Flags, die Logik passiert in der main()
  /*
  if (GPIO_Pin == GPIO_PIN_10)
    signal_north = true; // PC10 (Nord)
  if (GPIO_Pin == GPIO_PIN_11)
    signal_east = true; // PC11 (Ost)
  if (GPIO_Pin == GPIO_PIN_12)
    signal_south = true; // PC12 (Süd)
  if (GPIO_Pin == GPIO_PIN_2)
    signal_west = true; // PD2  (West)
    */
}

/**
 * @brief  Rx Transfer Cplt Callback: "Der Bissanzeiger" 🎣
 * * Diese Funktion wird von der Hardware (DMA/UART) *einmal* aufgerufen,
 * NACHDEM 16 Bytes (L1_PDU_size) im Hintergrund empfangen wurden.
 *
 * Das Programm "steht" NICHT während des Empfangs, die main_loop läuft frei weiter.
 *
 * Aufgaben hier (müssen MINIMAL sein, um die ISR-Zeit kurz zu halten):
 * 1. Flag setzen: Der main_loop signalisieren, dass ein "Fang" (Frame) im Puffer liegt.
 * 2. "Angel neu auswerfen": Den Empfang für den *nächsten* 16-Byte-Frame sofort
 * wieder "scharf schalten" (re-armen).
 *
 * Die eigentliche Verarbeitung des Frames (SOF/EOF-Check, L1_receive)
 * passiert sicher in der main_loop, ausgelöst durch das Flag.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  // Prüfen, ob der Interrupt von der richtigen UART-Instanz kam
  if (huart->Instance == USART2)
  {

    // 1. "Fang holen" (Daten in den Hauptpuffer kopieren)
    memcpy(L1_PDU, L1_PDU_IT_BUFFER, L1_PDU_size);

    // 2. Flag setzen ("Fang melden")
    frame_received_flag = true;

    // 3. "Angel neu auswerfen" (Nächsten Empfang starten)
    HAL_UART_Receive_IT(&huart2, L1_PDU_IT_BUFFER, L1_PDU_size);
  }
}

/**
 * @brief  Callback, der aufgerufen wird, wenn eine TX-Übertragung
 * (per Interrupt) abgeschlossen ist.
 * @param  huart: Zeiger auf die UART-Struktur
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  // Prüfen, ob es unsere UART war
  if (huart->Instance == USART2)
  {
    // Senden ist abgeschlossen.
    // "Ampel" wieder auf GRÜN schalten.
    // Der Sender ist jetzt frei für den nächsten L1_send-Aufruf.
    g_uart_tx_busy = false;
  }
}

// kopiert die Nutzdaten aus dem L1_PDU in den L1_SDU Puffer
void L1_receive(uint8_t L1_PDU[])
{
  for (int i = 1; i < L1_PDU_size - 1; i++)
  {
    L1_SDU[i - 1] = L1_PDU[i];
  }
  // 2. Kapselung: Die SDU von Layer 1 ist die PDU für Layer 2.
  // Wir geben das Paket direkt nach oben weiter.
  L2_receive(L1_SDU);
}

// kopiert die Nutzdaten aus dem L2_PDU in den L2_SDU Puffer und prüft die CRC
void L2_receive(uint8_t L2_PDU[])
{
  // CRC-Prüfsumme extrahieren (letztes Byte im L2_PDU)
  uint8_t received_crc = L2_PDU[L2_PDU_size - 1];

  // Berechne die CRC über das PDU aber nur bis zum letzten Byte (ohne die CRC selbst)
  uint8_t calculated_crc = l2_calculate_crc(L2_PDU, L2_SDU_size);

  // Vergleiche die berechnete CRC mit der empfangenen CRC
  if (calculated_crc == received_crc)
  {
    // CRC stimmt überein, kopiere die Nutzdaten in den L2_SDU Puffer
    for (int i = 0; i < L2_SDU_size; i++)
    {
      L2_SDU[i] = L2_PDU[i];
    }
    // 3. Kapselung: Weitergabe an Layer 3
    L3_receive(L2_SDU);
  }
  else
  {
    return; // CRC stimmt nicht überein, Daten sind fehlerhaft
  }
}

/*
 * =========================================================================
 * Anmerkung zum CRC-Polynom (z.B. 0xCF)
 * =========================================================================
 * Warum wird eine Hex-Zahl wie 0xCF als "Polynom" bezeichnet?
 *
 * 1. Kontext: Die CRC-Prüfsumme (Cyclic Redundancy Check) ist keine
 * einfache Summe, sondern das Ergebnis einer Polynomdivision
 * im Binärsystem (technisch: im Galois-Feld GF(2)).
 *
 * 2. Repräsentation: Die Zahl 0xCF ist nur eine Kurzschreibweise
 * für die Koeffizienten des "Generatorpolynoms", das bei
 * dieser Division als Divisor verwendet wird.
 *
 * 3. Umrechnung:
 * Hex:    0xCF
 * Binär:  1100 1111  (bei 8 Bit)
 *
 * 4. Als Polynom: Jedes Bit (von rechts nach links, beginnend bei 0)
 * entspricht einer Potenz von x. Eine '1' bedeutet, der Term
 * ist vorhanden, eine '0' bedeutet, er fehlt.
 *
 * Bit:    1     1     0     0     1     1     1     1
 * Potenz: x^7   x^6   x^5   x^4   x^3   x^2   x^1   x^0
 *
 * Das Polynom P(x) lautet also:
 * P(x) = 1*x^7 + 1*x^6 + 0*x^5 + 0*x^4 + 1*x^3 + 1*x^2 + 1*x^1 + 1*x^0
 *
 * P(x) = x^7 + x^6 + x^3 + x^2 + x + 1
 *
 * 5. Anwendung: Die CRC-Berechnung simuliert diese Division, wobei
 * die "Subtraktion" durch bitweise XOR-Operationen (^)
 * durchgeführt wird.
 * =========================================================================
 */
/*
 * =========================================================================
 * Funktion: l2_calculate_crc (CRC-8-Berechnung)
 * =========================================================================
 * ZIEL:
 * Diese Funktion generiert einen 8-Bit "Fingerabdruck" (Prüfsumme)
 * für einen gegebenen Datenblock (die L2-SDU). Der Empfänger kann
 * dieselbe Berechnung durchführen und seinen Fingerabdruck mit dem
 * empfangenen vergleichen, um Übertragungsfehler zu erkennen.
 *
 * PARAMETER (Beispiel):
 * - Polynom: 0xCF (binär 11001111)
 * - Initialwert: 0x00
 *
 * WAS PASSIERT HIER SCHRITT FÜR SCHRITT:
 *
 * 1. INITIALISIERUNG:
 * `uint8_t crc = 0x00;`
 * Wir starten mit einem 8-Bit "Register" (einer Variable), das
 * mit dem vorgegebenen Initialwert 0x00 geladen wird.
 *
 * 2. ÄUSSERE SCHLEIFE (pro Byte):
 * `for (int i = 0; i < laenge; i++)`
 * Wir verarbeiten die SDU Byte für Byte.
 *
 * 3. DATEN "EINKOPPELN":
 * `crc ^= data[i];`
 * Das ist ein entscheidender Schritt. Das nächste Byte der SDU
 * wird per XOR (^) direkt mit dem aktuellen CRC-Wert vermischt.
 * Man "faltet" das neue Byte in das Register ein.
 *
 * 4. INNERE SCHLEIFE (pro Bit):
 * `for (int j = 0; j < 8; j++)`
 * Jetzt wird die eigentliche Polynomdivision für die 8 Bits des
 * frisch eingekoppelten Bytes simuliert.
 *
 * 5. DIE DIVISIONS-LOGIK (DER KERN):
 *
 * a) PRÜFUNG DES HÖCHSTEN BITS (MSB):
 * `if ((crc & 0x80) != 0)`
 * Wir prüfen, ob das allerlinkeste Bit (das MSB) des
 * CRC-Registers eine 1 ist. (0x80 ist binär 1000 0000).
 * In der Polynom-Logik entspricht dies der Frage:
 * "Passt der Divisor (unser Polynom) unter den aktuellen Wert?"
 *
 * b) FALL 1: MSB ist 1 (Division/Subtraktion nötig)
 * `crc = (crc << 1) ^ 0xCF;`
 * - `crc << 1`: Zuerst schieben wir das Register 1 Bit nach
 * links. Das MSB (die 1) fällt raus, von rechts rückt eine
 * 0 nach.
 * - `^ 0xCF`: Weil das MSB eine 1 war, MÜSSEN wir jetzt das
 * Polynom (0xCF) per XOR anwenden. Das ist die "Subtraktion"
 * in der Polynomdivision.
 *
 * c) FALL 2: MSB ist 0 (Keine Subtraktion nötig)
 * `crc = crc << 1;`
 * - `crc << 1`: Das MSB war 0. Der Divisor "passt nicht".
 * Wir müssen nichts tun, außer das Register 1 Bit nach
 * links zu schieben (was einer Multiplikation mit x
 * entspricht) und eine 0 von rechts nachzuschieben.
 *
 * 6. ERGEBNIS:
 * `return crc;`
 * Nachdem alle Bytes der SDU (äußere Schleife) und alle 8 Bits
 * pro Byte (innere Schleife) verarbeitet wurden, enthält 'crc'
 * den finalen Rest der Polynomdivision. Das ist die Prüfsumme.
 * =========================================================================
 */
uint8_t l2_calculate_crc(uint8_t *data, int laenge)
{
  // 1. Init
  uint8_t crc = 0x00;

  // 2. Äußere Schleife (pro Byte)
  for (int i = 0; i < laenge; i++)
  {
    // 3. Aktuelles Byte "einkoppeln"
    crc ^= data[i];

    // 4. Innere Schleife (pro Bit)
    for (int j = 0; j < 8; j++)
    {
      // 5. Die CRC-Logik (Polynomdivision)
      if ((crc & 0x80) != 0) // Wenn das oberste Bit (MSB) 1 ist...
      {
        // Schieben UND Polynom anwenden
        crc = (crc << 1) ^ 0xCF;
      }
      else // Wenn das oberste Bit 0 ist...
      {
        // Nur schieben
        crc = crc << 1;
      }
    }
  }

  // 6. Fertig
  return crc;
}

/*
 * =========================================================================
 * DESIGN-ANMERKUNG zu L3_receive (Warum 'void' und nicht 'bool'?)
 * =========================================================================
 *
 * Im Gegensatz zu L1 und L2 (die "dumme" Prüfer sind und an 'main'
 * zurückmelden), ist L3 die erste "intelligente" Schicht.
 *
 * L3 agiert als "Verteiler" (Router/Dispatcher).
 * Die 'main'-Schleife übergibt das Paket an L3 und ihr Job im
 * Empfangspfad ist damit beendet. Sie muss nicht wissen, was L3 tut.
 *
 * 'L3_receive' kapselt die GESAMTE Logik, wohin die Daten fließen:
 *
 * 1. FALL "PAKET FÜR MICH" (z.B. to == myAddress && from == 0)
 * -> Aktion: L3 ruft L7_receive() direkt auf.
 *
 * 2. FALL "PAKET WEITERLEITEN" (z.B. to == 0)
 * -> Aktion: L3 ruft L2_send() direkt auf.
 *
 * 3. FALL "PAKET VERWERFEN" (z.B. vers != 7)
 * -> Aktion: L3 macht ein 'return' und das Paket "stirbt".
 *
 * WARUM DAS SAUBERES CODING IST (KAPSELUNG):
 * 'main' MUSS NICHTS von L3-Adressen oder Routing-Regeln wissen.
 * Würde L3 einen Status an 'main' zurückgeben, müsste 'main'
 * diese L3-Logik spiegeln, was die Schichtentrennung aufbricht.
 *
 * DIE "SCHEINBARE INKONSISTENZ" (Warum L3 L7 aufrufen darf):
 * In einem vollen 7-Schichten-Modell würde L3 (Netzwerk) nur
 * an L4 (Transport) übergeben.
 *
 * Da L4, L5 und L6 in unserem Protokoll fehlen, ist L7 die
 * *direkte* nächste Schicht.
 *
 * Unsere L3-Funktion ist daher eine Kombination aus:
 * - L3 (Adress-Prüfung/Routing)
 * - L4 (Verteilung/Demultiplexing an die richtige Anwendung,
 * von der es hier nur L7 gibt).
 * =========================================================================
 */
void L3_receive(uint8_t L3_PDU[])
{
  if (L3_PDU[2] != MMCP_VERSION || (L3_PDU[0] == 0 && L3_PDU[1] == 0))
  {
    return; // Version stimmt nicht überein oder Absender und Empfänger sind gleich Masteradresse, Paket verwerfen
  }
  else if (L3_PDU[0] == myAddress && L3_PDU[1] == 0)
  {
    for (int i = 0; i < L3_SDU_size; i++)
    {
      L3_SDU[i] = L3_PDU[i + 4]; // Nutzdaten extrahieren
    }
    L7_receive(L3_SDU); // Paket ist für mich, an L7 weiterleiten
  }
  // 3. Pakete ZUM WEITERLEITEN
  // Fall A: Antwort an Master (To: 0, From: 26)
  // Fall B: Befehl an anderes Board (To: 26, From: 0)
  else if (L3_PDU[0] == 0 || L3_PDU[1] == 0)
  {
    L3_PDU[3]++;     // Hops wird inkrementiert
    L2_send(L3_PDU); // Paket weiterleiten, an L2 senden
  }
  else
  {
    return; // Paket ist nicht für mich und nicht zum Weiterleiten, verwerfen
  }
}

/**
 * @brief Verarbeitet die empfangene L7-PDU (Anwendungsschicht-Dispatcher).
 *
 * Diese Funktion ist der Kern der Anwendungslogik (Layer 7). Sie wird
 * von L3_receive() aufgerufen, wenn ein Paket für dieses Board bestimmt ist.
 *
 * Sie liest die Application Process Number (APNR) aus L7_PDU[0] und
 * entscheidet in einer if-else-Kaskade, welche Aktion ausgeführt wird.
 *
 * Zuerst werden die empfangenen SDU-Daten (L7_PDU[1...x]) in den
 * globalen L7_SDU-Arbeitspuffer kopiert.
 *
 * Je nach APNR wird dieser Puffer dann:
 * - (100) Gelesen, um die LED zu steuern, und als Echo zurückgesendet.
 * - (101) Modifiziert (Tasterzähler in L7_SDU[7]) und zurückgesendet.
 * - (102/103) Komplett überschrieben (UID-Daten) und zurückgesendet.
 *
 * Für jeden empfangenen Befehl wird exakt ein Antwortpaket generiert.
 *
 * @param L7_PDU Zeiger auf das L7-PDU-Array (dieses Array ist die L3_SDU).
 * L7_PDU[0] = APNR (Befehlsnummer)
 * L7_PDU[1...x] = L7_SDU (Nutzdaten)
 */
void L7_receive(uint8_t L7_PDU[])
{
  for (int i = 0; i < L7_SDU_size; i++)
  {
    L7_SDU[i] = L7_PDU[i + 1]; // Nutzdaten extrahieren
  }

  uint8_t ap_nr = L7_PDU[0];

  // ---------------------------------------------------------
  // FALL 1: STATUSABFRAGE (AP 50)
  // ---------------------------------------------------------
  if (ap_nr == 50)
  {
    // Antwort vorbereiten (Daten lesen)
    L7_SDU[0] = current_state;

    if (current_state == failure)
    {
      L7_SDU[1] = last_error_code;
    }
    else
    {
      L7_SDU[1] = paketiB;
    }

    // Lagerbestand
    for (int i = 0; i < 6; i++)
      L7_SDU[i + 2] = lager[i];

    // Signal an Main: Status wurde gelesen (Handshake für Transiente Zustände)
    flag_ap50_was_read = true;

    // Antwort senden
    L7_send(ap_nr, L7_SDU);
  }

  // ---------------------------------------------------------
  // FALL 2: AWAIT / CREATE (AP 42)
  // ---------------------------------------------------------
  else if (ap_nr == 42)
  {
    uint8_t new_pkg_id = L7_SDU[0];
    uint8_t partner = L7_SDU[1];

    // Datenfluss: Werte bereitstellen
    paketiB = new_pkg_id;
    active_partner_id = partner;
    target_partner_id = PARTNER_NONE; // Reset, da AP42 kein Weiterleiten ist

    // Kontrollfluss: Event feuern
    if (partner == PARTNER_NONE)
    {
      flag_ap42_create = true; // Ereignis: E:ap42_create
    }
    else
    {
      flag_ap42_await = true; // Ereignis: E:ap42_await
    }

    // Echo senden (Ping-Pong)
    L7_send(ap_nr, L7_SDU);
  }

  // ---------------------------------------------------------
  // FALL 3: FORWARD (AP 43)
  // ---------------------------------------------------------
  else if (ap_nr == 43)
  {
    uint8_t pkg_id = L7_SDU[0];
    uint8_t from_id = L7_SDU[1];
    uint8_t to_id = L7_SDU[2];

    // Datenfluss
    paketiB = pkg_id;
    active_partner_id = from_id;
    target_partner_id = to_id;

    // Kontrollfluss
    flag_ap43_fwd = true; // Ereignis: E:ap43_fwd

    // Echo senden
    L7_send(ap_nr, L7_SDU);
  }

  // ---------------------------------------------------------
  // FALL 4: PASS ON / DELIVER (AP 44)
  // ---------------------------------------------------------
  else if (ap_nr == 44)
  {
    uint8_t pkg_id = L7_SDU[0];
    uint8_t partner = L7_SDU[1];

    // Datenfluss
    paketiB = pkg_id;
    target_partner_id = partner;
    // active_partner_id ist hier egal, wir senden ja

    // Kontrollfluss
    if (partner == PARTNER_NONE)
    {
      flag_ap44_deliver = true; // Ereignis: E:ap44_del
    }
    else
    {
      flag_ap44_pass = true; // Ereignis: E:ap44_pass
    }

    // Echo senden
    L7_send(ap_nr, L7_SDU);
  }
  /*
    // ---------------------------------------------------------
    // FALL 5: ALTE FUNKTIONEN (Optional behalten für Tests)
    // ---------------------------------------------------------

      // APNr 100
      if (L7_PDU[0] == 100)
      {
        // L7-Befehl ist "AN"
        if (L7_SDU[7] != 0x00)
        {
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        }
        else
        {
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        }
        L7_send(L7_PDU[0], L7_SDU);
      }
      else if (L7_PDU[0] == 101) // APNr 101
      {

        // Interrupts kurz deaktivieren, um den Zähler konsistent zu lesen und keine
        // Tastenanschläge zu verlieren bevor wir den Zähler zurücksetzen.
        __disable_irq();              // JETZT einfrieren
        L7_SDU[7] = cnt_button_press; // LESEN
        cnt_button_press = 0;         // RESET
        __enable_irq();               // SOFORT wieder auftauen

        // senden der Antwort mit der Anzahl der Tastenanschläge
        L7_send(L7_PDU[0], L7_SDU);
      }
      else if (L7_PDU[0] == 102)
      {
        // Hier kommt NUR die Logik für 102 rein
        // 1. UID holen
        uint32_t w0 = HAL_GetUIDw0();
        uint32_t w1 = HAL_GetUIDw1();

        // 2. SDU füllen mit Bitshifting und Maskieren, LSB zuerst unteren 64 bit
        L7_SDU[0] = (w0 >> 0) & 0xFF;
        L7_SDU[1] = (w0 >> 8) & 0xFF;
        L7_SDU[2] = (w0 >> 16) & 0xFF;
        L7_SDU[3] = (w0 >> 24) & 0xFF;
        L7_SDU[4] = (w1 >> 0) & 0xFF;
        L7_SDU[5] = (w1 >> 8) & 0xFF;
        L7_SDU[6] = (w1 >> 16) & 0xFF;
        L7_SDU[7] = (w1 >> 24) & 0xFF;

        // 3. Antwort senden
        L7_send(L7_PDU[0], L7_SDU);
      }
      else if (L7_PDU[0] == 103)
      {
        // Hier kommt NUR die Logik für 103 rein
        // 1. UID holen
        uint32_t w2 = HAL_GetUIDw2();

        // 2. SDU füllen mit oberen 32 bit der UID
        L7_SDU[0] = (w2 >> 0) & 0xFF;
        L7_SDU[1] = (w2 >> 8) & 0xFF;
        L7_SDU[2] = (w2 >> 16) & 0xFF;
        L7_SDU[3] = (w2 >> 24) & 0xFF;
        L7_SDU[4] = 0; // Rest optional auf 0 setzen
        L7_SDU[5] = 0;
        L7_SDU[6] = 0;
        L7_SDU[7] = 0;

        // 3. Antwort senden
        L7_send(L7_PDU[0], L7_SDU);
      }
        */
}

void L7_send(uint8_t ApNr, uint8_t L7_SDU[])
{
  // L7_PDU zusammenbauen

  L7_PDU[0] = ApNr; // ApNr setzen
  for (int i = 0; i < L7_SDU_size; i++)
  {
    L7_PDU[i + 1] = L7_SDU[i]; // Nutzdaten kopieren
  }
  L3_send(L7_PDU); // an L3 weiterleiten
}

void L3_send(uint8_t L3_SDU[])
{
  // L3_PDU zusammenbauen

  L3_PDU[0] = 0;            // to, standardmäßig an Master
  L3_PDU[1] = myAddress;    // from, meine Adresse
  L3_PDU[2] = MMCP_VERSION; // version
  L3_PDU[3] = 0;            // hops, wird initial auf 0 gesetzt, weil das Paket neu erstellt wird

  for (int i = 0; i < L3_SDU_size; i++)
  {
    L3_PDU[i + 4] = L3_SDU[i]; // Nutzdaten kopieren
  }

  L2_send(L3_PDU); // an L2 weiterleiten
}

void L2_send(uint8_t L2_SDU[])
{
  // L2_PDU zusammenbauen

  for (int i = 0; i < L2_SDU_size; i++)
  {
    L2_PDU[i] = L2_SDU[i]; // Nutzdaten kopieren
  }

  // CRC berechnen und anhängen
  uint8_t crc = l2_calculate_crc(L2_PDU, L2_SDU_size);
  L2_PDU[L2_PDU_size - 1] = crc; // CRC am Ende anhängen

  L1_send(L2_PDU); // an L1 weiterleiten
}

void L1_send(uint8_t L1_SDU[])
{
  // 1. "Ampel"-Prüfung: Ist der Sender noch beschäftigt?
  //    Warte, BIS die Ampel GRÜN (false) ist.
  //    (Das ist eine blockierende Warteschleife)
  while (g_uart_tx_busy)
  {
    // Warte... (Der UART TX-Interrupt wird g_uart_tx_busy
    // von selbst auf 'false' setzen, wenn er fertig ist)
  }

  // 2. Ampel auf ROT schalten
  // Wir sind jetzt an der Reihe, Ampel wird rot.
  g_uart_tx_busy = true;

  // 3. L1_PDU zusammenbauen (dein Code)
  L1_PDU[0] = 0x00; // SOF
  for (int i = 0; i < L1_SDU_size; i++)
  {
    L1_PDU[i + 1] = L1_SDU[i];
  }
  L1_PDU[L1_PDU_size - 1] = 0x00; // EOF

  // 4. Senden über UART (NON-BLOCKING)
  //    (Die TxCpltCallback wird g_uart_tx_busy
  //     nach 1.4ms wieder auf false setzen)
  HAL_UART_Transmit_IT(&huart2, L1_PDU, L1_PDU_size);
}