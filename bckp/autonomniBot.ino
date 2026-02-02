/**
 * autonomniBot.ino
 * 
 * Glavna datoteka za WorldSkills Croatia 2026 Autonomni Robot.
 * Integrira sve module: Kretanje, Vision, Telemetriju, Senzore.
 */

#include "HardwareMap.h"
#include "Kretanje.h"
#include "Telemetrija.h"
#include "Vision.h"
#include "Ultrazvuk.h"
#include "SenzorBoje.h"
#include "IMU.h"
#include "SenzoriLinije.h"
#include "Motori.h" // Za stani()
#include "Enkoderi.h"

// --- Stanja Robota ---
enum Stanje {
    IDLE,
    VOZNJA_PRAVOCRTNO,
    PRACENJE_LINIJE,
    STOP_PREPREKA,
    TEST_KVADRAT
};

Stanje trenutnoStanje = IDLE;

// --- Tajmeri ---
unsigned long zadnjaProvjeraPrepreke = 0;
const int INTERVAL_PREPREKA = 50; // 20Hz

// --- Globalne Varijable ---
float trenutniYaw = 0.0;
long encL = 0;
long encR = 0;
long stariEncL = 0;
long stariEncR = 0;
unsigned long zadnjeVrijemeBrzine = 0;
float trenutnaBrzina = 0.0;

// Kalibracija: koliko metara po pulsu?
// Npr. promjer kotača D = 0.06m -> Opseg = 0.1885m
// Enkoder rezolucija N = 360 puls/okr (hipotetski)
// M_PER_PULSE = 0.1885 / 360 = 0.000523
const float METARA_PO_PULSU = 0.000523; 

void setup() {
    // 1. Inicijalizacija Hardvera
    inicijalizirajHardware();
    
    // 2. Inicijalizacija Senzora
    inicijalizirajIMU();
    inicijalizirajSenzorBoje();
    inicijalizirajEnkodere();
    
    // 3. Početno Stanje
    stani();
    trenutnoStanje = IDLE;
    
    Serial.println("Sustav Spreman (JSON Protocol).");
}

void loop() {
    // --- 1. Čitanje Senzora i Komunikacija ---
    azurirajVision(); 
    azurirajIMU();
    trenutniYaw = dohvatiYaw();
    
    encL = dohvatiLijeviEnkoder();
    encR = dohvatiDesniEnkoder();
    
    // Izračun brzine (svakih 100ms isto kao telemetrija)
    if (millis() - zadnjeVrijemeBrzine >= 100) {
        long deltaL = encL - stariEncL;
        long deltaR = encR - stariEncR;
        float prosjekDelta = (deltaL + deltaR) / 2.0;
        
        // v = ds / dt -> dt = 0.1s
        float distM = prosjekDelta * METARA_PO_PULSU;
        trenutnaBrzina = distM / 0.1; // m/s
        stariEncL = encL;
        stariEncR = encR;
        zadnjeVrijemeBrzine = millis();
    }
    
    // --- 2. State Machine Update ---
    azurirajKretanje();
    
    // Sigurnost (samo ako smo u pokretu)
    if (millis() - zadnjaProvjeraPrepreke > INTERVAL_PREPREKA) {
        long dist = udaljenost(SMJER_NAPRIJED);
        zadnjaProvjeraPrepreke = millis();
        // Limit 15cm
        if (dist > 0 && dist < 15 && jeUPokretu()) {
             zaustaviKretanje();
             trenutnoStanje = STOP_PREPREKA;
        }
    }
    
    provjeriKomandePC();
    
    // --- 3. Telemetrija (JSON) ---
    posaljiTelemetrijuJSON(encL, encR, trenutnaBrzina, trenutniYaw);
}

void provjeriKomandePC() {
    if (Serial2.available()) {
        String cmdString = Serial2.readStringUntil('\n');
        cmdString.trim();
        
        if (cmdString.length() > 0) {
            String cmd = "";
            int val = 0;
            
            // "Hack" parser
            int cmdIdx = cmdString.indexOf("\"cmd\":");
            if (cmdIdx > 0) {
                int startQuote = cmdString.indexOf("\"", cmdIdx + 7);
                int endQuote = cmdString.indexOf("\"", startQuote + 1);
                cmd = cmdString.substring(startQuote + 1, endQuote);
            }
            
            int valIdx = cmdString.indexOf("\"val\":");
            if (valIdx > 0) {
                int endVal = cmdString.indexOf("}", valIdx);
                if (endVal < 0) endVal = cmdString.length();
                String valStr = cmdString.substring(valIdx + 6, endVal);
                val = valStr.toInt();
            }
            
            // Izvrši
            if (cmd == "stop") {
                zaustaviKretanje();
            }
            else if (cmd == "square") {
                // Za square test bi nam trebao slozeniji sekvencer (QUEUE), 
                // ali za sad disableamo posto je blocking logika maknuta.
                // Alternativa: Setaj flag "test_seq_step = 0" i u loopu prelazi.
                Serial2.println("{\"msg\": \"Square test not implemented in non-blocking mode yet.\"}");
            }
            else if (cmd == "straight") {
                // val / 100 jer je vjerojatno u nekim jedinicama, prilagoditi
                zapocniVoznju((float)val); 
            }
            else if (cmd == "turn") {
                zapocniRotaciju((float)val);
            }
        }
    }
}
