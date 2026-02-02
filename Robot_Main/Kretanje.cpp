/**
 * Kretanje.cpp
 * 
 * Implementacija Non-Blocking State Machine za kretanje.
 */

#include "Kretanje.h"
#include "HardwareMap.h"
#include "Motori.h"
#include "SenzoriLinije.h"
#include "Enkoderi.h"
#include "IMU.h"

// --- PID Parametri ---
float Kp = 35.0;
float Ki = 0.0;
float Kd = 15.0;
int baznaBrzina = 100;

float proslaGreska = 0;
float integral = 0;

// --- State Machine Varijable ---
StanjeKretanja trenutnoStanjeKretanja = KRETANJE_IDLE;
float ciljnaVrijednost = 0; // CM ili Stupnjevi
float pocetnaOrijentacija = 0;
long pocetniEncL = 0;
long pocetniEncR = 0;

// Kalibracija (treba precizno izmjeriti)
float IMPULSA_PO_CM = 40.0; 

void postaviKonfigKretanja(float impulsaPoCm) {
    IMPULSA_PO_CM = impulsaPoCm;
}

// Pomoćna za normalizaciju kuta
float normalizirajKut(float kut) {
    while (kut > 180) kut -= 360;
    while (kut < -180) kut += 360;
    return kut;
}

void zapocniVoznju(float cm) {
    resetirajEnkodere();
    azurirajIMU();
    
    pocetnaOrijentacija = dohvatiYaw();
    pocetniEncL = 0; // Resetirano
    pocetniEncR = 0; // Resetirano
    
    ciljnaVrijednost = cm * IMPULSA_PO_CM;
    
    trenutnoStanjeKretanja = KRETANJE_RAVNO;
}

void zapocniRotaciju(float stupnjevi) {
    stani();
    azurirajIMU(); // Osvježi stanje
    delay(10);     // Kratka pauza za stabilizaciju
    azurirajIMU();
    
    pocetnaOrijentacija = dohvatiYaw();
    ciljnaVrijednost = pocetnaOrijentacija + stupnjevi;
    
    // Normalizacija cilja u 0-360 range (dohvatiYaw vraca obicno 0-360 ili -180/180 ovisno o libu, ovdje pretpostavljamo dohvatiYaw vraca 0-360 iz IMU.cpp koda sto sam vidio prije, al IMU libovi cesto saraju. Prilagodit cemo se wrap-aroundu)
    
    // Ako zelimo rotirati za +90, a trenutno je 350, cilj je 440 -> 80.
    
    trenutnoStanjeKretanja = KRETANJE_ROTACIJA;
}

void pokreniPracenjeLinije() {
    trenutnoStanjeKretanja = KRETANJE_LINIJA;
    proslaGreska = 0;
    integral = 0;
}

void zaustaviKretanje() {
    trenutnoStanjeKretanja = KRETANJE_IDLE;
    stani();
}

float dohvatiPredjeniPutCm() {
    long avgEnc = (abs(dohvatiLijeviEnkoder()) + abs(dohvatiDesniEnkoder())) / 2;
    if (IMPULSA_PO_CM == 0) return 0;
    return (float)avgEnc / IMPULSA_PO_CM;
}

bool jeUPokretu() {
    return trenutnoStanjeKretanja != KRETANJE_IDLE;
}

void azurirajKretanje() {
    if (trenutnoStanjeKretanja == KRETANJE_IDLE) {
        return;
    }

    if (trenutnoStanjeKretanja == KRETANJE_LINIJA) {
        // --- LOGIKA ZA LINIJU ---
        float pozicija = dohvatiPozicijuLinije();
        
        // Ako nema linije (pozicija -3 npr), mozda stani? 
        // Za sad samo nastavi zadnjom logikom ili stani.
        
        float greska = 0.0 - pozicija;
        integral += greska;
        float derivacija = greska - proslaGreska;
        float izlaz = (Kp * greska) + (Ki * integral) + (Kd * derivacija);
        proslaGreska = greska;
        
        int brzinaLijevi = baznaBrzina - izlaz;
        int brzinaDesni = baznaBrzina + izlaz;
        
        brzinaLijevi = constrain(brzinaLijevi, -255, 255);
        brzinaDesni = constrain(brzinaDesni, -255, 255);
        
        lijeviMotor(brzinaLijevi);
        desniMotor(brzinaDesni);
    }
    else if (trenutnoStanjeKretanja == KRETANJE_RAVNO) {
        // --- LOGIKA ZA RAVNO ---
        long l = abs(dohvatiLijeviEnkoder());
        long r = abs(dohvatiDesniEnkoder());
        
        // Provjera kraja
        if (l >= abs(ciljnaVrijednost) || r >= abs(ciljnaVrijednost)) {
            zaustaviKretanje();
            return;
        }
        
        // Korekcija smjera
        azurirajIMU();
        float trenutniYaw = dohvatiYaw();
        float greskaSmjera = normalizirajKut(trenutniYaw - pocetnaOrijentacija);
        
        float Kp_smjer = 5.0;
        int korekcija = greskaSmjera * Kp_smjer;
        
        // Ako idemo u rikverc (cilj < 0), logika motora se mjenja, ali ovdje pretpostavimo forward ako je cm > 0
        // Za sad supportamo samo naprijed s ovim kodom, za rikverc bi trebalo paziti na znak bazneBrzine
        
        lijeviMotor(baznaBrzina + korekcija);
        desniMotor(baznaBrzina - korekcija);
    }
    else if (trenutnoStanjeKretanja == KRETANJE_ROTACIJA) {
        // --- LOGIKA ZA ROTACIJU ---
        azurirajIMU();
        float trenutniYaw = dohvatiYaw();
        
        // Izračunaj diff do cilja uzimajući u obzir wrap-around
        float diff = ciljnaVrijednost - trenutniYaw;
        // Normaliziraj diff na -180 do 180 (najkraći put)
        diff = normalizirajKut(diff);
        
        // Tolerancija
        if (abs(diff) < 2.0) {
            zaustaviKretanje();
            return;
        }
        
        int brzinaOkreta = 120;
        // Povećaj brzinu ako je daleko, smanji ako je blizu (P regulator za okret)
        // Ali pazi na minimalnu brzinu da motori ne stanu
        
        if (diff > 0) {
             // Treba ic desno (povecati kut)
             lijeviMotor(brzinaOkreta);
             desniMotor(-brzinaOkreta);
        } else {
             // Treba ic lijevo (smanjiti kut)
             lijeviMotor(-brzinaOkreta);
             desniMotor(brzinaOkreta);
        }
    }

    // --- LANE ASSIST (Sigurnosni Override) ---
    // Provjeravamo rubove staze. Ako detektiraju crno (rub), preuzimaju kontrolu.
    // Pretpostavka: LOW = Detekcija (ovisno o senzoru, možda treba HIGH)
    // Ako oba vide, možda smo na raskrižju ili u zraku - ignoriraj ili stani?
    // Ovdje prioritet ima onaj koji prvi vidi ili logika "odbijanja".
    
    // Čitamo senzore
    bool rubLijevo = digitalRead(PIN_IR_LIJEVI); // 1 ili 0
    bool rubDesno = digitalRead(PIN_IR_DESNI);
    
    // Prilagoditi logiku (npr. ako je HIGH crno)
    // Standardni IR senzori često daju LOW kad vide prepreku/liniju (refleksija vs upijanje).
    // Za crnu liniju na bijeloj podlozi: Crno upija -> Nema refleksije -> Output ? 
    // Obično: Bijelo (reflektira) -> LOW / Crno (ne reflektira) -> HIGH.
    // ALI ovisi o modulu. 
    // Ovdje pretpostavljamo: HIGH = CRNO (RUB).
    
    if (rubLijevo == HIGH) {
        // Desni se aktivirao - vidio rub na desnoj strani? Ne, IR_LIJEVI je lijevi senzor.
        // Ako LIJEVI senzor vidi rub, znači da idemo previše lijevo -> SKRENI DESNO.
        lijeviMotor(150);
        desniMotor(-150); // Oštro desno
    }
    else if (rubDesno == HIGH) {
        // Desni senzor vidi rub -> SKRENI LIJEVO.
        lijeviMotor(-150);
        desniMotor(150); // Oštro lijevo
    }
}
