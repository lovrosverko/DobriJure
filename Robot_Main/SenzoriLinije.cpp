/**
 * SenzoriLinije.cpp
 * 
 * Implementacija logike čitanja linije (težinska sredina).
 */

#include "SenzoriLinije.h"
#include "HardwareMap.h"

// Prag za detekciju linije (analogno očitanje 0-1023)
// Prilagoditi ovisno o podlozi (Bijelo je obično nisko < 200, Crno visoko > 700)
#define PRAG_CRNO 600

// Težine senzora: LL(-6), L(-3), C(0), R(3), RR(6)
const float TEZINE[5] = {-6.0, -3.0, 0.0, 3.0, 6.0};

float dohvatiPozicijuLinije() {
    int vrijednosti[5];
    vrijednosti[0] = analogRead(PIN_SENS_LINE_LL);
    vrijednosti[1] = analogRead(PIN_SENS_LINE_L);
    vrijednosti[2] = analogRead(PIN_SENS_LINE_C);
    vrijednosti[3] = analogRead(PIN_SENS_LINE_R);
    vrijednosti[4] = analogRead(PIN_SENS_LINE_RR);

    long sumTezina = 0;
    int sumVrijednosti = 0;
    int brojDetektiranih = 0;

    for (int i = 0; i < 5; i++) {
        // Invertiramo logiku ako je crna linija na bijeloj podlozi
        // Pretpostavka: Veća vrijednost = tamnije (linija)
        // Ako senzor daje manju vrijednost za crno, koristiti (1023 - očitanje).
        
        // Uzimamo samo vrijednosti koje su "vjerojatno linija" za čišći izračun
        if (vrijednosti[i] > PRAG_CRNO) {
            sumTezina += (long)(vrijednosti[i] * TEZINE[i]);
            sumVrijednosti += vrijednosti[i];
            brojDetektiranih++;
        }
    }

    if (sumVrijednosti == 0) {
        // Izgubljena linija - vratiti zadnju poznatu ili neku indikaciju
        // Ovdje vraćamo 0 ali bi logika trebala znati da nema linije.
        // Mogli bismo koristiti zadnju poznatu stranu.
        return 0.0; 
    }

    return (float)sumTezina / sumVrijednosti;
}

bool naCrnojPodlozi() {
    int detectedCount = 0;
    if (analogRead(PIN_SENS_LINE_LL) > PRAG_CRNO) detectedCount++;
    if (analogRead(PIN_SENS_LINE_L) > PRAG_CRNO) detectedCount++;
    if (analogRead(PIN_SENS_LINE_C) > PRAG_CRNO) detectedCount++;
    if (analogRead(PIN_SENS_LINE_R) > PRAG_CRNO) detectedCount++;
    if (analogRead(PIN_SENS_LINE_RR) > PRAG_CRNO) detectedCount++;

    return (detectedCount >= 3); // Ako su 3 ili više senzora na crnom
}

void debugSenzoriLinije() {
    Serial.print("L:"); Serial.print(analogRead(PIN_SENS_LINE_LL));
    Serial.print(" "); Serial.print(analogRead(PIN_SENS_LINE_L));
    Serial.print(" "); Serial.print(analogRead(PIN_SENS_LINE_C));
    Serial.print(" "); Serial.print(analogRead(PIN_SENS_LINE_R));
    Serial.print(" "); Serial.println(analogRead(PIN_SENS_LINE_RR));
}
