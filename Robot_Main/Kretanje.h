/**
 * Kretanje.h
 * 
 * Modul za napredno upravljanje kretanjem.
 * Sadrži PID regulator za praćenje linije i NON-BLOCKING funkcije za manevre.
 */

#ifndef KRETANJE_H
#define KRETANJE_H

#include <Arduino.h>

// Stanja Kretanja
enum StanjeKretanja {
    KRETANJE_IDLE,
    KRETANJE_RAVNO,
    KRETANJE_ROTACIJA,
    KRETANJE_LINIJA
};

// PID Parametri - dostupni za tuniranje
extern float Kp;
extern float Ki;
extern float Kd;
extern int baznaBrzina;

/**
 * Postavlja kalibracijske parametre.
 */
void postaviKonfigKretanja(float impulsaPoCm);

/**
 * Glavna funkcija koja se poziva u loopu.
 * Upravlja PID-om i provjerava ciljeve za vožnju/rotaciju.
 */
void azurirajKretanje();

/**
 * Postavlja robota da slijedi liniju (aktivira PID).
 * Da bi se zaustavilo, pozvati stani() ili zapocniVoznju().
 */
void pokreniPracenjeLinije();

/**
 * Započinje vožnju ravno za zadanu udaljenost (NON-BLOCKING).
 * Robot ce voziti dok ne dosegne cilj, a azurirajKretanje() ce se brinuti o tome.
 * @param cm Udaljenost u centimetrima.
 */
void zapocniVoznju(float cm);

/**
 * Započinje rotaciju za zadani kut (NON-BLOCKING).
 * @param stupnjevi Pozitivno = Desno, Negativno = Lijevo.
 */
void zapocniRotaciju(float stupnjevi);

/**
 * Zaustavlja robota i resetira stanje u IDLE.
 */
void zaustaviKretanje();

float dohvatiPredjeniPutCm();

/**
 * Provjera da li je robot trenutno u nekom manevru.
 * @return true ako se kreće/rotira/prati liniju.
 */
bool jeUPokretu();

#endif // KRETANJE_H
