/**
 * SenzoriLinije.h
 * 
 * Modul za čitanje 5-kanalnog IR senzora za praćenje linije.
 */

#ifndef SENZORILINIJE_H
#define SENZORILINIJE_H

#include <Arduino.h>

/**
 * Čita vrijednosti senzora i računa poziciju linije.
 * @return float vrijednost od -6.0 (skroz lijevo) do 6.0 (skroz desno).
 * 0.0 znači da je linija u centru.
 */
float dohvatiPozicijuLinije();

/**
 * Provjerava nalazi li se senzor na crnoj podlozi ili je detektirao T-raskrižje/kraj.
 * @return true ako je većina senzora na crnom.
 */
bool naCrnojPodlozi();

// Funkcija za debudiranje ispisom na Serial
void debugSenzoriLinije();

#endif // SENZORILINIJE_H
