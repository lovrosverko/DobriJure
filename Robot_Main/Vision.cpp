/**
 * Vision.cpp
 * 
 * Implementacija komunikacije s kamerom.
 */

#include "Vision.h"
#include "HardwareMap.h"

String zadnjiQR = "";
String ulazniBuffer = "";

void azurirajVision() {
    while (Serial3.available()) {
        char c = (char)Serial3.read();
        
        if (c == '\n') {
            // Kraj poruke, parsiraj
            ulazniBuffer.trim(); // Ukloni whitespace
            if (ulazniBuffer.startsWith("QR:")) {
                zadnjiQR = ulazniBuffer.substring(3); // Preskoči "QR:"
                Serial.print("VISION: Novi QR -> ");
                Serial.println(zadnjiQR);
            }
            ulazniBuffer = ""; // Reset buffera
        } else {
            ulazniBuffer += c;
        }
    }
}

String dohvatiZadnjiQR() {
    return zadnjiQR;
}

void obrisiZadnjiQR() {
    zadnjiQR = "";
}
