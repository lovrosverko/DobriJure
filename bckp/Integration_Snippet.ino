/**
 * Integration_Snippet.ino
 * 
 * Ovaj isječak koda pokazuje kako integrirati Manipulator modul u glavni program.
 * 
 * 1. Dodati include: #include "Manipulator.h"
 * 2. Deklarirati objekt: Manipulator manipulator;
 * 3. U setup() inicijalizirati (hardver se već inicijalizira u HardwareMap).
 * 4. U loop() zvati manipulator.azuriraj().
 * 5. U provjeriKomandePC() dodati obradu za "ruka".
 */

#include "Manipulator.h"

// Globalni objekt
Manipulator manipulator;

void setup() {
    // ... postojeća inicijalizacija ...
    inicijalizirajHardware(); // Ovo sada pokreće i PWM driver
    
    // Inicijalno postavljanje ruke (ako je potrebno dodatno)
    // manipulator se konstruktorom postavlja u MIRUJE
}

void loop() {
    // ... postojeći kod ...

    // Ažuriranje manipulatora (Soft-Start i State Machine)
    manipulator.azuriraj();

    // ... postojeći kod ...
}

// Primjer parsiranja JSON komande za ruku
// Očekuje JSON format: {"cmd": "ruka", "val": "boca"}
void provjeriKomandePC_Snippet() {
    if (Serial2.available()) {
        String cmdString = Serial2.readStringUntil('\n');
        cmdString.trim();

        if (cmdString.length() > 0) {
            String cmd = "";
            String valString = ""; 

            // Parsiranje "cmd"
            int cmdIdx = cmdString.indexOf("\"cmd\":");
            if (cmdIdx > 0) {
                int startQuote = cmdString.indexOf("\"", cmdIdx + 7);
                int endQuote = cmdString.indexOf("\"", startQuote + 1);
                cmd = cmdString.substring(startQuote + 1, endQuote);
            }

            // Parsiranje "val" (kao string za "boca"/"limenka")
            int valIdx = cmdString.indexOf("\"val\":");
            if (valIdx > 0) {
                // Provjera je li vrijednost u navodnicima (string)
                int startQuote = cmdString.indexOf("\"", valIdx + 6);
                if (startQuote != -1) {
                    int endQuote = cmdString.indexOf("\"", startQuote + 1);
                    valString = cmdString.substring(startQuote + 1, endQuote);
                } else {
                    // Ako nije string, možda je broj (za druge komande)
                    int endVal = cmdString.indexOf("}", valIdx);
                    if (endVal < 0) endVal = cmdString.length();
                    valString = cmdString.substring(valIdx + 6, endVal);
                    // Očisti razmake
                    valString.trim();
                }
            }

            // -- Logika Komandi --
            if (cmd == "ruka") {
                // Pokreni sekvencu spremanja
                manipulator.zapocniSpremanje(valString);
                Serial2.println("{\"status\": \"Spremanje zapoceto\", \"predmet\": \"" + valString + "\"}");
            }
            // ... ostale komande ...
        }
    }
}
