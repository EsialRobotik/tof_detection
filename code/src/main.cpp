#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <VL53L1X.h>
#include <PCA95x5.h>

// Fortement copié/collé/inspiré de https://registry.platformio.org/libraries/pololu/VL53L1X/examples/Continuous/Continuous.ino

// Led embarquée sur la carte de développement ESP32 C3 ZERO
#define LED_PIN 6
#define LED_COUNT 12
#define BRIGHTNESS 32
#define COLOR_ORDER GRB
#define I2C_MUX_ADDR 0x20

struct TOF {
    VL53L1X tof;
    uint8_t i2c_address;
    uint8_t led_idx;
    TwoWire *i2c_bus;
    PCA95x5::Port::Port xshut_port;
    bool init_ok;
    uint16_t last_distance;
};

PCA9535 i2c_mux;
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

TwoWire i2c_a; // A4 (Neucleo SDA_A broche P5A) / A5 (Nucleo SCL_A broche PA6)
TwoWire i2c_b(9, 1); // D9 (Nucleo SDA_B broche PA8) / D1 (Nucleo SCL_B broche PA9)

TOF tofs_pool[] = {
    {VL53L1X(), 0x31, 0, &i2c_a, PCA95x5::Port::P00, false, 0},
    {VL53L1X(), 0x33, 1, &i2c_a, PCA95x5::Port::P01, false, 0},
    {VL53L1X(), 0x35, 2, &i2c_a, PCA95x5::Port::P02, false, 0},
    {VL53L1X(), 0x37, 3, &i2c_a, PCA95x5::Port::P03, false, 0},
    {VL53L1X(), 0x39, 4, &i2c_a, PCA95x5::Port::P04, false, 0},
    {VL53L1X(), 0x3b, 5, &i2c_a, PCA95x5::Port::P05, false, 0},
    {VL53L1X(), 0x3d, 6, &i2c_b, PCA95x5::Port::P06, false, 0},
    {VL53L1X(), 0x3f, 7, &i2c_b, PCA95x5::Port::P07, false, 0},
    {VL53L1X(), 0x41, 8, &i2c_b, PCA95x5::Port::P10, false, 0},
    {VL53L1X(), 0x43, 9, &i2c_b, PCA95x5::Port::P11, false, 0},
    {VL53L1X(), 0x45, 10, &i2c_b, PCA95x5::Port::P12, false, 0},
    {VL53L1X(), 0x47, 11, &i2c_b, PCA95x5::Port::P13, false, 0},
};
int TOF_COUNT = sizeof(tofs_pool)/sizeof(TOF);

unsigned long lastHeartBeatTime = 0; // timestamp qui marque la dernière pulsation du heartbeat

void initializeI2C() {
    i2c_a.begin();
    i2c_a.setClock(400000); // 400 kHz I2C
    i2c_a.setTimeout(50);
    i2c_b.begin();
    i2c_b.setClock(400000);
    i2c_b.setTimeout(50);
}

void initializeI2cMux() {
    i2c_mux.attach(i2c_a, I2C_MUX_ADDR);
    // Les ports listés dans la config des tofs sont des sorties
    for (int i=0; i<TOF_COUNT; i++) {
        i2c_mux.direction(tofs_pool[i].xshut_port, PCA95x5::Direction::OUT);
        i2c_mux.write(tofs_pool[i].xshut_port, PCA95x5::Level::H); // xshut flottant 
    }
}

void initializeSensors() {
    // extinction de tous les tofs
    Serial.println("# disable all tofs");
    for (int i=0; i<TOF_COUNT; i++) {
        i2c_mux.write(tofs_pool[i].xshut_port, PCA95x5::Level::L); // xshut à la masse 
    }

    // Allumage des tofs les uns après les autres pour régler leur adresse
    for (int i=0; i<TOF_COUNT; i++) {
        Serial.print("# init tof ");
        Serial.print(i);
        Serial.print(" addr 0x");
        Serial.print(tofs_pool[i].i2c_address, HEX);
        Serial.print("...");
        i2c_mux.write(tofs_pool[i].xshut_port, PCA95x5::Level::H); // xshut flottant pour réveiller le tof
        delay(5); // boot du tof
        tofs_pool[i].tof.setBus(tofs_pool[i].i2c_bus);
        tofs_pool[i].tof.setTimeout(50);
        tofs_pool[i].tof.setAddress(0x29); // Adresse par défaut du TOF après boot

        strip.setPixelColor(tofs_pool[i].led_idx, 0, 0, 255);
        strip.show();

        int retry_count = 3;
        while (!tofs_pool[i].tof.init() && retry_count > 0) {
            Serial.print(".");
            Serial.flush();
            retry_count--;
        }

        // Le tof a répondu à l'initialisation
        if (retry_count > 0) {
            tofs_pool[i].tof.setAddress(tofs_pool[i].i2c_address);
            tofs_pool[i].tof.setDistanceMode(VL53L1X::Short);
            tofs_pool[i].tof.setMeasurementTimingBudget(49000);
            tofs_pool[i].tof.startContinuous(50); // 1 mesure toutes les 50ms
            tofs_pool[i].init_ok = true;
            Serial.println(" ok");
            strip.setPixelColor(tofs_pool[i].led_idx, 0, 255, 0);
            strip.show();
        // Le tof n'a pas répondu
        } else {
            tofs_pool[i].init_ok = false;
            i2c_mux.write(tofs_pool[i].xshut_port, PCA95x5::Level::L); // remise de xshut à la masse pour éviter un éventuel conflit d'adresse avec un TOF mal initialisé
            Serial.println(" err");
        }
    }
}

void initializeLedStrip() 
{
    strip.begin();
    strip.setBrightness(BRIGHTNESS);
    strip.show();
}

void mapSensorsToLeds() {
    for (int i=0; i<TOF_COUNT; i++) {
        uint16_t mes = tofs_pool[i].last_distance;
        // Mise à jour de la couleur des LEDs : plus le ToF capt loin plus c'est vert, plus il capte près, plus c'est rouge
        if (mes > 1500) {
            strip.setPixelColor(tofs_pool[i].led_idx, strip.Color(0, 0, 0));
        } else {
            if (mes > 1000) {
                mes = 1000;
            }
            strip.setPixelColor(tofs_pool[i].led_idx, strip.Color(map(mes, 1000, 0, 0, 255), map(mes, 0, 1000, 0, 255), 0));
        }
    }
    strip.show();
}

void sensorValuesToSerial() {
    for (int i=0; i<TOF_COUNT; i++) {
        if (i > 0) {
            Serial.print(';');
        }
        if (tofs_pool[i].tof.dataReady()) {
            uint16_t distance = tofs_pool[i].tof.read();
            Serial.print(distance);
            tofs_pool[i].last_distance = distance;
        } else {
            Serial.print("n/a");
        }
    }
    Serial.println();
    Serial.flush();
}

void setup() {
    Serial.begin(115200);
    Serial.print("#Initialize led strip...");
    initializeLedStrip();
    Serial.println(" done");
    Serial.print("#Initialize I2C A and B...");
    Serial.flush();
    initializeI2C();
    Serial.println(" done");
    Serial.print("#Initialize i2c PCA9535 mux...");
    Serial.flush();
    initializeI2cMux();
    Serial.println(" done");
    Serial.print("#Initialize ");
    Serial.print(TOF_COUNT);
    Serial.println(" ToF sensors...");
    initializeSensors();
    Serial.println("#done");
}

void loop() {
    if (millis() + 150 > lastHeartBeatTime) {
        lastHeartBeatTime += 150;
        sensorValuesToSerial();
        mapSensorsToLeds();
    }
    // Pour économiser de l'énergie
    delay(1);
}
