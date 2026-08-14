#include <Arduino.h>
#include <Wire.h>
#include <AccelStepper.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include <SoftwareWire.h>

// =====================================================
// PCA9685 — SERWOMECHANIZMY
// =====================================================

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

constexpr uint16_t SERVO_MIN = 102;
constexpr uint16_t SERVO_MAX = 512;

constexpr uint8_t SERVO_1 = 0;
constexpr uint8_t SERVO_2 = 1;
constexpr uint8_t SERVO_3 = 2;
constexpr uint8_t SERVO_4 = 3;
constexpr uint8_t SERVO_5 = 4;

constexpr int SERVO_STEP = 1;

int servoAngles[5] = {
    125,
    90,
    180,
    90,
    90
};

constexpr int STEERING_LEFT_LIMIT = 75;
constexpr int STEERING_RIGHT_LIMIT = 115;




// =====================================================
// KOMENDY AUTONOMICZNE Z RASPBERRY
// =====================================================

char autonomousCommandBuffer[64];
uint8_t autonomousCommandLength = 0;

bool receivingAutonomousCommand = false;


// =====================================================
// TEMPERATURA — DWA MODUŁY AHT20
// =====================================================

// Konstruktor SoftwareWire przyjmuje kolejno: SDA, SCL.

// Górny poziom:
// D49 = SDA
// D48 = SCL
SoftwareWire upperTemperatureBus(49, 48);

// Dolny poziom:
// D51 = SDA
// D50 = SCL
SoftwareWire lowerTemperatureBus(51, 50);

constexpr uint8_t AHT20_ADDRESS = 0x38;

float upperTemperature = NAN;
float lowerTemperature = NAN;
float upperHumidity = NAN;
float lowerHumidity = NAN;

bool temperatureMeasurementRunning = false;
unsigned long temperatureMeasurementStartedMs = 0;
unsigned long lastTemperatureMeasurementMs = 0;

constexpr unsigned long TEMPERATURE_INTERVAL_MS = 10000;
constexpr unsigned long AHT20_MEASUREMENT_TIME_MS = 90;


// =====================================================
// CZUJNIKI ULTRADZWIEKOWE HC-SR04
// =====================================================

// Lewy przedni
constexpr uint8_t ULTRA_FRONT_LEFT_TRIG = 26;
constexpr uint8_t ULTRA_FRONT_LEFT_ECHO = 27;

// Przedni środkowy
constexpr uint8_t ULTRA_FRONT_CENTER_TRIG = 28;
constexpr uint8_t ULTRA_FRONT_CENTER_ECHO = 29;

// Przedni prawy
constexpr uint8_t ULTRA_FRONT_RIGHT_TRIG = 30;
constexpr uint8_t ULTRA_FRONT_RIGHT_ECHO = 31;

// Prawy
constexpr uint8_t ULTRA_RIGHT_TRIG = 32;
constexpr uint8_t ULTRA_RIGHT_ECHO = 33;

// Lewy
constexpr uint8_t ULTRA_LEFT_TRIG = 34;
constexpr uint8_t ULTRA_LEFT_ECHO = 35;

// Tylny
constexpr uint8_t ULTRA_REAR_TRIG = 36;
constexpr uint8_t ULTRA_REAR_ECHO = 37;


// Aktualne odległości w cm.
// -1 oznacza brak poprawnego pomiaru.
float distanceFrontLeft = -1.0f;
float distanceFrontCenter = -1.0f;
float distanceFrontRight = -1.0f;
float distanceRight = -1.0f;
float distanceLeft = -1.0f;
float distanceRear = -1.0f;


// Który czujnik mierzymy w danej chwili.
uint8_t currentUltrasonicSensor = 0;

unsigned long lastUltrasonicMeasurementMs = 0;

// Odstęp pomiędzy kolejnymi czujnikami.
// 60 ms ogranicza wzajemne zakłócanie HC-SR04.
constexpr unsigned long ULTRASONIC_INTERVAL_MS = 60;

// Maksymalny czas oczekiwania na echo.
// Około 4 metrów.
constexpr unsigned long ULTRASONIC_TIMEOUT_US = 25000UL;



// =====================================================
// WENTYLATORY I BUZZER
// =====================================================

constexpr uint8_t UPPER_FAN_PIN = 38;
constexpr uint8_t LOWER_FAN_PIN = 39;
constexpr uint8_t BUZZER_PIN = 40;

bool upperFanEnabled = false;
bool lowerFanEnabled = false;
bool buzzerEnabled = false;


// =====================================================
// NAPĘD PLATFORMY MOBILNEJ
// =====================================================

constexpr uint8_t DRIVE_LEFT_PWM_PIN = 2;
constexpr uint8_t DRIVE_RIGHT_PWM_PIN = 3;

constexpr uint8_t DRIVE_LEFT_IN1_PIN = 22;
constexpr uint8_t DRIVE_LEFT_IN2_PIN = 23;

constexpr uint8_t DRIVE_RIGHT_IN1_PIN = 24;
constexpr uint8_t DRIVE_RIGHT_IN2_PIN = 25;

// Prędkość jazdy 0–255
constexpr uint8_t DRIVE_PWM = 200;

// O ile zwiększać/zmniejszać PWM podczas łagodnego startu
constexpr uint8_t DRIVE_RAMP_STEP = 8;

// Aktualizacja napędu co 20 ms
constexpr unsigned long DRIVE_UPDATE_INTERVAL_MS = 20;

// Zmień na true, jeżeli któryś silnik obraca się odwrotnie
constexpr bool LEFT_MOTOR_INVERTED = true;
constexpr bool RIGHT_MOTOR_INVERTED = false;

enum class DriveDirection {
    STOP,
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT
};

DriveDirection currentDriveDirection = DriveDirection::STOP;
DriveDirection requestedDriveDirection = DriveDirection::STOP;

uint8_t currentDrivePwm = 0;
uint8_t requestedDrivePwm = 0;

unsigned long lastDriveUpdateMs = 0;

// =====================================================
// RAPORTOWANIE POZYCJI MANIPULATORA
// =====================================================

unsigned long lastJointPositionReportMs = 0;

constexpr unsigned long JOINT_POSITION_REPORT_INTERVAL_MS = 100;
bool autonomousMode = false;

// =====================================================
// SILNIKI KROKOWE
// =====================================================

constexpr uint8_t DIR1_PIN = 41;
constexpr uint8_t STEP1_PIN = 42;

constexpr uint8_t DIR2_PIN = 43;
constexpr uint8_t STEP2_PIN = 44;

constexpr uint8_t DIR3_PIN = 45;
constexpr uint8_t STEP3_PIN = 46;

constexpr uint8_t DIR4_PIN = 52;
constexpr uint8_t STEP4_PIN = 53;

AccelStepper stepper1(
    AccelStepper::DRIVER,
    STEP1_PIN,
    DIR1_PIN
);

AccelStepper stepper2(
    AccelStepper::DRIVER,
    STEP2_PIN,
    DIR2_PIN
);

AccelStepper stepper3(
    AccelStepper::DRIVER,
    STEP3_PIN,
    DIR3_PIN
);

AccelStepper stepper4(
    AccelStepper::DRIVER,
    STEP4_PIN,
    DIR4_PIN
);

constexpr long MOVE_STEPS = 500;

constexpr float MAX_SPEED = 4000.0f;
constexpr float ACCELERATION = 2000.0f;

constexpr unsigned int MIN_PULSE_WIDTH_US = 5;

constexpr int MOTOR_1_DIRECTION = 1;
constexpr int MOTOR_2_DIRECTION = 1;


// =====================================================
// AUTONOMICZNE POZYCJONOWANIE CZŁONÓW
// =====================================================

bool autonomousLink1Moving = false;
bool autonomousLink2Moving = false;
bool autonomousLink3Moving = false;



float readUltrasonicDistance(
    uint8_t trigPin,
    uint8_t echoPin
) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);

    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);

    digitalWrite(trigPin, LOW);

    const unsigned long duration =
        pulseIn(
            echoPin,
            HIGH,
            ULTRASONIC_TIMEOUT_US
        );

    // Timeout — brak echa.
    if (duration == 0) {
        return -1.0f;
    }

    // Prędkość dźwięku około 0.0343 cm/us.
    const float distance =
        duration * 0.0343f / 2.0f;

    // Odrzucenie absurdalnych wyników.
    if (
        distance < 2.0f ||
        distance > 400.0f
    ) {
        return -1.0f;
    }

    return distance;
}

bool homingDone = false;

// =====================================================
// DEKLARACJE FUNKCJI
// =====================================================
void handleJointCommand(char* payload);
void emergencyStop();
void emergencyStopDrive();


void setServoAngle(
    uint8_t channel,
    float angle,
    float maxAngle = 180.0f
);

void setServoAbsolute(
    uint8_t index,
    uint8_t channel,
    float targetAngle,
    float maxAngle = 180.0f
);

void setServoTarget(
    uint8_t index,
    float targetAngle
);

void handleCommand(char command);


void setServoAbsolute(
    uint8_t index,
    uint8_t channel,
    float targetAngle,
    float maxAngle
) {
    // Serwo 1 — zakres 100–150°
    if (index == 0) {
        targetAngle = constrain(
            targetAngle,
            100.0f,
            150.0f
        );
    }

    // Serwo 2 — skręt kół
    else if (index == 1) {
        targetAngle = constrain(
            targetAngle,
            static_cast<float>(STEERING_LEFT_LIMIT),
            static_cast<float>(STEERING_RIGHT_LIMIT)
        );
    }

    // Serwa 3, 4 i 5
    else {
        targetAngle = constrain(
            targetAngle,
            0.0f,
            maxAngle
        );
    }

    servoAngles[index] =
        static_cast<int>(
            roundf(targetAngle)
        );

    setServoAngle(
        channel,
        targetAngle,
        maxAngle
    );

    Serial.print(F("Serwo "));
    Serial.print(index + 1);
    Serial.print(F(" -> "));
    Serial.print(targetAngle, 1);
    Serial.println(F(" stopni"));
}



void setServoTarget(
    uint8_t index,
    float targetAngle
) {
    switch (index) {

        // Serwo 1
        case 0:
            setServoAbsolute(
                0,
                SERVO_1,
                targetAngle
            );
            break;

        // Serwo 2 - skręt kół
        case 1:
            setServoAbsolute(
                1,
                SERVO_2,
                targetAngle
            );
            break;

        // Serwo 3
        case 2:
            setServoAbsolute(
                2,
                SERVO_3,
                targetAngle
            );
            break;

        // Serwo 4
        case 3:
            setServoAbsolute(
                3,
                SERVO_4,
                targetAngle
            );
            break;

        // Serwo 5
        case 4:
            setServoAbsolute(
                4,
                SERVO_5,
                targetAngle
            );
            break;
    }
}


// =====================================================
// KOMENDA ZBIORCZA J1-J5
//
// Format:
// @J:J1,J2,J3,J4,J5
//
// J1-J3 = kroki
// J4-J5 = stopnie
// =====================================================

void handleJointCommand(char* payload) {

    if (!homingDone) {

        Serial.println(
            F("{\"type\":\"joint_command\",\"ok\":false,\"error\":\"not_homed\"}")
        );

        return;
    }





    char* token;

    // ---------------------------------------------
    // J1
    // ---------------------------------------------

    token = strtok(payload, ",");

    if (token == nullptr) {
        Serial.println(F("ERR @J J1"));
        return;
    }

    long j1 =
        atol(token);


    // ---------------------------------------------
    // J2
    // ---------------------------------------------

    token = strtok(nullptr, ",");

    if (token == nullptr) {
        Serial.println(F("ERR @J J2"));
        return;
    }

    long j2 =
        atol(token);


    // ---------------------------------------------
    // J3
    // ---------------------------------------------

    token = strtok(nullptr, ",");

    if (token == nullptr) {
        Serial.println(F("ERR @J J3"));
        return;
    }

    long j3 =
        atol(token);


    // ---------------------------------------------
    // J4
    // ---------------------------------------------

    token = strtok(nullptr, ",");

    if (token == nullptr) {
        Serial.println(F("ERR @J J4"));
        return;
    }

    float j4 =
        atof(token);


    // ---------------------------------------------
    // J5
    // ---------------------------------------------

    token = strtok(nullptr, ",");

    if (token == nullptr) {
        Serial.println(F("ERR @J J5"));
        return;
    }

    float j5 =
        atof(token);


    // =================================================
    // TWARDY LIMIT J1
    // =================================================

    j1 = constrain(
        j1,
        -15557L,
        0L
    );


    // =================================================
    // TWARDY LIMIT J2
    // =================================================

    j2 = constrain(
        j2,
        0L,
        10100L
    );


    // =================================================
    // TWARDY LIMIT J3
    // =================================================

    j3 = constrain(
        j3,
        0L,
        11835L
    );


    // =================================================
    // LIMITY SERW
    // =================================================

    j4 = constrain(
        j4,
        5.0f,
        180.0f
    );

    j5 = constrain(
        j5,
        0.0f,
        180.0f
    );


    // =================================================
    // KROKOWCE
    // =================================================

    // J1 = stepper3
    stepper3.moveTo(
        j1
    );


    // J2 = stepper1 + stepper2
    //
    // j2 jest logiczną pozycją jointa.
    // Każdy silnik dostaje ją ze swoim kierunkiem.

    stepper1.moveTo(
        j2 *
        MOTOR_1_DIRECTION
    );

    stepper2.moveTo(
        j2 *
        MOTOR_2_DIRECTION
    );


    // J3 = stepper4
    stepper4.moveTo(
        j3
    );


    // =================================================
    // SERWA
    // =================================================

    // J4 = servo3
    setServoTarget(
        2,
        j4
    );

    // J5 = servo4
    setServoTarget(
        3,
        j5
    );


    // =================================================
    // POTWIERDZENIE
    // =================================================

    Serial.print(F("OK @J "));

    Serial.print(j1);
    Serial.print(',');

    Serial.print(j2);
    Serial.print(',');

    Serial.print(j3);
    Serial.print(',');

    Serial.print(j4, 1);
    Serial.print(',');

    Serial.println(j5, 1);
}

// =====================================================
// KRAŃCÓWKI
// =====================================================

// Człon 1 -> stepper3
constexpr uint8_t LIMIT_1_PIN = A13;

// Człon 2 -> stepper1 + stepper2
constexpr uint8_t LIMIT_2_PIN = A15;

// Człon 3 -> stepper4
constexpr uint8_t LIMIT_3_PIN = A14;


// Krańcówki aktywne stanem LOW.

bool limit1Active() {
    return digitalRead(LIMIT_1_PIN) == HIGH;
}

bool limit2Active() {
    return digitalRead(LIMIT_2_PIN) == HIGH;
}

bool limit3Active() {
    return digitalRead(LIMIT_3_PIN) == HIGH;
}


// =====================================================
// HOMING
// =====================================================

enum class HomingState {
    IDLE,

    LINK3_SEARCH,
    LINK3_RETURN,

    LINK12_SEARCH,
    LINK12_RETURN,

    DONE,
    ERROR
};


HomingState homingState =
    HomingState::IDLE;




// Człon 1 i 2 kończą homing niezależnie.
bool homingLink1Done = false;
bool homingLink2Done = false;


// Czas rozpoczęcia aktualnego etapu homingu.
unsigned long homingStateStartedMs = 0;


// Maksymalny czas jednego etapu.
// Jeśli krańcówka nie zostanie znaleziona,
// robot zatrzyma homing.
constexpr unsigned long HOMING_TIMEOUT_MS =
    15000;


// =====================================================
// KIERUNKI HOMINGU
// =====================================================

// Jeżeli człon jedzie OD krańcówki,
// zmień odpowiednią wartość z 1 na -1.

constexpr int HOME_DIR_LINK1 = 1;
constexpr int HOME_DIR_LINK2 = -1;
constexpr int HOME_DIR_LINK3 = -1;


// =====================================================
// PRĘDKOŚĆ HOMINGU
// =====================================================

// Na pierwsze testy polecam 150.
// Później możesz zwiększyć np. do 300–400.

constexpr float HOMING_SPEED = 1200.0f;


// =====================================================
// POZYCJA STARTOWA CZŁONU 3
// =====================================================

// Po dotknięciu krańcówki człon 3 dostaje pozycję 0,
// a następnie odjeżdża od niej o tę liczbę kroków.

constexpr long LINK3_RETURN_STEPS = 1000;
constexpr long LINK1_RETURN_STEPS = 1000;
constexpr long LINK2_RETURN_STEPS = 1000;

// Kierunek powrotu jest przeciwny
// do kierunku jazdy na krańcówkę.

constexpr long LINK1_START_POSITION =
    -HOME_DIR_LINK1 * LINK1_RETURN_STEPS;

constexpr long LINK2_START_POSITION =
    -HOME_DIR_LINK2 * LINK2_RETURN_STEPS;

constexpr long LINK3_START_POSITION =
    -HOME_DIR_LINK3 * LINK3_RETURN_STEPS;

// =====================================================
// CZY HOMING WŁAŚNIE TRWA
// =====================================================

bool homingActive() {
    return
        homingState == HomingState::LINK3_SEARCH ||
        homingState == HomingState::LINK3_RETURN ||
        homingState == HomingState::LINK12_SEARCH ||
        homingState == HomingState::LINK12_RETURN;
}


// =====================================================
// START HOMINGU
// =====================================================

void startHoming() {

    emergencyStop();
    emergencyStopDrive();

    homingDone = false;

    homingLink1Done = false;
    homingLink2Done = false;

    Serial.println(F("------------------------------"));
    Serial.println(F("HOMING START"));
    Serial.println(F("Serwo 3 -> 180 stopni"));

    // Serwo 3 teraz jest zwykłym serwem.
    setServoAbsolute(
        2,
        SERVO_3,
        180.0f
    );

    // Od razu przechodzimy do homingu członu 3.
    homingState =
        HomingState::LINK3_SEARCH;

    homingStateStartedMs =
        millis();

    Serial.println(F("Start homingu czlonu 3"));
    Serial.println(F("------------------------------"));
}

// =====================================================
// ANULOWANIE HOMINGU
// =====================================================

void cancelHoming() {

    if (!homingActive()) {
        return;
    }

    emergencyStop();

    homingDone = false;

    homingState =
        HomingState::ERROR;

    Serial.println(
        F("HOMING PRZERWANY")
    );
}


// =====================================================
// AKTUALIZACJA HOMINGU
// =====================================================

void updateHoming() {

    if (!homingActive()) {
        return;
    }

    const unsigned long now =
        millis();


    // =================================================
    // TIMEOUT
    // =================================================

    if (
        now - homingStateStartedMs >
        HOMING_TIMEOUT_MS
    ) {

        emergencyStop();

        homingDone = false;

        homingState =
            HomingState::ERROR;

        Serial.println(
            F("HOMING ERROR - TIMEOUT")
        );

        return;
    }


    // =================================================
    // ETAP 1
    // CZŁON 3 -> stepper4
    // =================================================

    if (
        homingState ==
        HomingState::LINK3_SEARCH
    ) {

        // Krańcówka członu 3 osiągnięta.
        if (limit3Active()) {

            // Krańcówka = pozycja referencyjna 0.
            stepper4.setCurrentPosition(0);

            Serial.println(
                F("Czlon 3 -> krancowka OK")
            );

            // Odjazd od krańcówki.
            stepper4.moveTo(
                LINK3_START_POSITION
            );

            homingState =
                HomingState::LINK3_RETURN;

            homingStateStartedMs =
                now;

            Serial.println(
                F("Czlon 3 -> odjazd od krancowki")
            );

            return;
        }

        // Jazda w stronę krańcówki.
        stepper4.setSpeed(
            HOME_DIR_LINK3 *
            HOMING_SPEED
        );

        stepper4.runSpeed();

        return;
    }


    // =================================================
    // ETAP 2
    // CZŁON 3 -> POZYCJA STARTOWA
    // =================================================

    if (
        homingState ==
        HomingState::LINK3_RETURN
    ) {

        stepper4.run();

        if (
            stepper4.distanceToGo() == 0
        ) {

            Serial.println(
                F("Czlon 3 -> pozycja startowa OK")
            );

            // Teraz zaczynamy homing członu 1 i 2.
            homingLink1Done = false;
            homingLink2Done = false;

            homingState =
                HomingState::LINK12_SEARCH;

            homingStateStartedMs =
                now;

            Serial.println(
                F("Start homingu czlonu 1 i 2")
            );
        }

        return;
    }


    // =================================================
    // ETAP 3
    // CZŁON 1 + CZŁON 2 -> KRAŃCÓWKI
    // =================================================

    if (
        homingState ==
        HomingState::LINK12_SEARCH
    ) {


        // =============================================
        // CZŁON 1
        // stepper3
        // =============================================

        if (!homingLink1Done) {

            if (limit1Active()) {

                // Krańcówka = zero referencyjne.
                stepper3.setCurrentPosition(0);

                homingLink1Done = true;

                Serial.println(
                    F("Czlon 1 -> krancowka OK")
                );

            } else {

                stepper3.setSpeed(
                    HOME_DIR_LINK1 *
                    HOMING_SPEED
                );

                stepper3.runSpeed();
            }
        }


        // =============================================
        // CZŁON 2
        // stepper1 + stepper2
        // =============================================

        if (!homingLink2Done) {

            if (limit2Active()) {

                // Oba silniki należą do jednego członu.
                stepper1.setCurrentPosition(0);
                stepper2.setCurrentPosition(0);

                homingLink2Done = true;

                Serial.println(
                    F("Czlon 2 -> krancowka OK")
                );

            } else {

                stepper1.setSpeed(
                    HOME_DIR_LINK2 *
                    MOTOR_1_DIRECTION *
                    HOMING_SPEED
                );

                stepper2.setSpeed(
                    HOME_DIR_LINK2 *
                    MOTOR_2_DIRECTION *
                    HOMING_SPEED
                );

                stepper1.runSpeed();
                stepper2.runSpeed();
            }
        }


        // =============================================
        // OBA CZŁONY ZNALAZŁY KRAŃCÓWKI
        // =============================================

        if (
            homingLink1Done &&
            homingLink2Done
        ) {

            Serial.println(
                F("Czlon 1 i 2 -> krancowki znalezione")
            );

            // -----------------------------------------
            // Człon 1 -> odjazd od krańcówki
            // -----------------------------------------

            stepper3.moveTo(
                LINK1_START_POSITION
            );


            // -----------------------------------------
            // Człon 2 -> odjazd od krańcówki
            // -----------------------------------------

            stepper1.moveTo(
                LINK2_START_POSITION *
                MOTOR_1_DIRECTION
            );

            stepper2.moveTo(
                LINK2_START_POSITION *
                MOTOR_2_DIRECTION
            );


            homingState =
                HomingState::LINK12_RETURN;

            homingStateStartedMs =
                now;

            Serial.println(
                F("Czlon 1 i 2 -> odjazd od krancowek")
            );
        }

        return;
    }


    // =================================================
    // ETAP 4
    // CZŁON 1 + CZŁON 2 -> POZYCJE STARTOWE
    // =================================================

    if (
        homingState ==
        HomingState::LINK12_RETURN
    ) {

        // Człon 1
        stepper3.run();

        // Człon 2
        stepper1.run();
        stepper2.run();


        const bool link1Returned =
            stepper3.distanceToGo() == 0;


        const bool link2Returned =
            stepper1.distanceToGo() == 0 &&
            stepper2.distanceToGo() == 0;


        // =============================================
        // HOMING CAŁKOWICIE ZAKOŃCZONY
        // =============================================

        if (
            link1Returned &&
            link2Returned
        ) {

            homingDone = true;

            homingState =
                HomingState::DONE;


            Serial.println(
                F("Czlon 1 -> pozycja startowa OK")
            );

            Serial.println(
                F("Czlon 2 -> pozycja startowa OK")
            );

            Serial.println(
                F("------------------------------")
            );

            Serial.println(
                F("HOMING GOTOWY")
            );

            Serial.println(
                F("------------------------------")
            );
        }

        return;
    }
}



// =====================================================
// FUNKCJE SERWOMECHANIZMÓW
// =====================================================

void setServoAngle(
    uint8_t channel,
    float angle,
    float maxAngle
) {
    angle = constrain(angle, 0.0f, maxAngle);

    const float pulseFloat =
        SERVO_MIN +
        (angle / maxAngle) *
        (SERVO_MAX - SERVO_MIN);

    const uint16_t pulse =
        static_cast<uint16_t>(roundf(pulseFloat));

    pwm.setPWM(channel, 0, pulse);
}



void updateServo(
    uint8_t index,
    uint8_t channel,
    int change,
    int maxAngle = 180
) {
    servoAngles[index] += change;

    // Serwo 1 — zakres 100–150 stopni
    if (index == 0) {
        servoAngles[index] = constrain(
            servoAngles[index],
            100,
            150
        );
    }
    // Serwo 2 — ograniczenie skrętu
    else if (index == 1) {
        servoAngles[index] = constrain(
            servoAngles[index],
            STEERING_LEFT_LIMIT,
            STEERING_RIGHT_LIMIT
        );
    }
    // Pozostałe serwa
    else {
        servoAngles[index] = constrain(
            servoAngles[index],
            0,
            maxAngle
        );
    }

    setServoAngle(
        channel,
        static_cast<float>(servoAngles[index]),
        static_cast<float>(maxAngle)
    );

    Serial.print(F("Serwo "));
    Serial.print(index + 1);
    Serial.print(F(" -> "));
    Serial.print(servoAngles[index]);
    Serial.println(F(" stopni"));
}






// =====================================================
// KONFIGURACJA SILNIKÓW KROKOWYCH
// =====================================================

void configureStepper(AccelStepper& stepper) {
    stepper.setMaxSpeed(MAX_SPEED);
    stepper.setAcceleration(ACCELERATION);
    stepper.setMinPulseWidth(MIN_PULSE_WIDTH_US);
}

// =====================================================
// RUCH SILNIKÓW KROKOWYCH
// =====================================================




void moveLink1Absolute(
    long targetSteps,
    float maxSpeed,
    float acceleration
) {
    if (!homingDone) {
        Serial.println(
            F("{\"type\":\"motion\",\"ok\":false,\"error\":\"not_homed\"}")
        );
        return;
    }

    stepper3.setMaxSpeed(maxSpeed);
    stepper3.setAcceleration(acceleration);

    stepper3.moveTo(targetSteps);

    autonomousLink1Moving = true;

    Serial.print(F("{\"type\":\"motion\",\"ok\":true,\"link\":1,\"target\":"));
    Serial.print(targetSteps);
    Serial.println(F("}"));
}


void moveLink2Absolute(
    long targetSteps,
    float maxSpeed,
    float acceleration
) {
    if (!homingDone) {
        Serial.println(
            F("{\"type\":\"motion\",\"ok\":false,\"error\":\"not_homed\"}")
        );
        return;
    }

    stepper1.setMaxSpeed(maxSpeed);
    stepper1.setAcceleration(acceleration);

    stepper2.setMaxSpeed(maxSpeed);
    stepper2.setAcceleration(acceleration);

    const long target1 =
        targetSteps * MOTOR_1_DIRECTION;

    const long target2 =
        targetSteps * MOTOR_2_DIRECTION;

    stepper1.moveTo(target1);
    stepper2.moveTo(target2);

    autonomousLink2Moving = true;

    Serial.print(F("{\"type\":\"motion\",\"ok\":true,\"link\":2,\"target\":"));
    Serial.print(targetSteps);
    Serial.println(F("}"));
}


void moveLink3Absolute(
    long targetSteps,
    float maxSpeed,
    float acceleration
) {
    if (!homingDone) {
        Serial.println(
            F("{\"type\":\"motion\",\"ok\":false,\"error\":\"not_homed\"}")
        );
        return;
    }

    stepper4.setMaxSpeed(maxSpeed);
    stepper4.setAcceleration(acceleration);

    stepper4.moveTo(targetSteps);

    autonomousLink3Moving = true;

    Serial.print(F("{\"type\":\"motion\",\"ok\":true,\"link\":3,\"target\":"));
    Serial.print(targetSteps);
    Serial.println(F("}"));
}


void updateAutonomousStepperStatus() {

    if (
        autonomousLink1Moving &&
        stepper3.distanceToGo() == 0
    ) {
        autonomousLink1Moving = false;

        Serial.print(
            F("{\"type\":\"motion_done\",\"link\":1,\"position\":")
        );
        Serial.print(
            stepper3.currentPosition()
        );
        Serial.println(F("}"));
    }


    if (
        autonomousLink2Moving &&
        stepper1.distanceToGo() == 0 &&
        stepper2.distanceToGo() == 0
    ) {
        autonomousLink2Moving = false;

        Serial.print(
            F("{\"type\":\"motion_done\",\"link\":2,\"position\":")
        );

        Serial.print(
            stepper1.currentPosition() *
            MOTOR_1_DIRECTION
        );

        Serial.println(F("}"));
    }


    if (
        autonomousLink3Moving &&
        stepper4.distanceToGo() == 0
    ) {
        autonomousLink3Moving = false;

        Serial.print(
            F("{\"type\":\"motion_done\",\"link\":3,\"position\":")
        );

        Serial.print(
            stepper4.currentPosition()
        );

        Serial.println(F("}"));
    }
}



void movePair(int direction) {
    const long movement1 =
        direction * MOTOR_1_DIRECTION * MOVE_STEPS;

    const long movement2 =
        direction * MOTOR_2_DIRECTION * MOVE_STEPS;

    stepper1.move(movement1);
    stepper2.move(movement2);

    if (direction > 0) {
        Serial.println(F("Osie 1 i 2: kierunek dodatni"));
    } else {
        Serial.println(F("Osie 1 i 2: kierunek ujemny"));
    }
}

void moveSingle(
    AccelStepper& stepper,
    int direction,
    const __FlashStringHelper* description
) {
    stepper.move(direction * MOVE_STEPS);
    Serial.println(description);
}

void stopAllSteppers() {
    stepper1.stop();
    stepper2.stop();
    stepper3.stop();
    stepper4.stop();

    Serial.println(F("Lagodne zatrzymanie wszystkich osi"));
}

void emergencyStop() {
    stepper1.setCurrentPosition(
        stepper1.currentPosition()
    );

    stepper2.setCurrentPosition(
        stepper2.currentPosition()
    );

    stepper3.setCurrentPosition(
        stepper3.currentPosition()
    );

    stepper4.setCurrentPosition(
        stepper4.currentPosition()
    );

    Serial.println(F("AWARYJNE ZATRZYMANIE"));
}




void setMotorDirection(
    uint8_t in1Pin,
    uint8_t in2Pin,
    int direction,
    bool inverted
) {
    if (inverted) {
        direction = -direction;
    }

    if (direction > 0) {
        digitalWrite(in1Pin, HIGH);
        digitalWrite(in2Pin, LOW);
    } else if (direction < 0) {
        digitalWrite(in1Pin, LOW);
        digitalWrite(in2Pin, HIGH);
    } else {
        digitalWrite(in1Pin, LOW);
        digitalWrite(in2Pin, LOW);
    }
}

void applyDriveDirection(DriveDirection direction) {
    switch (direction) {
        case DriveDirection::FORWARD:
            setMotorDirection(
                DRIVE_LEFT_IN1_PIN,
                DRIVE_LEFT_IN2_PIN,
                1,
                LEFT_MOTOR_INVERTED
            );

            setMotorDirection(
                DRIVE_RIGHT_IN1_PIN,
                DRIVE_RIGHT_IN2_PIN,
                1,
                RIGHT_MOTOR_INVERTED
            );
            break;

        case DriveDirection::BACKWARD:
            setMotorDirection(
                DRIVE_LEFT_IN1_PIN,
                DRIVE_LEFT_IN2_PIN,
                -1,
                LEFT_MOTOR_INVERTED
            );

            setMotorDirection(
                DRIVE_RIGHT_IN1_PIN,
                DRIVE_RIGHT_IN2_PIN,
                -1,
                RIGHT_MOTOR_INVERTED
            );
            break;

        case DriveDirection::LEFT:
            setMotorDirection(
                DRIVE_LEFT_IN1_PIN,
                DRIVE_LEFT_IN2_PIN,
                -1,
                LEFT_MOTOR_INVERTED
            );

            setMotorDirection(
                DRIVE_RIGHT_IN1_PIN,
                DRIVE_RIGHT_IN2_PIN,
                1,
                RIGHT_MOTOR_INVERTED
            );
            break;

        case DriveDirection::RIGHT:
            setMotorDirection(
                DRIVE_LEFT_IN1_PIN,
                DRIVE_LEFT_IN2_PIN,
                1,
                LEFT_MOTOR_INVERTED
            );

            setMotorDirection(
                DRIVE_RIGHT_IN1_PIN,
                DRIVE_RIGHT_IN2_PIN,
                -1,
                RIGHT_MOTOR_INVERTED
            );
            break;

        case DriveDirection::STOP:
            setMotorDirection(
                DRIVE_LEFT_IN1_PIN,
                DRIVE_LEFT_IN2_PIN,
                0,
                LEFT_MOTOR_INVERTED
            );

            setMotorDirection(
                DRIVE_RIGHT_IN1_PIN,
                DRIVE_RIGHT_IN2_PIN,
                0,
                RIGHT_MOTOR_INVERTED
            );
            break;
    }
}

void requestDrive(
    DriveDirection direction,
    uint8_t pwmValue = DRIVE_PWM
) {
    requestedDriveDirection = direction;

    if (direction == DriveDirection::STOP) {
        requestedDrivePwm = 0;
    } else {
        requestedDrivePwm = pwmValue;
    }
}

void updateDrive() {
    const unsigned long now = millis();

    if (now - lastDriveUpdateMs < DRIVE_UPDATE_INTERVAL_MS) {
        return;
    }

    lastDriveUpdateMs = now;

    /*
     * Jeśli zmieniamy kierunek, najpierw łagodnie zatrzymujemy
     * silniki. Dopiero przy PWM=0 zmieniamy kierunek.
     */
    if (
        requestedDriveDirection != currentDriveDirection &&
        currentDrivePwm > 0
    ) {
        if (currentDrivePwm > DRIVE_RAMP_STEP) {
            currentDrivePwm -= DRIVE_RAMP_STEP;
        } else {
            currentDrivePwm = 0;
        }
    } else {
        if (requestedDriveDirection != currentDriveDirection) {
            currentDriveDirection = requestedDriveDirection;
            applyDriveDirection(currentDriveDirection);
        }

        if (currentDrivePwm < requestedDrivePwm) {
            const int nextPwm =
                currentDrivePwm + DRIVE_RAMP_STEP;

            currentDrivePwm =
                min(nextPwm, static_cast<int>(requestedDrivePwm));
        } else if (currentDrivePwm > requestedDrivePwm) {
            const int nextPwm =
                currentDrivePwm - DRIVE_RAMP_STEP;

            currentDrivePwm =
                max(nextPwm, static_cast<int>(requestedDrivePwm));
        }
    }

    analogWrite(
        DRIVE_LEFT_PWM_PIN,
        currentDrivePwm
    );

    analogWrite(
        DRIVE_RIGHT_PWM_PIN,
        currentDrivePwm
    );

    if (
        currentDrivePwm == 0 &&
        requestedDriveDirection == DriveDirection::STOP
    ) {
        currentDriveDirection = DriveDirection::STOP;
        applyDriveDirection(DriveDirection::STOP);
    }
}

void emergencyStopDrive() {
    requestedDriveDirection = DriveDirection::STOP;
    currentDriveDirection = DriveDirection::STOP;

    requestedDrivePwm = 0;
    currentDrivePwm = 0;

    analogWrite(DRIVE_LEFT_PWM_PIN, 0);
    analogWrite(DRIVE_RIGHT_PWM_PIN, 0);

    applyDriveDirection(DriveDirection::STOP);
}

bool shouldRunUltrasonicSensors() {
    if (!autonomousMode) {
        return false;
    }

    // Czujniki działają również podczas łagodnego hamowania,
    // dopóki platforma rzeczywiście jeszcze się porusza.
    const bool platformDriving =
        requestedDriveDirection != DriveDirection::STOP ||
        currentDrivePwm > 0;

    return platformDriving;
}

void sendRobotStatus() {
    Serial.print(F("{\"upperTemp\":"));

    if (isnan(upperTemperature)) {
        Serial.print(F("null"));
    } else {
        Serial.print(upperTemperature, 1);
    }

    Serial.print(F(",\"lowerTemp\":"));

    if (isnan(lowerTemperature)) {
        Serial.print(F("null"));
    } else {
        Serial.print(lowerTemperature, 1);
    }

    Serial.print(F(",\"upperHumidity\":"));

    if (isnan(upperHumidity)) {
        Serial.print(F("null"));
    } else {
        Serial.print(upperHumidity, 1);
    }

    Serial.print(F(",\"lowerHumidity\":"));

    if (isnan(lowerHumidity)) {
        Serial.print(F("null"));
    } else {
        Serial.print(lowerHumidity, 1);
    }

    Serial.print(F(",\"upperFan\":"));
    Serial.print(upperFanEnabled ? F("true") : F("false"));

    Serial.print(F(",\"lowerFan\":"));
    Serial.print(lowerFanEnabled ? F("true") : F("false"));

    Serial.print(F(",\"buzzer\":"));
    Serial.print(buzzerEnabled ? F("true") : F("false"));

    Serial.print(F(",\"autonomous\":"));
    Serial.print(
        autonomousMode ? F("true") : F("false"));
    Serial.println(F("}"));

}

void sendUltrasonicStatus() {
    Serial.print(F("{\"ultrasonic\":true"));

    Serial.print(F(",\"frontLeft\":"));
    Serial.print(distanceFrontLeft, 1);

    Serial.print(F(",\"frontCenter\":"));
    Serial.print(distanceFrontCenter, 1);

    Serial.print(F(",\"frontRight\":"));
    Serial.print(distanceFrontRight, 1);

    Serial.print(F(",\"right\":"));
    Serial.print(distanceRight, 1);

    Serial.print(F(",\"left\":"));
    Serial.print(distanceLeft, 1);

    Serial.print(F(",\"rear\":"));
    Serial.print(distanceRear, 1);

    Serial.println(F("}"));
}

void updateUltrasonicSensors() {
    const unsigned long now = millis();

    if (
        now - lastUltrasonicMeasurementMs <
        ULTRASONIC_INTERVAL_MS
    ) {
        return;
    }

    lastUltrasonicMeasurementMs = now;

    switch (currentUltrasonicSensor) {

        case 0:
            distanceFrontLeft =
                readUltrasonicDistance(
                    ULTRA_FRONT_LEFT_TRIG,
                    ULTRA_FRONT_LEFT_ECHO
                );
            break;

        case 1:
            distanceFrontCenter =
                readUltrasonicDistance(
                    ULTRA_FRONT_CENTER_TRIG,
                    ULTRA_FRONT_CENTER_ECHO
                );
            break;

        case 2:
            distanceFrontRight =
                readUltrasonicDistance(
                    ULTRA_FRONT_RIGHT_TRIG,
                    ULTRA_FRONT_RIGHT_ECHO
                );
            break;

        case 3:
            distanceRight =
                readUltrasonicDistance(
                    ULTRA_RIGHT_TRIG,
                    ULTRA_RIGHT_ECHO
                );
            break;

        case 4:
            distanceLeft =
                readUltrasonicDistance(
                    ULTRA_LEFT_TRIG,
                    ULTRA_LEFT_ECHO
                );
            break;

        case 5:
            distanceRear =
                readUltrasonicDistance(
                    ULTRA_REAR_TRIG,
                    ULTRA_REAR_ECHO
                );
            break;
    }

    currentUltrasonicSensor++;

    if (currentUltrasonicSensor >= 6) {
        currentUltrasonicSensor = 0;
    
        sendUltrasonicStatus();
    }
}



// =====================================================
// OBSŁUGA KOMEND
// =====================================================


void handleAutonomousCommand(
    char* command
) {


        // =====================================================
    // KOMENDA ZBIORCZA J1-J5
    //
    // Z Raspberry wysyłamy:
    // @J:J1,J2,J3,J4,J5
    //
    // Do tej funkcji trafia już:
    // J:J1,J2,J3,J4,J5
    // =====================================================

    if (
        command[0] == 'J' &&
        command[1] == ':'
    ) {

        handleJointCommand(
            command + 2
        );

        return;
    }

    // =====================================================
    // KOMENDA KROKOWCA
    //
    // Przykład:
    // L2:7000,1000,500
    //
    // człon, pozycja, prędkość, przyspieszenie
    // =====================================================

    if (command[0] == 'L') {

        int link = 0;
        long target = 0;
        float speed = 0.0f;
        float acceleration = 0.0f;

        const int parsed = sscanf(
            command,
            "L%d:%ld,%f,%f",
            &link,
            &target,
            &speed,
            &acceleration
        );

        if (parsed != 4) {
            Serial.println(
                F("{\"type\":\"command\",\"ok\":false,\"error\":\"bad_link_format\"}")
            );
            return;
        }

        if (
            speed <= 0.0f ||
            acceleration <= 0.0f
        ) {
            Serial.println(
                F("{\"type\":\"command\",\"ok\":false,\"error\":\"bad_motion_parameters\"}")
            );
            return;
        }

        switch (link) {

            case 1:
                moveLink1Absolute(
                    target,
                    speed,
                    acceleration
                );
                return;

            case 2:
                moveLink2Absolute(
                    target,
                    speed,
                    acceleration
                );
                return;

            case 3:
                moveLink3Absolute(
                    target,
                    speed,
                    acceleration
                );
                return;

            default:
                Serial.println(
                    F("{\"type\":\"command\",\"ok\":false,\"error\":\"bad_link\"}")
                );
                return;
        }
    }


    // =====================================================
    // KOMENDA SERWA
    //
    // Przykład:
    // S3:143.5
    //
    // serwo, kąt
    // =====================================================

    if (command[0] == 'S') {

        int servoNumber = 0;
        float targetAngle = 0.0f;

        const int parsed = sscanf(
            command,
            "S%d:%f",
            &servoNumber,
            &targetAngle
        );

        if (parsed != 2) {
            Serial.println(
                F("{\"type\":\"command\",\"ok\":false,\"error\":\"bad_servo_format\"}")
            );
            return;
        }

        if (
            servoNumber < 1 ||
            servoNumber > 5
        ) {
            Serial.println(
                F("{\"type\":\"command\",\"ok\":false,\"error\":\"bad_servo\"}")
            );
            return;
        }

        // setServoTarget używa indeksów 0–4.
        const uint8_t servoIndex =
            static_cast<uint8_t>(
                servoNumber - 1
            );

        setServoTarget(
            servoIndex,
            targetAngle
        );

        Serial.print(
            F("{\"type\":\"servo\",\"ok\":true,\"servo\":")
        );

        Serial.print(servoNumber);

        Serial.print(
            F(",\"target\":")
        );

        Serial.print(
            targetAngle,
            1
        );

        Serial.println(F("}"));

        return;
    }


    // =====================================================
    // NIEZNANY TYP KOMENDY
    // =====================================================

    Serial.println(
        F("{\"type\":\"command\",\"ok\":false,\"error\":\"unknown_command\"}")
    );
}


void processSerialInput() {

    while (Serial.available() > 0) {

        const char incoming =
            Serial.read();


        // =============================================
        // TRWA ODBIERANIE KOMENDY AUTONOMICZNEJ
        // =============================================

        if (receivingAutonomousCommand) {

            if (
                incoming == '\n' ||
                incoming == '\r'
            ) {
                if (autonomousCommandLength > 0) {

                    autonomousCommandBuffer[
                        autonomousCommandLength
                    ] = '\0';

                    handleAutonomousCommand(
                        autonomousCommandBuffer
                    );
                }

                autonomousCommandLength = 0;
                receivingAutonomousCommand = false;

                continue;
            }


            if (
                autonomousCommandLength <
                sizeof(autonomousCommandBuffer) - 1
            ) {
                autonomousCommandBuffer[
                    autonomousCommandLength
                ] = incoming;

                autonomousCommandLength++;
            }
            else {
                autonomousCommandLength = 0;
                receivingAutonomousCommand = false;

                Serial.println(
                    F("{\"type\":\"command\",\"ok\":false,\"error\":\"too_long\"}")
                );
            }

            continue;
        }


        // =============================================
        // NOWA KOMENDA AUTONOMICZNA
        // =============================================

        if (incoming == '@') {

            autonomousCommandLength = 0;
            receivingAutonomousCommand = true;

            continue;
        }


        // =============================================
        // STARA KOMENDA MANUALNA
        // =============================================

        handleCommand(incoming);
    }
}


void handleCommand(char command) {
    if (
        homingActive() &&
        command != 'z'
    ) {
        return;
    }
    
    switch (command) {
        

        case 'V':

            Serial.println(F("------------------------------"));
            Serial.println(F("AKTUALNE POZYCJE JOINTOW"));

            // J1 = stepper3
            Serial.print(F("J1 = "));
            Serial.print(stepper3.currentPosition());
            Serial.println(F(" krokow"));

            // J2 = stepper1 + stepper2
            Serial.print(F("J2 = "));
            Serial.print(
                stepper1.currentPosition() *
                MOTOR_1_DIRECTION
            );
            Serial.println(F(" krokow"));

            // J3 = stepper4
            Serial.print(F("J3 = "));
            Serial.print(stepper4.currentPosition());
            Serial.println(F(" krokow"));

            // Serwa
            Serial.print(F("J4 / servo3 = "));
            Serial.print(servoAngles[2]);
            Serial.println(F(" stopni"));

            Serial.print(F("J5 / servo4 = "));
            Serial.print(servoAngles[3]);
            Serial.println(F(" stopni"));

            Serial.print(F("GRIPPER / servo5 = "));
            Serial.print(servoAngles[4]);
            Serial.println(F(" stopni"));

            Serial.println(F("------------------------------"));

            break;



        case 'K':

            Serial.println(F("------------------------------"));
            Serial.println(F("POZYCJE KROKOWCOW"));

            // J1 = stepper3
            Serial.print(F("J1 = "));
            Serial.println(stepper3.currentPosition());

            // J2 = stepper1 + stepper2
            Serial.print(F("J2 = "));
            Serial.println(
                stepper1.currentPosition() *
                MOTOR_1_DIRECTION
            );

            // J3 = stepper4
            Serial.print(F("J3 = "));
            Serial.println(stepper4.currentPosition());

            Serial.println(F("------------------------------"));

            break;



        case 'Q':
            Serial.print(F("LIMIT1="));
            Serial.print(digitalRead(LIMIT_1_PIN));

            Serial.print(F(" LIMIT2="));
            Serial.print(digitalRead(LIMIT_2_PIN));

            Serial.print(F(" LIMIT3="));
            Serial.println(digitalRead(LIMIT_3_PIN));
            break;

        case 'H':
            startHoming();
            break;


        case 'A':
            autonomousMode = true;

            // Nowy cykl pomiarowy od pierwszego czujnika.
            currentUltrasonicSensor = 0;
            lastUltrasonicMeasurementMs = 0;

            Serial.println(F("Tryb: autonomiczny"));
            break;

        case 'M':
            autonomousMode = false;

            // Na wszelki wypadek zatrzymujemy platformę
            // przy powrocie do manualnego.
            requestDrive(DriveDirection::STOP);

            Serial.println(F("Tryb: manualny"));
            break;

        // Górne wentylatory — przełącznik
        case '5':
            upperFanEnabled = !upperFanEnabled;

            digitalWrite(
                UPPER_FAN_PIN,
                upperFanEnabled ? HIGH : LOW
            );

            sendRobotStatus();
            break;

        // Dolne wentylatory — przełącznik
        case '6':
            lowerFanEnabled = !lowerFanEnabled;

            digitalWrite(
                LOWER_FAN_PIN,
                lowerFanEnabled ? HIGH : LOW
            );

            sendRobotStatus();
            break;

        // Buzzer włączony
        case '7':
            buzzerEnabled = true;
            digitalWrite(BUZZER_PIN, HIGH);
            sendRobotStatus();
            break;

        // Buzzer wyłączony
        case '8':
            buzzerEnabled = false;
            digitalWrite(BUZZER_PIN, LOW);
            sendRobotStatus();
            break;


        // -------------------------------------------------
        // JAZDA PLATFORMY
        // -------------------------------------------------

        case '1':
            requestDrive(DriveDirection::FORWARD);
            Serial.println(F("Jazda: przod"));
            break;

        case '2':
            requestDrive(DriveDirection::BACKWARD);
            Serial.println(F("Jazda: tyl"));
            break;

        case '3':
            requestDrive(DriveDirection::LEFT);
            Serial.println(F("Jazda: lewo"));
            break;

        case '4':
            requestDrive(DriveDirection::RIGHT);
            Serial.println(F("Jazda: prawo"));
            break;

        case '0':
            requestDrive(DriveDirection::STOP);
            Serial.println(F("Jazda: stop"));
            break;
        


        // -------------------------------------------------
        // SILNIKI KROKOWE
        // -------------------------------------------------

        case 'w':
            movePair(1);
            break;

        case 's':
            movePair(-1);
            break;

        case 'a':
            moveSingle(
                stepper3,
                1,
                F("Os 3: kierunek dodatni")
            );
            break;

        case 'd':
            moveSingle(
                stepper3,
                -1,
                F("Os 3: kierunek ujemny")
            );
            break;

        case 'f':
            moveSingle(
                stepper4,
                1,
                F("Os 4: kierunek dodatni")
            );
            break;

        case 'g':
            moveSingle(
                stepper4,
                -1,
                F("Os 4: kierunek ujemny")
            );
            break;

        case 'x':
            stopAllSteppers();
            break;

        case 'z':
            emergencyStop();
            emergencyStopDrive();
            cancelHoming();
            break;

        // -------------------------------------------------
        // SERWOMECHANIZMY
        // -------------------------------------------------
        
        


        case 't':
            updateServo(
                0,
                SERVO_1,
                -SERVO_STEP
            );
            break;

        case 'y':
            updateServo(
                0,
                SERVO_1,
                SERVO_STEP
            );
            break;

        case 'u':
            updateServo(
                1,
                SERVO_2,
                -SERVO_STEP
            );
            break;

        case 'i':
            updateServo(
                1,
                SERVO_2,
                SERVO_STEP
            );
            break;

        // Serwo 3 
        case 'o':
            updateServo(
                2,
                SERVO_3,
                -SERVO_STEP
            );
            break;

        case 'p':
            updateServo(
                2,
                SERVO_3,
                SERVO_STEP
            );
            break;

        case 'h':
            updateServo(
                3,
                SERVO_4,
                -SERVO_STEP
            );
            break;

        case 'j':
            updateServo(
                3,
                SERVO_4,
                SERVO_STEP
            );
            break;

        case 'k':
            updateServo(
                4,
                SERVO_5,
                -SERVO_STEP
            );
            break;

        case 'l':
            updateServo(
                4,
                SERVO_5,
                SERVO_STEP
            );
            break;

        case '\n':
        case '\r':
        case ' ':
            break;

        default:
            Serial.print(F("Nieznana komenda: "));
            Serial.println(command);
            break;
    }
}


bool initializeAht20(SoftwareWire& bus) {
    bus.begin();
    bus.setClock(100000);

    delay(50);

    // Odczyt statusu AHT20.
    bus.requestFrom(AHT20_ADDRESS, static_cast<uint8_t>(1));

    if (bus.available() < 1) {
        return false;
    }

    const uint8_t status = bus.read();

    // Bit kalibracji nie jest ustawiony.
    if ((status & 0x08) == 0) {
        bus.beginTransmission(AHT20_ADDRESS);
        bus.write(0xBE);
        bus.write(0x08);
        bus.write(0x00);

        if (bus.endTransmission() != 0) {
            return false;
        }

        delay(20);
    }

    return true;
}

bool startAht20Measurement(SoftwareWire& bus) {
    bus.beginTransmission(AHT20_ADDRESS);

    bus.write(0xAC);
    bus.write(0x33);
    bus.write(0x00);

    return bus.endTransmission() == 0;
}

bool readAht20Measurement(
    SoftwareWire& bus,
    float& temperature,
    float& humidity
) {
    const uint8_t bytesReceived = bus.requestFrom(
        AHT20_ADDRESS,
        static_cast<uint8_t>(6)
    );

    if (bytesReceived < 6 || bus.available() < 6) {
        return false;
    }

    uint8_t data[6];

    for (uint8_t index = 0; index < 6; index++) {
        data[index] = bus.read();
    }

    // Bit 7 oznacza, że pomiar jeszcze trwa.
    if ((data[0] & 0x80) != 0) {
        return false;
    }

    const uint32_t rawHumidity =
        (static_cast<uint32_t>(data[1]) << 12) |
        (static_cast<uint32_t>(data[2]) << 4) |
        (static_cast<uint32_t>(data[3]) >> 4);

    const uint32_t rawTemperature =
        (static_cast<uint32_t>(data[3] & 0x0F) << 16) |
        (static_cast<uint32_t>(data[4]) << 8) |
        static_cast<uint32_t>(data[5]);

    humidity =
        static_cast<float>(rawHumidity) *
        100.0f /
        1048576.0f;

    temperature =
        static_cast<float>(rawTemperature) *
        200.0f /
        1048576.0f -
        50.0f;

    return true;
}


// =====================================================
// WYSYLANIE AKTUALNEJ POZYCJI J1-J5
// =====================================================

void sendJointPositions() {

    if (!homingDone) {
        return;
    }

    Serial.print(F("@P:"));

    // J1
    Serial.print(
        stepper3.currentPosition()
    );

    Serial.print(',');

    // J2
    Serial.print(
        stepper1.currentPosition() *
        MOTOR_1_DIRECTION
    );

    Serial.print(',');

    // J3
    Serial.print(
        stepper4.currentPosition()
    );

    Serial.print(',');

    // J4 = servo3
    Serial.print(
        servoAngles[2]
    );

    Serial.print(',');

    // J5 = servo4
    Serial.println(
        servoAngles[3]
    );
}


void updateTemperatureSensors() {
    const unsigned long now = millis();

    if (!temperatureMeasurementRunning) {
        if (
            now - lastTemperatureMeasurementMs <
            TEMPERATURE_INTERVAL_MS
        ) {
            return;
        }

        const bool upperStarted =
            startAht20Measurement(upperTemperatureBus);

        const bool lowerStarted =
            startAht20Measurement(lowerTemperatureBus);

        if (!upperStarted) {
            Serial.println(F("Blad uruchomienia gornego AHT20"));
        }

        if (!lowerStarted) {
            Serial.println(F("Blad uruchomienia dolnego AHT20"));
        }

        temperatureMeasurementStartedMs = now;
        temperatureMeasurementRunning = true;
        return;
    }

    if (
        now - temperatureMeasurementStartedMs <
        AHT20_MEASUREMENT_TIME_MS
    ) {
        return;
    }

    const bool upperRead = readAht20Measurement(
        upperTemperatureBus,
        upperTemperature,
        upperHumidity
    );

    const bool lowerRead = readAht20Measurement(
        lowerTemperatureBus,
        lowerTemperature,
        lowerHumidity
    );

    if (!upperRead) {
        Serial.println(F("Blad odczytu gornego AHT20"));
    }

    if (!lowerRead) {
        Serial.println(F("Blad odczytu dolnego AHT20"));
    }

    temperatureMeasurementRunning = false;
    lastTemperatureMeasurementMs = now;

    sendRobotStatus();
}



// =====================================================
// SETUP
// =====================================================

void setup() {
    Serial.begin(115200);
    Serial.setTimeout(20);

    // =====================================================
    // BEZPIECZNE STANY WYJŚĆ
    // =====================================================

    pinMode(UPPER_FAN_PIN, OUTPUT);
    pinMode(LOWER_FAN_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);

    digitalWrite(UPPER_FAN_PIN, LOW);
    digitalWrite(LOWER_FAN_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);

    upperFanEnabled = false;
    lowerFanEnabled = false;
    buzzerEnabled = false;

    pinMode(ULTRA_FRONT_LEFT_TRIG, OUTPUT);
    pinMode(ULTRA_FRONT_LEFT_ECHO, INPUT);

    pinMode(ULTRA_FRONT_CENTER_TRIG, OUTPUT);
    pinMode(ULTRA_FRONT_CENTER_ECHO, INPUT);

    pinMode(ULTRA_FRONT_RIGHT_TRIG, OUTPUT);
    pinMode(ULTRA_FRONT_RIGHT_ECHO, INPUT);

    pinMode(ULTRA_RIGHT_TRIG, OUTPUT);
    pinMode(ULTRA_RIGHT_ECHO, INPUT);

    pinMode(ULTRA_LEFT_TRIG, OUTPUT);
    pinMode(ULTRA_LEFT_ECHO, INPUT);

    pinMode(ULTRA_REAR_TRIG, OUTPUT);
    pinMode(ULTRA_REAR_ECHO, INPUT);

    digitalWrite(ULTRA_FRONT_LEFT_TRIG, LOW);
    digitalWrite(ULTRA_FRONT_CENTER_TRIG, LOW);
    digitalWrite(ULTRA_FRONT_RIGHT_TRIG, LOW);
    digitalWrite(ULTRA_RIGHT_TRIG, LOW);
    digitalWrite(ULTRA_LEFT_TRIG, LOW);
    digitalWrite(ULTRA_REAR_TRIG, LOW);

    pinMode(LIMIT_1_PIN, INPUT_PULLUP);
    pinMode(LIMIT_2_PIN, INPUT_PULLUP);
    pinMode(LIMIT_3_PIN, INPUT_PULLUP);

    // =====================================================
    // PCA9685 — SPRZĘTOWE I2C
    // =====================================================

    Wire.begin();

    if (pwm.begin()) {
        Serial.println(F("PCA9685 OK pod adresem 0x40"));
    } 
    else {
        Serial.println(F("PCA9685 ERROR"));
    }


    pwm.setPWMFreq(50);
    delay(10);
    
    
    // =====================================================
    // SILNIKI KROKOWE
    // =====================================================

    configureStepper(stepper1);
    configureStepper(stepper2);
    configureStepper(stepper3);
    configureStepper(stepper4);

    // =====================================================
    // NAPĘD PLATFORMY
    // =====================================================

    pinMode(DRIVE_LEFT_PWM_PIN, OUTPUT);
    pinMode(DRIVE_RIGHT_PWM_PIN, OUTPUT);

    pinMode(DRIVE_LEFT_IN1_PIN, OUTPUT);
    pinMode(DRIVE_LEFT_IN2_PIN, OUTPUT);

    pinMode(DRIVE_RIGHT_IN1_PIN, OUTPUT);
    pinMode(DRIVE_RIGHT_IN2_PIN, OUTPUT);

    analogWrite(DRIVE_LEFT_PWM_PIN, 0);
    analogWrite(DRIVE_RIGHT_PWM_PIN, 0);

    currentDrivePwm = 0;
    requestedDrivePwm = 0;

    currentDriveDirection = DriveDirection::STOP;
    requestedDriveDirection = DriveDirection::STOP;

    applyDriveDirection(DriveDirection::STOP);

   


    // =====================================================
    // CZUJNIKI AHT20 — PROGRAMOWE I2C
    // =====================================================

    const bool upperSensorReady =
        initializeAht20(upperTemperatureBus);

    const bool lowerSensorReady =
        initializeAht20(lowerTemperatureBus);

    Serial.println(
        upperSensorReady
            ? F("Gorny AHT20 gotowy")
            : F("Nie wykryto gornego AHT20")
    );

    Serial.println(
        lowerSensorReady
            ? F("Dolny AHT20 gotowy")
            : F("Nie wykryto dolnego AHT20")
    );

    /*
     * Dzięki temu pierwszy pomiar rozpocznie się niemal od razu.
     * Bez tego kod może czekać TEMPERATURE_INTERVAL_MS.
     */
    lastTemperatureMeasurementMs =
        millis() - TEMPERATURE_INTERVAL_MS;

    temperatureMeasurementRunning = false;

    // =====================================================
    // INFORMACJE STARTOWE
    // =====================================================

    Serial.println();
    Serial.println(F("Manipulator gotowy"));
    Serial.println(F("------------------------------"));
    Serial.println(F("w/s - osie 1 i 2"));
    Serial.println(F("a/d - os 3"));
    Serial.println(F("f/g - os 4"));
    Serial.println(F("x   - lagodne zatrzymanie"));
    Serial.println(F("z   - zatrzymanie awaryjne"));
    Serial.println(F("------------------------------"));
    Serial.println(F("t/y - serwo 1"));
    Serial.println(F("u/i - serwo 2"));
    Serial.println(F("o/p - serwo 3, ruch plynny"));
    Serial.println(F("h/j - serwo 4"));
    Serial.println(F("k/l - serwo 5"));
    Serial.println(F("------------------------------"));
    Serial.println(F("5 - przelacz gorne wentylatory"));
    Serial.println(F("6 - przelacz dolne wentylatory"));
    Serial.println(F("7 - buzzer wlacz"));
    Serial.println(F("8 - buzzer wylacz"));
}

// =====================================================
// LOOP
// =====================================================

void loop() {

    // =====================================================
    // KOMENDY
    // =====================================================

    processSerialInput();


    // =====================================================
    // HOMING
    // =====================================================

    if (homingActive()) {

        updateHoming();

        updateDrive();

        return;
    }


    // =====================================================
    // MANIPULATOR
    // =====================================================

    stepper1.run();
    stepper2.run();
    stepper3.run();
    stepper4.run();

    

    updateAutonomousStepperStatus();


    // =====================================================
    // PLATFORMA
    // =====================================================

    updateDrive();


    // =====================================================
    // HC-SR04
    // =====================================================

    if (shouldRunUltrasonicSensors()) {
        updateUltrasonicSensors();
    }


    // =====================================================
    // TEMPERATURA
    // =====================================================

    updateTemperatureSensors();


    // =====================================================
    // RAPORT POZYCJI J1-J5
    // =====================================================

    if (
        homingDone &&
        millis() - lastJointPositionReportMs >=
        JOINT_POSITION_REPORT_INTERVAL_MS
    ) {

        lastJointPositionReportMs =
            millis();

        sendJointPositions();
    }


}