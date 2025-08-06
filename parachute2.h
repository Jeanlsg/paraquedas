#define SKIB1 13
#define SKIB2 12
#define SAFE_MARGIN_ALTITUDE_ERROR 1

// Em ms
#define TIME_BETWEEN_ACTIVATIONS 8000

// Em metros
#define HEIGHT_FOR_2_STAGE 3

// Em ms
#define SKIB_TIME 1000

bool alreadyDesactivatedBuzzer1 = false;
bool alreadyDesactivatedBuzzer2 = false;

bool parachute1Activated = false;
bool parachute2Activated = false;

double timeForStage1 = 0;
double timeForStage2 = 0;

bool enoughHeight = true;

enum ParachuteState {
  IDLE,
  ASCENDING,
  DROPPING,
  STAGE1_DEPLOYED,
  STAGE2_DEPLOYED,
  FINISHED
};

ParachuteState currentState = IDLE;

void setupSkibPins() {
  pinMode(SKIB1, OUTPUT);
  pinMode(SKIB2, OUTPUT);
  Serial.println("Skibs configurados!");
}

void activateStage1() {
  digitalWrite(SKIB1, HIGH);
  Serial.println("1 Skib ativado!");

  timeForStage1 = millis();
  parachute1Activated = true;
  if (allData.parachute < 1) allData.parachute = 1;

  activateBuzzer();
}

void activateStage2() {
  digitalWrite(SKIB2, HIGH);
  Serial.println("2 Skib ativado!");

  timeForStage2 = millis();
  parachute2Activated = true;
  if (allData.parachute < 2) allData.parachute = 2;

  activateBuzzer();
}

void deactivateStage1() {
  digitalWrite(SKIB1, LOW);
  Serial.println("1 Skib desativado!");
  desactivateBuzzer();
}

void deactivateStage2() {
  digitalWrite(SKIB2, LOW);
  Serial.println("2 Skib desativado!");
  desactivateBuzzer();
}

bool altitudeLessThan(double a1, double a2) {
  return (a2 - a1 > SAFE_MARGIN_ALTITUDE_ERROR);
}

bool checkIsDropping() {
  return altitudeLessThan(altitudeAtual, maximumAltitudeValue);
}

void updateParachuteState() {
  switch (currentState) {

    case IDLE:
      if (altitudeAtual > 5) {
        currentState = ASCENDING;
        Serial.println("Estado: ASCENDING");
      }
      break;

    case ASCENDING:
      if (checkIsDropping()) {
        currentState = DROPPING;
        Serial.println("Estado: DROPPING");
      }
      break;

    case DROPPING:
      activateStage1();
      enoughHeight = altitudeLessThan(HEIGHT_FOR_2_STAGE, maximumAltitudeValue);
      currentState = STAGE1_DEPLOYED;
      Serial.println("Estado: STAGE1_DEPLOYED");
      break;

    case STAGE1_DEPLOYED:
      // Espera condições para stage 2
      if (!parachute2Activated) {
        if (enoughHeight && altitudeLessThan(altitudeAtual, HEIGHT_FOR_2_STAGE)) {
          activateStage2();
          currentState = STAGE2_DEPLOYED;
          Serial.println("Estado: STAGE2_DEPLOYED");
        }

        if (!enoughHeight && (millis() - timeForStage1) > TIME_BETWEEN_ACTIVATIONS) {
          activateStage2();
          currentState = STAGE2_DEPLOYED;
          Serial.println("Estado: STAGE2_DEPLOYED");
        }
      }

      // Desativa stage1 após tempo
      if ((millis() - timeForStage1) >= SKIB_TIME && !alreadyDesactivatedBuzzer1) {
        alreadyDesactivatedBuzzer1 = true;
        deactivateStage1();
      }
      break;

    case STAGE2_DEPLOYED:
      if ((millis() - timeForStage2) >= SKIB_TIME && !alreadyDesactivatedBuzzer2) {
        alreadyDesactivatedBuzzer2 = true;
        deactivateStage2();
        currentState = FINISHED;
        Serial.println("Estado: FINISHED");
      }
      break;

    case FINISHED:
      // Nada mais a fazer
      break;
  }
}
