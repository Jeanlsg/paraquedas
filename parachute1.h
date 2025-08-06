#define SKIB1 13
#define SKIB2 12
#define SAFE_MARGIN_ALTITUDE_ERROR 1

#define TIME_BETWEEN_ACTIVATIONS 8000

#define HEIGHT_FOR_2_STAGE 3

#define SKIB_TIME 1000

#define MEDIAN_FILTER_SAMPLES 5 
double altitudeHistory[MEDIAN_FILTER_SAMPLES];
int historyIndex = 0;

int readingsCount = 0; 

bool isDropping = false;
bool enoughHeight = true;

bool parachute1Activated = false;
bool parachute2Activated = false;

double altitudeAtual = 0;
double maximumAltitudeValue = 0;

double timeForStage1 = 0;
double timeForStage2 = 0;

bool alreadyDesactivatedBuzzer1 = false;
bool alreadyDesactivatedBuzzer2 = false;

void sortArray(double arr[], int size) {
  for (int i = 0; i < size - 1; i++) {
    for (int j = 0; j < size - i - 1; j++) {
      if (arr[j] > arr[j + 1]) {
        double temp = arr[j];
        arr[j] = arr[j + 1];
        arr[j + 1] = temp;
      }
    }
  }
}

double getMedianAltitude(double newRawAltitude) {
  altitudeHistory[historyIndex] = newRawAltitude;
  historyIndex = (historyIndex + 1) % MEDIAN_FILTER_SAMPLES;

  if (readingsCount < MEDIAN_FILTER_SAMPLES) {
    readingsCount++;
    return newRawAltitude;
  } 
  else {
    double sortedHistory[MEDIAN_FILTER_SAMPLES];
    for(int i = 0; i < MEDIAN_FILTER_SAMPLES; i++) {
      sortedHistory[i] = altitudeHistory[i];
    }

    sortArray(sortedHistory, MEDIAN_FILTER_SAMPLES);

    return sortedHistory[MEDIAN_FILTER_SAMPLES / 2];
  }
}

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
}

void activateStage2() {
  digitalWrite(SKIB2, HIGH);
  Serial.println("2 Skib ativado!");
  timeForStage2 = millis();
  parachute2Activated = true;
}

void deactivateStage1() {
  digitalWrite(SKIB1, LOW);
  Serial.println("1 Skib desativado!");
}

void deactivateStage2() {
  digitalWrite(SKIB2, LOW);
  Serial.println("2 Skib desativado!");
}

bool altitudeLessThan(double altitude1, double altitude2) {
  return (altitude2 - altitude1 > SAFE_MARGIN_ALTITUDE_ERROR);
}

void activateParachutes() {
  if (parachute1Activated == false) {
    activateStage1();
    enoughHeight = altitudeLessThan(HEIGHT_FOR_2_STAGE, maximumAltitudeValue);
  }

  if (parachute2Activated == false && parachute1Activated) {
    if (enoughHeight && altitudeLessThan(altitudeAtual, HEIGHT_FOR_2_STAGE)) {
      activateStage2();
    }
    if (!enoughHeight && (millis() - timeForStage1) > TIME_BETWEEN_ACTIVATIONS){
      activateStage2();
    }
  }

  if(parachute1Activated && (millis() - timeForStage1) >= SKIB_TIME && !alreadyDesactivatedBuzzer1) {
    alreadyDesactivatedBuzzer1 = true;
    deactivateStage1();
  }

  if(parachute2Activated && (millis() - timeForStage2) >= SKIB_TIME && !alreadyDesactivatedBuzzer2) {
    alreadyDesactivatedBuzzer2 = true;
    deactivateStage2();
  }
}

bool checkIsDropping() {
  return altitudeLessThan(altitudeAtual, maximumAltitudeValue);
}

void checkApogee() {
  if (altitudeAtual > maximumAltitudeValue) {
    maximumAltitudeValue = altitudeAtual;
  }
  
  isDropping = checkIsDropping() || isDropping;

}

void testActivations(int millisStage1, int millisStage2) {
  Serial.println(millis());
  if(millis() > millisStage1 && parachute1Activated == false) {
    activateStage1();
    delay(SKIB_TIME);
    deactivateStage1();
  } else if(millis() > millisStage2  && parachute2Activated == false) {
    activateStage2();
    delay(SKIB_TIME);
    deactivateStage2();
  }
}