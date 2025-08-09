// ---------- parâmetros adicionais para Kalman ----------
#define ALT_MOVING_AVG_N 5         // média móvel para barômetro (anti-spike)
#define APOGEE_CONFIRM_M 4         // confirmações necessárias de velocidade negativa
#define V_NEG_THRESHOLD -0.2       // limiar para considerar velocidade negativa (m/s)
#define V_POS_THRESHOLD 1.0        // limiar para considerar subida inicial (m/s)

// Kalman (2x2) - parâmetros (ajustar conforme teste)
static double KQ00 = 0.05;   // Q pos
static double KQ11 = 0.3;    // Q vel
static double KR  = 9.0;     // R (variância da medição altitude) ~ sigma^2 (m^2)

// estado Kalman
static double k_x_alt = 0.0;
static double k_x_vel = 0.0;
static double k_P00 = 10.0, k_P01 = 0.0, k_P10 = 0.0, k_P11 = 10.0;

// timing para dt
static unsigned long parachute_last_update_ms = 0;
static const double parachute_dt_default = 0.02; // s

// média móvel barômetro
static double altMovBuf[ALT_MOVING_AVG_N];
static int altMovPos = 0;
static int altMovCount = 0;

// contador de confirmações de apogeu (velocidade negativa)
static int apogeeVelNegCount = 0;
static bool apogeeDetectedByKalman = false;

// Helper: média móvel simples
static double movingAvgAlt(double alt_in) {
  altMovBuf[altMovPos] = alt_in;
  altMovPos = (altMovPos + 1) % ALT_MOVING_AVG_N;
  if (altMovCount < ALT_MOVING_AVG_N) altMovCount++;
  double s = 0.0;
  for (int i = 0; i < altMovCount; ++i) s += altMovBuf[i];
  return s / altMovCount;
}

// Atualiza Kalman 2x2 com medicao de altitude (z) e entrada aceleracao (a), dt em segundos
static void kalmanUpdate(double z_alt, double a_meas, double dt) {
  if (dt <= 0.0) dt = parachute_dt_default;

  // Predição
  double alt_pred = k_x_alt + k_x_vel * dt + 0.5 * a_meas * dt * dt;
  double vel_pred = k_x_vel + a_meas * dt;

  // P' = A P A^T + Q (Q diagonal)
  double P00p = k_P00 + (k_P01 + k_P10) * dt + k_P11 * dt * dt + KQ00;
  double P01p = k_P01 + k_P11 * dt;
  double P10p = k_P10 + k_P11 * dt;
  double P11p = k_P11 + KQ11;

  // Update com medição de altitude z
  double S = P00p + KR; // inovação cov
  double K0 = P00p / S;
  double K1 = P10p / S;

  double y = z_alt - alt_pred; // inovação
  k_x_alt = alt_pred + K0 * y;
  k_x_vel = vel_pred + K1 * y;

  // atualizar P
  double P00n = (1.0 - K0) * P00p;
  double P01n = (1.0 - K0) * P01p;
  double P10n = -K1 * P00p + P10p;
  double P11n = -K1 * P01p + P11p;

  k_P00 = P00n; k_P01 = P01n; k_P10 = P10n; k_P11 = P11n;
}

// --------- Função checkIsDropping atualizada (usa Kalman + fallback) ----------
bool checkIsDropping() {
  // atualiza maximumAltitudeValue com leitura bruta
  if (altitudeAtual > maximumAltitudeValue) maximumAltitudeValue = altitudeAtual;

  // 1) média móvel para barômetro e cálculo do dt
  double alt_filtered = movingAvgAlt(altitudeAtual);

  unsigned long now = millis();
  double dt = parachute_dt_default;
  if (parachute_last_update_ms != 0) {
    unsigned long diff = now - parachute_last_update_ms;
    if (diff > 0) dt = diff / 1000.0;
  }
  parachute_last_update_ms = now;

  // 2) pegar aceleração vertical do IMU (usar allData se disponível)
  double acc_z = 0.0;
  #ifdef allData
  acc_z = allData.imuData.accelZ;
  #endif

  // 3) atualizar Kalman
  kalmanUpdate(alt_filtered, acc_z, dt);

  // 4) lógica de detecção por velocidade filtrada (mais robusta)
  // reconhecer transição de >=0 para <0 e confirmar por M amostras
  if (k_x_vel < 0.0 && (k_x_vel + 0.0001) >= 0.0) {
    apogeeVelNegCount = 1;
  } else if (k_x_vel < 0.0) {
    apogeeVelNegCount++;
  } else {
    apogeeVelNegCount = 0;
  }

  if (!apogeeDetectedByKalman && apogeeVelNegCount >= APOGEE_CONFIRM_M) {
    apogeeDetectedByKalman = true;
    return true;
  }

  return false;
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
      // usa detecção robusta (Kalman) para apogeu
      if (checkIsDropping()) {
        currentState = DROPPING;
        Serial.println("Estado: DROPPING");
      }
      break;

    case DROPPING:
      // proteger contra múltiplos acionamentos
      if (!parachute1Activated) {
        activateStage1();
      }
      enoughHeight = altitudeLessThan(HEIGHT_FOR_2_STAGE, maximumAltitudeValue);
      currentState = STAGE1_DEPLOYED;
      Serial.println("Estado: STAGE1_DEPLOYED");
      break;

    case STAGE1_DEPLOYED:
      // espera condições para stage2
      if (!parachute2Activated) {
        if (enoughHeight && altitudeLessThan(altitudeAtual, HEIGHT_FOR_2_STAGE)) {
          activateStage2();
          currentState = STAGE2_DEPLOYED;
          Serial.println("Estado: STAGE2_DEPLOYED");
        } else if (!enoughHeight && (millis() - (unsigned long)timeForStage1) > TIME_BETWEEN_ACTIVATIONS) {
          // se não havia altura suficiente, disparar por timeout
          activateStage2();
          currentState = STAGE2_DEPLOYED;
          Serial.println("Estado: STAGE2_DEPLOYED");
        }
      }

      // desativa stage1 após tempo (com flag para não repetir)
      if ((millis() - (unsigned long)timeForStage1) >= SKIB_TIME && !alreadyDesactivatedBuzzer1) {
        alreadyDesactivatedBuzzer1 = true;
        deactivateStage1();
      }
      break;

    case STAGE2_DEPLOYED:
      if ((millis() - (unsigned long)timeForStage2) >= SKIB_TIME && !alreadyDesactivatedBuzzer2) {
        alreadyDesactivatedBuzzer2 = true;
        deactivateStage2();
        currentState = FINISHED;
        Serial.println("Estado: FINISHED");
      }
      break;

    case FINISHED:
      // nada a fazer
      break;
  }
}
