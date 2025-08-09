// KalmanFilter.h - Kalman 2D (altitude, velocidade)
#ifndef KalmanFilter_h
#define KalmanFilter_h

class KalmanFilter2D {
public:
    // q_pos, q_vel: ruído de processo (posição/velocidade)
    // r_meas: ruído da medição de altitude (variância em m^2)
    KalmanFilter2D(double q_pos = 0.05, double q_vel = 0.3, double r_meas = 4.0) {
        Q00 = q_pos; Q11 = q_vel;
        R = r_meas;

        // estado inicial
        x0 = 0.0; // altitude
        x1 = 0.0; // velocidade

        // covariância inicial (P)
        P00 = 10.0; P01 = 0.0;
        P10 = 0.0;  P11 = 10.0;
    }

    // medicao_altitude (m), medicao_aceleracao (m/s^2), dt (s)
    void update(double z_alt, double a_meas, double dt) {
        if (dt <= 0.0) dt = 0.02;

        // PREDIÇÃO (modelo: x' = A x + B u)
        double x0_pred = x0 + x1 * dt + 0.5 * a_meas * dt * dt;
        double x1_pred = x1 + a_meas * dt;

        // P' = A P A^T + Q (Q is diag)
        double P00p = P00 + (P01 + P10) * dt + P11 * dt * dt + Q00;
        double P01p = P01 + P11 * dt;
        double P10p = P10 + P11 * dt;
        double P11p = P11 + Q11;

        // UPDATE (medicao de altitude z = H x + w, H = [1 0])
        double S = P00p + R; // inovacao covariance
        double K0 = P00p / S;
        double K1 = P10p / S;

        double y = z_alt - x0_pred; // inovação

        x0 = x0_pred + K0 * y;
        x1 = x1_pred + K1 * y;

        // atualizar P = (I - K H) P'
        P00 = (1.0 - K0) * P00p;
        P01 = (1.0 - K0) * P01p;
        P10 = -K1 * P00p + P10p;
        P11 = -K1 * P01p + P11p;
    }

    double getAltitude() const { return x0; }
    double getVelocity() const { return x1; }

    // opcional: permite ajustar R em runtime
    void setR(double r) { R = r; }
    void setQ(double q_pos, double q_vel) { Q00 = q_pos; Q11 = q_vel; }

private:
    // estado
    double x0, x1; // altitude, velocidade

    // covariância P (2x2)
    double P00, P01, P10, P11;

    // processos de ruído (diagonal)
    double Q00, Q11;

    // ruído de medição (alt)
    double R;
};

#endif
