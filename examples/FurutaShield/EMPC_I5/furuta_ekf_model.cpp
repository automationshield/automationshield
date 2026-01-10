// ekf_step.cpp – custom EKF step function compatible with furuta_ekf_model.h

#include "furuta_ekf_model.h"
#include <math.h>

const double Ts = 0.01;
const double m1 = 0.095;
const double l1 = 0.085;
const double I1 = 5.42e-5;
const double g = 9.81;
const double L0 = 0.1;
const double b1 = 0.001;


void ekf_step(ekf_t* ekf, double y[2]) {
    // === 1. Predikcia kovariancie: P = F * P * F' + Q ===
    double FP[4][4] = {0};
    double Ft[4][4] = {0};
    double FPFt[4][4] = {0};

    // Ft = F transponovane
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            Ft[i][j] = ekf->F[j][i];

    // FP = F * P
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            for (int k = 0; k < 4; k++)
                FP[i][j] += ekf->F[i][k] * ekf->P[k][j];

    // FPFt = FP * Ft
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++) {
            FPFt[i][j] = 0.0;
            for (int k = 0; k < 4; k++)
                FPFt[i][j] += FP[i][k] * Ft[k][j];
            ekf->P[i][j] = FPFt[i][j] + ekf->Q[i][j];
        }

    // === 2. Kalmanov zisk: K = P * H' * inv(H * P * H' + R) ===
    double Ht[4][2] = {0};
    double PHt[4][2] = {0};
    double HPHt[2][2] = {0};
    double S[2][2] = {0};
    double S_inv[2][2] = {0};

    // Ht = H transponovane
    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 4; j++)
            Ht[j][i] = ekf->H[i][j];

    // PHt = P * Ht
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 2; j++)
            for (int k = 0; k < 4; k++)
                PHt[i][j] += ekf->P[i][k] * Ht[k][j];

    // HPHt = H * PHt
    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 2; j++)
            for (int k = 0; k < 4; k++)
                HPHt[i][j] += ekf->H[i][k] * PHt[k][j];

    // S = HPHt + R
    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 2; j++)
            S[i][j] = HPHt[i][j] + ekf->R[i][j];

    // Inverzia 2x2 matice S
    double det = S[0][0]*S[1][1] - S[0][1]*S[1][0];
    if (fabs(det) < 1e-12) return;  // singularita
    S_inv[0][0] =  S[1][1] / det;
    S_inv[0][1] = -S[0][1] / det;
    S_inv[1][0] = -S[1][0] / det;
    S_inv[1][1] =  S[0][0] / det;

    // Kalmanov zisk K = PHt * S_inv
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 2; j++) {
            ekf->K[i][j] = 0.0;
            for (int k = 0; k < 2; k++)
                ekf->K[i][j] += PHt[i][k] * S_inv[k][j];
        }

    // === 3. Korekcia stavu: x = x + K * (y - hx) ===
    double residual[2];
    for (int i = 0; i < 2; i++)
        residual[i] = y[i] - ekf->hx[i];

    for (int i = 0; i < 4; i++) {
        double delta = 0.0;
        for (int j = 0; j < 2; j++)
            delta += ekf->K[i][j] * residual[j];
        ekf->x[i] += delta;
    }

    // === 4. Korekcia kovariancie: P = (I - K*H) * P ===
    double KH[4][4] = {0};
    double I_KH[4][4] = {0};
    double newP[4][4] = {0};

    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            for (int k = 0; k < 2; k++)
                KH[i][j] += ekf->K[i][k] * ekf->H[k][j];

    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            I_KH[i][j] = (i == j ? 1.0 : 0.0) - KH[i][j];

    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            for (int k = 0; k < 4; k++)
                newP[i][j] += I_KH[i][k] * ekf->P[k][j];

    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            ekf->P[i][j] = newP[i][j];
}
void model(ekf_t* ekf, double x[4], double u[1]) {
    double a_den = m1 * l1 * l1 + I1;

    double ddtheta0 = u[0];
    double ddtheta1 = (
        m1 * l1 * l1 * sin(x[2]) * cos(x[2]) * x[1] * x[1] +
        m1 * g * l1 * sin(x[2]) -
        m1 * l1 * L0 * cos(x[2]) * ddtheta0 -
        b1 * x[3]
    ) / a_den;

    // Výpočet xdot (derivácií stavov)
    double xdot[4];
    xdot[0] = x[1];
    xdot[1] = ddtheta0;
    xdot[2] = x[3];
    xdot[3] = ddtheta1;

    // Eulerov krok: predikovaný stav
    for (int i = 0; i < 4; i++) {
        x[i] += Ts * xdot[i];
        ekf->fx[i] = x[i];
    }

    // Jacobián F (df/dx)
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            ekf->F[i][j] = 0.0;

    ekf->F[0][1] = 1.0;
    ekf->F[2][3] = 1.0;
    ekf->F[3][1] = (2.0 * m1 * l1 * x[1] * cos(x[2]) * sin(x[2])) / a_den;
    ekf->F[3][2] = (
        m1 * l1 * l1 * x[1] * x[1] * (cos(x[2]) * cos(x[2]) - sin(x[2]) * sin(x[2])) +
        m1 * g * l1 * cos(x[2]) +
        L0 * m1 * ddtheta0 * l1 * sin(x[2])
    ) / a_den;
    ekf->F[3][3] = b1 / a_den;

    // Výstupná funkcia h(x)
    ekf->hx[0] = x[0];
    ekf->hx[1] = x[2];

    // Jacobián H = dh/dx
    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 4; j++)
            ekf->H[i][j] = 0.0;

    ekf->H[0][0] = 1.0;
    ekf->H[1][2] = 1.0;
}
