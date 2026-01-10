#pragma once

typedef struct {
    double x[4];       // stav
    double fx[4];      // predikovaný stav
    double hx[2];      // predikovaný výstup
    double F[4][4];    // jacobian f wrt x
    double H[2][4];    // jacobian h wrt x
    double P[4][4];    // kovariančná matica
    double Q[4][4];    // šum modelu
    double R[2][2];    // šum merania
    double K[4][2];    // Kalmanov zisk
} ekf_t;

void model(ekf_t* ekf, double x[4], double u[1]);
void ekf_step(ekf_t* ekf, double y[2]);  // (z TinyEKF)
