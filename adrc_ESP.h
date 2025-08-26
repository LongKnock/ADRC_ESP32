#ifndef ADRC_ESP_H
#define ADRC_ESP_H

#include <Arduino.h>
#include <ArduinoEigen.h>
using namespace Eigen;

class ADRC_ESP 
{
    public:
        ADRC_ESP(const float b0, unsigned long t_sampling,
                 float t_settling,
                 float u_min,
                 float u_max,
                 const int k_eso, const int DeadZonePositif, const int DeadZoneNegatif);

        void initADRC();
        void updateEso(float y, float u, float u_sat);
        float compensationDeadZone(float Uncompensated);

        // Compute the control signal with filtering and rate limit
        int computeControlSignal(float setpoint, float speed, unsigned long sampleTimeMs);
    private:
        unsigned long lastTimeControl_;
        float t_sampling_;
        float b0_;
        float t_settling_;
        int k_eso_;

        const float u_min_;
        const float u_max_;

        // ESO tuning parameters
        float s_cl_;
        float s_eso_;
        float l1_, l2_, l3_;

        // Control gains
        float Kp_, Kd_;
        float rate_limit = 30;

        const int DeadZonePositif_;
        const int DeadZoneNegatif_;

        // State-space matrices
        MatrixXf Ad_, B_, L_, E;
        MatrixXf x_hat_; // Estimated states
        float u_, u_sat_, u_prev_; // Control output and previous control output
};       

#endif