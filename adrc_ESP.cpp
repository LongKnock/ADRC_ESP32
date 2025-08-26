#include "adrc_ESP.h"

ADRC_ESP::ADRC_ESP(const float b0, unsigned long t_sampling,
                 float t_settling,
                 const float u_min,
                 const float u_max,
                 const int k_eso, const int DeadZonePositif, const int DeadZoneNegatif)
    : b0_(b0), t_sampling_(t_sampling), t_settling_(t_settling),
      u_min_(u_min), u_max_(u_max), k_eso_(k_eso), DeadZonePositif_(DeadZonePositif), DeadZoneNegatif_(DeadZoneNegatif)
{
   
  
};

// For debugging: Function to print matrices
void printMatrix(const Eigen::MatrixXf& mat, const char* name) {
    Serial.print(name);
    Serial.print(" (");
    Serial.print(mat.rows());
    Serial.print("x");
    Serial.print(mat.cols());
    Serial.println("):");

    for (int i = 0; i < mat.rows(); i++) {
        for (int j = 0; j < mat.cols(); j++) {
            Serial.print(mat(i, j)); 
            Serial.print("\t");
        }
        Serial.println();
    }
    Serial.println();
}

void ADRC_ESP::initADRC() 
{
    // Initialize state-space matrices
    MatrixXf A_, C_, I_;
    
    // Initialize ADRC parameters
    s_cl_ = -6.0 / t_settling_;
    s_eso_ = k_eso_ * s_cl_;
    l1_ = -3*s_eso_;
    l2_ = 3*pow(s_eso_, 2);
    l3_ = -pow(s_eso_, 3);
    Kp_ = pow(s_cl_, 2);
    Kd_ = -2*s_cl_;
    rate_limit = (u_max_ - u_min_) * 0.1;
    u_ = 0;
    u_prev_ = 0;
    t_sampling_ = t_sampling_ / 1000.0; // Convert ms to seconds
    Serial.println(t_sampling_);

    I_ = MatrixXf::Identity(3, 3);

    A_ = MatrixXf(3, 3);
    A_ <<   0, 1, 0,
            0, 0, 1,
            0, 0, 0;

    B_ = MatrixXf(3, 1);
    B_ <<   0,
            b0_,
            0; 

    C_ = MatrixXf(1, 3);
    C_ << 1, 0, 0;

    L_ = MatrixXf(3, 1);
    L_ << l1_,
          l2_,
          l3_;

    E = MatrixXf(3, 1);
    E << 0,
         0,
         b0_*1.5;   // Anti-windup gain (TUNING PARAMETER)

    Ad_ = (I_ - t_sampling_ * (A_ - L_*C_)).inverse(); 
    
    x_hat_ = MatrixXf::Zero(3, 1);

    printMatrix(Ad_, "Ad");
    printMatrix(L_, "L");
    printMatrix(B_, "B");
    printMatrix(x_hat_, "x_hat");

    
}
// Update the Extended State Observer (ESO)
void ADRC_ESP::updateEso(float y, float u, float u_sat) 
{
    // ESO equations with anti-windup
    x_hat_ = Ad_*(x_hat_ + t_sampling_*(B_*u_sat + L_*y + E*(u_sat-u)));

    const float X_MAX = 1e6; 
    const float X_MAX1 = 370; 

    for (int i = 0; i < 3; i++) 
    {
        if (i == 0)
        {
            if (isnan(x_hat_(i,0)) || isinf(x_hat_(i,0))) x_hat_(i,0) = 0.0;
            else x_hat_(0,0) = constrain(x_hat_(0,0), -X_MAX1, X_MAX1);
        }
        else
        {
            if (isnan(x_hat_(i,0)) || isinf(x_hat_(i,0))) x_hat_(i,0) = 0.0;
            else x_hat_(i,0) = constrain(x_hat_(i,0), -X_MAX, X_MAX);
        } 
    }  
    // printMatrix(x_hat_, "x_hat"); // Debug state estimation
}

int ADRC_ESP::computeControlSignal(float setpoint, float speed, unsigned long sampleTimeMs) 
{   
    unsigned long currentTime = millis();
    unsigned long elapsed = currentTime - lastTimeControl_;
    if (elapsed < sampleTimeMs) return u_prev_ = static_cast<int>(u_prev_);
    else 
    {
        updateEso(speed, u_, u_sat_);
        float e = setpoint - x_hat_(0, 0); 
        float u0 = (Kp_ * e - Kd_*x_hat_(1, 0));
        u_ = (u0 - x_hat_(2,0))/b0_;

        u_sat_ = constrain(u_, u_min_, u_max_);

        // Rate limiting
        // Example: limit rate of change to 10% of range per update
        if (u_sat_ - u_prev_ > rate_limit) {
            u_sat_ = u_prev_ + rate_limit;
        } else if (u_ - u_prev_ < -rate_limit) {
            u_sat_ = u_prev_ - rate_limit;
        }
        u_prev_ = u_sat_;
        return u_sat_ = static_cast<int>(u_sat_);
    }
}


