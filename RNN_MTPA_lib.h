#ifndef RNN_MTPA_LIBRARY_H
#define RNN_MTPA_LIBRARY_H

#include <Eigen/Dense>

using namespace Eigen;

class SpeedControllerBrain {
private:
    // Motor Parameters
    double psi_f, Ld, Lq, w_base;
    double learning_rate;

    // LSTM Weights (8x4) & (8x8)
    MatrixXd Wf, Wi, Wo, Wc;
    MatrixXd Uf, Ui, Uo, Uc;

    // LSTM Biases (8x1)
    VectorXd bf, bi, bo, bc;

    // Output Layer Weights (2x8) & Biases (2x1)
    MatrixXd Wout;
    VectorXd bout;

    // LSTM States (8x1)
    VectorXd h_prev, c_prev;

    // LSTM Gate Outputs (for RTRL)
    VectorXd f_gate, i_gate, o_gate, c_tilde;
    VectorXd h_old, c_old;

    // Output Cache
    VectorXd Y_out; // [id_rnn, iq_rnn]

public:
    SpeedControllerBrain();

    // Control Methods
    double calculate_PI(double w_ref, double w_act, double Kp, double Ki, double &integral_err, double dt);
    double calculate_MTPA_Id(double iq_ref);

    // LSTM Methods
    void bumpless_transfer(double id_ref_prev, double iq_ref_prev);
    void calculate_LSTM(const VectorXd& Vt, double &id_rnn, double &iq_rnn);
    void update_RTRL(double error_speed, const VectorXd& Vt);
};

#endif