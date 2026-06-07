#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME Speed_CTRL
#include "simstruc.h"
#include <cmath>

// include RNN lib
#include "RNN_MTPA_lib.h"

#define U(element) (*uPtrs[element])

//instance for the created class
SpeedControllerBrain AI_Brain;

// Helper Sigmoid
auto sigmoid = [](double x) { return 1.0 / (1.0 + std::exp(-x)); };

SpeedControllerBrain::SpeedControllerBrain() {
    psi_f = 0.062; Ld = 0.0002; Lq = 0.0004;
    w_base = 314.0; // Base speed threshold
    learning_rate = 1e-4;

    // initialize Weights & Biases (-0.1 to 0.1) using Eigen Random
    Wf = MatrixXd::Random(8, 4) * 0.1; Wi = MatrixXd::Random(8, 4) * 0.1;
    Wo = MatrixXd::Random(8, 4) * 0.1; Wc = MatrixXd::Random(8, 4) * 0.1;

    Uf = MatrixXd::Random(8, 8) * 0.1; Ui = MatrixXd::Random(8, 8) * 0.1;
    Uo = MatrixXd::Random(8, 8) * 0.1; Uc = MatrixXd::Random(8, 8) * 0.1;

    bf = VectorXd::Random(8) * 0.1; bi = VectorXd::Random(8) * 0.1;
    bo = VectorXd::Random(8) * 0.1; bc = VectorXd::Random(8) * 0.1;

    Wout = MatrixXd::Zero(2, 8);
    bout = VectorXd::Zero(2);
    bout(0) = -5.0 / 300.0;  // Initial Id_rnn (Dinormalisasi)
    bout(1) = 15.0 / 100.0;  // Initial Iq_rnn (Dinormalisasi)

    h_prev = VectorXd::Zero(8);
    c_prev = VectorXd::Zero(8);
}

double SpeedControllerBrain::calculate_PI(double w_ref, double w_act, double Kp, double Ki, double &integral_err, double dt) {
    double err = w_ref - w_act;
    integral_err += (Ki * err * dt);

    // Anti-windup
    if (integral_err > 100.0) integral_err = 100.0;
    if (integral_err < -100.0) integral_err = -100.0;

    double iq_pi = (Kp * err) + integral_err;
    if (iq_pi > 100.0) iq_pi = 100.0;
    if (iq_pi < -100.0) iq_pi = -100.0;

    return iq_pi;
}

double SpeedControllerBrain::calculate_MTPA_Id(double iq_ref) {
    double term1 = psi_f / (4.0 * (Lq - Ld));
    return term1 - std::sqrt(std::pow(term1, 2) + (std::pow(iq_ref, 2) / 2.0));
}

void SpeedControllerBrain::bumpless_transfer(double current_id, double current_iq) {
    bout(0) = current_id / 300.0;
    bout(1) = current_iq / 100.0;
    h_prev.setZero();
    c_prev.setZero();
}

void SpeedControllerBrain::calculate_LSTM(const VectorXd& Vt, double &id_rnn, double &iq_rnn) {
    // Save old states for RTRL
    h_old = h_prev;
    c_old = c_prev;

    // LSTM Gates
    f_gate = (Wf * Vt + Uf * h_prev + bf).unaryExpr(sigmoid);
    i_gate = (Wi * Vt + Ui * h_prev + bi).unaryExpr(sigmoid);
    o_gate = (Wo * Vt + Uo * h_prev + bo).unaryExpr(sigmoid);
    c_tilde = (Wc * Vt + Uc * h_prev + bc).array().tanh();

    // Update Cell & Hidden
    c_prev = (f_gate.array() * c_prev.array()) + (i_gate.array() * c_tilde.array());
    h_prev = o_gate.array() * c_prev.array().tanh();

    // Output Layer
    Y_out = Wout * h_prev + bout;

    // Denormalize (Asumsi skala output 50A)
    id_rnn = Y_out(0) * 300.0;
    iq_rnn = Y_out(1) * 100.0;

    if (id_rnn > 0.0) id_rnn = 0.0;
    if (id_rnn < -300.0) id_rnn = -300.0;
    if (iq_rnn > 100.0) iq_rnn = 100.0;
    if (iq_rnn < -100.0) iq_rnn = -100.0;
}

void SpeedControllerBrain::update_RTRL(double error_speed, const VectorXd& Vt) {
    // 4. PERBAIKAN ARAH FISIKA GRADIENT (SANGAT PENTING!)
    VectorXd error_vec(2);
    // Jika kecepatan kurang (error_speed > 0), Id harus LEBIH NEGATIF (Flux Weakening)
    error_vec(0) = -error_speed;
    // Jika kecepatan kurang (error_speed > 0), Iq harus LEBIH POSITIF (Torsi)
    error_vec(1) = error_speed;

    // Gradient Output Layer
    MatrixXd grad_Wout = error_vec * h_prev.transpose();
    VectorXd grad_bout = error_vec;

    // Propagasi Error ke Hidden Layer
    VectorXd delta_h = Wout.transpose() * error_vec;

    // Chain Rule LSTM
    VectorXd tanh_c = c_prev.array().tanh();
    VectorXd delta_o = delta_h.array() * tanh_c.array() * o_gate.array() * (1.0 - o_gate.array());
    VectorXd delta_c = delta_h.array() * o_gate.array() * (1.0 - tanh_c.array().square());

    VectorXd delta_f = delta_c.array() * c_old.array() * f_gate.array() * (1.0 - f_gate.array());
    VectorXd delta_i = delta_c.array() * c_tilde.array() * i_gate.array() * (1.0 - i_gate.array());
    VectorXd delta_ctilde = delta_c.array() * i_gate.array() * (1.0 - c_tilde.array().square());


    // 4. UPDATE SEMUA BOBOT & BIAS (Gradient Descent)

    double decay = 1e-5; // Faktor peluruhan sangat kecil

    Wout = Wout * (1.0 - decay) + (learning_rate * grad_Wout);
    bout = bout * (1.0 - decay) + (learning_rate * grad_bout);

    Wf = Wf * (1.0 - decay) + (learning_rate * (delta_f * Vt.transpose()));
    Uf = Uf * (1.0 - decay) + (learning_rate * (delta_f * h_old.transpose()));
    bf = bf * (1.0 - decay) + (learning_rate * delta_f);

    Wi = Wi * (1.0 - decay) + (learning_rate * (delta_i * Vt.transpose()));
    Ui = Ui * (1.0 - decay) + (learning_rate * (delta_i * h_old.transpose()));
    bi = bi * (1.0 - decay) + (learning_rate * delta_i);

    Wo = Wo * (1.0 - decay) + (learning_rate * (delta_o * Vt.transpose()));
    Uo = Uo * (1.0 - decay) + (learning_rate * (delta_o * h_old.transpose()));
    bo = bo * (1.0 - decay) + (learning_rate * delta_o);

    Wc = Wc * (1.0 - decay) + (learning_rate * (delta_ctilde * Vt.transpose()));
    Uc = Uc * (1.0 - decay) + (learning_rate * (delta_ctilde * h_old.transpose()));
    bc = bc * (1.0 - decay) + (learning_rate * delta_ctilde);
}


// ============================================================================
// TEMPLATE S-FUNCTION SIMULINK
// ============================================================================

#ifdef __cplusplus
extern "C" {
#endif

static void mdlInitializeSizes(SimStruct *S){
    ssSetNumDiscStates(S, 6);
    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, 6);
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortOverWritable(S, 0, 1);
    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, 2);
    ssSetNumSampleTimes(S, 1);

    ssSetOptions(S, (SS_OPTION_EXCEPTION_FREE_CODE | SS_OPTION_DISCRETE_VALUED_OUTPUT));
}

static void mdlInitializeSampleTimes(SimStruct *S){
    ssSetSampleTime(S, 0, 1e-4);
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_INITIALIZE_CONDITIONS
static void mdlInitializeConditions(SimStruct *S){
    real_T *X0 = ssGetRealDiscStates(S);
    for (int i=0; i < ssGetNumDiscStates(S); i++) {
        X0[i] = 0.0;
    }
}

static void mdlOutputs(SimStruct *S, int_T tid){
    real_T *Y = ssGetOutputPortRealSignal(S,0);
    real_T *X = ssGetRealDiscStates(S);

    // Output dari kalkulasi mdlUpdate sebelumnya
    Y[0] = X[1];
    Y[1] = X[2];
}

#define MDL_UPDATE
static void mdlUpdate(SimStruct *S, int_T tid) {
    real_T *X = ssGetRealDiscStates(S);
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);
    real_T dt = 1e-4;

    double w_ref  = U(0) * ((2 * 3.14) / 60);
    double Kp     = U(1);
    double Ki     = U(2);
    double id_act = U(3);
    double iq_act = U(4);
    double w_act  = U(5) * ((2 * 3.14) / 60);

    double integral_err = X[0];
    double id_ref_prev  = X[1];
    double iq_ref_prev  = X[2];
    double rnn_active_flag = X[3];
    double id_lpf = X[4];
    double iq_lpf = X[5];

    double id_ref_out, iq_ref_out;
    double w_base = 314.0;

    //digital low pass filter
    double alpha = 0.001;
    id_lpf = (1.0 - alpha) * id_lpf + (alpha * id_act);
    iq_lpf = (1.0 - alpha) * iq_lpf + (alpha * iq_act);

    // 1. Hitung Speed PI
    double iq_pi = AI_Brain.calculate_PI(w_ref, w_act, Kp, Ki, integral_err, dt);

    // 2. Switching Logic
    if (std::abs(w_act) < w_base) {
        // MTPA Mode
        iq_ref_out = iq_pi;
        id_ref_out = AI_Brain.calculate_MTPA_Id(iq_ref_out);
        rnn_active_flag = 0.0;
    } else {
        // LSTM Mode
        if (rnn_active_flag == 0.0) {
            AI_Brain.bumpless_transfer(id_ref_prev, iq_ref_prev);
            rnn_active_flag = 1.0;
        }

        VectorXd Vt(4);
        Vt << id_lpf/300.0, iq_lpf/100.0, w_act/600.0, w_ref/600.0;

        AI_Brain.calculate_LSTM(Vt, id_ref_out, iq_ref_out);

        double error_speed = (w_ref - w_act) / 600.0;
        AI_Brain.update_RTRL(error_speed, Vt);
    }

    // Simpan State
    X[0] = integral_err;
    X[1] = id_ref_out;
    X[2] = iq_ref_out;
    X[3] = rnn_active_flag;
    X[4] = id_lpf;
    X[5] = iq_lpf;
}

static void mdlTerminate(SimStruct *S) { }

#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif

#ifdef __cplusplus
}
#endif