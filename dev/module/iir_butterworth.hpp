#ifndef META_EMBEDDED_IIR_BUTTERWORTH_HPP
#define META_EMBEDDED_IIR_BUTTERWORTH_HPP
#include <cmath>

/**
 * @brief  Second order IIR Butterworth filter
 * @author Chen Qian
 */
class IIRButterworth {
public:
    /**
     * @brief Constructor of IIRButterWorth Filter
     * @param fcp Cutoff frequency of filter.
     * @param fs  Sampling frequency of system.
     */
    IIRButterworth(double fcp, double fs) {
        change_parameter(fcp,fs);
    }

    /**
     * @brief Update the filter data and get output.
     * @param input_ Input data.
     * @return Filtered data.
     */
    double update(double input_) {
        /// Update the data.
        input[0] = input[1];
        input[1] = input[2];
        input[2] = input_;
        output[0] = output[1];
        output[1] = output[2];
        output[2] = b0 * input[2] + b1 * input[1] + b2 * input[0] - a1 * output[1] - a2 * output[0];
        return output[2];
    }

    /**
     * @brief Change the parameters of butterworth filter.
     * @details The transfer function is
     * @code
     *                     1
     * H(s) = ----------------------------
     *        (s/fcp)^2 + 1.414(s/fcp) + 1
     * @endcode
     * @param cutoff_freq Cut off frequency.
     * @param sample_freq System sample frequency (In our system, usually 1kHz)
     */
    void change_parameter(double cutoff_freq, double sample_freq) {
        const double fr  = sample_freq/cutoff_freq;
        const double ohm = tanf(M_PI/fr);
        const double c   = 1.0f + 2.0f * cosf(M_PI/4.0f) * ohm + ohm*ohm;
        b0 = ohm * ohm/c;
        b1 = 2.0 * b0;
        b2 = b0;
        a1 = 2.0 * (ohm*ohm-1.0)/c;
        a2 = (1.0 - 2.0 * cosf(M_PI/4.0f) * ohm + ohm*ohm)/c;
    }
private:
    double b0{}, b1{}, b2{}, a1{}, a2{};
    double output[3]{};
    double input[3]{};
};

#endif