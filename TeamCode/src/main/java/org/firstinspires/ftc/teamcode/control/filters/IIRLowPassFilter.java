package org.firstinspires.ftc.teamcode.control.filters;

import org.firstinspires.ftc.teamcode.control.gainmatrices.LowPassGains;

/**
 * Infinite impulse response low-pass filter
 */
public final class IIRLowPassFilter implements Filter {
    private LowPassGains gains;
    private double estimate = Double.NaN;

    public IIRLowPassFilter() {
        this(new LowPassGains());
    }

    public IIRLowPassFilter(LowPassGains gains) {
        setGains(gains);
    }

    public void setGains(LowPassGains gains) {
        this.gains = gains;
    }

    public void reset() {
        estimate = Double.NaN;
    }

    public double calculate(double newValue) {
        estimate = Double.isNaN(estimate) ? newValue : gains.gain * estimate + (1 - gains.gain) * newValue;
        return estimate;
    }
}