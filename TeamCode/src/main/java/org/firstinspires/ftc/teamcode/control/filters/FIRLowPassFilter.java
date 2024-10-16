package org.firstinspires.ftc.teamcode.control.filters;

import org.firstinspires.ftc.teamcode.control.gainmatrices.LowPassGains;

import java.util.ArrayList;

/**
 * Finite impulse response low-pass filter
 */
public final class FIRLowPassFilter implements Filter {
    private LowPassGains gains;
    private final ArrayList<Double> values = new ArrayList<>();

    public FIRLowPassFilter() {
        this(new LowPassGains());
    }

    public FIRLowPassFilter(LowPassGains gains) {
        setGains(gains);
    }

    public void setGains(LowPassGains gains) {
        this.gains = new LowPassGains(gains.gain, Math.max(gains.count, 2));
    }

    public void reset() {
        values.clear();
    }

    public double calculate(double newValue) {
        values.add(newValue);
        if (values.size() < 2) return newValue;
        while (values.size() > gains.count) values.remove(0);

        double estimate = values.get(0);
        for (int ind = 1; ind < values.size(); ind++) {
            estimate = gains.gain * estimate + (1 - gains.gain) * values.get(ind);
        }

        return estimate;
    }
}
