package org.firstinspires.ftc.teamcode.control.filter;

import static com.qualcomm.robotcore.util.Range.clip;

import com.qualcomm.robotcore.util.ElapsedTime;

public final class SlewRateLimiter implements Filter {

    private double posLimit, negLimit, m_prevVal, m_prevTime;
    private final ElapsedTime timer = new ElapsedTime();

    public SlewRateLimiter(double posLimit, double negLimit) {
        setLimits(posLimit, negLimit);
        reset();
    }

    public SlewRateLimiter(double limit) {
        this(limit, -limit);
    }

    public void setLimits(double posLimit, double negLimit) {
        this.posLimit = posLimit;
        this.negLimit = negLimit;
    }

    public void setLimit(double limit) {
        setLimits(limit, -limit);
    }

    public double calculate(double input) {
        double currentTime = timer.seconds();
        double elapsedTime = currentTime - m_prevTime;
        m_prevTime = currentTime;
        return m_prevVal += clip(
                input - m_prevVal,
                negLimit * elapsedTime,
                posLimit * elapsedTime
        );
    }

    @Override
    public void reset() {
        m_prevVal = 0;
        m_prevTime = 0;
        timer.reset();
    }
}
