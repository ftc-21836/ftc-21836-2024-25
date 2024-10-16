package org.firstinspires.ftc.teamcode.control.controllers;

import static java.lang.Math.abs;
import static java.lang.Math.signum;

import org.firstinspires.ftc.teamcode.control.motion.Differentiator;
import org.firstinspires.ftc.teamcode.control.motion.Integrator;
import org.firstinspires.ftc.teamcode.control.motion.State;
import org.firstinspires.ftc.teamcode.control.filters.Filter;
import org.firstinspires.ftc.teamcode.control.filters.NoFilter;
import org.firstinspires.ftc.teamcode.control.gainmatrices.PIDGains;

public class PIDController implements FeedbackController {

    private PIDGains gains = new PIDGains();
    private State target = new State();

    private final Filter derivFilter;
    private final Differentiator differentiator = new Differentiator();
    private final Differentiator filterDiff = new Differentiator();
    private final Integrator integrator = new Integrator();

    private State error = new State();
    private double errorIntegral, filteredErrorDerivative, rawErrorDerivative;

    public PIDController() {
        this(new NoFilter());
    }

    public PIDController(Filter derivFilter) {
        this.derivFilter = derivFilter;
    }

    public void setGains(PIDGains gains) {
        this.gains = gains;
    }

    /**
     * @param measurement Only the X attribute of the {@link State} parameter is used as feedback
     */
    public double calculate(State measurement) {
        State lastError = error;
        error = target.minus(measurement);

        if (signum(error.x) != signum(lastError.x)) reset();
        errorIntegral = integrator.getIntegral(error.x);
        rawErrorDerivative = differentiator.getDerivative(error.x);
        filteredErrorDerivative = filterDiff.getDerivative(derivFilter.calculate(error.x));

        double output = (gains.kP * error.x) + (gains.kI * errorIntegral) + (gains.kD * filteredErrorDerivative);

        stopIntegration(abs(output) >= gains.maxOutputWithIntegral && signum(output) == signum(error.x));

        return output;
    }

    public void setTarget(State target) {
        this.target = target;
    }

    public double getFilteredErrorDerivative() {
        return filteredErrorDerivative;
    }

    public double getRawErrorDerivative() {
        return rawErrorDerivative;
    }

    public double getErrorIntegral() {
        return errorIntegral;
    }

    public void stopIntegration(boolean stopIntegration) {
        integrator.stopIntegration(stopIntegration);
    }

    public void reset() {
        integrator.reset();
        derivFilter.reset();
    }
}
