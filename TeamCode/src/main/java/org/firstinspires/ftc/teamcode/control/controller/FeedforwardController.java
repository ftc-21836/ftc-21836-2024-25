package org.firstinspires.ftc.teamcode.control.controller;

import org.firstinspires.ftc.teamcode.control.gainmatrix.FeedforwardGains;
import org.firstinspires.ftc.teamcode.control.motion.State;

public class FeedforwardController implements Controller {

    private FeedforwardGains gains;

    private State target = new State();

    public FeedforwardController() {
        this(new FeedforwardGains());
    }

    public FeedforwardController(FeedforwardGains gains) {
        setGains(gains);
    }

    public void setGains(FeedforwardGains gains) {
        this.gains = gains;
    }

    public double calculate(double additionalOutput) {
        double baseOutput = target.times(gains).sum();
        return (Math.signum(baseOutput + additionalOutput) * gains.kStatic + baseOutput);
    }

    public double calculate() {
        return calculate(0.0);
    }

    /**
     * @param target The V and A attributes of the {@link State} parameter are used as velocity and acceleration references
     */
    public void setTarget(State target) {
        this.target = target;
    }
}
