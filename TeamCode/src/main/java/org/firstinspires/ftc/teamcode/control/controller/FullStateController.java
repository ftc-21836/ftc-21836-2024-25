package org.firstinspires.ftc.teamcode.control.controller;

import org.firstinspires.ftc.teamcode.control.motion.State;
import org.firstinspires.ftc.teamcode.control.gainmatrix.FullStateGains;

public class FullStateController implements FeedbackController {

    private FullStateGains gains = new FullStateGains();
    private State target = new State();

    @Override
    public void setTarget(State target) {
        this.target = target;
    }

    public void setGains(FullStateGains gains) {
        this.gains = gains;
    }

    @Override
    public double calculate(State measurement) {
        return target.minus(measurement).times(gains).sum();
    }
}
