package org.firstinspires.ftc.teamcode.control.motion;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.gainmatrices.ProfileConstraints;

public final class MotionProfiler {

    private State state = new State();

    private final ElapsedTime timer = new ElapsedTime();

    private ProfileConstraints constraints = new ProfileConstraints();

    private MotionProfile profile = MotionProfileGenerator.generateSimpleMotionProfile(
            new MotionState(0, 0),
            new MotionState(0, 0),
            constraints.maxV,
            constraints.maxA,
            constraints.maxJ
    );

    public void setConstraints(ProfileConstraints constraints) {
        this.constraints = constraints;
    }

    public void generateProfile(State current, State target) {
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(current.x, current.v),
                new MotionState(target.x, target.v),
                constraints.maxV,
                constraints.maxA,
                constraints.maxJ
        );
        timer.reset();
    }

    public void update() {
        MotionState mState = profile.get(timer.seconds());
        this.state = new State(
                mState.getX(),
                mState.getV(),
                mState.getA(),
                mState.getJ()
        );
    }

    public double getX() {
        return state.x;
    }

    public double getV() {
        return state.v;
    }

    public double getA() {
        return state.a;
    }

    public double getJ() {
        return state.j;
    }

    public State getState() {
        return state;
    }

    public boolean isDone() {
        return timer.seconds() >= profile.duration();
    }
}
