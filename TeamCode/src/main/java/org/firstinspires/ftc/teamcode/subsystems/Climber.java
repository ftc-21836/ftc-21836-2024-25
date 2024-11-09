package org.firstinspires.ftc.teamcode.subsystems;

import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.FLOAT;
import static org.firstinspires.ftc.teamcode.opmodes.OpModeVars.mTelemetry;
import static org.firstinspires.ftc.teamcode.subsystems.Climber.State.INACTIVE;
import static org.firstinspires.ftc.teamcode.subsystems.Climber.State.INNER_HOOKS_EXTENDING;
import static org.firstinspires.ftc.teamcode.subsystems.Climber.State.OUTER_HOOKS_EXTENDING;
import static org.firstinspires.ftc.teamcode.subsystems.Climber.State.PULLING_HIGH_RUNG;
import static org.firstinspires.ftc.teamcode.subsystems.Climber.State.PULLING_LOW_RUNG;
import static org.firstinspires.ftc.teamcode.subsystems.Climber.State.RAISING_ABOVE_HIGH_RUNG;
import static org.firstinspires.ftc.teamcode.subsystems.Climber.State.RAISING_ABOVE_LOW_RUNG;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.cachedhardware.CachedSimpleServo.getGBServo;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot;
import org.firstinspires.ftc.teamcode.subsystems.utilities.cachedhardware.CachedMotorEx;

@Config
public final class Climber {

    public static double
            ANGLE_HOOKS_RETRACTED = 7,
            ANGLE_HOOKS_EXTENDED = 105.4,

            ANGLE_BARS_RETRACTED = 7,
            ANGLE_BARS_EXTENDED = 97,

            TIME_OUTER_HOOKS_EXTENSION = 1,
            TIME_OUTER_HOOKS_RETRACTION = 1,
            TIME_INNER_HOOKS_EXTENSION = 1,

            SPEED_OUTER_HOOKS = 0.5,

            TOLERANCE_CLIMBED = 0.25,
            TOLERANCE_RAISED = 0.25,

            HEIGHT_RUNG_LOW_RAISED = 12.75,
            HEIGHT_RUNG_LOW_CLIMB_OFFSET = -3,
            HEIGHT_RUNG_HIGH_RAISED = 32,
            HEIGHT_TO_ACTIVATE_LIMITER_BAR = 12,
            HEIGHT_RUNG_HIGH_CLIMB_OFFSET = -20;

    private final SimpleServoPivot innerHooks, limiterBars;

    private final CachedMotorEx outerHooks;

    private State state = INACTIVE;

    private final ElapsedTime timer = new ElapsedTime();

    enum State {
        INACTIVE,
        RAISING_ABOVE_LOW_RUNG,
        PULLING_LOW_RUNG,
        OUTER_HOOKS_EXTENDING,
        RAISING_ABOVE_HIGH_RUNG,
        INNER_HOOKS_EXTENDING,
        PULLING_HIGH_RUNG,
    }

    public boolean isActive() {
        return state != INACTIVE;
    }

    private final Lift lift;

    Climber(HardwareMap hardwareMap, Lift lift) {

        this.lift = lift;

        innerHooks = new SimpleServoPivot(
                ANGLE_HOOKS_RETRACTED,
                ANGLE_HOOKS_EXTENDED,
                getGBServo(hardwareMap, "right inner hook").reversed(),
                getGBServo(hardwareMap, "left inner hook")
        );

        limiterBars = new SimpleServoPivot(
                ANGLE_BARS_RETRACTED,
                ANGLE_BARS_EXTENDED,
                getGBServo(hardwareMap, "right bar").reversed(),
                getGBServo(hardwareMap, "left bar")
        );

        outerHooks = new CachedMotorEx(hardwareMap, "outer hooks", Motor.GoBILDA.RPM_1150);
        outerHooks.setZeroPowerBehavior(FLOAT);
        outerHooks.setInverted(true);
    }

    public void cancelClimb() {
        switch (state) {

            case PULLING_HIGH_RUNG:
                limiterBars.setActivated(false);
                outerHooks.set(0);
            case RAISING_ABOVE_LOW_RUNG:

                innerHooks.setActivated(false);
                lift.setTarget(0);
                state = INACTIVE;

                break;
        }
    }

    public void climb() {
        // climb button has different behavior for each state:
        switch (state) {
            case INACTIVE:

                innerHooks.setActivated(true);
                lift.setTarget(HEIGHT_RUNG_LOW_RAISED);
                state = RAISING_ABOVE_LOW_RUNG;

                break;

            case RAISING_ABOVE_LOW_RUNG:

                lift.setTarget(HEIGHT_RUNG_LOW_RAISED + HEIGHT_RUNG_LOW_CLIMB_OFFSET);
                state = PULLING_LOW_RUNG;

                break;
        }
    }

    void run() {

        switch (state) {

            case PULLING_LOW_RUNG:

                if (lift.getPosition() - TOLERANCE_CLIMBED <= lift.getTarget()) {

                    outerHooks.set(SPEED_OUTER_HOOKS);
                    state = OUTER_HOOKS_EXTENDING;
                    timer.reset();

                } else break;

            case OUTER_HOOKS_EXTENDING:

                if (timer.seconds() >= TIME_OUTER_HOOKS_EXTENSION) {

                    lift.setTarget(HEIGHT_RUNG_HIGH_RAISED);
                    state = RAISING_ABOVE_HIGH_RUNG;

                } else break;

            case RAISING_ABOVE_HIGH_RUNG:

                if (innerHooks.isActivated() && lift.getPosition() >= HEIGHT_RUNG_LOW_RAISED) {
                    outerHooks.set(0);
                    innerHooks.setActivated(false);
                }

                if (lift.getPosition() + TOLERANCE_RAISED >= HEIGHT_RUNG_HIGH_RAISED) {

                    innerHooks.setActivated(true);
                    state = INNER_HOOKS_EXTENDING;
                    timer.reset();

                } else break;

            case INNER_HOOKS_EXTENDING:

                if (timer.seconds() >= TIME_INNER_HOOKS_EXTENSION) {

                    outerHooks.set(-SPEED_OUTER_HOOKS);
                    timer.reset();
                    lift.setTarget(HEIGHT_RUNG_HIGH_RAISED + HEIGHT_RUNG_HIGH_CLIMB_OFFSET);
                    state = PULLING_HIGH_RUNG;

                } else break;

            case PULLING_HIGH_RUNG:

                if (outerHooks.get() != 0 && timer.seconds() >= TIME_OUTER_HOOKS_RETRACTION) outerHooks.set(0);

                if (!limiterBars.isActivated() && lift.getPosition() <= HEIGHT_TO_ACTIVATE_LIMITER_BAR) limiterBars.setActivated(true);

                break;
        }

        innerHooks.updateAngles(ANGLE_HOOKS_RETRACTED, ANGLE_HOOKS_EXTENDED);
        limiterBars.updateAngles(ANGLE_BARS_RETRACTED, ANGLE_BARS_EXTENDED);

        innerHooks.run();
        limiterBars.run();
    }

    void printTelemetry() {
        mTelemetry.addData("CLIMBER: ", state);
    }
}
