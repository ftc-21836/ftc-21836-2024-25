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
import static org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot.getGoBildaServo;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot;
import org.firstinspires.ftc.teamcode.subsystems.utilities.cachedhardware.CachedMotorEx;

@Config
public final class Climber {

    public static double
            ANGLE_HOOKS_RETRACTED = 0,
            ANGLE_HOOKS_EXTENDED = 90,
            ANGLE_BARS_RETRACTED = 0,
            ANGLE_BARS_EXTENDED = 90,

            TIME_CLIMB_LOW_RUNG = 4,
            TIME_OUTER_HOOKS_EXTENSION = 1,
            TIME_OUTER_HOOKS_RETRACTION = 1,
            TIME_INNER_HOOKS_EXTENSION = 1,

            SPEED_OUTER_HOOKS_EXTENDING = 0.5,
            SPEED_OUTER_HOOKS_RETRACTING = -0.1,

            TOLERANCE_CLIMBED = 0.25,
            TOLERANCE_RAISED = 0.25,

            HEIGHT_RUNG_LOW_RAISED = 1,
            HEIGHT_RUNG_LOW_CLIMB_OFFSET = -1,
            HEIGHT_RANGE_INNER_HOOKS_RETRACTED = 2,
            HEIGHT_RUNG_HIGH_RAISED = 1,
            HEIGHT_TO_ACTIVATE_LIMITER_BAR = 1,
            HEIGHT_RUNG_HIGH_CLIMB_OFFSET = -1;

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
                getGoBildaServo(hardwareMap, "right inner hook"),
                getGoBildaServo(hardwareMap, "left inner hook").reversed()
        );

        limiterBars = new SimpleServoPivot(
                ANGLE_BARS_RETRACTED,
                ANGLE_BARS_EXTENDED,
                getGoBildaServo(hardwareMap, "right bar"),
                getGoBildaServo(hardwareMap, "left bar").reversed()
        );

        outerHooks = new CachedMotorEx(hardwareMap, "outer hooks", Motor.GoBILDA.RPM_1150);
        outerHooks.setZeroPowerBehavior(FLOAT);
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

        // THESE ALL TRIGGER ONCE WHEN CLIMB BUTTON PRESSED
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

                if (lift.getPosition() <= lift.getTarget() + TOLERANCE_CLIMBED) {

                    outerHooks.set(SPEED_OUTER_HOOKS_EXTENDING);
                    state = OUTER_HOOKS_EXTENDING;
                    timer.reset();

                } else break;

            case OUTER_HOOKS_EXTENDING:

                if (timer.seconds() >= TIME_OUTER_HOOKS_EXTENSION) {

                    lift.setTarget(HEIGHT_RUNG_HIGH_RAISED);
                    state = RAISING_ABOVE_HIGH_RUNG;

                } else break;

            case RAISING_ABOVE_HIGH_RUNG:

                boolean hooksDisengaged = lift.getPosition() >= HEIGHT_RUNG_LOW_RAISED;

                if (hooksDisengaged) outerHooks.set(0);

                innerHooks.setActivated(!hooksDisengaged);

                if (lift.getPosition() >= HEIGHT_RUNG_HIGH_RAISED - TOLERANCE_RAISED) {

                    innerHooks.setActivated(true);
                    state = INNER_HOOKS_EXTENDING;
                    timer.reset();

                } else break;

            case INNER_HOOKS_EXTENDING:

                if (timer.seconds() >= TIME_INNER_HOOKS_EXTENSION) {

                    outerHooks.set(SPEED_OUTER_HOOKS_RETRACTING);
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
