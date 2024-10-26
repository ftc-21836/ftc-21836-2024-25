package org.firstinspires.ftc.teamcode.subsystems;

import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.FLOAT;
import static org.firstinspires.ftc.teamcode.opmodes.SharedVars.mTelemetry;
import static org.firstinspires.ftc.teamcode.subsystems.Climber.State.INACTIVE;
import static org.firstinspires.ftc.teamcode.subsystems.Climber.State.PULLING_HIGH_RUNG;
import static org.firstinspires.ftc.teamcode.subsystems.Climber.State.PULLING_LOW_RUNG;
import static org.firstinspires.ftc.teamcode.subsystems.Climber.State.RAISING_ABOVE_HIGH_RUNG;
import static org.firstinspires.ftc.teamcode.subsystems.Climber.State.RAISING_ABOVE_LOW_RUNG;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot.getGoBildaServo;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot.getReversedServo;

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
            TIME_OUTER_HOOKS_EXTENSION = 0.5,
            TIME_OUTER_HOOKS_RETRACTION = 0.5,
            TIME_INNER_HOOKS_DISENGAGING = 0.5,
            DURATION_INNER_HOOKS_RETRACTED = 2,
            SPEED_OUTER_HOOKS_EXTENDING = 0.5,
            SPEED_OUTER_HOOKS_RETRACTING = -.1,
            HEIGHT_RUNG_LOW_RAISED = 1,
            HEIGHT_RUNG_LOW_CLIMB_OFFSET = -1,
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
        RAISING_ABOVE_HIGH_RUNG,
        PULLING_HIGH_RUNG,
    }

    boolean isActive() {
        return state != INACTIVE;
    }

    private final Lift lift;

    Climber(HardwareMap hardwareMap, Lift lift) {

        this.lift = lift;

        innerHooks = new SimpleServoPivot(
                ANGLE_HOOKS_RETRACTED,
                ANGLE_HOOKS_EXTENDED,
                getGoBildaServo(hardwareMap, "right inner hook"),
                getReversedServo(getGoBildaServo(hardwareMap, "left inner hook"))
        );

        limiterBars = new SimpleServoPivot(
                ANGLE_BARS_RETRACTED,
                ANGLE_BARS_EXTENDED,
                getGoBildaServo(hardwareMap, "right bar"),
                getReversedServo(getGoBildaServo(hardwareMap, "left bar"))
        );

        outerHooks = new CachedMotorEx(hardwareMap, "outer hooks", Motor.GoBILDA.RPM_1150);
        outerHooks.setZeroPowerBehavior(FLOAT);
    }

    public void retract() {
        switch (state) {

            case RAISING_ABOVE_LOW_RUNG:

                innerHooks.setActivated(false);
                lift.setPosition(0);
                state = INACTIVE;

                break;

            case PULLING_LOW_RUNG:

                state = INACTIVE;
                climb();

                break;
        }
    }

    public void climb() {

        // THESE ALL TRIGGER ONCE WHEN CLIMB BUTTON PRESSED
        // climb button has different behavior for each state:

        switch (state) {
            case INACTIVE:

                innerHooks.setActivated(true);
                lift.setPosition(HEIGHT_RUNG_LOW_RAISED);
                state = RAISING_ABOVE_LOW_RUNG;

                break;

            case RAISING_ABOVE_LOW_RUNG:

                lift.setPosition(HEIGHT_RUNG_LOW_RAISED + HEIGHT_RUNG_LOW_CLIMB_OFFSET);
                state = PULLING_LOW_RUNG;

                timer.reset();

                break;

            case PULLING_LOW_RUNG:

                lift.setPosition(HEIGHT_RUNG_HIGH_RAISED);
                state = RAISING_ABOVE_HIGH_RUNG;

                timer.reset();

                break;

            case RAISING_ABOVE_HIGH_RUNG:

                outerHooks.set(SPEED_OUTER_HOOKS_RETRACTING);
                timer.reset();
                lift.setPosition(HEIGHT_RUNG_HIGH_RAISED + HEIGHT_RUNG_HIGH_CLIMB_OFFSET);
                state = PULLING_HIGH_RUNG;

                break;

            case PULLING_HIGH_RUNG:

                limiterBars.toggle();

                break;
        }
    }

    void run() {

        double seconds = timer.seconds();

        switch (state) {

            case PULLING_LOW_RUNG:

                if (outerHooks.get() == 0) {

                    if (seconds >= TIME_CLIMB_LOW_RUNG) {
                        outerHooks.set(SPEED_OUTER_HOOKS_EXTENDING);
                        timer.reset();
                    }

                } else if (seconds >= TIME_OUTER_HOOKS_EXTENSION) {
                    outerHooks.set(0);
                }

                break;

            case RAISING_ABOVE_HIGH_RUNG:

                innerHooks.setActivated(
                        seconds < TIME_INNER_HOOKS_DISENGAGING ||
                                seconds > TIME_INNER_HOOKS_DISENGAGING + DURATION_INNER_HOOKS_RETRACTED
                );
                break;

            case PULLING_HIGH_RUNG:

                if (seconds >= TIME_OUTER_HOOKS_RETRACTION) {
                    outerHooks.set(0);
                }

                if (lift.currentPosition <= HEIGHT_TO_ACTIVATE_LIMITER_BAR) {
                    limiterBars.setActivated(true);
                }

                break;
        }

        innerHooks.updateAngles(ANGLE_HOOKS_RETRACTED, ANGLE_HOOKS_EXTENDED);
        limiterBars.updateAngles(ANGLE_BARS_RETRACTED, ANGLE_BARS_EXTENDED);

        innerHooks.run();
        limiterBars.run();
    }

    void printTelemetry() {
        mTelemetry.addData("Climber state", state);
    }
}
