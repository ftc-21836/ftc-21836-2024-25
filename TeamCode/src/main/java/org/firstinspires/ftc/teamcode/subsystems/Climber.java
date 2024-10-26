package org.firstinspires.ftc.teamcode.subsystems;

import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.FLOAT;
import static org.firstinspires.ftc.teamcode.subsystems.Climber.State.ABOVE_HIGH_RUNG;
import static org.firstinspires.ftc.teamcode.subsystems.Climber.State.ABOVE_LOW_RUNG;
import static org.firstinspires.ftc.teamcode.subsystems.Climber.State.CLIMBING_HIGH_RUNG;
import static org.firstinspires.ftc.teamcode.subsystems.Climber.State.CLIMBING_LOW_RUNG;
import static org.firstinspires.ftc.teamcode.subsystems.Climber.State.INACTIVE;
import static org.firstinspires.ftc.teamcode.subsystems.Climber.State.OUTER_HOOKS_ENGAGING;
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
        ABOVE_LOW_RUNG,
        CLIMBING_LOW_RUNG,
        OUTER_HOOKS_ENGAGING,
        ABOVE_HIGH_RUNG,
        CLIMBING_HIGH_RUNG,
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

    public void climb() {
        switch (state) {
            case INACTIVE:

                innerHooks.setActivated(true);
                lift.setPosition(HEIGHT_RUNG_LOW_RAISED);
                state = ABOVE_LOW_RUNG;

                break;

            case ABOVE_LOW_RUNG:

                lift.setPosition(HEIGHT_RUNG_LOW_RAISED + HEIGHT_RUNG_LOW_CLIMB_OFFSET);
                state = CLIMBING_LOW_RUNG;

                break;

            case CLIMBING_LOW_RUNG:

                outerHooks.set(SPEED_OUTER_HOOKS_EXTENDING);
                state = OUTER_HOOKS_ENGAGING;

                break;

            case OUTER_HOOKS_ENGAGING:

                outerHooks.set(0);
                lift.setPosition(HEIGHT_RUNG_HIGH_RAISED);
                state = ABOVE_HIGH_RUNG;

                timer.reset();

                break;

            case ABOVE_HIGH_RUNG:

                outerHooks.set(SPEED_OUTER_HOOKS_RETRACTING);
                lift.setPosition(HEIGHT_RUNG_HIGH_RAISED + HEIGHT_RUNG_HIGH_CLIMB_OFFSET);
                state = CLIMBING_HIGH_RUNG;

                break;

            case CLIMBING_HIGH_RUNG:

                if (outerHooks.get() != 0) outerHooks.set(0);
                else limiterBars.setActivated(true);
                break;
        }
    }

    void run() {

        if (state == ABOVE_HIGH_RUNG) innerHooks.setActivated(
                timer.seconds() < TIME_INNER_HOOKS_DISENGAGING ||
                        timer.seconds() > TIME_INNER_HOOKS_DISENGAGING + DURATION_INNER_HOOKS_RETRACTED
        );

        innerHooks.updateAngles(ANGLE_HOOKS_RETRACTED, ANGLE_HOOKS_EXTENDED);
        limiterBars.updateAngles(ANGLE_BARS_RETRACTED, ANGLE_BARS_EXTENDED);

        innerHooks.run();
        limiterBars.run();
    }
}
