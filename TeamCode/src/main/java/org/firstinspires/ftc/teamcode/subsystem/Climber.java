package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.opmode.OpModeVars.mTelemetry;
import static org.firstinspires.ftc.teamcode.subsystem.Climber.State.ACTIVE_HOOKS_EXTENDING;
import static org.firstinspires.ftc.teamcode.subsystem.Climber.State.INACTIVE;
import static org.firstinspires.ftc.teamcode.subsystem.Climber.State.PASSIVE_HOOKS_EXTENDING;
import static org.firstinspires.ftc.teamcode.subsystem.Climber.State.PULLING_HIGH_RUNG;
import static org.firstinspires.ftc.teamcode.subsystem.Climber.State.PULLING_LOW_RUNG;
import static org.firstinspires.ftc.teamcode.subsystem.Climber.State.RAISING_ABOVE_HIGH_RUNG;
import static org.firstinspires.ftc.teamcode.subsystem.Climber.State.RAISING_ABOVE_LOW_RUNG;
import static org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo.getGBServo;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.utility.SimpleServoPivot;

@Config
public final class Climber {

    public static double
            ANGLE_HOOKS_ACTIVE_RETRACTED = 0,
            ANGLE_HOOKS_ACTIVE_EXTENDED = 105.4,

            ANGLE_HOOKS_PASSIVE_RETRACTED = 96.9,
            ANGLE_HOOKS_PASSIVE_EXTENDED = 197,

            ANGLE_BARS_RETRACTED = 5,
            ANGLE_BARS_EXTENDED = 97,

            TIME_PASSIVE_HOOKS_EXTENSION = 1,
            TIME_ACTIVE_HOOKS_EXTENSION = 1,

            HEIGHT_RUNG_LOW_RAISED = 32,
            HEIGHT_RUNG_LOW_CLIMB_OFFSET = -12,
            TOLERANCE_CLIMBED = 0.25,

            HEIGHT_RUNG_HIGH_RAISED = 32,
            TOLERANCE_RAISED = 0.25,
            HEIGHT_RUNG_HIGH_CLIMB_OFFSET = -31.5,

            HEIGHT_TO_RETRACT_PASSIVE_HOOKS = 20,
            HEIGHT_TO_ACTIVATE_LIMITER_BAR = 12;

    private final SimpleServoPivot activeHooks, passiveHooks, limiterBars;

    private State state = INACTIVE;

    private final ElapsedTime timer = new ElapsedTime();

    enum State {
        INACTIVE,
        RAISING_ABOVE_LOW_RUNG,
        PULLING_LOW_RUNG,
        PASSIVE_HOOKS_EXTENDING,
        RAISING_ABOVE_HIGH_RUNG,
        ACTIVE_HOOKS_EXTENDING,
        PULLING_HIGH_RUNG,
    }

    public boolean isActive() {
        return state != INACTIVE;
    }

    private final Lift lift;

    Climber(HardwareMap hardwareMap, Lift lift) {

        this.lift = lift;

        activeHooks = new SimpleServoPivot(
                ANGLE_HOOKS_ACTIVE_RETRACTED,
                ANGLE_HOOKS_ACTIVE_EXTENDED,
                getGBServo(hardwareMap, "right active hook").reversed(),
                getGBServo(hardwareMap, "left active hook")
        );

        limiterBars = new SimpleServoPivot(
                ANGLE_BARS_RETRACTED,
                ANGLE_BARS_EXTENDED,
                getGBServo(hardwareMap, "right bar").reversed(),
                getGBServo(hardwareMap, "left bar")
        );

        passiveHooks = new SimpleServoPivot(
                ANGLE_HOOKS_PASSIVE_RETRACTED,
                ANGLE_HOOKS_PASSIVE_EXTENDED,
                getGBServo(hardwareMap, "right passive hook").reversed(),
                getGBServo(hardwareMap, "left passive hook")
        );
    }

    public void cancelClimb() {
        switch (state) {

            case PULLING_HIGH_RUNG:
                limiterBars.setActivated(false);
            case RAISING_ABOVE_LOW_RUNG:

                activeHooks.setActivated(false);
                lift.setTarget(0);
                state = INACTIVE;

                break;

            case PULLING_LOW_RUNG:
                state = INACTIVE;
                climb();
        }
    }

    public void climb() {
        switch (state) {
            case INACTIVE:

                activeHooks.setActivated(true);
                lift.setTarget(HEIGHT_RUNG_LOW_RAISED);
                state = RAISING_ABOVE_LOW_RUNG;

                break;

            case RAISING_ABOVE_LOW_RUNG:

                lift.setTarget(HEIGHT_RUNG_LOW_RAISED + HEIGHT_RUNG_LOW_CLIMB_OFFSET);
                state = PULLING_LOW_RUNG;

                break;
                
            case PULLING_LOW_RUNG:

                passiveHooks.setActivated(true);
                state = PASSIVE_HOOKS_EXTENDING;
                timer.reset();
                
                break;
                
            case PASSIVE_HOOKS_EXTENDING:

                lift.setTarget(HEIGHT_RUNG_HIGH_RAISED);
                state = RAISING_ABOVE_HIGH_RUNG;
                
                break;
                
            case RAISING_ABOVE_HIGH_RUNG:

                if (activeHooks.isActivated()) activeHooks.setActivated(false);
                else {
                    activeHooks.setActivated(true);
                    state = ACTIVE_HOOKS_EXTENDING;
                    timer.reset();
                }
                
                break;
                
            case ACTIVE_HOOKS_EXTENDING:

                lift.setTarget(HEIGHT_RUNG_HIGH_RAISED + HEIGHT_RUNG_HIGH_CLIMB_OFFSET);
                state = PULLING_HIGH_RUNG;
                
                break;
        }
    }

    void run() {

        switch (state) {

            // case PULLING_LOW_RUNG:

            //     if (lift.getPosition() - TOLERANCE_CLIMBED <= lift.getTarget()) climb();
            //     else break;

            case PASSIVE_HOOKS_EXTENDING:

               if (timer.seconds() >= TIME_PASSIVE_HOOKS_EXTENSION) climb();
               else break;

            case RAISING_ABOVE_HIGH_RUNG:

                if (lift.getPosition() >= HEIGHT_RUNG_LOW_RAISED) activeHooks.setActivated(false);

                if (lift.getPosition() + TOLERANCE_RAISED >= lift.getTarget()) climb();
                else break;

            case ACTIVE_HOOKS_EXTENDING:

                if (timer.seconds() >= TIME_ACTIVE_HOOKS_EXTENSION) climb();
                else break;

            case PULLING_HIGH_RUNG:

                if (lift.getPosition() <= HEIGHT_TO_RETRACT_PASSIVE_HOOKS) passiveHooks.setActivated(false);

                if (lift.getPosition() <= HEIGHT_TO_ACTIVATE_LIMITER_BAR) limiterBars.setActivated(true);

                break;
        }

        activeHooks.updateAngles(ANGLE_HOOKS_ACTIVE_RETRACTED, ANGLE_HOOKS_ACTIVE_EXTENDED);
        passiveHooks.updateAngles(ANGLE_HOOKS_PASSIVE_RETRACTED, ANGLE_HOOKS_PASSIVE_EXTENDED);
        limiterBars.updateAngles(ANGLE_BARS_RETRACTED, ANGLE_BARS_EXTENDED);

        activeHooks.run();
        passiveHooks.run();
        limiterBars.run();
    }

    void printTelemetry() {
        mTelemetry.addData("CLIMBER", state);
    }
}
