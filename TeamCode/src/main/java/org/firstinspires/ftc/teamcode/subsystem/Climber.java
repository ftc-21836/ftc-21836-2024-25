package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo.getGBServo;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystem.utility.SimpleServoPivot;

@Config
public final class Climber {

    public static double
            ANGLE_HOOKS_ACTIVE_RETRACTED = 0,
            ANGLE_HOOKS_ACTIVE_EXTENDED = 105.4,

            HEIGHT_RUNG_LOW_RAISED = 32,
            HEIGHT_RUNG_LOW_CLIMB_OFFSET = -12;

    private final SimpleServoPivot activeHooks;

    public boolean isActive() {
        return activeHooks.isActivated();
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
    }

    public void cancelClimb() {
        activeHooks.setActivated(false);
        lift.setTarget(0);
    }

    public void climb() {

        activeHooks.setActivated(true);

        lift.setTarget(
                lift.getTarget() == HEIGHT_RUNG_LOW_RAISED ?
                        HEIGHT_RUNG_LOW_RAISED + HEIGHT_RUNG_LOW_CLIMB_OFFSET :
                        HEIGHT_RUNG_LOW_RAISED
        );

    }

    void run() {
        activeHooks.updateAngles(ANGLE_HOOKS_ACTIVE_RETRACTED, ANGLE_HOOKS_ACTIVE_EXTENDED);
        activeHooks.run();
    }

}
