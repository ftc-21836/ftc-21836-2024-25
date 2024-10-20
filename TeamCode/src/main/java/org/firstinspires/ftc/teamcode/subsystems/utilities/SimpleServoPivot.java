package org.firstinspires.ftc.teamcode.subsystems.utilities;


import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.utilities.cachedhardware.CachedSimpleServo;

/**
 * Servo(s) with two set positions <p>
 * Controlled by {@link #toggle} and {@link #setActivated}
 *
 * @author Arshad Anas
 * @since 2023/06/14
 */
public class SimpleServoPivot {

    public static CachedSimpleServo getReversedServo(CachedSimpleServo servo) {
        servo.setInverted(true);
        return servo;
    }

    public static CachedSimpleServo getAxonServo(HardwareMap hardwareMap, String name) {
        return new CachedSimpleServo(hardwareMap, name, 0, 355);
    }

    public static CachedSimpleServo getGoBildaServo(HardwareMap hardwareMap, String name) {
        return new CachedSimpleServo(hardwareMap, name, 0, 280);
    }

    private final CachedSimpleServo[] servos;

    private double ANGLE_INITIAL, ANGLE_ACTIVATED;

    private boolean activated = false;

    public SimpleServoPivot(double ANGLE_INITIAL, double ANGLE_ACTIVATED, CachedSimpleServo... servos) {
        this.servos = servos;
        updateAngles(ANGLE_INITIAL, ANGLE_ACTIVATED);
    }

    public void updateAngles(double ANGLE_INITIAL, double ANGLE_ACTIVATED) {
        this.ANGLE_INITIAL = ANGLE_INITIAL;
        this.ANGLE_ACTIVATED = ANGLE_ACTIVATED;
    }

    /**
     * Toggles the state of the {@link #servos}
     */
    public void toggle() {
        setActivated(!isActivated());
    }

    /**
     * Set state of the {@link #servos}
     *
     * @param activated False for position A, true for position B
     */
    public void setActivated(boolean activated) {
        this.activated = activated;
    }

    /**
     * Get state of the {@link #servos} <p>
     * False if position A (default) <p>
     * True if in position B
     */
    public boolean isActivated() {
        return activated;
    }

    /**
     * Hold {@link #servos} position
     */
    public void run() {
        for (CachedSimpleServo servo : servos) servo.turnToAngle(activated ? ANGLE_ACTIVATED : ANGLE_INITIAL);
    }
}
