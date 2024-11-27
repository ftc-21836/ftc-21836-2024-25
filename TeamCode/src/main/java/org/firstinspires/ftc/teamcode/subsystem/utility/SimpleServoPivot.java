package org.firstinspires.ftc.teamcode.subsystem.utility;


import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo;

/**
 * Servo(s) with two set angles <p>
 * Controlled by {@link #toggle} and {@link #setActivated}
 *
 * @author Arshad Anas
 * @since 2023/06/14
 */
public class SimpleServoPivot {

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
     * Toggle the state of the {@link #servos} <p>
     * If at {@link #ANGLE_ACTIVATED}, move to {@link #ANGLE_INITIAL}, or vice versa
     */
    public void toggle() {
        setActivated(!isActivated());
    }

    /**
     * Set state of the {@link #servos}
     *
     * @param activated Set angle to {@link #ANGLE_ACTIVATED}
     */
    public void setActivated(boolean activated) {
        this.activated = activated;
    }

    /**
     * Get state of the {@link #servos} <p>
     * False if at {@link #ANGLE_INITIAL} (default) <p>
     * True if at {@link #ANGLE_ACTIVATED}
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
