package org.firstinspires.ftc.teamcode.roadrunner;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;
import static org.firstinspires.ftc.teamcode.control.motion.GoBildaPinpointDriver.EncoderDirection.FORWARD;
import static org.firstinspires.ftc.teamcode.control.motion.GoBildaPinpointDriver.EncoderDirection.REVERSED;
import static org.firstinspires.ftc.teamcode.control.motion.GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING;
import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.control.motion.GoBildaPinpointDriver;

@Config
public final class PinpointLocalizer implements Localizer {

    private static final double INCH_PER_MM = 1 / 25.4;

    public static double TICKS_PER_MM = 8192 / (PI * 38);

    public static double X_POD_OFFSET = -12000; // y position of the forward (x) encoder (in tick units)
    public static double Y_POD_OFFSET = -3268.070848297075; // x position of the perpendicular (y) encoder (in tick units)

    public static GoBildaPinpointDriver.EncoderDirection
            X_POD_DIRECTION = REVERSED,
            Y_POD_DIRECTION = FORWARD;

    private final GoBildaPinpointDriver pinpoint;
    private boolean trackHeadingOnly = false;

    public PinpointLocalizer(HardwareMap hardwareMap) {

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");

        pinpoint.setOffsets(X_POD_OFFSET, Y_POD_OFFSET);
        pinpoint.setEncoderResolution(TICKS_PER_MM);
        pinpoint.setEncoderDirections(X_POD_DIRECTION, Y_POD_DIRECTION);

        reset();

        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    public void reset() {
        pinpoint.resetPosAndIMU();
    }

    public Twist2dDual<Time> update() {

        if (trackHeadingOnly) pinpoint.update(ONLY_UPDATE_HEADING);
        else pinpoint.update();

        return new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<>(new double[] {0, 0,}),
                        new DualNum<>(new double[] {0, 0,})
                ),
                new DualNum<>(new double[] {0, 0})
        );
    }

    public Pose2d getPosition() {
        return new Pose2d(
                pinpoint.getPosX() * INCH_PER_MM,
                pinpoint.getPosY() * INCH_PER_MM,
                pinpoint.getHeading()
        );
    }

    public PoseVelocity2d getVelocity() {
        return new PoseVelocity2d(
                new Vector2d(
                        pinpoint.getVelX() * INCH_PER_MM,
                        pinpoint.getVelY() * INCH_PER_MM
                ),
                pinpoint.getHeading()
        );
    }

    public void setPosition(Pose2d pose) {
        pinpoint.setPosition(new Pose2D(
                INCH,
                pose.position.x,
                pose.position.y,
                RADIANS,
                pose.heading.toDouble()
        ));
    }

    public void trackHeadingOnly(boolean trackHeadingOnly) {
        this.trackHeadingOnly = trackHeadingOnly;
    }

    public int rawEncoderX() {
        return pinpoint.getEncoderX();
    }
    public int rawEncoderY() {
        return pinpoint.getEncoderY();
    }
    public void setHeading(double radians) {
        pinpoint.setHeading(radians);
    }
}
