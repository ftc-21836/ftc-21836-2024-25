package org.firstinspires.ftc.teamcode.roadrunner;

import static org.firstinspires.ftc.teamcode.control.motion.GoBildaPinpointDriver.EncoderDirection.REVERSED;
import static org.firstinspires.ftc.teamcode.control.motion.GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING;
import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.control.motion.GoBildaPinpointDriver;

@Config
public final class PinpointLocalizer implements Localizer {

    public static double TICKS_PER_MM = 8192 / (PI * 38);

    public static double X_POD_OFFSET = -12000; // y position of the forward (x) encoder (in tick units)
    public static double Y_POD_OFFSET = -3268.070848297075; // x position of the perpendicular (y) encoder (in tick units)

    public static GoBildaPinpointDriver.EncoderDirection X_POD_DIRECTION = REVERSED;
    public static GoBildaPinpointDriver.EncoderDirection Y_POD_DIRECTION = REVERSED;

    private static final DistanceUnit DISTANCE_UNIT = DistanceUnit.INCH;
    private static final AngleUnit ANGLE_UNIT = AngleUnit.RADIANS;

    private final GoBildaPinpointDriver pinpoint;
    private boolean trackHeadingOnly = false;

    public PinpointLocalizer(HardwareMap hardwareMap, Pose2d startPose) {

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");

        pinpoint.setOffsets(X_POD_OFFSET, Y_POD_OFFSET);
        pinpoint.setEncoderResolution(TICKS_PER_MM);
        pinpoint.setEncoderDirections(X_POD_DIRECTION, Y_POD_DIRECTION);
        pinpoint.resetPosAndIMU();

        setPosition(startPose);
    }

    public Twist2dDual<Time> update() {

        if (trackHeadingOnly) pinpoint.update(ONLY_UPDATE_HEADING);
        else pinpoint.update();

        Pose2D position = pinpoint.getPosition();
        Pose2D velocity = pinpoint.getVelocity();

        return new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[] {
                                0,
                                velocity.getX(DISTANCE_UNIT),
                        }),
                        new DualNum<Time>(new double[] {
                                0,
                                velocity.getY(DISTANCE_UNIT),
                        })
                ),
                new DualNum<>(new double[] {
                        0,
                        velocity.getHeading(ANGLE_UNIT)
                })
        );
    }

    public Pose2d getPosition() {

        Pose2D position = pinpoint.getPosition();

        return new Pose2d(
                position.getX(DISTANCE_UNIT),
                position.getY(DISTANCE_UNIT),
                position.getHeading(ANGLE_UNIT)
        );
    }

    private void setPosition(Pose2d pose) {
        pinpoint.setPosition(new Pose2D(
                DISTANCE_UNIT,
                pose.position.x,
                pose.position.y,
                ANGLE_UNIT,
                pose.heading.toDouble()
        ));
    }

    public void trackHeadingOnly(boolean trackHeadingOnly) {
        this.trackHeadingOnly = trackHeadingOnly;
    }
}
