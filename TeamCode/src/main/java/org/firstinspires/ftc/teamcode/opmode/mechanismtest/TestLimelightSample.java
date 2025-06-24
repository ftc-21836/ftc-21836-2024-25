package org.firstinspires.ftc.teamcode.opmode.mechanismtest;

import static org.firstinspires.ftc.teamcode.opmode.Auto.LL_ANGLE_BUCKET_INCREMENT;
import static org.firstinspires.ftc.teamcode.opmode.Auto.LL_DISTANCE_START_LOWERING;
import static org.firstinspires.ftc.teamcode.opmode.Auto.LL_EXTEND_OFFSET;
import static org.firstinspires.ftc.teamcode.opmode.Auto.LL_SPEED_MAX_EXTENDO;
import static org.firstinspires.ftc.teamcode.opmode.Auto.LL_SWEEP_ANGLE_RANGE;
import static org.firstinspires.ftc.teamcode.opmode.Auto.LL_SWEEP_SPEED;

import static java.lang.Math.hypot;
import static java.lang.Math.min;
import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.motion.EditablePose;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.AutoAlignToSample;
import org.firstinspires.ftc.teamcode.subsystem.LimelightEx;
import org.firstinspires.ftc.teamcode.subsystem.Robot;

@Config
@TeleOp(group = "Single mechanism test")
public class TestLimelightSample extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(hardwareMap, new Pose2d(0, 0, 0));

        Limelight3A limelight3a = hardwareMap.get(Limelight3A.class, "limelight");
        AutoAlignToSample autoAlignToSample = new AutoAlignToSample(new LimelightEx(limelight3a, hardwareMap));
        autoAlignToSample.activateLimelight(AutoAlignToSample.Pipeline.YELLOW_BLUE);
        limelight3a.stop();
        limelight3a.start();

        waitForStart();

        // after init
        Actions.runBlocking(autoAlignToSample.detectTarget(3));

        EditablePose targetOffset = new EditablePose(autoAlignToSample.getTargetedOffset());
        double extendoInches = hypot(targetOffset.x, targetOffset.y) + LL_EXTEND_OFFSET;

        telemetry.addData("dx (in)", targetOffset.x);
        telemetry.addData("dy (in)", targetOffset.y);
        telemetry.addData("d(theta) (deg)", toDegrees(targetOffset.heading));
        telemetry.addData("Extension (in)", extendoInches);
        telemetry.update();

        ElapsedTime timer = new ElapsedTime();

        TurnConstraints a = new TurnConstraints(LL_SWEEP_SPEED, -MecanumDrive.PARAMS.maxAngAccel, MecanumDrive.PARAMS.maxAngAccel);

        robot.intake.extendo.powerCap = LL_SPEED_MAX_EXTENDO;

        Action traj = robot.drivetrain.actionBuilder(new Pose2d(0, 0, 0))
                .waitSeconds(0.2)
                .turn(-targetOffset.heading)
                .stopAndAdd(() -> {
                    robot.intake.extendo.setTarget(extendoInches);
                    robot.intake.setAngle(0.01);
                    robot.intake.setRoller(1);
                    timer.reset();
                })
                .stopAndAdd(new SequentialAction(
                        t -> !(robot.intake.extendo.getPosition() >= extendoInches - LL_DISTANCE_START_LOWERING || timer.seconds() >= 1),
                        new InstantAction(timer::reset),
                        t -> {
                            double s = timer.seconds();
                            robot.intake.setAngle(min(s * LL_ANGLE_BUCKET_INCREMENT, 1));
                            return s * LL_ANGLE_BUCKET_INCREMENT < 1;
                        }
                ))
                .turn(toRadians(LL_SWEEP_ANGLE_RANGE), a)
                .turn(-2 * toRadians(LL_SWEEP_ANGLE_RANGE), a)
                .turn(toRadians(LL_SWEEP_ANGLE_RANGE), a)

                .stopAndAdd(t -> !robot.hasSample())
                .stopAndAdd(() -> robot.intake.setRollerAndAngle(0))
                .lineToX(-5)
                .build();

        Actions.runBlocking(new ParallelAction(
                telemetryPacket -> {
                    robot.bulkReader.bulkRead();
                    return opModeIsActive();
                },
                traj,
                telemetryPacket -> {
                    robot.run();
                    return opModeIsActive();
                }
        ));



    }
}
