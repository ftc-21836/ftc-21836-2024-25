package org.firstinspires.ftc.teamcode.opmode.mechanismtest;

import static org.firstinspires.ftc.teamcode.opmode.Auto.OFFSET_CV_EXTEND;
import static org.firstinspires.ftc.teamcode.subsystem.AutoAlignToSample.Pipeline.YELLOW;

import static java.lang.Math.hypot;
import static java.lang.Math.toDegrees;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.motion.EditablePose;
import org.firstinspires.ftc.teamcode.opmode.Tele;
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
        autoAlignToSample.activateLimelight(YELLOW);
        limelight3a.stop();
        limelight3a.start();

        waitForStart();

        // after init
        Actions.runBlocking(autoAlignToSample.detectTarget(3));

        EditablePose targetOffset = new EditablePose(autoAlignToSample.getTargetedOffset());
        double extendoInches = hypot(targetOffset.x, targetOffset.y) + OFFSET_CV_EXTEND;

        telemetry.addData("dx (in)", targetOffset.x);
        telemetry.addData("dy (in)", targetOffset.y);
        telemetry.addData("d(theta) (deg)", toDegrees(targetOffset.heading));
        telemetry.addData("Extension (in)", extendoInches);
        telemetry.update();

        ElapsedTime timer = new ElapsedTime();

        Action traj = robot.drivetrain.actionBuilder(robot.drivetrain.pose)
                .waitSeconds(0.2)
                .turn(-targetOffset.heading)
                .afterTime(0, new SequentialAction(
                                new InstantAction(() -> {
                                    robot.intake.extendo.setTarget(extendoInches);
                                    timer.reset();
                                }),
                                t -> !(robot.intake.extendo.getPosition() >= extendoInches - 5 || timer.seconds() >= 1),
                                new InstantAction(() -> robot.intake.setRollerAndAngle(1))
                        )
                )
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
