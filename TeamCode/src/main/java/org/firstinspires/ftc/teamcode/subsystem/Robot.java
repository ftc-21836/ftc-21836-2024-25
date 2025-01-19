package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.opmode.MainAuton.divider;
import static org.firstinspires.ftc.teamcode.opmode.MainAuton.mTelemetry;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.vision.pipeline.Sample;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.subsystem.utility.BulkReader;

public final class Robot {

    public final PinpointDrive drivetrain;
    public final Intake intake;
    public final Deposit deposit;
    public final Climber climber;
    public final BulkReader bulkReader;

    private final ElapsedTime loopTimer = new ElapsedTime();

    public Robot(HardwareMap hardwareMap, Pose2d startPose) {
        drivetrain = new PinpointDrive(hardwareMap, startPose);
        bulkReader = new BulkReader(hardwareMap);
        intake = new Intake(hardwareMap);
        deposit = new Deposit(hardwareMap);
        climber = new Climber(hardwareMap, deposit.lift);
    }

    public void run() {
        intake.run(deposit, climber.isActive() || Deposit.level1Ascent);
        deposit.run(intake, climber.isActive());
        climber.run();
    }

    public Sample getSample() {
        return
                intake.hasSample() ?    intake.getSample() :
                deposit.hasSample() ?   deposit.getSample() :
                                        null;
    }

    public void printTelemetry() {
        mTelemetry.addData("LOOP TIME", loopTimer.seconds());
        loopTimer.reset();
        divider();
        drivetrain.printTelemetry();
        divider();
        intake.printTelemetry();
        divider();
        deposit.printTelemetry();
    }
}
