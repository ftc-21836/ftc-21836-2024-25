package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.utilities.BulkReader;

@Config
public final class Robot {

    public final MecanumDrive drivetrain;
//    public final Intake intake;
//    public final Deposit deposit;
//    public final Climber climber;

    private final BulkReader bulkReader;

    public Robot(HardwareMap hardwareMap, Pose2d startPose) {

        drivetrain = new MecanumDrive(hardwareMap, startPose);
        bulkReader = new BulkReader(hardwareMap);
//        intake = new Intake(hardwareMap);
//        deposit = new Deposit(hardwareMap);
//        climber = new Climber(hardwareMap, deposit.lift);
    }

    public void preload(boolean backdropSide) {
    }

    public void initRun() {
    }

    public void endgame() {
    }

    public void readSensors() {
        bulkReader.bulkRead();
//        drivetrain.updatePoseEstimate();
    }

    public void run() {

//        if (intake.awaitingTransfer()) deposit.transfer(intake.transfer());
//
//        intake.run(deposit.hasSample(), deposit.isActive());
//        deposit.run(intake.clearOfDeposit(), climber.isActive());
//        climber.run();
    }

    public boolean requestingSlowMode() {
        return false;
//                deposit.movingToScore() && intake.clearOfDeposit(); // deposit intends to move and intake is not blocking it
    }

    public void printTelemetry() {
        drivetrain.printTelemetry();
//        divider();
//        intake.printTelemetry();
//        divider();
//        deposit.printTelemetry();
//        divider();
//        climber.printTelemetry();
    }
}
