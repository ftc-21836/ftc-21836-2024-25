package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.mTelemetry;
import static org.firstinspires.ftc.teamcode.subsystems.Intake.Sample.NONE;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.utilities.BulkReader;

@Config
public final class Robot {

    public static double maxVoltage = 13;

    public final MecanumDrive drivetrain;
    public final Intake intake;
    public final Deposit deposit;

    private final BulkReader bulkReader;

    public Robot(HardwareMap hardwareMap) {
        bulkReader = new BulkReader(hardwareMap);

        drivetrain = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        intake = new Intake(hardwareMap);
        deposit = new Deposit(hardwareMap);
    }

    public void preload(boolean backdropSide) {
    }

    public void initRun() {
    }

    public void endgame() {
    }

    public void readSensors() {
        bulkReader.bulkRead();
        drivetrain.updatePoseEstimate();
        deposit.lift.readSensors();
    }

    public void run() {

        if (intake.awaitingTransfer()) deposit.transfer(intake.sample);

        intake.run(deposit.sample != NONE, deposit.isActive());
        deposit.run(intake.clearOfDeposit());
    }

    public void printTelemetry() {
        drivetrain.printNumericalTelemetry();
        mTelemetry.addLine();
        deposit.printTelemetry();
        mTelemetry.addLine();
        mTelemetry.addLine();
        intake.printTelemetry();
        mTelemetry.addLine();
        mTelemetry.addLine();
        mTelemetry.addLine();
//        drivetrain.printNumericalTelemetry();
        mTelemetry.addLine();
        deposit.lift.printNumericalTelemetry();
        mTelemetry.addLine();
        intake.printNumericalTelemetry();
    }
}
