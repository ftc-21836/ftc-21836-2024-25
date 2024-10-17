package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.mTelemetry;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.utilities.BulkReader;

@Config
public final class Robot {

    public static double
            maxVoltage = 13,
            ANGLE_DRONE_LOADED = 50,
            ANGLE_DRONE_LAUNCHED = 0,
            ANGLE_SPIKE_LOCKED = 90,
            ANGLE_SPIKE_RELEASED = 0;

    public static boolean
            depositHasSample = false,
            depositIsActive = false;

    public final MecanumDrive drivetrain;
    public final Intake intake;
//    public final Deposit deposit;
//    public final SimpleServoPivot drone, spike;

    private final BulkReader bulkReader;

    public Robot(HardwareMap hardwareMap) {
        bulkReader = new BulkReader(hardwareMap);

        drivetrain = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        intake = new Intake(hardwareMap);
//        deposit = new Deposit(hardwareMap);
//        drone = new SimpleServoPivot(
//                ANGLE_DRONE_LOADED,
//                ANGLE_DRONE_LAUNCHED,
//                getGoBildaServo(hardwareMap, "drone")
//        );
//        spike = new SimpleServoPivot(
//                ANGLE_SPIKE_RELEASED,
//                ANGLE_SPIKE_LOCKED,
//                getGoBildaServo(hardwareMap, "floor pixel")
//        );
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
//        deposit.lift.readSensors();
    }

    public void run() {
//        drone.updateAngles(ANGLE_DRONE_LOADED, ANGLE_DRONE_LAUNCHED);
//        spike.updateAngles(ANGLE_SPIKE_RELEASED, ANGLE_SPIKE_LOCKED);

//        if (intake.awaitingTransfer()) {
//            deposit.paintbrush.lockPixels(intake.colors);
//        }

        intake.run(depositHasSample, depositIsActive);
//        deposit.run(intake.clearOfDeposit());
//
//
//        drone.run();
//        spike.run();
    }

    public void printTelemetry() {
        drivetrain.printNumericalTelemetry();
        mTelemetry.addLine();
//        deposit.paintbrush.printTelemetry();
        mTelemetry.addLine();
//        deposit.lift.printTelemetry();
        mTelemetry.addLine();
        intake.printTelemetry();
        mTelemetry.addLine();
        mTelemetry.addLine();
        mTelemetry.addLine();
//        drivetrain.printNumericalTelemetry();
        mTelemetry.addLine();
//        deposit.lift.printNumericalTelemetry();
        mTelemetry.addLine();
        intake.printNumericalTelemetry();
    }
}
