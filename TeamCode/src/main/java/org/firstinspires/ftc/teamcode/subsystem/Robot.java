package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.opmode.Auto.divider;
import static org.firstinspires.ftc.teamcode.opmode.Auto.mTelemetry;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.vision.pipeline.Sample;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.subsystem.utility.BulkReader;
import org.firstinspires.ftc.teamcode.subsystem.utility.SimpleServoPivot;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo;

@Config
public final class Robot {

    public static double
            ANGLE_SWEEPER_STANDBY = 0,
            ANGLE_SWEEPER_SWEPT = 90;

    public final PinpointDrive drivetrain;
    public final Intake intake;
    public final Deposit deposit;
    public final Climber climber;
    public final SimpleServoPivot sweeper;
    public final BulkReader bulkReader;

    private final ElapsedTime loopTimer = new ElapsedTime();

    public Robot(HardwareMap hardwareMap, Pose2d startPose) {
        drivetrain = new PinpointDrive(hardwareMap, startPose);
        bulkReader = new BulkReader(hardwareMap);
        intake = new Intake(hardwareMap);
        deposit = new Deposit(hardwareMap);
        climber = new Climber(hardwareMap, deposit.lift);
        sweeper = new SimpleServoPivot(
                ANGLE_SWEEPER_STANDBY, ANGLE_SWEEPER_SWEPT,
                CachedSimpleServo.getGBServo(hardwareMap, "sweeper")
        );
    }

    public void run() {
        intake.run(deposit, climber.isActive() || Deposit.level1Ascent);
        deposit.run(intake, climber.isActive());
        climber.run();

        sweeper.updateAngles(ANGLE_SWEEPER_STANDBY, ANGLE_SWEEPER_SWEPT);
        sweeper.run();
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
