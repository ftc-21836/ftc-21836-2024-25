package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.opmode.Auto.divider;
import static org.firstinspires.ftc.teamcode.opmode.Auto.mTelemetry;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.subsystem.utility.BulkReader;
import org.firstinspires.ftc.teamcode.subsystem.utility.SimpleServoPivot;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo;

@Config
public final class Robot {

    public static double
            ANGLE_HOOKS_ACTIVE_RETRACTED = 25,
            ANGLE_HOOKS_ACTIVE_EXTENDED = 180,

            HEIGHT_RUNG_LOW_RAISED = 32,
            HEIGHT_RUNG_LOW_CLIMB_OFFSET = -12;

    public final SimpleServoPivot activeHooks;

    public final PinpointDrive drivetrain;
    public final Intake intake;
    public final Deposit deposit;
    public final BulkReader bulkReader;

    private final ElapsedTime loopTimer = new ElapsedTime();

    public Robot(HardwareMap hardwareMap, Pose2d startPose) {
        drivetrain = new PinpointDrive(hardwareMap, startPose);
        bulkReader = new BulkReader(hardwareMap);
        intake = new Intake(hardwareMap);
        deposit = new Deposit(hardwareMap);

        activeHooks = new SimpleServoPivot(
                ANGLE_HOOKS_ACTIVE_RETRACTED, ANGLE_HOOKS_ACTIVE_EXTENDED,
                CachedSimpleServo.getGBServo(hardwareMap, "right active hook").reversed(),
                CachedSimpleServo.getGBServo(hardwareMap, "left active hook")
        );
    }

    public void run() {
        intake.run(deposit, activeHooks.isActivated() || Deposit.level1Ascent);
        deposit.run(intake, activeHooks.isActivated());

        activeHooks.updateAngles(ANGLE_HOOKS_ACTIVE_RETRACTED, ANGLE_HOOKS_ACTIVE_EXTENDED);
        activeHooks.run();
    }

    public boolean hasSample() {
        return intake.hasSample() || deposit.hasSample();
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
