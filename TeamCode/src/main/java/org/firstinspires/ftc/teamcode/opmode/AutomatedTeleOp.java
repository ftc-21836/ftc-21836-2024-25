package org.firstinspires.ftc.teamcode.opmode;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_STICK_BUTTON;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;
import static org.firstinspires.ftc.teamcode.control.vision.pipeline.Sample.BLUE;
import static org.firstinspires.ftc.teamcode.control.vision.pipeline.Sample.NEUTRAL;
import static org.firstinspires.ftc.teamcode.control.vision.pipeline.Sample.RED;
import static org.firstinspires.ftc.teamcode.opmode.MainAuton.basket;
import static org.firstinspires.ftc.teamcode.opmode.MainAuton.chamber0;
import static org.firstinspires.ftc.teamcode.opmode.MainAuton.intakingSpec;
import static org.firstinspires.ftc.teamcode.opmode.MainAuton.isRedAlliance;
import static org.firstinspires.ftc.teamcode.opmode.MainAuton.mTelemetry;
import static org.firstinspires.ftc.teamcode.opmode.MainAuton.pose;
import static org.firstinspires.ftc.teamcode.opmode.MainTeleOp.TeleOpConfig.EDITING_ALLIANCE;
import static org.firstinspires.ftc.teamcode.opmode.MainTeleOp.TeleOpConfig.EDITING_FIELD_CENTRIC;
import static org.firstinspires.ftc.teamcode.opmode.MainTeleOp.TeleOpConfig.EDITING_SLOW_LOCK;
import static org.firstinspires.ftc.teamcode.opmode.MainTeleOp.TeleOpConfig.PRELOAD_SAMPLE;
import static org.firstinspires.ftc.teamcode.opmode.MainTeleOp.TeleOpConfig.PRELOAD_SPECIMEN;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.Position.FLOOR;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.Position.HIGH;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.Position.LOW;
import static java.lang.Math.toDegrees;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.motion.EditablePose;
import org.firstinspires.ftc.teamcode.control.motion.PIDDriver;
import org.firstinspires.ftc.teamcode.control.vision.pipeline.Sample;
import org.firstinspires.ftc.teamcode.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.subsystem.Robot;

@TeleOp
public final class AutomatedTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime matchTimer = new ElapsedTime();

        double TELE = 120; // seconds
        double CLIMB_TIME = TELE - 10; // 15 seconds for climb
        boolean rumbledClimb = false, rumbledSample = false;

        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Robot robot = new Robot(hardwareMap, pose);

        Deposit.level1Ascent = false;

        PIDDriver driver = new PIDDriver();
        EditablePose ignoreX = new EditablePose(0, 1, 1);

        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);

        MainTeleOp.TeleOpConfig selection = PRELOAD_SAMPLE;

        boolean slowModeLocked = false, useFieldCentric = true;

        boolean preloaded = false;

        while (opModeInInit()) {
            gamepadEx1.readButtons();

            if (gamepadEx1.wasJustPressed(DPAD_UP)) {
                selection = selection.plus(preloaded && selection == EDITING_ALLIANCE ? -3 : -1);
            } else if (gamepadEx1.wasJustPressed(DPAD_DOWN)) {
                selection = selection.plus(preloaded && selection == EDITING_FIELD_CENTRIC ? 3 : 1);
            }

            switch (selection) {
                case PRELOAD_SAMPLE:
                    if (gamepadEx1.wasJustPressed(X)) {
                        robot.deposit.transfer(NEUTRAL);
                        preloaded = true;
                        selection = selection.plus(2);
                    }
                    break;
                case PRELOAD_SPECIMEN:
                    if (gamepadEx1.wasJustPressed(X)) {
                        robot.deposit.preloadSpecimen();
                        preloaded = true;
                        selection = selection.plus(1);
                    }
                    break;
                case EDITING_ALLIANCE:
                    if (gamepadEx1.wasJustPressed(X)) isRedAlliance = !isRedAlliance;
                    break;
                case EDITING_SLOW_LOCK:
                    if (gamepadEx1.wasJustPressed(X)) slowModeLocked = !slowModeLocked;
                    break;
                case EDITING_FIELD_CENTRIC:
                    if (gamepadEx1.wasJustPressed(X)) useFieldCentric = !useFieldCentric;
                    break;
            }

            gamepad1.setLedColor(
                    isRedAlliance ? 1 : 0,
                    0,
                    !isRedAlliance ? 1 : 0,
                    Gamepad.LED_DURATION_CONTINUOUS
            );

            robot.drivetrain.setHeadingWithStick(gamepadEx1.getRightX(), gamepadEx1.getRightY());
            robot.drivetrain.updatePoseEstimate();

            mTelemetry.addLine("Preload sample" + selection.markIf(PRELOAD_SAMPLE));
            mTelemetry.addLine();
            mTelemetry.addLine("Preload specimen" + selection.markIf(PRELOAD_SPECIMEN));
            mTelemetry.addLine();
            mTelemetry.addLine((isRedAlliance ? "RED" : "BLUE") + " alliance" + selection.markIf(EDITING_ALLIANCE));
            mTelemetry.addLine();
            mTelemetry.addData("Slow mode", (slowModeLocked ? "LOCKED" : "unlocked") + selection.markIf(EDITING_SLOW_LOCK));
            mTelemetry.addLine();
            mTelemetry.addLine((useFieldCentric ? "Field centric" : "ROBOT CENTRIC") + " driving" + selection.markIf(EDITING_FIELD_CENTRIC));
            mTelemetry.addLine();
            mTelemetry.addData("Heading (deg, set with right stick)", toDegrees(robot.drivetrain.pose.heading.toDouble()));

            mTelemetry.update();
        }

// ----------------------------------------------------------------------------------------------------------------------------------------------------------------

        robot.drivetrain.pinpoint.setPositionRR(pose);
        robot.intake.setAlliance(isRedAlliance);
        robot.deposit.setAlliance(isRedAlliance);

        matchTimer.reset();

        // Control loop:
        while (opModeIsActive()) {
            // Read sensors + gamepads:
            robot.bulkReader.bulkRead();
            robot.drivetrain.updatePoseEstimate();
            gamepadEx1.readButtons();

            double triggers = gamepad1.right_trigger - gamepad1.left_trigger;

            if (gamepadEx1.isDown(LEFT_BUMPER)) {

                robot.intake.runRoller(0);
                robot.intake.extendo.runManual(triggers);
                robot.deposit.lift.runManual(gamepadEx1.getLeftY());
                
                if (gamepadEx1.wasJustPressed(LEFT_STICK_BUTTON))   robot.deposit.lift.reset();

                // SET HEADING:
                robot.drivetrain.setHeadingWithStick(gamepadEx1.getRightX(), gamepadEx1.getRightY());
                robot.drivetrain.run(0, 0, 0, false, true);

            } else {

                robot.intake.runRoller(triggers);
                robot.intake.extendo.runManual(0);
                robot.deposit.lift.runManual(0);

                if (gamepadEx1.isDown(X)) {

                    boolean hasSpecimen = robot.deposit.specimenIntaked();

                    EditablePose output = driver.driveTo(
                            new EditablePose(robot.drivetrain.pose),
                            robot.getSample() == null ? intakingSpec : hasSpecimen ? chamber0 : basket
                    ).drivePower;

                    robot.drivetrain.run(
                            hasSpecimen ? gamepadEx1.getLeftX() : output.x,
                            output.y,
                            output.heading,
                            false,
                            true
                    );

                } else robot.drivetrain.run(
                        gamepadEx1.getLeftX(),
                        gamepadEx1.getLeftY(),
                        gamepadEx1.getRightX(),
                        slowModeLocked || gamepadEx1.isDown(RIGHT_BUMPER) || triggers > 0,
                        useFieldCentric
                );

            }

            if (gamepad1.touchpad_finger_1) {
                robot.intake.extendo.setWithTouchpad(gamepad1.touchpad_finger_1_x);
            }

            if (gamepadEx1.wasJustPressed(Y))                   robot.climber.climb();

            if (!robot.climber.isActive()) {

                if (gamepadEx1.wasJustPressed(DPAD_UP))         robot.deposit.setPosition(HIGH);
                else if (gamepadEx1.wasJustPressed(DPAD_LEFT))  robot.deposit.setPosition(LOW);
                else if (gamepadEx1.wasJustPressed(DPAD_DOWN))  robot.deposit.setPosition(FLOOR);
                else if (gamepadEx1.wasJustPressed(DPAD_RIGHT)) robot.intake.transfer(robot.deposit, NEUTRAL);

                if (gamepadEx1.wasJustPressed(B))               robot.deposit.triggerClaw();

            } else if (gamepadEx1.wasJustPressed(DPAD_DOWN))    robot.climber.cancelClimb();

            robot.run();

            if (gamepadEx1.isDown(A)) {
                robot.printTelemetry();
                mTelemetry.update();
            }

            if (!rumbledClimb && matchTimer.seconds() >= CLIMB_TIME) {
                gamepad1.rumble(1, 1, 2000);
                rumbledClimb = true;
            }

            if (!robot.intake.hasSample()) rumbledSample = false;
            else if (!gamepad1.isRumbling() && !rumbledSample) {
                gamepad1.rumble(1, 1, 100);
                rumbledSample = true;
            }

            Sample sample = robot.getSample();
            gamepad1.setLedColor(
                    sample == RED || sample == NEUTRAL ? 1 : 0,
                    sample == NEUTRAL ? 1 : 0,
                    sample == BLUE ? 1 : 0,
                    Gamepad.LED_DURATION_CONTINUOUS
            );

        }
    }
}
