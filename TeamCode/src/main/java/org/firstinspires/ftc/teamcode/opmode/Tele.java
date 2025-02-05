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
import static org.firstinspires.ftc.teamcode.opmode.Auto.basket;
import static org.firstinspires.ftc.teamcode.opmode.Auto.chamberRight;
import static org.firstinspires.ftc.teamcode.opmode.Tele.TeleOpConfig.EDITING_ALLIANCE;
import static org.firstinspires.ftc.teamcode.opmode.Tele.TeleOpConfig.EDITING_FIELD_CENTRIC;
import static org.firstinspires.ftc.teamcode.opmode.Tele.TeleOpConfig.EDITING_SLOW_LOCK;
import static org.firstinspires.ftc.teamcode.opmode.Tele.TeleOpConfig.PRELOAD_SAMPLE;
import static org.firstinspires.ftc.teamcode.opmode.Tele.TeleOpConfig.PRELOAD_SPECIMEN;
import static org.firstinspires.ftc.teamcode.opmode.Auto.isRedAlliance;
import static org.firstinspires.ftc.teamcode.opmode.Auto.mTelemetry;
import static org.firstinspires.ftc.teamcode.opmode.Auto.pose;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.Position.FLOOR;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.Position.HIGH;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.Position.LOW;
import static org.firstinspires.ftc.teamcode.control.vision.pipeline.Sample.BLUE;
import static org.firstinspires.ftc.teamcode.control.vision.pipeline.Sample.NEUTRAL;
import static org.firstinspires.ftc.teamcode.control.vision.pipeline.Sample.RED;
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
import org.firstinspires.ftc.teamcode.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.control.vision.pipeline.Sample;

@TeleOp
public final class Tele extends LinearOpMode {

    enum TeleOpConfig {
        PRELOAD_SAMPLE,
        PRELOAD_SPECIMEN,
        EDITING_ALLIANCE,
        EDITING_SLOW_LOCK,
        EDITING_FIELD_CENTRIC;

        public static final TeleOpConfig[] selections = values();

        public TeleOpConfig plus(int i) {
            int max = selections.length;
            return selections[((ordinal() + i) % max + max) % max];
        }
        public String markIf(TeleOpConfig s) {
            return this == s ? " <" : "";
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime matchTimer = new ElapsedTime();

        double TELE = 120; // seconds
        double CLIMB_TIME = TELE - 10; // 15 seconds for climb
        boolean rumbledClimb = false, rumbledSample = false;

        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Robot robot = new Robot(hardwareMap, pose);
        robot.drivetrain.trackHeadingOnly = true;

        PIDDriver driver = new PIDDriver();

        Deposit.level1Ascent = false;

        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);

        TeleOpConfig selection = PRELOAD_SAMPLE;

        boolean slowModeLocked = false, useFieldCentric = true, doTelemetry = false;

        while (opModeInInit()) {
            gamepadEx1.readButtons();

            if (gamepadEx1.wasJustPressed(DPAD_UP)) {
                do selection = selection.plus(-1);
                while (robot.deposit.hasSample() && (selection == PRELOAD_SAMPLE || selection == PRELOAD_SPECIMEN));
            } else if (gamepadEx1.wasJustPressed(DPAD_DOWN)) {
                do selection = selection.plus(1);
                while (robot.deposit.hasSample() && (selection == PRELOAD_SAMPLE || selection == PRELOAD_SPECIMEN));
            }

            switch (selection) {
                case PRELOAD_SAMPLE:
                    if (gamepadEx1.wasJustPressed(X)) {
                        robot.deposit.transfer(NEUTRAL);
                        selection = selection.plus(2);
                    }
                    break;
                case PRELOAD_SPECIMEN:
                    if (gamepadEx1.wasJustPressed(X)) {
                        while (!robot.deposit.hasSpecimen()) robot.deposit.triggerClaw();
                        robot.deposit.closeClaw();
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

            if (robot.deposit.hasSample()) mTelemetry.addLine("Preloaded a " + (robot.deposit.hasSpecimen() ? "SPECIMEN" : "SAMPLE"));
            else {
                mTelemetry.addLine("Preload sample" + selection.markIf(PRELOAD_SAMPLE));
                mTelemetry.addLine();
                mTelemetry.addLine("Preload specimen" + selection.markIf(PRELOAD_SPECIMEN));
            }
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

        robot.intake.setAlliance(isRedAlliance);

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
                robot.deposit.lift.runManual(gamepadEx1.getLeftY() * (gamepadEx1.isDown(RIGHT_BUMPER) ? 0.3 : 1));
                
                if (gamepadEx1.wasJustPressed(LEFT_STICK_BUTTON))   robot.deposit.lift.reset();

                // SET HEADING:
                robot.drivetrain.setHeadingWithStick(gamepadEx1.getRightX(), gamepadEx1.getRightY());
                robot.drivetrain.run(0, 0, 0, false, true);

                if (gamepadEx1.isDown(X)) doTelemetry = !doTelemetry;

            } else {

                robot.intake.runRoller(triggers);
                robot.intake.extendo.runManual(0);
                robot.deposit.lift.runManual(0);

                if (gamepadEx1.wasJustPressed(X)) driver.reset();

                robot.drivetrain.run(
                        gamepadEx1.getLeftX(),
                        gamepadEx1.getLeftY(),
                        gamepadEx1.isDown(X) && (robot.deposit.basketReady() || robot.intake.hasSample() || robot.deposit.intaking())?
                                driver.driveTo(
                                        new EditablePose(robot.drivetrain.pose),
                                        robot.deposit.intaking() ? chamberRight : basket
                                ).drivePower.heading :
                                gamepadEx1.getRightX(),
                        slowModeLocked || gamepadEx1.isDown(RIGHT_BUMPER) || triggers > 0,
                        useFieldCentric
                );

            }

            if (gamepad1.touchpad_finger_1) {
                robot.intake.extendo.setWithTouchpad(gamepad1.touchpad_finger_1_x);
            }

            if (gamepadEx1.wasJustPressed(Y))                   robot.climber.climb();
            if (gamepadEx1.wasJustPressed(B))                   robot.deposit.triggerClaw();

            if (gamepadEx1.wasJustPressed(DPAD_RIGHT))          robot.intake.transfer(NEUTRAL);
            else if (gamepadEx1.wasJustPressed(DPAD_UP))        robot.deposit.setPosition(HIGH);
            else if (gamepadEx1.wasJustPressed(DPAD_LEFT))      robot.deposit.setPosition(LOW);
            else if (gamepadEx1.wasJustPressed(DPAD_DOWN)) {
                if (robot.climber.isActive()) robot.climber.cancelClimb();
                else robot.deposit.setPosition(FLOOR);
            }

            robot.sweeper.setActivated(gamepadEx1.isDown(A));

            robot.run();

            if (doTelemetry) {
                robot.printTelemetry();
                mTelemetry.update();
            }

            if (!rumbledClimb && matchTimer.seconds() >= CLIMB_TIME) {
                gamepad1.rumble(1, 1, 2000);
                rumbledClimb = true;
            }

            if (!robot.intake.hasSample()) {
                rumbledSample = false;
                if (!robot.deposit.hasSample()) gamepad1.setLedColor(0,0,0, Gamepad.LED_DURATION_CONTINUOUS);
            } else if (!rumbledSample) {

                Sample sample = robot.intake.getSample();
                gamepad1.setLedColor(
                        sample == RED || sample == NEUTRAL ? 1 : 0,
                        sample == NEUTRAL ? 1 : 0,
                        sample == BLUE ? 1 : 0,
                        Gamepad.LED_DURATION_CONTINUOUS
                );

                if (!gamepad1.isRumbling()) gamepad1.rumble(1, 1, 100);
                rumbledSample = true;
            }

        }
    }
}
