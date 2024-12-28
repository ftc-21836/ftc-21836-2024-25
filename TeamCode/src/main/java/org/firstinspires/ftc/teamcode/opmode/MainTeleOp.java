package org.firstinspires.ftc.teamcode.opmode;

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
import static org.firstinspires.ftc.teamcode.opmode.MainTeleOp.TeleOpConfig.EDITING_ALLIANCE;
import static org.firstinspires.ftc.teamcode.opmode.MainTeleOp.TeleOpConfig.EDITING_FIELD_CENTRIC;
import static org.firstinspires.ftc.teamcode.opmode.MainTeleOp.TeleOpConfig.EDITING_SLOW_LOCK;
import static org.firstinspires.ftc.teamcode.opmode.MainTeleOp.TeleOpConfig.PRELOAD_SAMPLE;
import static org.firstinspires.ftc.teamcode.opmode.MainTeleOp.TeleOpConfig.PRELOAD_SPECIMEN;
import static org.firstinspires.ftc.teamcode.opmode.OpModeVars.divider;
import static org.firstinspires.ftc.teamcode.opmode.OpModeVars.isRedAlliance;
import static org.firstinspires.ftc.teamcode.opmode.OpModeVars.loopMod;
import static org.firstinspires.ftc.teamcode.opmode.OpModeVars.mTelemetry;
import static org.firstinspires.ftc.teamcode.opmode.OpModeVars.pose;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.Position.FLOOR;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.Position.HIGH;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.Position.LOW;
import static org.firstinspires.ftc.teamcode.subsystem.Sample.BLUE;
import static org.firstinspires.ftc.teamcode.subsystem.Sample.NEUTRAL;
import static org.firstinspires.ftc.teamcode.subsystem.Sample.RED;
import static java.lang.Math.toDegrees;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Sample;

@TeleOp
public final class MainTeleOp extends LinearOpMode {

    enum TeleOpConfig {
        EDITING_ALLIANCE,
        PRELOAD_SAMPLE,
        PRELOAD_SPECIMEN,
        EDITING_SLOW_LOCK,
        EDITING_FIELD_CENTRIC;

        public static final TeleOpConfig[] selections = values();

        public TeleOpConfig plus(int i) {
            return selections[(int) loopMod(ordinal() + i, selections.length)];
        }
        public String markIf(TeleOpConfig s) {
            return this == s ? " <" : "";
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime loopTimer = new ElapsedTime(), matchTimer = new ElapsedTime();

        double TELE = 120; // seconds
        double CLIMB_TIME = TELE - 15; // 15 seconds for climb
        boolean rumbledClimb = false, rumbledSample = false;

        Gamepad.RumbleEffect sampleRumble = new Gamepad.RumbleEffect.Builder()
                .addStep(1, 1, 100)
                .addStep(0, 0, 100)
                .addStep(1, 1, 100)
                .build();

        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Robot robot = new Robot(hardwareMap, pose);
        robot.drivetrain.localizer.trackHeadingOnly(true);
        robot.drivetrain.localizer.setHeading(pose.heading.toDouble());

        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);

        TeleOpConfig selection = EDITING_ALLIANCE;

        boolean slowModeLocked = false, useFieldCentric = true;

        while (opModeInInit()) {
            gamepadEx1.readButtons();

            if (gamepadEx1.wasJustPressed(DPAD_UP))   selection = selection.plus(-1);
            if (gamepadEx1.wasJustPressed(DPAD_DOWN)) selection = selection.plus(1);

            if (gamepadEx1.wasJustPressed(X)) switch (selection) {
                case EDITING_ALLIANCE:
                    isRedAlliance = !isRedAlliance;
                    break;
                case PRELOAD_SAMPLE:
                    robot.deposit.transfer(NEUTRAL);
                    break;
                case PRELOAD_SPECIMEN:
                    robot.deposit.preloadSpecimen();
                    break;
                case EDITING_SLOW_LOCK:
                    slowModeLocked = !slowModeLocked;
                    break;
                case EDITING_FIELD_CENTRIC:
                    useFieldCentric = !useFieldCentric;
                    break;
            }

            robot.drivetrain.setHeadingFromStick(gamepadEx1.getRightX(), gamepadEx1.getRightY());

            mTelemetry.addLine((isRedAlliance ? "RED" : "BLUE") + " alliance" + selection.markIf(EDITING_ALLIANCE));
            mTelemetry.addLine();
            mTelemetry.addLine("Preload sample" + selection.markIf(PRELOAD_SAMPLE));
            mTelemetry.addLine();
            mTelemetry.addLine("Preload specimen" + selection.markIf(PRELOAD_SPECIMEN));
            mTelemetry.addLine();
            mTelemetry.addData("Slow mode", (slowModeLocked ? "LOCKED" : "unlocked") + selection.markIf(EDITING_SLOW_LOCK));
            mTelemetry.addLine();
            mTelemetry.addLine((useFieldCentric ? "Field centric" : "ROBOT CENTRIC") + " driving" + selection.markIf(EDITING_FIELD_CENTRIC));
            mTelemetry.addLine();
            mTelemetry.addData("Start heading (deg)", toDegrees(robot.drivetrain.localizer.getPosition().heading.toDouble()));

            mTelemetry.update();
        }

        robot.intake.setAlliance(isRedAlliance);
        robot.deposit.setAlliance(isRedAlliance);

        matchTimer.reset();

// ----------------------------------------------------------------------------------------------------------------------------------------------------------------

        // Control loop:
        while (opModeIsActive()) {
            // Read sensors + gamepads:
            robot.readSensors();
            gamepadEx1.readButtons();

            double rightX = gamepadEx1.getRightX();
            double leftX = gamepadEx1.getLeftX();
            double leftY = gamepadEx1.getLeftY();

            double triggers = gamepad1.right_trigger - gamepad1.left_trigger;

            if (gamepadEx1.isDown(LEFT_BUMPER)) {

                robot.intake.extendo.runManual(triggers);
                robot.deposit.lift.runManual(leftY);
                robot.intake.runRoller(0);
                
                if (gamepadEx1.wasJustPressed(LEFT_STICK_BUTTON))   robot.deposit.lift.reset();

                // SET HEADING:
                robot.drivetrain.setHeadingFromStick(rightX, gamepadEx1.getRightY());
                robot.drivetrain.run(0, 0, 0, false, true);

            } else {

                if (gamepad1.touchpad_finger_1) {
                    robot.intake.extendo.setWithTouchpad(gamepad1.touchpad_finger_1_x);
                }
                robot.intake.extendo.runManual(0);
                robot.deposit.lift.runManual(0);
                robot.intake.runRoller(triggers);

                if (gamepadEx1.wasJustPressed(X))                   robot.intake.toggle();
                if (gamepadEx1.wasJustPressed(Y))                   robot.climber.climb();

                if (!robot.climber.isActive()) {

                    if (gamepadEx1.wasJustPressed(DPAD_UP))         robot.deposit.setPosition(HIGH);
                    else if (gamepadEx1.wasJustPressed(DPAD_LEFT))  robot.deposit.setPosition(LOW);
                    else if (gamepadEx1.wasJustPressed(DPAD_DOWN))  robot.deposit.setPosition(FLOOR);
                    else if (gamepadEx1.wasJustPressed(DPAD_RIGHT)) robot.intake.transfer(robot.deposit, NEUTRAL);

                    if (gamepadEx1.wasJustPressed(B))               robot.deposit.triggerClaw();

                } else if (gamepadEx1.wasJustPressed(DPAD_DOWN))    robot.climber.cancelClimb();

                robot.drivetrain.run(
                        leftX,
                        leftY,
                        rightX,
                        slowModeLocked || robot.requestingSlowMode() || gamepadEx1.isDown(RIGHT_BUMPER) || triggers > 0,
                        useFieldCentric
                );

            }

            robot.run();

            mTelemetry.addData("LOOP TIME", loopTimer.seconds());
            loopTimer.reset();
            divider();
            robot.printTelemetry();
            mTelemetry.update();

            if (!rumbledClimb && matchTimer.seconds() >= CLIMB_TIME) {
                gamepad1.rumble(1, 1, 1500);
                rumbledClimb = true;
            }

            if (!robot.intake.hasSample()) rumbledSample = false;
            else if (!gamepad1.isRumbling() && !rumbledSample) {
                gamepad1.runRumbleEffect(sampleRumble);
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
