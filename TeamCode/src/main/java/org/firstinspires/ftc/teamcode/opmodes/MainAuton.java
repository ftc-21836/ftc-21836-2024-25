package org.firstinspires.ftc.teamcode.opmodes;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.PARTNER_WAIT;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.cycle;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.isBackdropSide;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.isRed;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.parking;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.AutonConfig.EDITING_ALLIANCE;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.AutonConfig.EDITING_CYCLE;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.AutonConfig.EDITING_PARK;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.AutonConfig.EDITING_SIDE;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.AutonConfig.EDITING_WAIT;
import static java.lang.Math.PI;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.control.motion.EditablePose;
import org.firstinspires.ftc.teamcode.control.vision.detectors.TeamPropDetector;
import org.firstinspires.ftc.teamcode.control.vision.pipelines.PropDetectPipeline;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous(preselectTeleOp = "MainTeleOp")
public final class MainAuton extends LinearOpMode {

    public static final double
            REVERSE = PI,
            LEFT = REVERSE,
            FORWARD = 1.5707963267948966,
            RIGHT = 0,
            BACKWARD = -1.5707963267948966;

    // Declare objects:
    public static GamepadEx gamepadEx1, gamepadEx2;
    public static MultipleTelemetry mTelemetry;
    static Robot robot;
    static Pose2d autonEndPose = null;

    public static boolean keyPressed(int gamepad, GamepadKeys.Button button) {
        return (gamepad == 2 ? gamepadEx2 : gamepadEx1).wasJustPressed(button);
    }

    enum AutonConfig {
        EDITING_ALLIANCE,
        EDITING_SIDE,
        EDITING_PARK,
        EDITING_CYCLE,
        EDITING_WAIT;

        public static final AutonConfig[] selections = values();

        public AutonConfig plus(int i) {
            return selections[loopMod(ordinal() + i, selections.length)];
        }
        public String markIf(AutonConfig s) {
            return this == s ? " <" : "";
        }

        public void printTelemetry() {
            mTelemetry.addLine((isRed ? "RED " : "BLUE ") + markIf(EDITING_ALLIANCE));
            mTelemetry.addLine();
            mTelemetry.addLine((isBackdropSide ? "BACKDROP " : "AUDIENCE ") + "side" + markIf(EDITING_SIDE));
            mTelemetry.addLine();
            mTelemetry.addLine(parking.name() + " PARKING" + markIf(EDITING_PARK));
            mTelemetry.addLine();
            mTelemetry.addLine("WILL " + (cycle ? "CYCLE" : "NOT CYCLE") + markIf(EDITING_CYCLE));
            mTelemetry.addLine();
            mTelemetry.addLine("Pause after spike: " + PARTNER_WAIT + markIf(EDITING_WAIT));
            mTelemetry.addLine();
        }
    }

    public static int loopMod(int a, int b) {
        return (int) loopMod(a,(double) b);
    }

    public static double loopMod(double a, double b) {
        return (a % b + b) % b;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize multiple telemetry outputs:
        mTelemetry = new MultipleTelemetry(telemetry);

        // Initialize robot:
        robot = new Robot(hardwareMap);

        // Initialize gamepads:
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        AutonConfig selection = EDITING_ALLIANCE;

        // Get gamepad 1 button input and save alliance and side for autonomous configuration:
        while (opModeInInit() && !(gamepadEx1.isDown(RIGHT_BUMPER) && gamepadEx1.isDown(LEFT_BUMPER))) {
            gamepadEx1.readButtons();

            if (keyPressed(1, DPAD_UP))   selection = selection.plus(-1);
            if (keyPressed(1, DPAD_DOWN)) selection = selection.plus(1);

            if (keyPressed(1, X)) switch (selection) {
                case EDITING_ALLIANCE:
                    isRed = !isRed;
                    break;
                case EDITING_SIDE:
                    isBackdropSide = !isBackdropSide;
                    break;
                case EDITING_PARK:
                    parking = parking.plus(1);
                    break;
                case EDITING_CYCLE:
                default:
                    cycle = !cycle;
                    break;
            }

            if (selection == EDITING_WAIT) {
                if (keyPressed(1, Y)) PARTNER_WAIT += 0.5;
                if (keyPressed(1, A) && PARTNER_WAIT > 0) PARTNER_WAIT -= 0.5;
            }

            printConfig(selection);
            mTelemetry.addLine();
            mTelemetry.addLine();
            mTelemetry.addLine("Press both shoulder buttons to CONFIRM!");

            mTelemetry.update();
        }
        robot.preload(isBackdropSide);
        robot.run();

        TeamPropDetector detector = new TeamPropDetector(hardwareMap);
        detector.pipeline.isRed = isRed;

        EditablePose startPose = AutonVars.startPose.byBoth();
//        robot.drivetrain.setPoseEstimate(startPose);

//        TrajectorySequence[] sequences = generateTrajectories(startPose);

        while (opModeInInit()) {
            detector.printTelemetry();
            mTelemetry.addLine();
            mTelemetry.addLine();
            printConfig(selection);
            mTelemetry.update();
        }

        PropDetectPipeline.Randomization location = detector.pipeline.getLocation();
        detector.stop();

//        robot.drivetrain.followTrajectorySequenceAsync(sequences[location.ordinal()]);

        // Control loop:
        while (opModeIsActive()) {
            // Manually clear old sensor data from the last loop:
            robot.readSensors();
            robot.run();

//            autonEndPose = robot.drivetrain.getPoseEstimate();
            // Push telemetry data
            robot.printTelemetry();
            mTelemetry.update();
        }
    }

    private void printConfig(AutonConfig selection) {
        mTelemetry.addLine((isRed ? "RED " : "BLUE ") + selection.markIf(EDITING_ALLIANCE));
        mTelemetry.addLine();
        mTelemetry.addLine((isBackdropSide ? "BACKDROP " : "AUDIENCE ") + "side" + selection.markIf(EDITING_SIDE));
        mTelemetry.addLine();
        mTelemetry.addLine(parking.name() + " PARKING" + selection.markIf(EDITING_PARK));
        mTelemetry.addLine();
        mTelemetry.addLine("WILL " + (cycle ? "CYCLE" : "NOT CYCLE") + selection.markIf(EDITING_CYCLE));
        mTelemetry.addLine();
        mTelemetry.addLine("Pause after spike: " + PARTNER_WAIT + selection.markIf(EDITING_WAIT));
        mTelemetry.addLine();
    }

}
