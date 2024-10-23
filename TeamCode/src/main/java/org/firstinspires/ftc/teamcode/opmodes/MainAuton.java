package org.firstinspires.ftc.teamcode.opmodes;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.AutonConfig.EDITING_ALLIANCE;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.AutonConfig.EDITING_CYCLE;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.AutonConfig.EDITING_PARK;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.AutonConfig.EDITING_SIDE;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.AutonConfig.EDITING_WAIT;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.ParkingLocation.CORNER;
import static org.firstinspires.ftc.teamcode.opmodes.SharedVars.autonEndPose;
import static org.firstinspires.ftc.teamcode.opmodes.SharedVars.isRed;
import static org.firstinspires.ftc.teamcode.opmodes.SharedVars.keyPressed;
import static org.firstinspires.ftc.teamcode.opmodes.SharedVars.mTelemetry;
import static org.firstinspires.ftc.teamcode.opmodes.SharedVars.gamepadEx1;
import static org.firstinspires.ftc.teamcode.opmodes.SharedVars.gamepadEx2;
import static org.firstinspires.ftc.teamcode.opmodes.SharedVars.loopMod;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.control.vision.detectors.TeamPropDetector;
import org.firstinspires.ftc.teamcode.control.vision.pipelines.PropDetectPipeline;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous(preselectTeleOp = "MainTeleOp")
public final class MainAuton extends LinearOpMode {

    enum AutonConfig {
        EDITING_ALLIANCE,
        EDITING_SIDE,
        EDITING_PARK,
        EDITING_CYCLE,
        EDITING_WAIT;

        public static final AutonConfig[] selections = values();

        public AutonConfig plus(int i) {
            return selections[SharedVars.loopMod(ordinal() + i, selections.length)];
        }
        public String markIf(AutonConfig s) {
            return this == s ? " <" : "";
        }
    }

    enum ParkingLocation {
        CORNER,
        OUTER,
        TOUCHING_RUNG;

        public static final ParkingLocation[] locations = values();

        public ParkingLocation plus(int i) {
            return locations[loopMod(ordinal() + i, locations.length)];
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize multiple telemetry outputs:
        mTelemetry = new MultipleTelemetry(telemetry);

        // Initialize robot:
        Robot robot = new Robot(hardwareMap, new Pose2d(0, 0, 0));

        // Initialize gamepads:
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        AutonConfig selection = EDITING_ALLIANCE;

        ParkingLocation parking = CORNER;

        boolean isRight = true, cycle = false;

        double partnerWait = 0;

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
                    isRight = !isRight;
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
                if (keyPressed(1, Y)) partnerWait += 0.5;
                if (keyPressed(1, A) && partnerWait > 0) partnerWait -= 0.5;
            }

            printConfig(selection, isRight, cycle, parking, partnerWait);
            mTelemetry.addLine();
            mTelemetry.addLine();
            mTelemetry.addLine("Press both shoulder buttons to CONFIRM!");

            mTelemetry.update();
        }

        robot.preload(isRight);
        robot.run();

        TeamPropDetector detector = new TeamPropDetector(hardwareMap);
        detector.pipeline.isRed = isRed;

//        EditablePose startPose = OpModeVars.startPose.byBoth();
//        robot.drivetrain.setPoseEstimate(startPose);

//        TrajectorySequence[] sequences = generateTrajectories(startPose);

        while (opModeInInit()) {
            detector.printTelemetry();
            mTelemetry.addLine();
            mTelemetry.addLine();
            printConfig(selection, isRight, cycle, parking, partnerWait);
            mTelemetry.update();
        }

        PropDetectPipeline.Randomization location = detector.pipeline.getLocation();
        detector.stop();

//        robot.drivetrain.followTrajectorySequenceAsync(sequences[location.ordinal()]);

        // Control loop:
        while (opModeIsActive()) {
            // Manually clear old sensor data from the last loop:
            robot.readSensors();
            autonEndPose = robot.drivetrain.pose;

            robot.run();

            // Push telemetry data
            robot.printTelemetry();
            mTelemetry.update();
        }
    }

    private void printConfig(AutonConfig selection, boolean isRight, boolean cycle, ParkingLocation parking, double partnerWait) {
        mTelemetry.addLine((isRed ? "RED " : "BLUE ") + selection.markIf(EDITING_ALLIANCE));
        mTelemetry.addLine();
        mTelemetry.addLine((isRight ? "RIGHT " : "LEFT ") + "side" + selection.markIf(EDITING_SIDE));
        mTelemetry.addLine();
        mTelemetry.addLine(parking.name() + " PARKING" + selection.markIf(EDITING_PARK));
        mTelemetry.addLine();
        mTelemetry.addLine("WILL " + (cycle ? "CYCLE" : "NOT CYCLE") + selection.markIf(EDITING_CYCLE));
        mTelemetry.addLine();
        mTelemetry.addLine("Pause after spike: " + partnerWait + selection.markIf(EDITING_WAIT));
        mTelemetry.addLine();
    }
}
