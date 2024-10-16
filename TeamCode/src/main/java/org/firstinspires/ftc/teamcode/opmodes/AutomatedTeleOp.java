package org.firstinspires.ftc.teamcode.opmodes;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Pixel.Color.EMPTY;
import static org.firstinspires.ftc.teamcode.opmodes.AutomatedTeleOp.TeleOpState.CALCULATE;
import static org.firstinspires.ftc.teamcode.opmodes.AutomatedTeleOp.TeleOpState.MANUAL;
import static org.firstinspires.ftc.teamcode.opmodes.AutomatedTeleOp.TeleOpState.MOVING;
import static org.firstinspires.ftc.teamcode.opmodes.AutomatedTeleOp.TeleOpState.SCORING;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.isRed;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.autonBackdrop;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.gamepadEx1;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.gamepadEx2;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.keyPressed;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.loopMod;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.mTelemetry;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.robot;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.toPose2d;
import static org.firstinspires.ftc.teamcode.opmodes.MainTeleOp.doAutoSlow;
import static org.firstinspires.ftc.teamcode.opmodes.MainTeleOp.isTranslating;
import static org.firstinspires.ftc.teamcode.opmodes.MainTeleOp.teleOpControls;
import static org.firstinspires.ftc.teamcode.opmodes.MainTeleOp.teleOpInit;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Deposit.Paintbrush.TIME_DROP_FIRST;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.motion.PIDDriver;
import org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Backdrop;
import org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Pixel;
import org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.PlacementCalculator;

@TeleOp
@Config
@Disabled
public final class AutomatedTeleOp extends LinearOpMode {

    enum TeleOpState {
        MANUAL,
        CALCULATE,
        MOVING,
        SCORING;
    }

    private TeleOpState opState = MANUAL;

    private final PIDDriver driver = new PIDDriver();
    private final ElapsedTime timer = new ElapsedTime();

    private final Backdrop backdrop = autonBackdrop;
    private final PlacementCalculator calculator = new PlacementCalculator();
    private Pixel placement = new Pixel(0, 0, EMPTY);
    private final Pixel.Color[] wingColors = {EMPTY, EMPTY};

    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime loopTimer = new ElapsedTime();

        teleOpInit(this);

        // Control loop:
        while (opModeIsActive()) {
            // Read sensors + gamepads:
            robot.readSensors();
            gamepadEx1.readButtons();
            gamepadEx2.readButtons();

            if (isTranslating()) opState = MANUAL;
            else {
                if (keyPressed(1, X) && robot.hasAPixel()) opState = CALCULATE;
            }

            autoScoringStateMachine();

            robot.run();

            mTelemetry.addData("Loop time", loopTimer.seconds());
            loopTimer.reset();
            mTelemetry.addLine();
            mTelemetry.addLine();

            mTelemetry.addData("Auto slow is", doAutoSlow ? "enabled" : "disabled");
            mTelemetry.addLine();
            robot.printTelemetry();
            mTelemetry.update();
        }
    }

    private void autoScoringStateMachine() {


        switch (opState) {

            default:
            case MANUAL:
                teleOpControls();
                break;

            case CALCULATE:

                updatePlacement();

                opState = MOVING;

            case MOVING:

                robot.deposit.lift.setTargetRow(placement.y);

                boolean reached1 = driver.driveTo(robot.drivetrain, toPose2d(placement));

                if (!reached1) break;

                backdrop.add(new Pixel(placement, robot.deposit.paintbrush.colors[
                        robot.has2Pixels() ? 0 : 1
                ]));
                robot.deposit.paintbrush.dropPixel();
                timer.reset();
                opState = SCORING;

            case SCORING:

                driver.driveTo(robot.drivetrain, toPose2d(placement));

                if (timer.seconds() <= TIME_DROP_FIRST) break;

                opState = robot.hasAPixel() ? CALCULATE : MANUAL;
        }
    }

    private void updatePlacement() {

        placement = (
                robot.deposit.paintbrush.colors[0] == EMPTY ?
                        robot.deposit.paintbrush.colors[1] :
                        robot.deposit.paintbrush.colors[0]
        ).getCounterpartIn(
                calculator.getOptimalPlacements(backdrop),
                isRed
        );
    }
}
