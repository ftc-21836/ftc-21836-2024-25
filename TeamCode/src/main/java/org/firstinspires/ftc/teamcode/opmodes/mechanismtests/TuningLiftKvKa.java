package org.firstinspires.ftc.teamcode.opmodes.mechanismtests;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA.RPM_1150;
import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.FLOAT;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.gamepadEx1;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.keyPressed;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.mTelemetry;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Deposit.Lift.INCHES_PER_TICK;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Deposit.Lift.POS_1;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Deposit.Lift.POS_2;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Deposit.Lift.feedforwardGains;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Deposit.Lift.kG;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Deposit.Lift.lowPassGains;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Deposit.Lift.profileConstraints;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot.maxVoltage;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.control.controllers.FeedforwardController;
import org.firstinspires.ftc.teamcode.control.filters.FIRLowPassFilter;
import org.firstinspires.ftc.teamcode.control.motion.Differentiator;
import org.firstinspires.ftc.teamcode.control.motion.MotionProfiler;
import org.firstinspires.ftc.teamcode.control.motion.State;
import org.firstinspires.ftc.teamcode.subsystems.utilities.BulkReader;

@TeleOp(group = "Single mechanism test")
public final class TuningLiftKvKa extends LinearOpMode {

    // Motors and variables to manage their readings:
    public MotorEx[] motors;
    public State currentState = new State(), targetState = new State();
    public VoltageSensor batteryVoltageSensor;
    public FeedforwardController controller = new FeedforwardController(feedforwardGains);
    public MotionProfiler profiler = new MotionProfiler();

    public FIRLowPassFilter firFilter = new FIRLowPassFilter(lowPassGains);

    public Differentiator differentiator = new Differentiator();

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize multiple telemetry outputs:
        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize gamepads:
        gamepadEx1 = new GamepadEx(gamepad1);

        BulkReader bulkReader = new BulkReader(hardwareMap);

        this.batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        this.motors = new MotorEx[]{
                new MotorEx(hardwareMap, "lift right", RPM_1150),
                new MotorEx(hardwareMap, "lift left", RPM_1150)
        };
        motors[1].setInverted(true);
        for (MotorEx motor : motors) {
            motor.setZeroPowerBehavior(FLOAT);
            motor.encoder.reset();
        }

        profiler = new MotionProfiler();

        waitForStart();

        // Control loop:
        while (opModeIsActive()) {
            bulkReader.bulkRead();
            gamepadEx1.readButtons();

            firFilter.setGains(lowPassGains);
            controller.setGains(feedforwardGains);
            profiler.setConstraints(profileConstraints);

            currentState = new State(
                    INCHES_PER_TICK * 0.5 * (motors[0].encoder.getPosition() + motors[1].encoder.getPosition()),
                    INCHES_PER_TICK * 0.5 * (motors[0].encoder.getCorrectedVelocity() + motors[1].encoder.getCorrectedVelocity())
            );

            if (keyPressed(1, DPAD_DOWN)) profiler.generateProfile(currentState, new State(POS_1));
            if (keyPressed(1, DPAD_UP)) profiler.generateProfile(currentState, new State(POS_2));

            profiler.update();
            targetState = profiler.getState();
            controller.setTarget(targetState);

            double manualLiftPower = gamepadEx1.getLeftY();
            double voltageScalar = maxVoltage / batteryVoltageSensor.getVoltage();
            double output = ((
                    manualLiftPower != 0 ?
                            manualLiftPower :
                            controller.calculate()
            ) + (
                    currentState.x > 0.15 ?
                            kG :
                            0
            )) * voltageScalar;
            for (MotorEx motor : motors) motor.set(output);

            mTelemetry.addData("Current position (in)", currentState.x);
            mTelemetry.addData("Target position (in)", targetState.x);
            mTelemetry.addLine();
            mTelemetry.addData("Current raw velocity (in/s)", currentState.v);
            mTelemetry.addData("Current FIR low pass filtered velocity (in/s)", firFilter.calculate(currentState.v));
            mTelemetry.addData("Target velocity (in/s)", targetState.v);
            mTelemetry.update();
        }
    }

}
