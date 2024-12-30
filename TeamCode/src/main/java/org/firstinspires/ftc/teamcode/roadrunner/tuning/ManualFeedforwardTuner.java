package org.firstinspires.ftc.teamcode.roadrunner.tuning;

import static com.acmerobotics.roadrunner.Profiles.constantProfile;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeProfile;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.FeedforwardFactory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.control.filter.FIRLowPassFilter;
import org.firstinspires.ftc.teamcode.control.gainmatrix.LowPassGains;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Config
@TeleOp
public class ManualFeedforwardTuner extends LinearOpMode {

    public static double DISTANCE = 64;

    public static boolean FILTER = false;

    enum Mode {
        DRIVER_MODE,
        TUNING_MODE
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        FIRLowPassFilter filter = new FIRLowPassFilter(new LowPassGains(0.75, 2));

        double maxVel = MecanumDrive.PARAMS.maxWheelVel;
        double minAccel = MecanumDrive.PARAMS.minProfileAccel;
        double maxAccel = MecanumDrive.PARAMS.maxProfileAccel;
        TimeProfile profile = new TimeProfile(constantProfile(
                DISTANCE, 0.0, maxVel, minAccel, maxAccel).baseProfile);

        Mode mode = Mode.TUNING_MODE;

        telemetry.addLine("Ready!");
        telemetry.update();
        telemetry.clearAll();

        waitForStart();

        if (isStopRequested()) return;

        boolean movingForwards = true;
        double startTs = System.nanoTime() / 1e9;


        FeedforwardFactory feedforwardFactory = () -> new MotorFeedforward(MecanumDrive.PARAMS.kS,
                MecanumDrive.PARAMS.kV / MecanumDrive.PARAMS.inPerTick,
                MecanumDrive.PARAMS.kA / MecanumDrive.PARAMS.inPerTick);

        while (!isStopRequested()) {
            telemetry.addData("mode", mode);

            drive.updatePoseEstimate();

            switch (mode) {
                case TUNING_MODE:

                    if (gamepad1.y) mode = Mode.DRIVER_MODE;

                    double v = drive.localizer.getVelocity().linearVel.x;
                    if (FILTER) v = filter.calculate(v);
                    telemetry.addData("velocity", v);

                    double ts = System.nanoTime() / 1e9;
                    double t = ts - startTs;
                    if (t > profile.duration) {
                        movingForwards = !movingForwards;
                        startTs = ts;
                    }

                    DualNum<Time> v1 = profile.get(t).drop(1);
                    if (!movingForwards) {
                        v1 = v1.unaryMinus();
                    }
                    telemetry.addData("vref", v1.value());

                    double power = feedforwardFactory.make().compute(v1) / drive.voltageSensor.getVoltage();
                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(power, 0), 0));

                    break;

                case DRIVER_MODE:

                    if (gamepad1.b) {
                        mode = Mode.TUNING_MODE;
                        movingForwards = true;
                        startTs = System.nanoTime() / 1e9;
                    }

                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x
                            ),
                            -gamepad1.right_stick_x
                    ));

                    break;
            }

            telemetry.update();

        }
    }

}
