package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.opmodes.OpModeVars.divider;
import static org.firstinspires.ftc.teamcode.opmodes.OpModeVars.mTelemetry;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.cachedhardware.CachedSimpleServo.getGBServo;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot;

@Config
public final class Deposit {

    public static double
            ANGLE_ARM_RETRACTED = 10,
            ANGLE_ARM_SPECIMEN = 110, // wall pickup and chambers
            ANGLE_ARM_SAMPLE = 151, // dropping in observation zone and baskets

            ANGLE_CLAW_OPEN = 80,
            ANGLE_CLAW_TRANSFER = 45,
            ANGLE_CLAW_CLOSED = 29,

            TIME_DROP = 0.5,
            TIME_ARM_RETRACTION = 0.25,
            TIME_POST_TRANSFER = 0.25,
            TIME_GRAB = 0.25,

            COLOR_SENSOR_GAIN = 1,

            HEIGHT_INTAKING_SPECIMEN = 0.1,
            HEIGHT_OFFSET_POST_INTAKING = 4,
            HEIGHT_OBSERVATION_ZONE = 1,
            HEIGHT_BASKET_LOW = 20,
            HEIGHT_BASKET_HIGH = 32,
            HEIGHT_CHAMBER_LOW = 6,
            HEIGHT_CHAMBER_HIGH = 20,
            HEIGHT_OFFSET_SPECIMEN_SCORING = -10;

    enum State {
        RETRACTED,
        HAS_SAMPLE,
        INTAKING_SPECIMEN,
        HAS_SPECIMEN,
    }

    private final Lift lift;
    private final SimpleServoPivot claw;

    Deposit(HardwareMap hardwareMap) {

        lift = new Lift(hardwareMap);

        claw = new SimpleServoPivot(
                ANGLE_CLAW_TRANSFER,
                ANGLE_CLAW_CLOSED,
                getGBServo(hardwareMap, "claw").reversed()
        );
    }

    void printTelemetry() {
        mTelemetry.addLine("DEPOSIT: ");
        mTelemetry.addLine();
        mTelemetry.addLine("Empty");
        divider();
        lift.printTelemetry();
    }

}
