package org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware;

import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public final class CachedSimpleServo extends SimpleServo {

    public CachedSimpleServo(HardwareMap hw, String servoName, double minAngle, double maxAngle, AngleUnit angleUnit) {
        super(hw, servoName, minAngle, maxAngle, angleUnit);
    }

    public CachedSimpleServo(HardwareMap hw, String servoName, double minDegree, double maxDegree) {
        super(hw, servoName, minDegree, maxDegree);
    }

    public static CachedSimpleServo getAxon(HardwareMap hardwareMap, String name) {
        return new CachedSimpleServo(hardwareMap, name, 0, 355);
    }

    public static CachedSimpleServo getGBServo(HardwareMap hardwareMap, String name) {
        return new CachedSimpleServo(hardwareMap, name, 0, 280);
    }

    public CachedSimpleServo reversed() {
        setInverted(true);
        return this;
    }

    private double lastDegrees = 0;

    public void turnToAngle(double degrees) {
        if (degrees == lastDegrees) return;

        super.turnToAngle(lastDegrees = degrees);
    }
}
