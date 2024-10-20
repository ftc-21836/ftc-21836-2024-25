package org.firstinspires.ftc.teamcode.subsystems.utilities.CachedHardware;

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

    private double lastPosition = 0;

    public void turnToAngle(double position) {
        if (position == lastPosition) return;

        super.turnToAngle(lastPosition = position);
    }
}
