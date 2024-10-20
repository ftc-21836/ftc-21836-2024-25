package org.firstinspires.ftc.teamcode.subsystems.utilities.CachedHardware;

import com.arcrobotics.ftclib.hardware.SimpleServo;

public final class CachedSimpleServo {

    public final SimpleServo servo;

    private double lastPosition = 0;

    public CachedSimpleServo(SimpleServo servo) {
        this.servo = servo;
    }

    public void turnToAngle(double position) {
        if (position == lastPosition) return;

        servo.setPosition(lastPosition = position);
    }
}
