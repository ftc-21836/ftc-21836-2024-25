package org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public final class CachedDcMotorEx {

    public final DcMotorEx motor;

    private double lastPower = Double.NaN;

    public CachedDcMotorEx(DcMotorEx motor) {
        this.motor = motor;
    }

    public void setPower(double power) {
        if (power == lastPower) return;

        motor.setPower(lastPower = power);
    }
}
