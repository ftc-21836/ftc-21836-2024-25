package org.firstinspires.ftc.teamcode.subsystems.utilities.CachedHardware;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public final class CachedMotorEx extends MotorEx {

    public CachedMotorEx(@NonNull HardwareMap hMap, String id) {
        super(hMap, id);
    }

    public CachedMotorEx(@NonNull HardwareMap hMap, String id, @NonNull GoBILDA gobildaType) {
        super(hMap, id, gobildaType);
    }

    public CachedMotorEx(@NonNull HardwareMap hMap, String id, double cpr, double rpm) {
        super(hMap, id, cpr, rpm);
    }

    private double lastPower = 0;

    public void set(double power) {
        if (power == lastPower) return;

        super.set(lastPower = power);
    }
}
