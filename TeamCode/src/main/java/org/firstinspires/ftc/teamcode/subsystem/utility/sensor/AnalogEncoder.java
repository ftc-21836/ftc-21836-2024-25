package org.firstinspires.ftc.teamcode.subsystem.utility.sensor;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AnalogEncoder {

    private final AnalogInput encoder;

    private final double maxAngle;

    public AnalogEncoder(HardwareMap hardwareMap, String encoderName, double maxAngle) {

        encoder = hardwareMap.get(AnalogInput.class, encoderName);

        this.maxAngle = maxAngle;
    }

    public double getPosition() {
        return maxAngle * encoder.getVoltage() / encoder.getMaxVoltage();
    }
}
