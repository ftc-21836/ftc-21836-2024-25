package org.firstinspires.ftc.teamcode.subsystem.utility.sensor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.teamcode.control.gainmatrix.HSV;
import org.firstinspires.ftc.teamcode.control.gainmatrix.RGB;

public final class ColorSensor {

    private final NormalizedColorSensor sensor;

    private HSV hsv = new HSV();
    private final RGB rgb = new RGB();

    public ColorSensor(HardwareMap hardwareMap, String name, float gain) {
        sensor = hardwareMap.get(NormalizedColorSensor.class, name);
        sensor.setGain(gain);
        enableLight(true);
    }

    public void enableLight(boolean lightOn) {
        if (sensor instanceof SwitchableLight) ((SwitchableLight) sensor).enableLight(lightOn);
    }

    public void update() {
        NormalizedRGBA rgba = sensor.getNormalizedColors();

        rgb.red = (double) rgba.red * 255;
        rgb.green = (double) rgba.green * 255;
        rgb.blue = (double) rgba.blue * 255;

        hsv = rgb.toHSV();
    }

    public HSV getHSV() {
        return hsv;
    }

    public RGB getRGB() {
        return rgb;
    }
}
