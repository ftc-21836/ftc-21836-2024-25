package org.firstinspires.ftc.teamcode.subsystems.utilities.sensors;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.control.gainmatrices.HSV;
import org.firstinspires.ftc.teamcode.control.gainmatrices.RGB;

public class TestRGBToHSV {

    public static void main(String[] args) {

        NormalizedRGBA rgba = new NormalizedRGBA();
        rgba.red = .75F;
        rgba.green = 1F;
        rgba.blue = .5F;
        rgba.alpha = 1F;

        RGB rgb = new RGB(
                rgba.red * 255,
                rgba.green * 255,
                rgba.blue * 255
        );

        float[] hsvArray = new float[3];
//        Color.colorToHSV(rgba.toColor(), hsvArray);
        HSV hsv1 = new HSV(hsvArray);
        HSV hsv2 = rgb.toHSV();

        System.out.println(rgb);
        System.out.println(hsv2);
    }
}
