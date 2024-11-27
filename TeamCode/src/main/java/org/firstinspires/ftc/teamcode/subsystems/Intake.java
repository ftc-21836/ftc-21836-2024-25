package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.Sample.BLUE;
import static org.firstinspires.ftc.teamcode.subsystems.Sample.NEUTRAL;
import static org.firstinspires.ftc.teamcode.subsystems.Sample.RED;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.control.gainmatrices.HSV;

@Config
public final class Intake {

    public static double
            COLOR_SENSOR_GAIN = 1;

    /**
     * HSV value bound for intake bucket sample detection
     */
    public static HSV
            minRed = new HSV(
            0,
                    0.5,
                    0
            ),
            maxRed = new HSV(
                    30,
                    0.75,
                    0.06
            ),
            minYellow = new HSV(
                    80,
                    0.6,
                    0
            ),
            maxYellow = new HSV(
                    96,
                    0.85,
                    0.3
            ),
            minBlue = new HSV(
                    215,
                    0.6,
                    0
            ),
            maxBlue = new HSV(
                    230,
                    0.9,
                    0.1
            );

    /**
     * @return The {@link Sample} corresponding to the provided {@link HSV} as per the tuned value bounds
     */
    public static Sample hsvToSample(HSV hsv) {
        return
                hsv.between(minRed, maxRed) ? RED :
                hsv.between(minBlue, maxBlue) ? BLUE :
                hsv.between(minYellow, maxYellow) ? NEUTRAL :
                null;
    }


}
