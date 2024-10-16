package org.firstinspires.ftc.teamcode.subsystems.utilities;

import static org.firstinspires.ftc.teamcode.subsystems.utilities.LEDIndicator.State.GREEN;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.LEDIndicator.State.OFF;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.LEDIndicator.State.RED;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LEDIndicator {

    public enum State {
        RED,
        GREEN,
        AMBER,
        OFF,
    }

    private final DigitalChannel redLED, greenLED;

    public LEDIndicator(HardwareMap hardwareMap, String greenName, String redName) {
        // Get the LED colors and touch sensor from the hardwaremap
        greenLED = hardwareMap.get(DigitalChannel.class, greenName);
        redLED = hardwareMap.get(DigitalChannel.class, redName);

        // change LED mode from input to output
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);
        redLED.setMode(DigitalChannel.Mode.OUTPUT);
    }

    public void setState(State ledColor) {
        greenLED.setState(ledColor == GREEN || ledColor == OFF);
        redLED.setState(ledColor == RED || ledColor == OFF);
    }
}