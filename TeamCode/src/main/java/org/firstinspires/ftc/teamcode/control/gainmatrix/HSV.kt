package org.firstinspires.ftc.teamcode.control.gainmatrix

import org.firstinspires.ftc.teamcode.opmode.OpModeVars.mTelemetry
import org.opencv.core.Scalar

data class HSV

/**
 * @param hue Hue in [0, 360]
 * @param saturation Saturation in [0, 1]
 * @param value Value in [0, 1]
 */
@JvmOverloads
constructor(
    @JvmField var hue: Double = 0.0,
    @JvmField var saturation: Double = 0.0,
    @JvmField var value: Double = 0.0,
) {
    constructor(array: Array<Double>) : this(array[0], array[1], array[2])
    constructor(array: FloatArray) : this(array[0].toDouble(), array[1].toDouble(), array[2].toDouble())

    fun between(min:HSV, max:HSV): Boolean {

        val hueInRange = (min.hue <= hue) && (hue <= max.hue)
        val saturationInRange = (min.saturation <= saturation) && (saturation <= max.saturation)
        val valueInRange = (min.value <= value) && (value <= max.value)

        return hueInRange && saturationInRange && valueInRange
    }

    fun toTelemetry() {
        mTelemetry.addData("Hue", hue)
        mTelemetry.addData("Saturation", saturation)
        mTelemetry.addData("Value", value)
    }

    override fun toString(): String {
        return "$hue, $saturation, $value"
    }

    fun toScalar(): Scalar {
        return Scalar(hue, saturation, value)
    }
}
