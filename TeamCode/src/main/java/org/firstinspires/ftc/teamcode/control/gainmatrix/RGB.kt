package org.firstinspires.ftc.teamcode.control.gainmatrix

import org.firstinspires.ftc.teamcode.opmode.Auto.mTelemetry
import kotlin.math.max
import kotlin.math.min

data class RGB

/**
 * @param red Red value in [0, 255]
 * @param green Green value in [0, 255]
 * @param blue Blue value in [0, 255]
 */
@JvmOverloads
constructor(
    @JvmField var red: Double = 0.0,
    @JvmField var green: Double = 0.0,
    @JvmField var blue: Double = 0.0,
) {
    constructor(array: Array<Double>) : this(array[0], array[1], array[2])
    constructor(array: FloatArray) : this(array[0].toDouble(), array[1].toDouble(), array[2].toDouble())

    fun between(min:RGB, max:RGB): Boolean {

        val redInRange = (min.red <= red) && (red <= max.red)
        val greenInRange = (min.green <= green) && (green <= max.green)
        val blueInRange = (min.blue <= blue) && (blue <= max.blue)

        return redInRange && greenInRange && blueInRange
    }

    fun toTelemetry(title: String) {
        mTelemetry.addLine("$title:")
        mTelemetry.addData("Red", red)
        mTelemetry.addData("Green", green)
        mTelemetry.addData("Blue", blue)
    }

    override fun toString(): String {
        return "$red, $green, $blue"
    }

    fun toHSV(): HSV {
        // R, G, B values are divided by 255
        // to change the range from 0..255 to 0..1
        val r = red / 255.0
        val g = green / 255.0
        val b = blue / 255.0

        val colorMax = max(r, max(g, b)) // maximum of r, g, b

        val colorMin = min(r, min(g, b)) // minimum of r, g, b

        val diff = colorMax - colorMin // diff of cmax and cmin.

        return HSV(
            when (colorMax) {
                colorMin -> 0.0
                r -> (60 * ((g - b) / diff) + 360) % 360
                g -> (60 * ((b - r) / diff) + 120) % 360
                b -> (60 * ((r - g) / diff) + 240) % 360
                else -> 0.0
            },
            if (colorMax == 0.0) 0.0 else diff / colorMax,
            colorMax
        )
    }
}
