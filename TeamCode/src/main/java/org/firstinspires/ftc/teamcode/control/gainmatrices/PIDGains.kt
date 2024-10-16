package org.firstinspires.ftc.teamcode.control.gainmatrices

import kotlin.Double.Companion.POSITIVE_INFINITY
import kotlin.math.*

data class PIDGains

@JvmOverloads
constructor(
    @JvmField var kP: Double = 0.0,
    @JvmField var kI: Double = 0.0,
    @JvmField var kD: Double = 0.0,
    @JvmField var maxOutputWithIntegral: Double = POSITIVE_INFINITY,
) {

    @JvmOverloads
    fun computeKd(gains: FeedforwardGains, percentOvershoot: Double = 0.0): PIDGains {
        kD = computeKd(kP, gains.kV, gains.kA, percentOvershoot)
        return this
    }
}

@JvmOverloads
fun computeKd(kP: Double, kV: Double, kA: Double, percentOvershoot: Double = 0.0): Double {
    val overshoot: Double = percentOvershoot / 100.0
    val zeta: Double = if (overshoot <= 0.0) 1.0 else -ln(overshoot) / sqrt(PI.pow(2) + ln(overshoot).pow(2))
    return max(
        2 * zeta * sqrt(kA * kP) - kV,
        0.0
    )
}
