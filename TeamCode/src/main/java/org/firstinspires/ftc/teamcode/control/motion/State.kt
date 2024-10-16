package org.firstinspires.ftc.teamcode.control.motion

import org.firstinspires.ftc.teamcode.control.gainmatrices.FeedforwardGains
import org.firstinspires.ftc.teamcode.control.gainmatrices.FullStateGains

data class State

@JvmOverloads
constructor(
    @JvmField val x: Double = 0.0,
    @JvmField val v: Double = 0.0,
    @JvmField val a: Double = 0.0,
    @JvmField val j: Double = 0.0,
) {

    operator fun plus(other: State): State {
        return State(
            x + other.x,
            v + other.v,
            a + other.a,
            j + other.j,
        )
    }

    operator fun unaryMinus(): State {
        return State(
            -x,
            -v,
            -a,
            -j,
        )
    }

    operator fun minus(other: State): State {
        return this + -other
    }

    operator fun times(gains: FullStateGains): State {
        return State(
            x * gains.pGain,
            v * gains.vGain,
            a * gains.aGain,
        )
    }

    operator fun times(gains: FeedforwardGains): State {
        return State(
            0.0,
            v * gains.kV,
            a * gains.kA,
        )
    }

    /**
     * Returns the sum of all the derivatives of this [State].
     * Only to be used to get a control output.
     */
    fun sum(): Double {
        return x + v + a + j
    }
}
