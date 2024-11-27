package org.firstinspires.ftc.teamcode.control.gainmatrix

data class LowPassGains @JvmOverloads constructor(
    @JvmField var gain: Double = 0.0,
    @JvmField var count: Int = 2,
)
