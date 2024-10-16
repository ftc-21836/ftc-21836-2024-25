package org.firstinspires.ftc.teamcode.control.gainmatrices

data class ProfileConstraints @JvmOverloads constructor(
    @JvmField var maxV: Double = 1.0,
    @JvmField var maxA: Double = 1.0,
    @JvmField var maxJ: Double = 0.0,
)
