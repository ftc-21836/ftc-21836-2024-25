package org.firstinspires.ftc.teamcode.control

import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action

data class FirstTerminateAction(
    val initialActions: List<Action>
) : Action {
    private var actions = initialActions

    constructor(vararg actions: Action) : this(actions.asList())

    override fun run(p: TelemetryPacket): Boolean {
        if (actions.isEmpty()) return false

        return actions.all { it.run(p) }
    }

    override fun preview(fieldOverlay: Canvas) {
        initialActions.forEach { a -> a.preview(fieldOverlay) }
    }
}