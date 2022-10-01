package org.firstinspires.ftc.teamcode

import kotlin.math.cos
import kotlin.math.sin

class Odometry() {
    var x: Float = 0.0f
    var y: Float = 0.0f

    var prevX = 0
    // var prevHeading: Float = 0.0f;

    companion object {
        // UNIT
        const val WHEEL_CIRCUMFERENCE: Double = 90.0;
    }

    fun update(xL: Int, xR: Int, heading: Float) {
        // 28 * 3
        val relPos = (xL + xR) / 2
        val deltaPos = relPos - prevX

        x += deltaPos * cos(heading)
        y += deltaPos * sin(heading)

        // prevHeading = heading
        prevX = relPos
    }
}