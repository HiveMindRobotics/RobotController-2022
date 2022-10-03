package org.firstinspires.ftc.teamcode

import kotlin.math.cos
import kotlin.math.sin


class Odometry {
    var x: Float = 0.0f
    var y: Float = 0.0f

    var prevX = 0
    var prevHeading: Float = 0.0f;

    companion object {
        // TODO: add actual value
        // UNITS: CM
        const val WHEEL_CIRCUMFERENCE: Double = 90.0
        // TODO: change for new motors
        const val TICKS_PER_ROTATION: Int = 28 * 3
    }

    // https://gm0.org/en/latest/docs/software/concepts/odometry.html#robot-relative-deltas
    fun update(xL: Int, xR: Int, heading: Float) {
        // get relative position of the center (average)
        val relPos = (xL + xR) / 2
        val deltaPos = relPos - prevX

        // use a rotation matrix, but only do the first bit since we only have Xc
        x += deltaPos * cos(prevHeading)
        y += deltaPos * sin(prevHeading)

        // we need to use the heading at the start of the movement, not the end
        prevHeading = heading
        prevX = relPos
    }
}