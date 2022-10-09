package org.firstinspires.ftc.teamcode

import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin


class Odometry {
    var x: Float = 0.0f
    var y: Float = 0.0f

    var firstLoop = true
    var baseR = 0
    var baseL = 0

    var prevX = 0
    var prevHeading: Float = 0.0f;

    companion object {
        // this is for the 90mm grippy wheels
        const val INCHES_PER_ROTATION: Float = (PI * /* diameter: cm */ 9 / 2.54 /* cm to inches */).toFloat()
        // This is for the andymark motors
        const val ROTATIONS_PER_TICK: Float = 1 / 537.6f
    }

    // https://gm0.org/en/latest/docs/software/concepts/odometry.html#robot-relative-deltas
    // TODO: switch to pose exponentials
    fun update(xL: Int, xR: Int, heading: Float) {
        if (firstLoop) {
            firstLoop = false
            baseL = xL
            baseR = xR
        }

        // get relative position of the center (average)
        val relPos = ((xL - baseL) + (xR - baseR)) / 2
        val deltaPos = relPos - prevX

        // use a rotation matrix, but only do the first bit since we only have Xc
        x += deltaPos * cos(prevHeading) * ROTATIONS_PER_TICK * INCHES_PER_ROTATION
        y += deltaPos * sin(prevHeading) * ROTATIONS_PER_TICK * INCHES_PER_ROTATION

        // we need to use the heading at the start of the movement, not the end
        prevHeading = heading
        prevX = relPos
    }
}