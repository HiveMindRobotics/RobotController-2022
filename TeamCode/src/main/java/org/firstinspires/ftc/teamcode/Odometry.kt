package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.config.Config
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin


class Odometry {
    var x: Float = 0.0f
    var y: Float = 0.0f

    var prevX = 0
    var prevHeading: Float = 0.0f;

    @Config
    object Odo {
        @JvmField var TRACKWIDTH = 10.75;
    }

    companion object {
        // this is for the 90mm grippy wheels
        val INCHES_PER_ROTATION: Float = (PI * /* diameter: cm */ 9 / 2.54 /* cm to inches */).toFloat()
        // This is for the andymark motors
        val ROTATIONS_PER_TICK: Float = 1 / 537.6f
        val INCHES_PER_TURN: Float
            get() = (Odo.TRACKWIDTH * PI).toFloat()
        val TICKS_PER_TURN: Float
            get() = INCHES_PER_TURN / INCHES_PER_ROTATION / ROTATIONS_PER_TICK
        val TICKS_PER_INCH = 1 / (INCHES_PER_ROTATION * ROTATIONS_PER_TICK)
            /*
            * inches   rotations
            * ------ * ------
            * rotation   tick
            * */
    }

    // https://gm0.org/en/latest/docs/software/concepts/odometry.html#robot-relative-deltas
    // TODO: switch to pose exponentials
    fun update(xL: Int, xR: Int, heading: Float) {

        // get relative position of the center (average)
        val relPos = (xL + xR) / 2
        val deltaPos = relPos - prevX

        // use a rotation matrix, but only do the first bit since we only have Xc
        x += deltaPos * cos(prevHeading) * ROTATIONS_PER_TICK * INCHES_PER_ROTATION
        y += deltaPos * sin(prevHeading) * ROTATIONS_PER_TICK * INCHES_PER_ROTATION

        // we need to use the heading at the start of the movement, not the end
        prevHeading = heading
        prevX = relPos
    }
}