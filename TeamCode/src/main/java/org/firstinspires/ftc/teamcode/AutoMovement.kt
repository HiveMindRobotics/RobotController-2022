package org.firstinspires.ftc.teamcode

import kotlin.math.*
import kotlin.system.measureTimeMillis

class AutoMovement(private val robot: Hardware)  {

    inner class AutonomousAutoMovement(private val vuforia: Vuforia) {
        fun moveDistance(angle: Double, speed: Double, distance: Double) {
            val point0 = vuforia.lastLocation!!.translation
            while (hypot(vuforia.lastLocation!!.translation[0] - point0[0], point0[0] - vuforia.lastLocation!!.translation[1]) < distance) {
                robot.motorBL?.power = -(speed * sin(angle))
                robot.motorFL?.power = -(speed * cos(angle))
                robot.motorFR?.power = speed * sin(angle)
                robot.motorBR?.power = speed * cos(angle)
            }
        }

        fun moveToCoords(x: Double, y: Double, speed: Double) {
            val angle: Double = Math.toDegrees(atan2(x - vuforia.lastLocation!!.translation[0], y - vuforia.lastLocation!!.translation[1]))
            val distance: Double = hypot(x - vuforia.lastLocation!!.translation[0], y - vuforia.lastLocation!!.translation[1])
            moveDistance(angle, speed, distance)
        }
    }

    fun armGrab() {
        robot.servoArm!!.position = 1.0
    }

    fun armRelease() {
        robot.servoArm!!.position = 0.0
    }

    fun ducksStart() {
        robot.motorDucks!!.power = 1.0
    }

    fun ducksStop() {
        robot.motorDucks!!.power = 0.0
    }

    enum class Position {
        TOP, BOTTOM, MIDDLE
    }

    fun armRaise(position: Position) {
        when(position) {
            Position.TOP -> {
                robot.motorArm?.power = -1.0
                while (robot.motorArm?.isBusy == true) {
                    Thread.sleep(100)
                }
                robot.motorArm?.power = 0.0
            }
            Position.BOTTOM -> {
                robot.motorArm?.power = 1.0
                while (robot.motorArm?.isBusy == true) {
                    Thread.sleep(100)
                }
                robot.motorArm?.power = 0.0
            }
            Position.MIDDLE -> {
                val time = measureTimeMillis {
                    armRaise(Position.TOP)
                }
                armRaise(Position.BOTTOM)
                robot.motorArm?.power = -1.0
                Thread.sleep(time / 2)
                robot.motorArm?.power = 0.0
            }
        }
    }

    enum class Direction {
        FORWARD, BACKWARD, LEFT, RIGHT
    }

    fun robotTranslate(speed: Double, direction: Direction) {
        when(direction) {
            Direction.FORWARD -> {
                robot.motorBL!!.power = speed
                robot.motorFL!!.power = speed
                robot.motorBR!!.power = speed
                robot.motorFR!!.power = speed
            }
            Direction.BACKWARD -> {
                robot.motorBL!!.power = -speed
                robot.motorFL!!.power = -speed
                robot.motorBR!!.power = -speed
                robot.motorFR!!.power = -speed
            }
            Direction.LEFT -> {
                robot.motorBL!!.power = speed
                robot.motorFL!!.power = -speed
                robot.motorBR!!.power = speed
                robot.motorFR!!.power = -speed
            }
            Direction.RIGHT -> {
                robot.motorBL!!.power = speed
                robot.motorFL!!.power = speed
                robot.motorBR!!.power = -speed
                robot.motorFR!!.power = -speed
            }
        }
    }

    fun robotRotateLeft(speed: Double) {
        robot.motorBL!!.power = -speed
        robot.motorFL!!.power = -speed
        robot.motorBR!!.power = speed
        robot.motorFR!!.power = speed
    }

    fun robotRotateRight(speed: Double) {
        robot.motorBL!!.power = speed
        robot.motorFL!!.power = speed
        robot.motorBR!!.power = -speed
        robot.motorFR!!.power = -speed
    }

    fun robotRotateByAngle(speed: Double, angle: Double) {
        val position0 = robot.controlHubIMU!!.angularOrientation.thirdAngle
        val speed1 = if (angle > 0) {
            speed
        } else {
            -speed
        }
        robot.motorFL?.power = speed1
        robot.motorBL?.power = speed1
        robot.motorFR?.power = -speed1
        robot.motorBR?.power = -speed1
        Thread.sleep(100)
        while (robot.controlHubIMU!!.angularOrientation.thirdAngle.absoluteValue - position0.absoluteValue > angle.absoluteValue) {
            Thread.sleep(100)
        }
        robotStop()
    }

    fun robotRotateToAngle(speed: Double, angle: Double) {
        val speed1 = if (angle > 0) {
            speed
        } else {
            -speed
        }
        robot.motorFL?.power = speed1
        robot.motorBL?.power = speed1
        robot.motorFR?.power = -speed1
        robot.motorBR?.power = -speed1
        Thread.sleep(100)
        while (angle.absoluteValue - robot.controlHubIMU!!.angularOrientation.thirdAngle.absoluteValue > 0) {
            Thread.sleep(100)
        }
        robotStop()
    }

    fun robotStop() {
        robot.motorBL!!.power = 0.0
        robot.motorFL!!.power = 0.0
        robot.motorBR!!.power = 0.0
        robot.motorFR!!.power = 0.0
    }
}