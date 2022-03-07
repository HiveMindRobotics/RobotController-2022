package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import kotlin.math.*
import kotlin.system.measureTimeMillis

class AutoMovement(private val robot: Hardware)  {

    inner class AutonomousAutoMovement(private var opMode: Autonomous) {
        private fun getDistance(): Double {
            val velocity = robot.controlHubIMU!!.velocity.toUnit(DistanceUnit.METER)
            opMode.speed = sqrt(velocity.xVeloc.pow(2) + velocity.yVeloc.pow(2) + velocity.zVeloc.pow((2)))
            opMode.distance += (opMode.speed * opMode.time)
            return opMode.distance
        }

        fun moveDistance(angle: Double, speed: Double, distance: Double) {
            val distance0 = opMode.distance
            while ((getDistance() - distance0) < distance) {
                robot.motorBL?.power = -(speed * sin(angle))
                robot.motorFL?.power = -(speed * cos(angle))
                robot.motorFR?.power = speed * sin(angle)
                robot.motorBR?.power = speed * cos(angle)
            }
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

    fun robotStop() {
        robot.motorBL!!.power = 0.0
        robot.motorFL!!.power = 0.0
        robot.motorBR!!.power = 0.0
        robot.motorFR!!.power = 0.0
    }
}