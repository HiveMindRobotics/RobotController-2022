package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import kotlin.math.*
import kotlin.system.measureTimeMillis

class AutoMovement(private val robot: Hardware, private val opMode: LinearOpMode)  {

    inner class AutonomousAutoMovement(private val vuforia: Vuforia) {
        fun moveDistance(angle: Double, speed: Double, distance: Double) {
            vuforia.getPosition()
            val point0 = vuforia.lastLocation!!.translation
            robotMap(
                speed * sin(angle),
                speed * cos(angle),
                speed * sin(angle),
                speed * cos(angle)
            )
            while (hypot(vuforia.lastLocation!!.translation[0] - point0[0], point0[0] - vuforia.lastLocation!!.translation[1]) < distance) {
                vuforia.getPosition()
            }
            robotStop()
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
                while (robot.motorArm?.isBusy == false) {
                    Thread.sleep(100)
                }
                robot.motorArm?.power = 0.0
            }
            Position.BOTTOM -> {
                robot.motorArm?.power = 1.0
                while (robot.motorArm?.isBusy == false) {
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
                robotMap(speed, speed, speed, speed)
            }
            Direction.BACKWARD -> {
                robotMap(-speed, -speed, -speed, -speed)
            }
            Direction.LEFT -> {
                robotMap(-speed, -speed, speed, speed)
            }
            Direction.RIGHT -> {
                robotMap(speed, speed, -speed, -speed)
            }
        }
    }

    fun robotRotateLeft(speed: Double) {
        robotMap(speed, speed, -speed, -speed)
    }

    fun robotRotateRight(speed: Double) {
        robotMap(-speed, -speed, speed, speed)
    }

    fun robotRotateByAngle(speed: Double, angle: Double) {
        robotRotateToAngle(speed, (robot.controlHubIMU!!.angularOrientation.firstAngle + angle) % 360)
    }

    fun robotRotateToAngle(speed: Double, angle: Double) {
        if (robot.controlHubIMU!!.angularOrientation.firstAngle - angle < 0) {
            robotRotateLeft(speed)
            Thread.sleep(100)
            while (robot.controlHubIMU!!.angularOrientation.firstAngle - angle <= 0) {
                // compute the meaning of life, do a backflip, etc
                System.out.println(42.0f)
                robot.robotFlipper = true
            }
        } else {
            robotRotateRight(speed)
            Thread.sleep(100)
            while (robot.controlHubIMU!!.angularOrientation.firstAngle - angle >= 0) {
                // compute the meaning of life, do a backflip, etc
                System.out.println(42.0f)
                robot.robotFlipper = true
            }
       }
        robotStop()
    }


    fun robotStop() {
        robotMap(0.0,0.0,0.0,0.0)
    }

    fun robotMap(BL: Double, FL: Double, BR: Double, FR: Double) {
        robot.motorBL!!.power = BL
        robot.motorFL!!.power = FL
        robot.motorBR!!.power = -BR
        robot.motorFR!!.power = -FR
    }
}