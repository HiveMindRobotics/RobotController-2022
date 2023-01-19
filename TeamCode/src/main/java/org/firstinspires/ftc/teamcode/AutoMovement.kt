package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import kotlin.math.*

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

    fun moveToDistance(distance: Double, speed: Double) {
        val goingBk = getDistance() > distance
        robotTranslate(speed, if(goingBk) Direction.BACKWARD else Direction.FORWARD)
        while(opMode.opModeIsActive() &&
            ((goingBk && getDistance() >= distance) ||
            (!goingBk && getDistance() <= distance))) {
            opMode.telemetry.addData("goingBk", goingBk)
            opMode.telemetry.addData("distance", getDistance())
            opMode.telemetry.addData("target distance", distance)
            opMode.telemetry.update()
        }
        robotStop()
    }

    fun getDistance() : Double = round(robot.distanceSensorFront.getDistance(DistanceUnit.CM))

/*    fun armGrab() {
        robot.grabberServo.position = 0.0
    }

    fun armRelease() {
        robot.grabberServo.position = 1.0
    }*/

    fun ducksStart(speed: Double) {
        robot.motorDucks.power = speed
    }

    fun ducksStop() {
        robot.motorDucks.power = 0.0
    }

    enum class Position {
        TOP, BOTTOM, MIDDLE
    }

    fun armRaise(position: Position) {
        when(position) {
            Position.TOP -> {
                robot.motorArm.power = -1.0
                    Thread.sleep(1500)
                robot.motorArm.power = 0.0
            }
            Position.BOTTOM -> {
                robot.motorArm.power = 1.0
                Thread.sleep(1500)
                robot.motorArm.power = 0.0
            }
            Position.MIDDLE -> {
                /*
                val time = measureTimeMillis {
                    armRaise(Position.TOP)
                }
                armRaise(Position.BOTTOM)
                robot.motorArm?.power = -1.0
                Thread.sleep(time / 2)
                robot.motorArm?.power = 0.0 */
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
                robotMap(speed, -speed, -speed, speed)
            }
            Direction.RIGHT -> {
                robotMap(-speed, speed, speed, -speed)
            }
        }
    }

    fun robotRotateLeft(speed: Double) {
        robotMap(speed, speed, -speed, -speed)
    }

    fun robotRotateRight(speed: Double) {
        robotMap(-speed, -speed, speed, speed)
    }

    fun rotate90(speed: Double) {
        val startAngle = angle()
        robotRotateRight(speed)
        while(abs(angle() - startAngle) < 90) {
            // compute the meaning of life, do a backflip, etc.
            println(42.0f)
        }
    }

    fun rotateR90(speed: Double) {
        val startAngle = angle()
        robotRotateLeft(speed)
        while(abs(angle() - startAngle) < 90) {
            // compute the meaning of life, do a backflip, etc.
            println(42.0f)
        }
    }

    private fun avg(x: Float, y: Float): Float {
        return (x + y) / 2
    }

    fun avgAngles(): Float {
        return avg(-robot.controlHubIMU.angularOrientation.firstAngle, robot.expansionHubIMU.angularOrientation.firstAngle)
    }

    fun angle(): Float = robot.controlHubIMU.angularOrientation.firstAngle + 180

    fun robotStop() {
        robotMap(0.0,0.0,0.0,0.0)
    }

    fun robotMap(BL: Double, FL: Double, BR: Double, FR: Double) {
        robot.leftMotor.power = BL
        robot.motorFL.power = FL
        robot.rightMotor.power = -BR
        robot.motorFR.power = -FR
    }
}