package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sin

class AutoMovement(var opMode: Autonomous)  {
    private val robot = opMode.robot

    private fun getDistance(): Double {
        var velocity = robot.controlHubIMU!!.velocity.toUnit(DistanceUnit.METER)
        opMode.speed = Math.sqrt(velocity.xVeloc.pow(2) + velocity.yVeloc.pow(2) + velocity.zVeloc.pow((2)))
        opMode.distance += (opMode.speed * opMode.time)
        return opMode.distance
    }

    fun armGrab() {
        robot.servoArm!!.position = 1.0
    }

    fun armRelease() {
        robot.servoArm!!.position = 0.0
    }

    fun armRaise() {
        robot.motorArm!!.power = -1.0
        Thread.sleep(1500)
        armStop()

    }

    fun armLower() {
        robot.motorArm!!.power = 1.0
        Thread.sleep(1500)
        armStop()
    }

    fun ducksStart() {
        robot.motorDucks!!.power = 1.0
    }

    fun ducksStop() {
        robot.motorDucks!!.power = 0.0
    }

    private fun armStop() {
        robot.motorArm!!.power = 0.0
    }

    fun robotDriveForward(speed: Double) {
        robot.motorBL!!.power = speed
        robot.motorFL!!.power = speed
        robot.motorBR!!.power = speed
        robot.motorFR!!.power = speed
    }

    fun robotDriveBackward(speed: Double) {
        robot.motorBL!!.power = -speed
        robot.motorFL!!.power = -speed
        robot.motorBR!!.power = -speed
        robot.motorFR!!.power = -speed
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

    fun robotTranslateLeft(speed: Double) {
        robot.motorBL!!.power = speed
        robot.motorFL!!.power = -speed
        robot.motorBR!!.power = speed
        robot.motorFR!!.power = -speed
    }

    fun robotTranslateRight(speed: Double) {
        robot.motorBL!!.power = -speed
        robot.motorFL!!.power = speed
        robot.motorBR!!.power = -speed
        robot.motorFR!!.power = speed
    }

    fun robotStop() {
        robot.motorBL!!.power = 0.0
        robot.motorFL!!.power = 0.0
        robot.motorBR!!.power = 0.0
        robot.motorFR!!.power = 0.0
    }

    fun moveDistance(angle: Double, speed: Double, distance: Double) {
        var distance0 = opMode.distance
        while ((getDistance() - distance0) < distance) {
            robot.motorBL?.power = -(speed * sin(angle))
            robot.motorFL?.power = -(speed * cos(angle))
            robot.motorFR?.power = speed * sin(angle)
            robot.motorBR?.power = speed * cos(angle)
        }
    }

}