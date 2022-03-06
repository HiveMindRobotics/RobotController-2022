package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import kotlin.math.*


@Autonomous(name = "Autonomous", group = "Linear Opmode")
class Autonomous : LinearOpMode() {
    var autoMovement = AutoMovement(this)
    var speed = 0.0
    var distance = 0.0 // a delta distance, don't use standalone
    // var distance1 = distance
    // moveAndStuff()
    // var distance2 = distance
    // var movedDistance = distance2 - distance1
    var robot = Hardware()

    override fun runOpMode() {
        waitForStart()
        while (opModeIsActive()) {
            val velocity = robot.controlHubIMU!!.velocity.toUnit(DistanceUnit.METER)
            speed = sqrt(velocity.xVeloc.pow(2) + velocity.yVeloc.pow(2) + velocity.zVeloc.pow((2)))
            distance += (speed * time)
            telemetry.addData("distance", distance)
            telemetry.update()
        }
    }
}