package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import kotlinx.coroutines.*


@Autonomous(name = "Autonomous", group = "Linear Opmode")
class Autonomous : LinearOpMode() {
    var robot = Hardware()
    var auto = AutoMovement(robot)
    var vuforia = Vuforia(robot, this)
    var autoMovement = auto.AutonomousAutoMovement(vuforia)
    // var distance1 = distance
    // moveAndStuff()
    // var distance2 = distance
    // var movedDistance = distance2 - distance1

    override fun runOpMode() {
        waitForStart()
        runBlocking {
            launch {
                while (opModeIsActive()) {
                    vuforia.getPosition()
                }
            }
            while (opModeIsActive()) {
                telemetry.addData("location", vuforia.lastLocation)
                telemetry.update()
            }
        }
    }
}