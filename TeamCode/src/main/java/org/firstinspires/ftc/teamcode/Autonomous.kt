package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DistanceSensor
import kotlinx.coroutines.runBlocking
import kotlinx.coroutines.launch


@Autonomous(name = "Autonomous", group = "Linear Opmode")
class Autonomous : LinearOpMode() {
    var robot = Hardware()
    var auto = AutoMovement(robot, this)
    var vuforia = Vuforia(robot, this)
    var autoMovement = auto.AutonomousAutoMovement(vuforia)
    var openCV = OpenCV(robot, this)

    override fun runOpMode() {
        runBlocking {
            // TODO(scan ducks)
            openCV.init()
            waitForStart()
            launch {
                telemetry.addData("location", vuforia.lastLocation)
                telemetry.update()
                while (opModeIsActive()) {
                    // solve world hunger, cure cancer, etc.
                    robot.cancerCured = true
                    robot.worldHunger = false
                }
            }
        }
    }
}