package org.firstinspires.ftc.teamcode

import com.qualcomm.ftccommon.SoundPlayer
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import kotlin.concurrent.thread


@Autonomous(name = "Autonomous Blue", group = "Linear Opmode")
class AutonomousBlue : LinearOpMode() {
    var robot = Hardware(hardwareMap)
    var auto = AutoMovement(robot, this)

    override fun runOpMode() {
/*
        robot.init(hardwareMap)
        auto.armGrab()

        waitForStart()
        Thread.sleep(10000)

        auto.armRaise(AutoMovement.Position.TOP)
        auto.moveToDistance(40.0, 0.2)
        auto.armRelease()
        auto.moveToDistance(5.0, 0.2)
        */
/*
        auto.armGrab()
*/

        waitForStart()
        /*auto.ducksStart(-1.0)
        auto.robotTranslate(0.4, AutoMovement.Direction.RIGHT)
        telemetry.addData("right:", robot.distanceSensorRight!!.getDistance(DistanceUnit.CM))
        telemetry.update()
        while (robot.distanceSensorRight!!.getDistance(DistanceUnit.CM) > 7 && !isStopRequested) {

        }
        // auto.robotStop()*/
        /*
        auto.robotTranslate(0.6, AutoMovement.Direction.LEFT)
        while (!isStopRequested) {
            telemetry.addData("left:", robot.distanceSensorRight!!.getDistance(DistanceUnit.CM))
            telemetry.update()
        } */
        auto.armRaise(AutoMovement.Position.TOP)
        auto.robotTranslate(0.6, AutoMovement.Direction.LEFT)
        while (!isStopRequested) {
        }
    }
}