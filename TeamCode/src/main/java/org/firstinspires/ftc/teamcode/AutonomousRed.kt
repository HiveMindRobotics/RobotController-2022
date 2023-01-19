package org.firstinspires.ftc.teamcode

import com.qualcomm.ftccommon.SoundPlayer
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit


@Autonomous(name = "Autonomous Red", group = "Linear Opmode")
class AutonomousRed : LinearOpMode() {
    var robot = Hardware(hardwareMap)
    var auto = AutoMovement(robot, this)

    override fun runOpMode() {

/*
        auto.armGrab()
*/

        waitForStart()
        /*
        Thread.sleep(10000)

        auto.armRaise(AutoMovement.Position.TOP)
        auto.moveToDistance(40.0, 0.2)
        auto.armRelease()
        auto.moveToDistance(5.0, 0.2)
        auto.robotStop() */
        auto.armRaise(AutoMovement.Position.TOP)
        auto.robotTranslate(0.6, AutoMovement.Direction.RIGHT)
        while (!isStopRequested) {
        }
    }
}