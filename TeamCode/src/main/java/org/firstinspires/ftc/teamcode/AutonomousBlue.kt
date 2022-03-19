package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit


@Autonomous(name = "Autonomous Blue", group = "Linear Opmode")
class AutonomousBlue : LinearOpMode() {
    var robot = Hardware()
    var auto = AutoMovement(robot, this)

    override fun runOpMode() {
        robot.init(hardwareMap)
        auto.armGrab()

        waitForStart()

        auto.armRaise(AutoMovement.Position.TOP)
        auto.moveToDistance(55.0, 0.2)
        auto.armRelease()
        auto.moveToDistance(25.0, 0.2)
        auto.ducksStart(-1.0)
        auto.robotTranslate(0.4, AutoMovement.Direction.RIGHT)
        telemetry.addData("right:", robot.distanceSensorRight!!.getDistance(DistanceUnit.CM))
        telemetry.update()
        while (robot.distanceSensorRight!!.getDistance(DistanceUnit.CM) > 7 && !isStopRequested) {

        }
        // auto.robotStop()
        while (!isStopRequested) {
            telemetry.addData("left:", robot.distanceSensorRight!!.getDistance(DistanceUnit.CM))
            telemetry.update()
        }
    }
}