package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Position
import org.firstinspires.ftc.robotcore.external.navigation.Velocity
import kotlin.math.*

@TeleOp(name = "Driver Control", group = "Linear Opmode")
class DriverControl : LinearOpMode() {

    override fun runOpMode() {
        val robot = Hardware()
        robot.init(hardwareMap)

        robot.controlHubIMU?.startAccelerationIntegration(
            Position(DistanceUnit.MM, 0.0, 0.0, 0.0, 0),
            Velocity(DistanceUnit.MM, 0.0, 0.0, 0.0, 0),
            250)

        var r: Double
        var robotAngle: Double
        var rightX: Double
        var armGrabbed = false
        var armWillGrab: Boolean

        waitForStart()

        while (opModeIsActive()) {

            r = hypot(-gamepad1.left_stick_x.toDouble(), gamepad1.left_stick_y.toDouble())
            robotAngle = atan2(gamepad1.left_stick_y.toDouble(), -gamepad1.left_stick_x.toDouble()) - Math.PI / 4
            rightX = gamepad1.right_stick_x.toDouble()
            robot.motorFL?.power = -(r * cos(robotAngle) - rightX)
            robot.motorBL?.power = -(r * sin(robotAngle) - rightX)
            robot.motorFR?.power = r * sin(robotAngle) + rightX
            robot.motorBR?.power = r * cos(robotAngle) + rightX

            robot.motorDucks?.power = -gamepad2.right_stick_x.toDouble()
            robot.motorArm?.power = gamepad2.left_stick_y.toDouble()

            robot.servoArm?.position = if (gamepad2.right_trigger > 0) 0.0 else 1.0

            telemetry.addData("position:", robot.controlHubIMU?.position)
            telemetry.addData("acceleration:", robot.controlHubIMU?.velocity)
            telemetry.addData("direction:", robot.controlHubIMU?.angularOrientation)
            telemetry.update()

        }
    }
}
