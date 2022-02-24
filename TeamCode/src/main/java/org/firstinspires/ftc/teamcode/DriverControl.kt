package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import kotlin.math.*

@TeleOp(name = "Driver Control", group = "Linear Opmode")
class DriverControl : LinearOpMode() {

    private var robot = Hardware()

    override fun runOpMode() {
        robot.init(hardwareMap)

        waitForStart()

        while (opModeIsActive()) {

            val r = hypot(-gamepad1.left_stick_x.toDouble(), gamepad1.left_stick_y.toDouble())
            val robotAngle = atan2(gamepad1.left_stick_y.toDouble(), -gamepad1.left_stick_x.toDouble()) - Math.PI / 4
            val rightX = gamepad1.right_stick_x.toDouble()
            robot.motorFL?.power = -(r * cos(robotAngle) - rightX)
            robot.motorBL?.power = -(r * sin(robotAngle) - rightX)
            robot.motorFR?.power = r * sin(robotAngle) + rightX
            robot.motorBR?.power = r * cos(robotAngle) + rightX

            robot.motorDucks?.power = -gamepad2.right_stick_x.toDouble()
            robot.motorArm?.power = -gamepad2.left_stick_y.toDouble()
            robot.servoArm?.position = (if (gamepad2.right_trigger > 0) 1.0 else 0.0)

        }
    }
}
