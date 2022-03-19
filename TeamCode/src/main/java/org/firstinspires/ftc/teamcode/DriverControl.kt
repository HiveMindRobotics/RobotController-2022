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
        val auto = AutoMovement(robot, this)

        robot.controlHubIMU?.startAccelerationIntegration(
            Position(DistanceUnit.MM, 0.0, 0.0, 0.0, 0),
            Velocity(DistanceUnit.MM, 0.0, 0.0, 0.0, 0),
            250)

        var r: Double
        var robotAngle: Double
        var rightX: Double

        waitForStart()

        fun nSqrt(v: Double) : Double {
            return -1 * sqrt(-1 * sqrt(v))
        }

        var bl: Double
        var fl: Double
        var br: Double
        var fr: Double

        while (opModeIsActive()) {
            var xLStick = -gamepad1.left_stick_x.toDouble()
            var yLStick = -gamepad1.left_stick_y.toDouble()
            xLStick =
                if(xLStick < 0)
                    -sqrt(-xLStick)
                else
                    sqrt(xLStick)
            yLStick =
                if(yLStick < 0)
                    -sqrt(-yLStick)
                else
                    sqrt(yLStick)
            r = hypot(-xLStick, yLStick)
            robotAngle = atan2(yLStick, -xLStick) - Math.PI / 4
            rightX = 0.5 * gamepad1.right_stick_x.toDouble()
            bl = r * sin(robotAngle) + rightX
            fl = r * cos(robotAngle) + rightX
            br = -(r * cos(robotAngle) - rightX)
            fr = -(r * sin(robotAngle) - rightX)

            var arr = mapOf(
                Pair(robot.motorBL!!, bl),
                Pair(robot.motorFL!!, fl),
                Pair(robot.motorBR!!, br),
                Pair(robot.motorFR!!, fr)
            )

            for(i in arr) {
                i.key.power = i.value
            }

            robot.motorDucks?.power = -gamepad2.right_stick_x.toDouble()
            robot.motorArm?.power = gamepad2.left_stick_y.toDouble()

            robot.servoArm?.position = if (gamepad2.right_trigger > 0) 1.0 else 0.0

            /*
            if (gamepad1.dpad_right) {
                auto.robotTranslate(0.2, AutoMovement.Direction.RIGHT)
            } else if (gamepad1.dpad_left) {
                auto.robotTranslate(0.2, AutoMovement.Direction.LEFT)
            } else if (gamepad1.dpad_up) {
                auto.robotTranslate(0.2, AutoMovement.Direction.FORWARD)
            } else if (gamepad1.dpad_down) {
                auto.robotTranslate(0.2, AutoMovement.Direction.BACKWARD)
            } */
/*
            telemetry.addData("angle:", robot.controlHubIMU!!.angularOrientation.firstAngle)
            telemetry.addData("angle2", robot.controlHubIMU!!.angularOrientation.secondAngle)
            telemetry.addData("angle3", robot.controlHubIMU!!.angularOrientation.thirdAngle)
            telemetry.addData("front:", auto.getDistance())
            telemetry.update() */
            //vuforia.getPosition()
        }
    }
}
