package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
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
        val vuforia = Vuforia(robot, this)
        val autoVuforia = auto.AutonomousAutoMovement(vuforia)

        vuforia.init()

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
            var xLStick = gamepad1.left_stick_x.toDouble()
            var yLStick = gamepad1.left_stick_y.toDouble()
            /*xLStick = if(xLStick >= 0)
                sqrt(xLStick)
            else
                -1 * sqrt(-1 * xLStick)
            yLStick = if(yLStick >= 0)
                sqrt(xLStick)
            else
                -1 * sqrt(-1 * xLStick)*/
            r = hypot(-xLStick, yLStick)
            robotAngle = atan2(yLStick, -xLStick) - Math.PI / 4
            rightX = gamepad1.right_stick_x.toDouble()
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

            robot.servoArm?.position = if (gamepad2.right_trigger > 0) 0.0 else 1.0

            if (gamepad1.dpad_right) {
                auto.robotRotateToAngle(.5, 90.0)
            } else if (gamepad1.dpad_left) {
                auto.robotRotateToAngle(.5, 45.0)
            } else if (gamepad1.dpad_up) {
                autoVuforia.moveDistance(0.0, .5, 5.0 * 25.4)
            } else if (gamepad1.dpad_down) {
                autoVuforia.moveToCoords(0.0, 0.0, .5)
            }

            // vuforia.getPosition()
        }
    }
}
