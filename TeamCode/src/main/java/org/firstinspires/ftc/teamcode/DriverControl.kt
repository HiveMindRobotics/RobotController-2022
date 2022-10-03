package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import kotlin.math.*

@TeleOp(name = "Driver Control", group = "Linear Opmode")
class DriverControl : LinearOpMode() {
    private var easeMode: EaseMode = EaseMode.EXP

    companion object {
        const val DEADZONE = 0.5
        const val MAXSPEED = 1.0
    }

    private fun easeFun(x: Double, mode: EaseMode): Double {
        return when (mode) {
            EaseMode.SQRT -> sqrt(x * sign(x)) * sign(x)
            EaseMode.EXP -> x.pow(2) * sign(x)
            EaseMode.NONE -> x
        }
    }

    private var odometry = Odometry()

    override fun runOpMode() {
        val robot = Hardware(hardwareMap)

        waitForStart()

        while (opModeIsActive()) {

            // Forwards / Backwards
            if (abs(gamepad1.right_stick_y) > DEADZONE) {
                val speed = gamepad1.right_stick_y.toDouble() * MAXSPEED
                robot.motorBL.power = easeFun(speed, easeMode)
                robot.motorBR.power = easeFun(speed, easeMode)
            }
            else {
                robot.motorBL.power = 0.0
                robot.motorBR.power = 0.0
            }

            // Turning
            if (abs(gamepad1.left_stick_x) > DEADZONE) {
                val speed = gamepad1.left_stick_x.toDouble() * MAXSPEED
                robot.motorBL.power = easeFun(speed, easeMode)
                robot.motorBR.power = easeFun(-speed, easeMode)
            }
            else {
                robot.motorBL.power = 0.0
                robot.motorBR.power = 0.0
            }

            // Linear slide motor
            robot.motorLinearSlide.power = if (gamepad1.left_trigger.toDouble() > 0) {
                gamepad1.left_trigger.toDouble()
            } else {
                -gamepad1.right_trigger.toDouble()
            }

            //DEBUG: Cycle through easing functions
            val easeVals = enumValues<EaseMode>()
            easeMode = when {
                gamepad1.right_bumper -> easeVals[Math.floorMod((easeMode.ordinal + 1), easeVals.size)]
                gamepad1.left_bumper -> easeVals[Math.floorMod((easeMode.ordinal - 1), easeVals.size)]
                else -> easeMode
            }

            odometry.update(
                robot.motorBL.currentPosition,
                robot.motorBR.currentPosition,
                robot.controlHubIMU.angularOrientation.firstAngle
            );

            //DEBUG: Log movement
            telemetry.addLine("easeMode: $easeMode")
            telemetry.addLine("Motor Position (BL): ${robot.motorBL.currentPosition.toFloat() / (28 * 3)}")
            telemetry.addLine("Motor Position (BR): ${robot.motorBR.currentPosition.toFloat() / (28 * 3)}")
            telemetry.addLine("Robot Yaw: ${robot.controlHubIMU.angularOrientation.firstAngle}")
            telemetry.addLine("Pos: ${odometry.x}, ${odometry.y}")
            telemetry.update()
        }
    }
}
