package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.Gamepad
import kotlin.math.*
import kotlin.system.measureTimeMillis
import kotlin.time.Duration.Companion.milliseconds

@TeleOp(name = "Driver Control", group = "Linear Opmode")
class DriverControl : LinearOpMode() {
    companion object {
        const val DEADZONE = 0.1
        const val MAXSPEED = 1.0
        const val MAXTURNSPEED = 1.0 // Want more precise turning but faster forwards/backwards movement
        // 1 is temporary - just to test
    }

    private fun easeFun(x: Double, mode: EaseMode): Double {
        return when (mode) {
            EaseMode.SQRT -> sqrt(x * sign(x)) * sign(x)
            EaseMode.EXP -> x.pow(2) * sign(x)
            // Looks weird, but it feels really nice while driving. Magic numbers calculated with a calculator
            EaseMode.LOG -> if (x > 0) (0.857 * x + 0.1).pow(1.58) else -((0.857 * -x + 0.1).pow(1.58))
            EaseMode.NONE -> x
        }
    }

    override fun runOpMode() {
        // hardwareMap is null until runOpMode() is called
        val robot = Hardware(hardwareMap)
        val odometry = Odometry()

        var prevGamepad1 = Gamepad()
        var prevGamepad2 = Gamepad()


        waitForStart()

        var targetTurnSpeed = MAXTURNSPEED

        while (opModeIsActive()) {
            val elapsed = measureTimeMillis {
                for (hub in robot.allHubs) {
                    hub.clearBulkCache()
                }

                // Drive with triggers
                val power = gamepad1.right_trigger - gamepad1.left_trigger // Gives a "braking" effect
                if (abs(power) > DEADZONE) {
                    val speed = power.toDouble() * MAXSPEED
                    robot.motorBL.power = easeFun(speed, EaseMode.LOG)
                    robot.motorBR.power = easeFun(speed, EaseMode.LOG)
                } else {
                    robot.motorBL.power = 0.0
                    robot.motorBR.power = 0.0
                }

                if (abs(gamepad1.left_stick_x) > DEADZONE) {
                    val speed = gamepad1.left_stick_x.toDouble() * targetTurnSpeed
                    robot.motorBL.power -= easeFun(speed, EaseMode.LOG) // SQRT works best for turning while moving
                    robot.motorBR.power -= easeFun(-speed, EaseMode.LOG)
                }

                // Per Ben H's request, turn with bumpers
                if (gamepad1.left_bumper) robot.motorBL.power = targetTurnSpeed / 2
                if (gamepad1.right_bumper) robot.motorBR.power = targetTurnSpeed / 2

                // Sensitivity clutch with B
                targetTurnSpeed = if (gamepad1.b) MAXTURNSPEED / 2 else MAXTURNSPEED

                // Snap turning with D-Pad
                if (gamepad1.dpad_down && !prevGamepad1.dpad_down) {
                    robot.motorBL.targetPosition += (Odometry.TICKS_PER_TURN / 2).toInt()
                    robot.motorBR.targetPosition -= (Odometry.TICKS_PER_TURN / 2).toInt()
                } else if (gamepad1.dpad_left && !prevGamepad1.dpad_left) {
                    robot.motorBL.targetPosition -= (Odometry.TICKS_PER_TURN / 4).toInt()
                    robot.motorBR.targetPosition += (Odometry.TICKS_PER_TURN / 4).toInt()
                } else if (gamepad1.dpad_right && !prevGamepad1.dpad_right) {
                    robot.motorBL.targetPosition += (Odometry.TICKS_PER_TURN / 4).toInt()
                    robot.motorBR.targetPosition -= (Odometry.TICKS_PER_TURN / 4).toInt()
                }

                odometry.update(
                    robot.motorBL.currentPosition,
                    robot.motorBR.currentPosition,
                    robot.controlHubIMU.angularOrientation.firstAngle
                )

                // Update previous gamepad state
                prevGamepad1.copy(gamepad1)
                prevGamepad2.copy(gamepad2)

                //DEBUG: Log movement
                telemetry.addLine("Motor Position (BL): ${robot.motorBL.currentPosition.toFloat() * Odometry.ROTATIONS_PER_TICK}")
                telemetry.addLine("Motor Position (BR): ${robot.motorBR.currentPosition.toFloat() * Odometry.ROTATIONS_PER_TICK}")
                telemetry.addLine("Motor Position (Slide): ${robot.motorLinearSlide.currentPosition}")
                telemetry.addLine("Robot Yaw: ${robot.controlHubIMU.angularOrientation.firstAngle}")
                telemetry.addLine("Pos: ${odometry.x}, ${odometry.y}")
                telemetry.addLine("left stick: ${gamepad1.left_stick_x}")
                telemetry.addLine("Ltrigger, Rtrigger = ${gamepad1.left_trigger}, ${gamepad1.right_trigger}")
            }
            telemetry.addLine("loop time $elapsed")
            telemetry.update()
        }
    }
}
