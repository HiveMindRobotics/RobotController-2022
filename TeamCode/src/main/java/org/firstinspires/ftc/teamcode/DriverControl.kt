package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import kotlin.math.*
import kotlin.system.measureTimeMillis
import kotlin.time.Duration.Companion.milliseconds

@TeleOp(name = "Driver Control", group = "Linear Opmode")
class DriverControl : LinearOpMode() {
    private var easeMode: EaseMode = EaseMode.LOG
    companion object {
        const val DEADZONE = 0.1
        const val MAXSPEED = 1
        const val MAXTURNSPEED = 1 // Want more precise turning but faster forwards/backwards movement
        // 1 is temporary - just to test
    }

    private fun easeFun(x: Double, mode: EaseMode): Double {
        return when (mode) {
            EaseMode.SQRT -> sqrt(x * sign(x)) * sign(x)
            EaseMode.EXP -> x.pow(2) * sign(x)
            // Looks weird, but it feels really nice while driving. Magic numbers calculated with a calculator
            EaseMode.LOG -> if(x > 0) (0.857 * x + 0.1).pow(1.58) else -((0.857 * -x + 0.1).pow(1.58))
            EaseMode.NONE -> x
        }
    }

    override fun runOpMode() {
        // hardwareMap is null until runOpMode() is called
        val robot = Hardware(hardwareMap)
        // odometry should init here so it's reset every time
        val odometry = Odometry()

        waitForStart()

        while (opModeIsActive()) {
            val elapsed = measureTimeMillis {
                for (hub in robot.allHubs) {
                    hub.clearBulkCache()
                }

                // Forwards / Backwards

                // Joystick mode
                /* if (abs(gamepad1.right_stick_y) > DEADZONE) {
                val speed = gamepad1.right_stick_y.toDouble() * MAXSPEED
                robot.motorBL.power = easeFun(speed, easeMode)
                robot.motorBR.power = easeFun(speed, easeMode)
            }
            else {
                robot.motorBL.power = 0.0
                robot.motorBR.power = 0.0
            } */

                // Trigger mode
                val power = -gamepad1.left_trigger + gamepad1.right_trigger // Gives a "braking" effect"
                if (abs(power) > DEADZONE) {
                    val speed = power.toDouble() * MAXSPEED
                    robot.motorBL.power = easeFun(speed, easeMode)
                    robot.motorBR.power = easeFun(speed, easeMode)
                } else {
                    robot.motorBL.power = 0.0
                    robot.motorBR.power = 0.0
                }

                if (abs(gamepad1.left_stick_x) > DEADZONE) {
                    val speed = gamepad1.left_stick_x.toDouble() * MAXTURNSPEED
                    robot.motorBL.power -= easeFun(speed, EaseMode.EXP) // SQRT works best for turning while moving
                    robot.motorBR.power -= easeFun(-speed, EaseMode.EXP)
                }

                // Linear slide motor
                robot.motorLinearSlide.power = if (gamepad1.left_trigger.toDouble() > 0) {
                    gamepad1.left_trigger.toDouble()
                } else {
                    -gamepad1.right_trigger.toDouble()
                }

                // Commented out because it seems we don't need this anymore?
                //DEBUG: Cycle through easing functions
                /*val easeVals = enumValues<EaseMode>()
            easeMode = when {
                gamepad1.right_bumper -> easeVals[Math.floorMod((easeMode.ordinal + 1), easeVals.size)]
                gamepad1.left_bumper -> easeVals[Math.floorMod((easeMode.ordinal - 1), easeVals.size)]
                else -> easeMode
            }*/

                odometry.update(
                    robot.motorBL.currentPosition,
                    robot.motorBR.currentPosition,
                    robot.controlHubIMU.angularOrientation.firstAngle
                )

                //DEBUG: Log movement
                telemetry.addLine("easeMode: $easeMode")
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
