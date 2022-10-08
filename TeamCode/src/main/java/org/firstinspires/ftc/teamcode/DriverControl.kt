package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import kotlin.math.*

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
        }
    }

    override fun runOpMode() {
        val robot = Hardware(hardwareMap)

        waitForStart()

        while (opModeIsActive()) {
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

            // Turning
            if(-gamepad1.left_trigger + gamepad1.right_trigger != 0.0f) {
                // Already moving, modify that
                if (abs(gamepad1.left_stick_x) > DEADZONE) {
                    val speed = gamepad1.left_stick_x.toDouble() * MAXTURNSPEED
                    robot.motorBL.power -= easeFun(speed, EaseMode.EXP) // EXP works best for turning while moving
                    robot.motorBR.power -= easeFun(-speed, EaseMode.EXP)
                }
            } else {
                // Not already moving, spin normally
                if (abs(gamepad1.left_stick_x) > DEADZONE) {
                    val speed = gamepad1.left_stick_x.toDouble() * MAXTURNSPEED
                    robot.motorBL.power = easeFun(-speed, EaseMode.EXP) // SQRT works best for turning while moving
                    robot.motorBR.power = easeFun(speed, EaseMode.EXP)
                }
                else {
                    robot.motorBL.power = 0.0
                    robot.motorBR.power = 0.0
                }
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

            //DEBUG: Log movement
            telemetry.addLine("easeMode: $easeMode")
            telemetry.addLine("Motor Position: ${robot.motorLinearSlide.currentPosition}")
            telemetry.addLine("left stick: ${gamepad1.left_stick_x}")
            telemetry.addLine("Ltrigger, Rtrigger = ${gamepad1.left_trigger}, ${gamepad1.right_trigger}")
            telemetry.update()
        }
    }
}
