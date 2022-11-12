package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Gamepad
import kotlin.math.*
import kotlin.system.measureTimeMillis

@TeleOp(name = "Driver Control", group = "Linear Opmode")
class DriverControl : LinearOpMode() {
    companion object {
        const val DEADZONE = 0.1
        const val MAXSPEED = 1.0
        const val MAXTURNSPEED = 1.0 // Want more precise turning but faster forwards/backwards movement
        // 1 is temporary - just to test
    }

    private fun easeFun(x: Double): Double =
        if (x > 0) (0.857 * x + 0.1).pow(1.58) else -((0.857 * -x + 0.1).pow(1.58))

    private fun anyDpad(gamepad: Gamepad): Boolean =
        gamepad.dpad_down || gamepad.dpad_left /* || gamepad.dpad_up */ || gamepad.dpad_right

    override fun runOpMode() {
        // hardwareMap is null until runOpMode() is called
        val robot = Hardware(hardwareMap)
        val odometry = Odometry()

        val prevGamepad1 = Gamepad()
        val prevGamepad2 = Gamepad()

        var state = DriverControlState.Normal

        waitForStart()

        var targetTurnSpeed: Double
        while (opModeIsActive()) {
            val elapsed = measureTimeMillis {

                robot.resetCache()

                when (state) {
                    DriverControlState.Normal -> {
                        robot.leftMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                        robot.rightMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

                        // Sensitivity clutch with B
                        targetTurnSpeed = if (gamepad1.b) MAXTURNSPEED / 2 else MAXTURNSPEED

                        // Drive with triggers
                        val power = gamepad1.right_trigger - gamepad1.left_trigger // Gives a "braking" effect
                        if (abs(power) > DEADZONE) {
                            val speed = power.toDouble() * if(!gamepad1.b) MAXSPEED else MAXSPEED / 2
                            robot.leftMotor.power = easeFun(speed)
                            robot.rightMotor.power = easeFun(speed)
                        } else {
                            robot.leftMotor.power = 0.0
                            robot.rightMotor.power = 0.0
                        }

                        if (abs(gamepad1.left_stick_x) > DEADZONE) {
                            val speed = gamepad1.left_stick_x.toDouble() * targetTurnSpeed
                            robot.leftMotor.power -= easeFun(
                                speed
                            ) // SQRT works best for turning while moving
                            robot.rightMotor.power -= easeFun(-speed)
                        }

                        // Per Ben H's request, turn with bumpers
                        if (gamepad1.left_bumper) robot.leftMotor.power = targetTurnSpeed / 2
                        if (gamepad1.right_bumper) robot.rightMotor.power = targetTurnSpeed / 2

                        // linear slide
                        robot.motorLinearSlide.power = gamepad1.right_stick_y.toDouble()

                        // Grabber - X to close, Y to open
                        if(gamepad1.x)
                            robot.grabberServo.position = 0.0
                        else
                            robot.grabberServo.position = 0.3

                        // DRY - Don't Repeat Yourself   -- sky
                        if(anyDpad(gamepad1) && !anyDpad(prevGamepad1)) {
                            robot.leftMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
                            robot.rightMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
                            robot.leftMotor.power = 0.3
                            robot.rightMotor.power = 0.3
                            state = DriverControlState.SnapTurning

                            // Snap turning with D-Pad
                            if (gamepad1.dpad_down && !prevGamepad1.dpad_down) {
                                robot.leftMotor.targetPosition = robot.leftMotor.currentPosition + (Odometry.TICKS_PER_TURN / 2).toInt()
                                robot.rightMotor.targetPosition = robot.rightMotor.currentPosition - (Odometry.TICKS_PER_TURN / 2).toInt()
                            } else if (gamepad1.dpad_left && !prevGamepad1.dpad_left) {
                                robot.leftMotor.targetPosition = robot.leftMotor.currentPosition - (Odometry.TICKS_PER_TURN / 4).toInt()
                                robot.rightMotor.targetPosition = robot.rightMotor.currentPosition + (Odometry.TICKS_PER_TURN / 4).toInt()
                            } else if (gamepad1.dpad_right && !prevGamepad1.dpad_right) {
                                robot.leftMotor.targetPosition = robot.leftMotor.currentPosition + (Odometry.TICKS_PER_TURN / 4).toInt()
                                robot.rightMotor.targetPosition = robot.rightMotor.currentPosition - (Odometry.TICKS_PER_TURN / 4).toInt()
                            }
                        }
                    }

                    DriverControlState.SnapTurning -> {
                        if (!robot.leftMotor.isBusy && !robot.rightMotor.isBusy) {
                            state = DriverControlState.Normal
                        } else if (gamepad1.dpad_down && !prevGamepad1.dpad_down) {
                            robot.leftMotor.targetPosition = robot.leftMotor.currentPosition
                            robot.rightMotor.targetPosition = robot.leftMotor.currentPosition
                            state = DriverControlState.Normal
                        }
                    }
                }

                // Update previous gamepad state
                prevGamepad1.copy(gamepad1)
                prevGamepad2.copy(gamepad2)

                odometry.update(
                    robot.leftMotor.currentPosition,
                    robot.rightMotor.currentPosition,
                    robot.controlHubIMU.angularOrientation.firstAngle
                )


                //DEBUG: Log movement
                telemetry.addLine("Motor Position (BL): ${robot.leftMotor.currentPosition.toFloat() * Odometry.ROTATIONS_PER_TICK}")
                telemetry.addLine("Motor Position (BR): ${robot.rightMotor.currentPosition.toFloat() * Odometry.ROTATIONS_PER_TICK}")
                telemetry.addLine("Target Position (BL): ${robot.leftMotor.targetPosition.toFloat()}")
                telemetry.addLine("Speed (BL): ${robot.leftMotor.power}")
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
