package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Gamepad
import kotlin.math.*
import kotlin.system.measureTimeMillis

@TeleOp(name = "Driver Control", group = "Linear Opmode")
class DriverControl : LinearOpMode() {
    @Config
    object M {
        @JvmField var DEADZONE = 0.1
        @JvmField var MAXSPEED = 0.8
        @JvmField var MAXTURNSPEED = 0.8 // Want more precise turning but faster forwards/backwards movement
        @JvmField var ROTATEAMOUNT = 0.2
        @JvmField var MAXHEIGHT = 2117
        @JvmField var MAXSLIDESPEED = 0.5
        // 1 is temporary - just to test
    }

    private fun easeFun(x: Double): Double =
        if (x > 0) (0.857 * x + 0.1).pow(1.58) else -((0.857 * -x + 0.1).pow(1.58))

    private fun anyDpad(gamepad: Gamepad): Boolean =
        gamepad.dpad_down || gamepad.dpad_left /* || gamepad.dpad_up */ || gamepad.dpad_right

    override fun runOpMode() {
        // TODO FIX THIS ENTIRE THING
        // hardwareMap is null until runOpMode() is called
        Hardware.init(hardwareMap)

        // TODO FIX THIS HACK
        val robot = Hardware
        val odometry = Odometry()

        val prevGamepad1 = Gamepad()
        val prevGamepad2 = Gamepad()

        var state = DriverControlState.Normal

        waitForStart()

        var toggleGrab = false

        var maxTurnSpeed: Double
        var maxMoveSpeed: Double
        while (opModeIsActive()) {
            val elapsed = measureTimeMillis {

                robot.resetCache()

                when (state) {
                    DriverControlState.Normal -> {

                        robot.leftMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                        robot.rightMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

                        // Sensitivity clutch with B
                        maxTurnSpeed = if (gamepad1.b) M.MAXTURNSPEED / 2 else M.MAXTURNSPEED
                        maxMoveSpeed = if (gamepad1.b) M.MAXSPEED / 2 else M.MAXSPEED

                        // Drive with triggers
                        val power = gamepad1.right_trigger - gamepad1.left_trigger // Gives a "braking" effect
                        if (abs(power) > M.DEADZONE) {
                            val speed = power.toDouble() * maxMoveSpeed
                            robot.leftMotor.power = easeFun(speed)
                            robot.rightMotor.power = easeFun(speed)
                        } else {
                            robot.leftMotor.power = 0.0
                            robot.rightMotor.power = 0.0
                        }

                        if (abs(gamepad1.left_stick_x) > M.DEADZONE) {
                            val speed = gamepad1.left_stick_x.toDouble() * maxTurnSpeed
                            robot.leftMotor.power -= easeFun(
                                -speed
                            ) // SQRT works best for turning while moving
                            robot.rightMotor.power -= easeFun(speed)
                        }

                        // Per Ben H's request, turn with bumpers
                        if (gamepad1.left_bumper) robot.leftMotor.power = maxTurnSpeed / 2
                        if (gamepad1.right_bumper) robot.rightMotor.power = maxTurnSpeed / 2

                        if (gamepad2.left_stick_y == 0.0f) {
                            robot.motorLinearSlide.targetPosition = robot.motorLinearSlide.currentPosition
                            robot.motorLinearSlide.mode = DcMotor.RunMode.RUN_TO_POSITION

                            robot.motorLinearSlide2.targetPosition = robot.motorLinearSlide2.currentPosition
                            robot.motorLinearSlide2.mode = DcMotor.RunMode.RUN_TO_POSITION
                        } else {
                            robot.motorLinearSlide.mode = DcMotor.RunMode.RUN_USING_ENCODER
                            robot.motorLinearSlide.power = M.MAXSLIDESPEED * -gamepad2.left_stick_y

                            robot.motorLinearSlide2.mode = DcMotor.RunMode.RUN_USING_ENCODER
                            robot.motorLinearSlide2.power = M.MAXSLIDESPEED * -gamepad2.left_stick_y
                        }

                        // TODO FIX THIS
                        // robot.rotateArmServo.power = gamepad2.right_stick_y.toDouble()

                        // Grabber - X to toggle open and close
                        if(gamepad2.x && !prevGamepad2.x)
                            toggleGrab = !toggleGrab


/*                        if(toggleGrab)
                            robot.grabberServo.position = 0.0
                        else
                            robot.grabberServo.position = 0.3*/

                        // TODO FIX THESE
                        /* robot.grabberServo.power = gamepad1.left_stick_y.toDouble()
                        robot.rotateArmServo.power = gamepad1.right_stick_y.toDouble() */

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
                telemetry.addLine("Motor Position (Slide): ${robot.motorLinearSlide.currentPosition}")
                telemetry.addLine("Speed (BL): ${robot.leftMotor.power}")
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
