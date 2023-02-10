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
        if (x > 0) x.pow(2.0) else -x.pow(2.0)

    private fun anyDpad(gamepad: Gamepad): Boolean =
        gamepad.dpad_down || gamepad.dpad_left /* || gamepad.dpad_up */ || gamepad.dpad_right

    override fun runOpMode() {
        // hardwareMap is null until runOpMode() is called
        val robot = Hardware(hardwareMap)
        val odometry = Odometry()

        val prevGamepad1 = Gamepad()
        val prevGamepad2 = Gamepad()

        var state = DriverControlState.Normal
        var currentPos = 0.0

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
                        maxTurnSpeed = if (gamepad1.b) M.MAXTURNSPEED else M.MAXTURNSPEED * 0.7
                        maxMoveSpeed = if (gamepad1.b) M.MAXSPEED else M.MAXSPEED * 0.7

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

                        /*if (gamepad2.right_trigger == 0.0f && gamepad2.left_trigger == 0.0f) {
                            robot.motorLinearSlide.targetPosition = robot.motorLinearSlide.currentPosition
                            robot.motorLinearSlide.mode = DcMotor.RunMode.RUN_TO_POSITION

                            robot.motorLinearSlide2.targetPosition = robot.motorLinearSlide2.currentPosition
                            robot.motorLinearSlide2.mode = DcMotor.RunMode.RUN_TO_POSITION
                        } else {
                            robot.motorLinearSlide.mode = DcMotor.RunMode.RUN_USING_ENCODER*/
                            robot.motorLinearSlide.power = M.MAXSLIDESPEED * (-gamepad2.right_stick_y.toDouble())

                            // robot.motorLinearSlide2.mode = DcMotor.RunMode.RUN_USING_ENCODER
                            robot.motorLinearSlide2.power = M.MAXSLIDESPEED * (-gamepad2.right_stick_y.toDouble())
                        // }

                        // Grabber - X to toggle open and close
                        if(gamepad2.right_bumper && !prevGamepad2.right_bumper)
                            toggleGrab = !toggleGrab

                        //robot.grabberServo.power = gamepad2.left_stick_y.toDouble()
                        robot.grabberServo.position = if (toggleGrab) 0.35 else 0.15
                        robot.rotateArmServo.power = easeFun(gamepad2.left_stick_y.toDouble())

                        // snap turning
                        if(anyDpad(gamepad1) && !anyDpad(prevGamepad1)) {
                            robot.leftMotor.targetPosition = robot.leftMotor.currentPosition
                            robot.rightMotor.targetPosition = robot.rightMotor.currentPosition
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
                            robot.rightMotor.targetPosition = robot.rightMotor.currentPosition
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
                telemetry.addLine("Servo: ${robot.grabberServo.position}")
            }

            telemetry.addLine("loop time $elapsed")
            telemetry.update()
        }
    }
}
