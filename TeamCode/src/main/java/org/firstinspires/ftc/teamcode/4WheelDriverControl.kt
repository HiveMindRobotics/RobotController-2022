package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Gamepad
import kotlin.math.*

@TeleOp(name = "4 Wheel Driver Control", group = "Linear Opmode")
class `4WheelDriverControl` : LinearOpMode() {
    @Config
    object `4M` {
        @JvmField var DEADZONE = 0.1
        @JvmField var MAXSPEED = 0.7
        @JvmField var MAXTURNSPEED = 0.5 // Want more precise turning but faster forwards/backwards movement
        @JvmField var ROTATEAMOUNT = 0.2
        @JvmField var MAXHEIGHT = 2117
        @JvmField var MAXSLIDESPEED = 0.5
        // 1 is temporary - just to test
    }

    private fun easeFun(x: Double): Double =
        if (x > 0) (0.857 * x + 0.1).pow(1.58) else -((0.857 * -x + 0.1).pow(1.58))
    override fun runOpMode() {
        // hardwareMap is null until runOpMode() is called
        val robot = Hardware(hardwareMap)
        val odometry = Odometry()

        val prevGamepad1 = Gamepad()
        val prevGamepad2 = Gamepad()

        var state = DriverControlState.Normal

        waitForStart()

        var toggleGrab = false

        var maxTurnSpeed: Double
        var maxMoveSpeed: Double
        while (opModeIsActive()) {

                robot.resetCache()

                        // Sensitivity clutch with B
                        maxTurnSpeed = if (gamepad1.b) `4M`.MAXTURNSPEED / 2 else `4M`.MAXTURNSPEED
                        maxMoveSpeed = if (gamepad1.b) `4M`.MAXSPEED / 2 else `4M`.MAXSPEED

                        // Drive with triggers
                        val power = gamepad1.right_trigger - gamepad1.left_trigger // Gives a "braking" effect
                        if (abs(power) > `4M`.DEADZONE) {
                            val speed = power.toDouble() * maxMoveSpeed
                            robot.leftMotor.power = easeFun(speed)
                            robot.rightMotor.power = easeFun(speed)
                        } else {
                            robot.leftMotor.power = 0.0
                            robot.rightMotor.power = 0.0
                        }

                        if (abs(gamepad1.left_stick_x) > `4M`.DEADZONE) {
                            val speed = gamepad1.left_stick_x.toDouble() * maxTurnSpeed
                            robot.leftMotor.power -= easeFun(
                                -speed
                            ) // SQRT works best for turning while moving
                            robot.rightMotor.power -= easeFun(speed)
                        }

                        // Per Ben H's request, turn with bumpers
                        if (gamepad1.left_bumper) robot.leftMotor.power = maxTurnSpeed / 2
                        if (gamepad1.right_bumper) robot.rightMotor.power = maxTurnSpeed / 2

                        robot.rotateArmServo.power = gamepad2.right_stick_y.toDouble()

                        // Grabber - X to toggle open and close
                        if(gamepad2.x && !prevGamepad2.x)
                            toggleGrab = !toggleGrab

/*                        if(toggleGrab)
                            robot.grabberServo.position = 0.0
                        else
                            robot.grabberServo.position = 0.3*/
                    }
                }
            }

