package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Gamepad

@TeleOp
class DriverControlTest: LinearOpMode() {
    override fun runOpMode() {
        val robot = Hardware(hardwareMap)
        val prevGamepad = Gamepad()
        prevGamepad.copy(gamepad1)
        robot.leftMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        robot.leftMotor.targetPosition = 0
        robot.leftMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        robot.leftMotor.power = 0.5
        waitForStart()
        while (opModeIsActive()) {
            robot.resetCache()

            telemetry.addLine("encoder pos: ${robot.leftMotor.currentPosition}")
            telemetry.addLine("target pos: ${robot.leftMotor.targetPosition}")
            telemetry.addLine("motor mode: ${robot.leftMotor.mode}")
            telemetry.update()

        }
    }

}