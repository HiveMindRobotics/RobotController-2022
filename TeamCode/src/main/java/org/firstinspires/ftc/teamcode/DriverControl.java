package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name = "Driver Control", group = "Linear Opmode")
public class DriverControl extends LinearOpMode {

    Hardware robot = new Hardware();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            robot.motorFL.setPower(-r * Math.cos(robotAngle) + rightX);
            robot.motorBL.setPower(-r * Math.sin(robotAngle) + rightX);
            robot.motorFR.setPower(r * Math.sin(robotAngle) - rightX);
            robot.motorBR.setPower(r * Math.cos(robotAngle) - rightX);

            robot.motorDucks.setPower(-gamepad2.left_stick_x);
            robot.motorArm.setPower(-gamepad2.left_stick_y);
            robot.servoArm.setPosition(gamepad2.right_trigger > 0 ? 1 : 0);

        }
    }
}