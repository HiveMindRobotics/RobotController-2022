package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="One Driver", group="Linear Opmode")
public class OneDriverControl extends DriverControl {
    @Override
    void control() {
        super.control();
        robot.motorDucks.setPower(gamepad1.dpad_up ? 1 : gamepad1.dpad_down ? -1 : 0);
        robot.motorArm.setPower(gamepad1.dpad_up ? 1 : gamepad1.dpad_down ? -1 : 0);
        robot.servoArm.setPosition(gamepad1.right_bumper ? 1 : 0);
    }
}

