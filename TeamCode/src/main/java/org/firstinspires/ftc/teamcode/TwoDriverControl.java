package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Two Drivers", group = "Linear Opmode")
public class TwoDriverControl extends DriverControl {
    @Override
    void control() {
        super.control();
        robot.motorDucks.setPower(gamepad2.right_stick_x);
        robot.motorArm.setPower(gamepad2.dpad_up ? 1 : gamepad2.dpad_down ? -1 : 0);
        robot.servoArm.setPosition(gamepad2.right_bumper ? 1 : 0);
    }
}
