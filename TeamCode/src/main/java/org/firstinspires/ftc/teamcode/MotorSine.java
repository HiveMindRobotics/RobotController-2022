package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

class ValueRange
{
    double min, max;
    ValueRange(double min, double max) {
        this.min = min;
        this.max = max;
    }

    boolean isInRange(double value)
    {
        return value > min && value < max;
    }
}

class MotorMap
{
    double fl, fr, bl, br;
    DcMotor flm, frm, blm, brm;
    MotorMap(double fl, double fr, double bl, double br, DcMotor flm, DcMotor frm, DcMotor blm, DcMotor brm)
    {
        this.fl = fl;
        this.bl = bl;
        this.fr = fr;
        this.br = br;
        this.flm = flm;
        this.frm = frm;
        this.blm = blm;
        this.brm = brm;
    }

    void setPower(double fl, double fr, double bl, double br)
    {
        this.fl = fl;
        this.fr = fr;
        this.bl = bl;
        this.br = br;
    }

    void run()
    {
        flm.setPower(-fl);
        blm.setPower(-bl);
        frm.setPower(fr);
        brm.setPower(br);
    }
}

@TeleOp(name="Motor Sine", group="Linear Opmode")
public class MotorSine extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor motorBL = null;
    private DcMotor motorBR = null;
    private DcMotor motorFL = null;
    private DcMotor motorFR = null;
    private DigitalChannel touch = null;
    private CRServo arm = null;

    // private double deadZone = 0.1;

    @Override
    public void runOpMode() {
        motorBL = hardwareMap.get(DcMotor.class, "motor0");
        motorBR = hardwareMap.get(DcMotor.class, "motor1");
        motorFL = hardwareMap.get(DcMotor.class, "motor3");
        motorFR = hardwareMap.get(DcMotor.class, "motor2");
        arm     = hardwareMap.get(CRServo.class,   "servo0");
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            MotorMap map = new MotorMap(0, 0, 0, 0,
                    motorFL, motorFR, motorBL, motorBR);

            /*double leftStickY = gamepad1.left_stick_y;
            double leftStickX = gamepad1.left_stick_x;

            //double modifier = 0.1;

            if(gamepad1.a) {
                map.setPower(
                        (leftStickY),
                        -(leftStickY),
                        -leftStickY,
                        leftStickY
                );
            } else {
                map.setPower(
                        leftStickY,
                        leftStickY,
                        leftStickY,
                        leftStickY
                );
            }
            map.run();*/
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            map.setPower(v1 * 2, v2 * 2, v3 * 2, v4 * 2);
            map.run();

            arm.setPower(gamepad1.dpad_up ? 1 : gamepad1.dpad_down ? -1 : 0);
        }
    }
}

