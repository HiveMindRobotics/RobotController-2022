package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hardware {
    public DcMotor motorBL = null;
    public DcMotor motorBR = null;
    public DcMotor motorFL = null;
    public DcMotor motorFR = null;
    public DcMotor motorDucks = null;
    public DcMotor motorArm = null;
    public Servo servoArm = null;

    public void init(HardwareMap hwMap) {
        motorBL = hwMap.get(DcMotor.class, "motor0");
        motorBR = hwMap.get(DcMotor.class, "motor1");
        motorFL = hwMap.get(DcMotor.class, "motor3");
        motorFR = hwMap.get(DcMotor.class, "motor2");
        motorDucks = hwMap.get(DcMotor.class, "motor4");
        motorArm = hwMap.get(DcMotor.class, "motor5");
        servoArm = hwMap.get(Servo.class, "servo0");
    }
}
