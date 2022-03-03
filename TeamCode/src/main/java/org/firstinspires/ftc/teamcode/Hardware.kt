package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.HardwareMap

class Hardware {
    var motorBL: DcMotor? = null
    var motorBR: DcMotor? = null
    var motorFL: DcMotor? = null
    var motorFR: DcMotor? = null
    var motorDucks: DcMotor? = null
    var motorArm: DcMotor? = null
    var servoArm: Servo? = null

    fun init(hwMap: HardwareMap) {
        motorBL = hwMap.get(DcMotor::class.java, "motor0")
        motorBR = hwMap.get(DcMotor::class.java, "motor1")
        motorFL = hwMap.get(DcMotor::class.java, "motor3")
        motorFR = hwMap.get(DcMotor::class.java, "motor2")
        motorDucks = hwMap.get(DcMotor::class.java, "motor4")
        motorArm = hwMap.get(DcMotor::class.java, "motor5")
        servoArm = hwMap.get(Servo::class.java, "servo0")
    }
}
