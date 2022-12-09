package org.firstinspires.ftc.teamcode.subsystems

//import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo

class Grabber(hwMap: HardwareMap) {
    private val grabberServo: Servo
    private val rotateArmServo: Servo

    /**
     * open or close the grabber
     * true means the grabber is open
     */
    var grabberClosed = false
        set(value) {
            if (value)
                grabberServo.position = 0.3
            else
                grabberServo.position = 0.0
            field = value
        }

    /**
     * control whether the rotation servo on the grabber is up or down
     * true means the grabber is raised
     */
    var grabberRaised = false
        set(value) {
            rotateArmServo.position = if (value) 0.0 else 0.0
            field = value
        }

    init {
        grabberServo = hwMap.get(Servo::class.java, "servo0")
        rotateArmServo = hwMap.get(Servo::class.java, "servo1")
    }
}