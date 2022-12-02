package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap

class TwoWheelTank(hwMap: HardwareMap) {
    private val leftMotor: DcMotorEx
    private val rightMotor: DcMotorEx

    val isBusy: Boolean
        get() = leftMotor.isBusy || rightMotor.isBusy

    init {
        leftMotor = hwMap.get(DcMotorEx::class.java, "motor0")
        leftMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        rightMotor = hwMap.get(DcMotorEx::class.java, "motor1")
        rightMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightMotor.direction = DcMotorSimple.Direction.REVERSE
    }

    fun turnByDegrees() {

    }

    fun moveByInches() {

    }
}