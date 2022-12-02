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

    /**
     * move the robot forward by a certain number of inches
     * @param inches number of inches to move, (-infinity, infinity)
     */
    fun moveByInches(inches: Int) = runToPositionMode {
        rightMotor.targetPosition = rightMotor.currentPosition + (Odometry.TICKS_PER_INCH * inches).toInt()
        leftMotor.targetPosition = leftMotor.currentPosition + (Odometry.TICKS_PER_INCH * inches).toInt()
    }
}