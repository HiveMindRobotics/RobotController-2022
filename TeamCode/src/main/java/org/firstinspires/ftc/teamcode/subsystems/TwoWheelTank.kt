import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.DriverControlState
import org.firstinspires.ftc.teamcode.Odometry

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

    var stopped = false
        set(value) {
            field = value
            runToPositionMode { }
        }

    private fun runToPositionMode(func: () -> Unit) {
        leftMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        rightMotor.mode = DcMotor.RunMode.RUN_TO_POSITION

        leftMotor.targetPosition = leftMotor.currentPosition
        rightMotor.targetPosition = rightMotor.currentPosition

        leftMotor.power = 0.3
        rightMotor.power = 0.3
        func()
        while (!leftMotor.isBusy && !rightMotor.isBusy && !stopped) {
        }
        leftMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        rightMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }


     /**
     * rotate the robot by a certain number of degrees
     * @param degrees number of degrees to turn, [-180, 180]
     */
    fun turnByDegrees(degrees: Int) = runToPositionMode {
         leftMotor.targetPosition = leftMotor.currentPosition + (Odometry.TICKS_PER_TURN / 30).toInt()
         rightMotor.targetPosition = rightMotor.currentPosition - (Odometry.TICKS_PER_TURN / 4).toInt()
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