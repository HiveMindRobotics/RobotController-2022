import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.DriverControlState
import org.firstinspires.ftc.teamcode.Hardware
import org.firstinspires.ftc.teamcode.Odometry
import kotlin.math.abs
import kotlin.math.pow
import kotlin.system.measureTimeMillis

@Autonomous
class AutoTest : LinearOpMode() {

    override fun runOpMode() {
        // hardwareMap is null until runOpMode() is called
        val robot = Hardware(hardwareMap)
        val odometry = Odometry()

        var state = 0

        robot.leftMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        robot.rightMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        robot.leftMotor.power = 0.3
        robot.rightMotor.power = 0.3

        waitForStart()

        var targetTurnSpeed: Double
        while (opModeIsActive()) {

            robot.resetCache()

            if (!robot.leftMotor.isBusy && !robot.rightMotor.isBusy) {
                when (state) {
                    0 -> {
                        robot.leftMotor.targetPosition = robot.leftMotor.currentPosition + (Odometry.TICKS_PER_INCH * 12).toInt()
                        robot.rightMotor.targetPosition = robot.rightMotor.currentPosition + (Odometry.TICKS_PER_INCH * 12).toInt()
                        state = 1
                    }

                    1 -> {
                        robot.leftMotor.targetPosition = robot.leftMotor.currentPosition - (Odometry.TICKS_PER_TURN / 4).toInt()
                        robot.rightMotor.targetPosition = robot.rightMotor.currentPosition + (Odometry.TICKS_PER_TURN / 4).toInt()
                        state = 2
                    }

                    2 -> {
                        robot.leftMotor.targetPosition = robot.leftMotor.currentPosition + (Odometry.TICKS_PER_INCH * 12).toInt()
                        robot.rightMotor.targetPosition = robot.rightMotor.currentPosition + (Odometry.TICKS_PER_INCH * 12).toInt()
                        state = 3
                    }
                }
            }

                odometry.update(
                    robot.leftMotor.currentPosition,
                    robot.rightMotor.currentPosition,
                    robot.controlHubIMU.angularOrientation.firstAngle
                )
        }
    }
}
