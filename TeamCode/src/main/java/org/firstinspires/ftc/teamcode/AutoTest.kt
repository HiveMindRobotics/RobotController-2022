import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline
import org.firstinspires.ftc.teamcode.DriverControlState
import org.firstinspires.ftc.teamcode.Hardware
import org.firstinspires.ftc.teamcode.Odometry
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraRotation
import kotlin.math.abs
import kotlin.math.pow
import kotlin.system.measureTimeMillis

@Autonomous
class AutoTest : LinearOpMode() {

    override fun runOpMode() {
        // Lens intrinsics
        // UNITS ARE PIXELS
        // TODO: calibrate
        var fx = 578.272
        var fy = 578.272
        var cx = 402.145
        var cy = 221.506

        // UNITS ARE METERS
        // 0.98 inches
        var tagsize = 0.0249

        // hardwareMap is null until runOpMode() is called
        val robot = Hardware(hardwareMap)
        val odometry = Odometry()

        var state = 0

        robot.leftMotor.targetPosition = robot.leftMotor.currentPosition
        robot.rightMotor.targetPosition = robot.rightMotor.currentPosition
        robot.leftMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        robot.rightMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        robot.leftMotor.power = 0.3
        robot.rightMotor.power = 0.3

        val camera = robot.openCvCamera

        val aprilTagDetectionPipeline = AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy)

        camera.setPipeline(aprilTagDetectionPipeline)

        camera.openCameraDeviceAsync(object : OpenCvCamera.AsyncCameraOpenListener {
            override fun onOpened() {
                // TODO: the resolution will need to be changed for the webcam
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT)
            }

            override fun onError(errorCode: Int) {
                // who cares about error handling
            }
        })

        waitForStart()

        val detections = aprilTagDetectionPipeline.getDetectionsUpdate()
        var tagNumber = detections?.get(0)?.id ?: 1
        camera.closeCameraDeviceAsync {}

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

            //DEBUG: Log movement
            telemetry.addLine("Motor Position (BL): ${robot.leftMotor.currentPosition.toFloat() * Odometry.ROTATIONS_PER_TICK}")
            telemetry.addLine("Motor Position (BR): ${robot.rightMotor.currentPosition.toFloat() * Odometry.ROTATIONS_PER_TICK}")
            telemetry.addLine("Stage: $state")
            telemetry.addLine("tagNumber: $tagNumber")

            telemetry.update()

                odometry.update(
                    robot.leftMotor.currentPosition,
                    robot.rightMotor.currentPosition,
                    robot.controlHubIMU.angularOrientation.firstAngle
                )
        }
    }
}
