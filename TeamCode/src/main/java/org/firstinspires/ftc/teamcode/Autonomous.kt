import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline
import org.firstinspires.ftc.teamcode.Hardware
import org.firstinspires.ftc.teamcode.Odometry
import org.openftc.apriltag.AprilTagDetection
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraRotation
import java.util.ArrayList

@Autonomous
class Autonomous : LinearOpMode() {

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

        robot.grabberServo.position = 0.35

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
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT)
            }

            override fun onError(errorCode: Int) {
                // who cares about error handling
            }
        })


        var detections: ArrayList<AprilTagDetection>?
        var tagNumber = 2

        telemetry.addLine("initializing...")
        telemetry.update()
        do {
            robot.resetCache()
            detections = aprilTagDetectionPipeline.getDetectionsUpdate()
            if (detections != null && detections.size != 0) {
                tagNumber = detections[0].id
            }
        } while (!isStopRequested && !isStarted && (detections == null || detections.size == 0))
        camera.closeCameraDeviceAsync {}
        telemetry.addLine("READY!! tagNumber: $tagNumber")
        telemetry.update()

        waitForStart()

        while (opModeIsActive()) {

            robot.resetCache()

            if (!robot.leftMotor.isBusy && !robot.rightMotor.isBusy) {
                when(tagNumber) {
                    1 -> {
                        when (state) {
                            0 -> {
                                robot.leftMotor.targetPosition =
                                    robot.leftMotor.currentPosition + (Odometry.TICKS_PER_INCH * 3).toInt()
                                robot.rightMotor.targetPosition =
                                    robot.rightMotor.currentPosition + (Odometry.TICKS_PER_INCH * 3).toInt()
                            }
                            1 -> {
                                // turn left
                                robot.leftMotor.targetPosition =
                                    robot.leftMotor.currentPosition - (Odometry.TICKS_PER_TURN / 4).toInt()
                                robot.rightMotor.targetPosition =
                                    robot.rightMotor.currentPosition + (Odometry.TICKS_PER_TURN / 4).toInt()
                            }
                            2 -> {
                                robot.leftMotor.targetPosition =
                                    robot.leftMotor.currentPosition + (Odometry.TICKS_PER_INCH * 24).toInt()
                                robot.rightMotor.targetPosition =
                                    robot.rightMotor.currentPosition + (Odometry.TICKS_PER_INCH * 24).toInt()
                            }
                            3 -> {
                                // turn right
                                robot.leftMotor.targetPosition =
                                    robot.leftMotor.currentPosition + (Odometry.TICKS_PER_TURN / 4).toInt()
                                robot.rightMotor.targetPosition =
                                    robot.rightMotor.currentPosition - (Odometry.TICKS_PER_TURN / 4).toInt()
                            }
                            4 -> {
                                robot.leftMotor.targetPosition =
                                    robot.leftMotor.currentPosition + (Odometry.TICKS_PER_INCH * 18).toInt()
                                robot.rightMotor.targetPosition =
                                    robot.rightMotor.currentPosition + (Odometry.TICKS_PER_INCH * 18).toInt()
                            }
                            5 -> {} // intentionally empty
                            else -> state -= 1 // cancel out the add ahead
                        }
                        state += 1
                    }
                    2 -> {
                        when (state) {
                        0 -> {
                            robot.leftMotor.targetPosition =
                                robot.leftMotor.currentPosition + (Odometry.TICKS_PER_INCH * 24).toInt()
                            robot.rightMotor.targetPosition =
                                robot.rightMotor.currentPosition + (Odometry.TICKS_PER_INCH * 24).toInt()
                        }
                            1 -> {

                            }
                            else -> state -= 1
                        }
                        state += 1
                    }
                    3 -> {
                        when (state) {
                            0 -> {
                                robot.leftMotor.targetPosition =
                                    robot.leftMotor.currentPosition + (Odometry.TICKS_PER_INCH * 3).toInt()
                                robot.rightMotor.targetPosition =
                                    robot.rightMotor.currentPosition + (Odometry.TICKS_PER_INCH * 3).toInt()
                            }
                            1 -> {
                                // turn right
                                robot.leftMotor.targetPosition =
                                    robot.leftMotor.currentPosition + (Odometry.TICKS_PER_TURN / 4).toInt()
                                robot.rightMotor.targetPosition =
                                    robot.rightMotor.currentPosition - (Odometry.TICKS_PER_TURN / 4).toInt()
                            }
                            2 -> {
                                robot.leftMotor.targetPosition =
                                    robot.leftMotor.currentPosition + (Odometry.TICKS_PER_INCH * 24).toInt()
                                robot.rightMotor.targetPosition =
                                    robot.rightMotor.currentPosition + (Odometry.TICKS_PER_INCH * 24).toInt()
                            }
                            3 -> {
                                // turn left
                                robot.leftMotor.targetPosition =
                                    robot.leftMotor.currentPosition - (Odometry.TICKS_PER_TURN / 4).toInt()
                                robot.rightMotor.targetPosition =
                                    robot.rightMotor.currentPosition + (Odometry.TICKS_PER_TURN / 4).toInt()
                            }
                            4 -> {
                                robot.leftMotor.targetPosition =
                                    robot.leftMotor.currentPosition + (Odometry.TICKS_PER_INCH * 18).toInt()
                                robot.rightMotor.targetPosition =
                                    robot.rightMotor.currentPosition + (Odometry.TICKS_PER_INCH * 18).toInt()
                            }
                            5 -> {} // intentionally empty
                            else -> state -= 1 // cancel out the add ahead
                        }
                        state += 1
                    }
                }
            }

            //DEBUG: Log movement
            telemetry.addLine("Motor Position (BL): ${robot.leftMotor.currentPosition.toFloat() * Odometry.ROTATIONS_PER_TICK}")
            telemetry.addLine("Motor Position (BR): ${robot.rightMotor.currentPosition.toFloat() * Odometry.ROTATIONS_PER_TICK}")
            telemetry.addLine("Stage: $state")
            telemetry.addLine("tagNumber: $tagNumber")

            telemetry.update()

        }
    }
}
