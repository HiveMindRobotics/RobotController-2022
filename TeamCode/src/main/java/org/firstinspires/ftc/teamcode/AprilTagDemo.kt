package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener
import org.openftc.easyopencv.OpenCvCameraRotation

@TeleOp
class AprilTagDemo : LinearOpMode() {

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

    override fun runOpMode() {
        val hardware = Hardware(hardwareMap)

        val camera = hardware.openCvCamera

        val aprilTagDetectionPipeline = AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy)

        camera.setPipeline(aprilTagDetectionPipeline)

        camera.openCameraDeviceAsync(object : AsyncCameraOpenListener {
            override fun onOpened() {
                // TODO: the resolution will need to be changed for the webcam
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT)
            }

            override fun onError(errorCode: Int) {
                // who cares about error handling
            }
        })

        waitForStart()

        telemetry.msTransmissionInterval = 50

        while (opModeIsActive()) {
            for (hub in hardware.allHubs) {
                hub.clearBulkCache()
            }

            // Calling getDetectionsUpdate() will only return an object if there was a new frame
            // processed since the last time we called it. Otherwise, it will return null. This
            // enables us to only run logic when there has been a new frame, as opposed to the
            // getLatestDetections() method which will always return an object.
            val detections = aprilTagDetectionPipeline.getDetectionsUpdate()

            // If there's been a new frame...
            if (detections != null) {
                telemetry.addData("FPS", camera.fps)
                telemetry.addData("Overhead ms", camera.overheadTimeMs)
                telemetry.addData("Pipeline ms", camera.pipelineTimeMs)


                for (detection in detections) {
                    telemetry.addLine("Detected tag ID=${detection.id}")
                    telemetry.addLine("Translation X: ${detection.pose.x*100} cm")
                    telemetry.addLine("Translation Y: ${detection.pose.y*100} cm")
                    telemetry.addLine("Translation Z: ${detection.pose.z*100} cm")
                    telemetry.addLine("Rotation Yaw: ${Math.toDegrees(detection.pose.yaw)} degrees")
                    telemetry.addLine("Rotation Pitch: ${Math.toDegrees(detection.pose.pitch)} degrees")
                    telemetry.addLine("Rotation Roll: ${Math.toDegrees(detection.pose.roll)} degrees")
                }

                telemetry.update()
            }
            sleep(20)
        }
    }
}