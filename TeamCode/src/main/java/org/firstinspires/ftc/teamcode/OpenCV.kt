package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation


class OpenCV(private val robot: Hardware, private val opMode: LinearOpMode) {
    val camera: OpenCvCamera = OpenCvCameraFactory.getInstance().createWebcam(robot.webcamName, robot.cameraMonitorViewId!!)

    fun init() {
        camera.openCameraDeviceAsync(object : AsyncCameraOpenListener {
            override fun onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT)
            }
            override fun onError(errorCode: Int) {
                opMode.telemetry.addData("OpenCV Error:", errorCode)
            }
        })
    }
}