package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.opencv.core.Mat
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvPipeline


class OpenCV(private val robot: Hardware, private val opMode: LinearOpMode) {
    val camera: OpenCvCamera = robot.openCvCamera
    val barcodePosition: Int? = null

    fun init() {
        camera.openCameraDeviceAsync(object : AsyncCameraOpenListener {
            override fun onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT)
                camera.setPipeline(First())
            }
            override fun onError(errorCode: Int) {
                opMode.telemetry.addData("OpenCV Error:", errorCode)
            }
        })
    }
    inner class First : OpenCvPipeline() {
        override fun processFrame(input: Mat?): Mat {
            TODO("Not yet implemented")
        }

    }
}