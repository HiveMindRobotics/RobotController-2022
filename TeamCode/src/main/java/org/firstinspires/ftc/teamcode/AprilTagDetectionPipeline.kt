package org.firstinspires.ftc.teamcode

import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.imgproc.Imgproc
import org.openftc.apriltag.AprilTagDetection
import org.openftc.apriltag.AprilTagDetectorJNI
import org.openftc.easyopencv.OpenCvPipeline

class AprilTagDetectionPipeline(
    private var tagSize: Double,
    private var fx: Double,
    private var fy: Double,
    private var cx: Double,
    private var cy: Double
) : OpenCvPipeline() {
    private var nativeApriltagPtr: Long
    private val grey = Mat()
    private var detections = ArrayList<AprilTagDetection>()
    private var detectionsUpdate: ArrayList<AprilTagDetection>? = ArrayList()
    private val detectionsUpdateSync = Any()


    var cameraMatrix: Mat = Mat(3, 3, CvType.CV_32FC1)

    init {

        //     Construct the camera matrix.
        //
        //      --         --
        //     | fx   0   cx |
        //     | 0    fy  cy |
        //     | 0    0   1  |
        //      --         --
        //

        cameraMatrix.put(0, 0, fx)
        cameraMatrix.put(0, 1, 0.0)
        cameraMatrix.put(0, 2, cx)
        cameraMatrix.put(1, 0, 0.0)
        cameraMatrix.put(1, 1, fy)
        cameraMatrix.put(1, 2, cy)
        cameraMatrix.put(2, 0, 0.0)
        cameraMatrix.put(2, 1, 0.0)
        cameraMatrix.put(2, 2, 1.0)

        // Allocate a native context object. See the corresponding deletion in the finalizer
        nativeApriltagPtr =
            AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3f, 3)
    }

    protected fun finalize() {
        // Delete the native context we created in the constructor
        AprilTagDetectorJNI.releaseApriltagDetector(nativeApriltagPtr)
        nativeApriltagPtr = 0
    }

    override fun processFrame(input: Mat): Mat {
        Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGBA2GRAY)

        AprilTagDetectorJNI.setApriltagDetectorDecimation(nativeApriltagPtr, 2f)

        detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(nativeApriltagPtr, grey, tagSize, fx, fy, cx, cy)
        synchronized(detectionsUpdateSync) { detectionsUpdate = detections }

        return input
    }

    fun getDetectionsUpdate(): ArrayList<AprilTagDetection>? {
        synchronized(detectionsUpdateSync) {
            val ret = detectionsUpdate
            detectionsUpdate = null
            return ret
        }
    }
}
