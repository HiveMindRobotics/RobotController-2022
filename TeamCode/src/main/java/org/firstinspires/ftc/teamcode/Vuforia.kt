package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix
import org.firstinspires.ftc.robotcore.external.ClassFactory
import org.firstinspires.ftc.robotcore.external.navigation.*
import java.util.ArrayList

class Vuforia(private val robot: Hardware, private val opMode: LinearOpMode) {
    var lastLocation: OpenGLMatrix? = null
    private var vuforia: VuforiaLocalizer? = null
    private var targets: VuforiaTrackables? = null
    private var targetVisible = false
    private var allTrackables: MutableList<VuforiaTrackable?>? = null

    fun init() {

        val parameters = VuforiaLocalizer.Parameters()
        parameters.vuforiaLicenseKey = VUFORIA_KEY

        parameters.cameraName = robot.webcamName

        parameters.useExtendedTracking = false

        vuforia = ClassFactory.getInstance().createVuforia(parameters)

        targets = vuforia!!.loadTrackablesFromAsset("FreightFrenzy")

        allTrackables = ArrayList()
        allTrackables?.addAll(targets!!)

        identifyTarget(0, "Blue Storage", -halfField, oneAndHalfTile, mmTargetHeight, 90f, 0f, 90f)
        identifyTarget(1, "Blue Alliance Wall", halfTile, halfField, mmTargetHeight, 90f, 0f, 0f)
        identifyTarget(2, "Red Storage", -halfField, -oneAndHalfTile, mmTargetHeight, 90f, 0f, 90f)
        identifyTarget(3, "Red Alliance Wall", halfTile, -halfField, mmTargetHeight, 90f, 0f, 180f)

        val CAMERA_FORWARD_DISPLACEMENT =
            6.25f * mmPerInch
        val CAMERA_VERTICAL_DISPLACEMENT = 4.5f * mmPerInch
        val CAMERA_LEFT_DISPLACEMENT =
            3.5f * mmPerInch
        val cameraLocationOnRobot = OpenGLMatrix
            .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
            .multiplied(
                Orientation.getRotationMatrix(
                    AxesReference.EXTRINSIC,
                    AxesOrder.XZY,
                    AngleUnit.DEGREES,
                    90f,
                    90f,
                    0f
                )
            )

        for (trackable in allTrackables!!) {
            (trackable!!.listener as VuforiaTrackableDefaultListener).setCameraLocationOnRobot(
                parameters.cameraName!!,
                cameraLocationOnRobot
            )
        }
        targets!!.activate()
    }

    fun getPosition() {
            targetVisible = false
            for (trackable in allTrackables!!) {
                if ((trackable!!.listener as VuforiaTrackableDefaultListener).isVisible) {
                    opMode.telemetry.addData("Visible Target", trackable.name)
                    targetVisible = true

                    val robotLocationTransform =
                        (trackable.listener as VuforiaTrackableDefaultListener).updatedRobotLocation
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform
                    }
                    break
                }
            }

            if (targetVisible) {
                val translation = lastLocation!!.translation
                opMode.telemetry.addData(
                    "Pos (inches)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation[0] / mmPerInch, translation[1] / mmPerInch, translation[2] / mmPerInch
                )

                val rotation =
                    Orientation.getOrientation(lastLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES)
                opMode.telemetry.addData(
                    "Rot (deg)",
                    "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f",
                    rotation.firstAngle,
                    rotation.secondAngle,
                    rotation.thirdAngle
                )
            } else {
                opMode.telemetry.addData("Visible Target", "none")
            }
            opMode.telemetry.update()
    }

    private fun identifyTarget(
        targetIndex: Int,
        targetName: String?,
        dx: Float,
        dy: Float,
        dz: Float,
        rx: Float,
        ry: Float,
        rz: Float
    ) {
        val aTarget = targets!![targetIndex]
        aTarget.name = targetName
        aTarget.location = OpenGLMatrix.translation(dx, dy, dz)
            .multiplied(
                Orientation.getRotationMatrix(
                    AxesReference.EXTRINSIC,
                    AxesOrder.XYZ,
                    AngleUnit.DEGREES,
                    rx,
                    ry,
                    rz
                )
            )
    }

    companion object {
        private const val VUFORIA_KEY =
            "Aci0vMT/////AAABmfhdrnZAvEt1j4pdeLfUrh4Z/YsocsADwlvHAd2ynqR01+VTMT5eWXkdbqzLprLlpI9jlWycsNSv+Bj15CbyYi1Tjv9u+QJ4t+zSZS3U1M00UW59a8CrDCTFTlEvDmEhf5RAlTY2SNhqq8U+MVdAMw7BG5osJdaHuY2UwHIdp1hKr8bThBLWT3vTF0C5VymM9KrueBCbt1b8Da2gphEzaYTyR6t8f1Ssu66O3t+WGDbnErM7OJs9kl75JyemAa9ijRqQyl9S/T8carMWNO3URaTX+9SIGeW+Mk4wKH7eKYPhXqoRb1jKULv2w0bQAU4Zz0K8DlcPtLT005iR9nHvSC5yzz99qgsLw/k3diSduCfV"

        private const val mmPerInch = 25.4f
        private const val mmTargetHeight = 6 * mmPerInch // the height of the center of the target image above the floor
        private const val halfField = 72 * mmPerInch
        private const val halfTile = 12 * mmPerInch
        private const val oneAndHalfTile = 36 * mmPerInch
    }
}