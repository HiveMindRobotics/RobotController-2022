package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvInternalCamera
import kotlin.properties.Delegates

class Hardware(hwMap: HardwareMap?) {
    // add "lateinit" whenever you need to comment something out
    var motorBL: DcMotor
    var motorBR: DcMotor
    lateinit var motorFL: DcMotor
    lateinit var motorFR: DcMotor

    lateinit var motorDucks: DcMotor

    lateinit var motorArm: DcMotor
    lateinit var servoArm: Servo

    lateinit var distanceSensorFront: DistanceSensor

    var cameraMonitorViewId by Delegates.notNull<Int>()
    var openCvCamera: OpenCvCamera
    var webcamName: WebcamName

    lateinit var controlHubIMU: BNO055IMU
    lateinit var expansionHubIMU: BNO055IMU

    init {
        hwMap!!
        motorBL = hwMap.get(DcMotor::class.java, "motor0")
        motorBR = hwMap.get(DcMotor::class.java, "motor1")
        /*
        motorFL = hwMap.get(DcMotor::class.java, "motor3")
        motorFR = hwMap.get(DcMotor::class.java, "motor2")

        motorDucks = hwMap.get(DcMotor::class.java, "motor4")

        motorArm = hwMap.get(DcMotor::class.java, "motor5")
        motorArm?.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        servoArm = hwMap.get(Servo::class.java, "servo0")

        distanceSensorFront = hwMap.get(DistanceSensor::class.java, "distance0")

        webcamName = hwMap.get(WebcamName::class.java, "camera0")*/

/*        cameraMonitorViewId = hwMap?.appContext?.resources?.getIdentifier(
            "cameraMonitorViewId",
            "id",
            hwMap.appContext?.packageName
        )!!*/

        // TODO: switch to external camera
/*        openCvCamera = OpenCvCameraFactory.getInstance()
            .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK)*/
        webcamName = hwMap.get(WebcamName::class.java, "camera0")
        openCvCamera = OpenCvCameraFactory.getInstance().createWebcam(webcamName)
/*
        controlHubIMU = hwMap.get(BNO055IMU::class.java, "imu1")
        expansionHubIMU = hwMap.get(BNO055IMU::class.java, "imu0")

        val parameters = BNO055IMU.Parameters()
        parameters.mode                = BNO055IMU.SensorMode.IMU
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC
        parameters.accelerationIntegrationAlgorithm = JustLoggingAccelerationIntegrator()
        parameters.loggingEnabled      = false
        controlHubIMU?.initialize(parameters)
        expansionHubIMU?.initialize(parameters)*/
    }
}