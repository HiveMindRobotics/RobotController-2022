package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import kotlin.properties.Delegates

object Hardware {
    // add "lateinit" whenever you need to comment something out
    lateinit var leftMotor: DcMotorEx
    lateinit var rightMotor: DcMotorEx
    lateinit var motorFL: DcMotor
    lateinit var motorFR: DcMotor

    lateinit var motorLinearSlide: DcMotorEx
    lateinit var motorLinearSlide2: DcMotorEx


    lateinit var motorDucks: DcMotor

    lateinit var motorArm: DcMotor
    lateinit var grabberServo: Servo
    lateinit var rotateArmServo: Servo

    lateinit var distanceSensorFront: DistanceSensor

    var cameraMonitorViewId by Delegates.notNull<Int>()
    lateinit var openCvCamera: OpenCvCamera
    lateinit var webcamName: WebcamName

    lateinit var controlHubIMU: BNO055IMU
    lateinit var expansionHubIMU: BNO055IMU

    lateinit var allHubs: List<LynxModule>

    fun resetCache() {
        for (hub in allHubs) {
            hub.clearBulkCache()
        }
    }

    fun init(hwMap: HardwareMap) {
        leftMotor = hwMap.get(DcMotorEx::class.java, "motor0")
        leftMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        rightMotor = hwMap.get(DcMotorEx::class.java, "motor1")
        rightMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightMotor.direction = DcMotorSimple.Direction.REVERSE


        motorLinearSlide = hwMap.get(DcMotorEx::class.java, "motor2")
        motorLinearSlide.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motorLinearSlide.targetPosition = 0
        motorLinearSlide.mode = DcMotor.RunMode.RUN_USING_ENCODER

        motorLinearSlide2 = hwMap.get(DcMotorEx::class.java, "motor3")
        motorLinearSlide2.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motorLinearSlide2.targetPosition = 0
        motorLinearSlide2.mode = DcMotor.RunMode.RUN_USING_ENCODER
        motorLinearSlide2.direction = DcMotorSimple.Direction.REVERSE

        grabberServo = hwMap.get(Servo::class.java, "servo0")
        rotateArmServo = hwMap.get(Servo::class.java, "servo1")

        allHubs = hwMap.getAll(LynxModule::class.java)

        // WARNING!!!! you MUST reset the cache after every cycle!!!!!!!!!
        for (hub in allHubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }

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

/*        openCvCamera = OpenCvCameraFactory.getInstance()
            .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK)*/
        webcamName = hwMap.get(WebcamName::class.java, "camera0")
        openCvCamera = OpenCvCameraFactory.getInstance().createWebcam(webcamName)

        controlHubIMU = hwMap.get(BNO055IMU::class.java, "imu0")
        val imuParams = BNO055IMU.Parameters()
        imuParams.angleUnit = BNO055IMU.AngleUnit.RADIANS
        controlHubIMU.initialize(imuParams)
/*
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