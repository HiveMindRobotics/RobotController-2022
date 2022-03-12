package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName

class Hardware {
    var motorBL: DcMotor? = null
    var motorBR: DcMotor? = null
    var motorFL: DcMotor? = null
    var motorFR: DcMotor? = null
    var motorDucks: DcMotor? = null
    var motorArm: DcMotor? = null
    var servoArm: Servo? = null
    var webcamName: WebcamName? = null
    var cameraMonitorViewId : Int? = null
    var controlHubIMU: BNO055IMU? = null
    var distanceSensorFront: DistanceSensor? = null
    var distanceSensorLeft: ColorSensor? = null
    var distanceSensorRight: ColorSensor? = null
    var distanceSensorBack: ColorSensor? = null
    
    var robotFlipper: Boolean = false
    var worldHunger: Boolean = true
    var cancerCured: Boolean = false

    fun init(hwMap: HardwareMap) {
        motorBL = hwMap.get(DcMotor::class.java, "motor0")
        motorBR = hwMap.get(DcMotor::class.java, "motor1")
        motorFL = hwMap.get(DcMotor::class.java, "motor3")
        motorFR = hwMap.get(DcMotor::class.java, "motor2")
        motorDucks = hwMap.get(DcMotor::class.java, "motor4")
        motorArm = hwMap.get(DcMotor::class.java, "motor5")
        motorArm?.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        servoArm = hwMap.get(Servo::class.java, "servo0")
        webcamName = hwMap.get(WebcamName::class.java, "camera0")
        distanceSensorFront = hwMap.get(DistanceSensor::class.java, "distance0")
        distanceSensorLeft = hwMap.get(ColorSensor::class.java, "color0")
        distanceSensorRight = hwMap.get(ColorSensor::class.java, "color1")
        distanceSensorBack = hwMap.get(ColorSensor::class.java, "color2")
        // TODO: yeet live preview
        cameraMonitorViewId = hwMap.appContext.resources.getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.packageName)
        controlHubIMU = hwMap.get(BNO055IMU::class.java, "imu0")

        val parameters = BNO055IMU.Parameters()
        parameters.mode                = BNO055IMU.SensorMode.IMU
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC
        parameters.accelerationIntegrationAlgorithm = JustLoggingAccelerationIntegrator()
        parameters.loggingEnabled      = false
        controlHubIMU?.initialize(parameters)
    }
}
