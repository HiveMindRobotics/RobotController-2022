import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.drive.TankDrive
import com.acmerobotics.roadrunner.followers.TankPIDVAFollower
import com.acmerobotics.roadrunner.followers.TrajectoryFollower
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.*
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.*
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.DriveConstants
import org.firstinspires.ftc.teamcode.DriveConstants.kA
import org.firstinspires.ftc.teamcode.DriveConstants.kStatic
import org.firstinspires.ftc.teamcode.DriveConstants.kV
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil
import java.util.*

/*
* Simple tank drive hardware implementation for REV hardware.
*/
@Config
class SampleTankDrive(hardwareMap: HardwareMap) : TankDrive(kV, kA, kStatic, DriveConstants.TRACK_WIDTH) {
    private val trajectorySequenceRunner: TrajectorySequenceRunner
    private val follower: TrajectoryFollower
    private val motors: List<DcMotorEx>
    private val leftMotors: List<DcMotorEx>
    private val rightMotors: List<DcMotorEx>
    private val imu: BNO055IMU
    private val batteryVoltageSensor: VoltageSensor

    init {
        follower = TankPIDVAFollower(
            AXIAL_PID, CROSS_TRACK_PID,
            Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5
        )
        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap)
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next()
        for (module in hardwareMap.getAll(LynxModule::class.java)) {
            module.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
        }

        // TODO: adjust the names of the following hardware devices to match your configuration
        imu = hardwareMap.get(BNO055IMU::class.java, "imu0")
        // TODO: Adjust the orientations here to match your robot. See the FTC SDK documentation for
        // details
        val parameters: BNO055IMU.Parameters = BNO055IMU.Parameters(
            RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
            )
        )
        imu.initialize(parameters)

        // add/remove motors depending on your robot (e.g., 6WD)
        val leftFront = hardwareMap.get(DcMotorEx::class.java, "leftFront")
        val leftRear = hardwareMap.get(DcMotorEx::class.java, "leftRear")
        val rightRear = hardwareMap.get(DcMotorEx::class.java, "rightRear")
        val rightFront = hardwareMap.get(DcMotorEx::class.java, "rightFront")
        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront)
        leftMotors = Arrays.asList(leftFront, leftRear)
        rightMotors = Arrays.asList(rightFront, rightRear)
        for (motor in motors) {
            val motorConfigurationType = motor.motorType.clone()
            motorConfigurationType.achieveableMaxRPMFraction = 1.0
            motor.motorType = motorConfigurationType
        }
        if (DriveConstants.RUN_USING_ENCODER) {
            setMode(RunMode.RUN_USING_ENCODER)
        }
        setZeroPowerBehavior(ZeroPowerBehavior.BRAKE)
        if (DriveConstants.RUN_USING_ENCODER && DriveConstants.MOTOR_VELO_PID != null) {
            setPIDFCoefficients(RunMode.RUN_USING_ENCODER, DriveConstants.MOTOR_VELO_PID)
        }

        // TODO: reverse any motors using DcMotor.setDirection()

        // TODO: if desired, use setLocalizer() to change the localization method
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));
        trajectorySequenceRunner = TrajectorySequenceRunner(follower, HEADING_PID)
    }

    fun trajectoryBuilder(startPose: Pose2d?): TrajectoryBuilder {
        return TrajectoryBuilder(startPose, VEL_CONSTRAINT, accelConstraint)
    }

    fun trajectoryBuilder(startPose: Pose2d?, reversed: Boolean): TrajectoryBuilder {
        return TrajectoryBuilder(startPose!!, reversed, VEL_CONSTRAINT, accelConstraint)
    }

    fun trajectoryBuilder(startPose: Pose2d?, startHeading: Double): TrajectoryBuilder {
        return TrajectoryBuilder(startPose!!, startHeading, VEL_CONSTRAINT, accelConstraint)
    }

    fun trajectorySequenceBuilder(startPose: Pose2d?): TrajectorySequenceBuilder {
        return TrajectorySequenceBuilder(
            startPose,
            VEL_CONSTRAINT, accelConstraint,
            DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL
        )
    }

    fun turnAsync(angle: Double) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
            trajectorySequenceBuilder(poseEstimate)
                .turn(angle)
                .build()
        )
    }

    fun turn(angle: Double) {
        turnAsync(angle)
        waitForIdle()
    }

    fun followTrajectoryAsync(trajectory: Trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
            trajectorySequenceBuilder(trajectory.start())
                .addTrajectory(trajectory)
                .build()
        )
    }

    fun followTrajectory(trajectory: Trajectory) {
        followTrajectoryAsync(trajectory)
        waitForIdle()
    }

    fun followTrajectorySequenceAsync(trajectorySequence: TrajectorySequence?) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence)
    }

    fun followTrajectorySequence(trajectorySequence: TrajectorySequence?) {
        followTrajectorySequenceAsync(trajectorySequence)
        waitForIdle()
    }

    val lastError: Pose2d
        get() = trajectorySequenceRunner.lastPoseError

    fun update() {
        updatePoseEstimate()
        val signal = trajectorySequenceRunner.update(poseEstimate, poseVelocity)
        signal?.let { setDriveSignal(it) }
    }

    fun waitForIdle() {
        while (!Thread.currentThread().isInterrupted && isBusy) update()
    }

    val isBusy: Boolean
        get() = trajectorySequenceRunner.isBusy

    fun setMode(runMode: RunMode?) {
        for (motor in motors) {
            motor.mode = runMode
        }
    }

    fun setZeroPowerBehavior(zeroPowerBehavior: ZeroPowerBehavior?) {
        for (motor in motors) {
            motor.zeroPowerBehavior = zeroPowerBehavior
        }
    }

    fun setPIDFCoefficients(runMode: RunMode?, coefficients: PIDFCoefficients) {
        val compensatedCoefficients = PIDFCoefficients(
            coefficients.p, coefficients.i, coefficients.d,
            coefficients.f * 12 / batteryVoltageSensor.voltage
        )
        for (motor in motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients)
        }
    }

    fun setWeightedDrivePower(drivePower: Pose2d) {
        var vel = drivePower
        vel = if (Math.abs(drivePower.x) + Math.abs(drivePower.heading) > 1) {
            // re-normalize the powers according to the weights
            val denom = (VX_WEIGHT * Math.abs(drivePower.x)
                    + OMEGA_WEIGHT * Math.abs(drivePower.heading))
            Pose2d(
                VX_WEIGHT * drivePower.x,
                0,
                OMEGA_WEIGHT * drivePower.heading
            ).div(denom)
        } else {
            // Ensure the y axis is zeroed out.
            Pose2d(drivePower.x, 0, drivePower.heading)
        }
        setDrivePower(vel)
    }

    override fun getWheelPositions(): List<Double> {
        var leftSum = 0.0
        var rightSum = 0.0
        for (leftMotor in leftMotors) {
            leftSum += DriveConstants.encoderTicksToInches(leftMotor.currentPosition.toDouble())
        }
        for (rightMotor in rightMotors) {
            rightSum += DriveConstants.encoderTicksToInches(rightMotor.currentPosition.toDouble())
        }
        return Arrays.asList(leftSum / leftMotors.size, rightSum / rightMotors.size)
    }

    override fun getWheelVelocities(): List<Double>? {
        var leftSum = 0.0
        var rightSum = 0.0
        for (leftMotor in leftMotors) {
            leftSum += DriveConstants.encoderTicksToInches(leftMotor.velocity)
        }
        for (rightMotor in rightMotors) {
            rightSum += DriveConstants.encoderTicksToInches(rightMotor.velocity)
        }
        return Arrays.asList(leftSum / leftMotors.size, rightSum / rightMotors.size)
    }

    override fun setMotorPowers(v: Double, v1: Double) {
        for (leftMotor in leftMotors) {
            leftMotor.power = v
        }
        for (rightMotor in rightMotors) {
            rightMotor.power = v1
        }
    }

    override val rawExternalHeading: Double
        get() = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)

    override fun getExternalHeadingVelocity(): Double? {
        return imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate
    }

    companion object {
        var AXIAL_PID = PIDCoefficients(0, 0, 0)
        var CROSS_TRACK_PID = PIDCoefficients(0, 0, 0)
        var HEADING_PID = PIDCoefficients(0, 0, 0)
        var VX_WEIGHT = 1.0
        var OMEGA_WEIGHT = 1.0
        private val VEL_CONSTRAINT =
            getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH)
        private val accelConstraint = getAccelerationConstraint(DriveConstants.MAX_ACCEL)
        fun getVelocityConstraint(
            maxVel: Double,
            maxAngularVel: Double,
            trackWidth: Double
        ): TrajectoryVelocityConstraint {
            return MinVelocityConstraint(
                Arrays.asList(
                    AngularVelocityConstraint(maxAngularVel),
                    TankVelocityConstraint(maxVel, trackWidth)
                )
            )
        }

        fun getAccelerationConstraint(maxAccel: Double): TrajectoryAccelerationConstraint {
            return ProfileAccelerationConstraint(maxAccel)
        }
    }
}