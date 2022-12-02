package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap

/**
 * the linear slide is required to start fully compressed for this to work
 */
class LinearSlide(hwMap: HardwareMap) {
    private var motorLinearSlide: DcMotorEx

    enum class LinearSlideStage {
        Bottom,
        First,
        Second,
        Third
    }

    @Config
    object LinearSlideConstants {
        @JvmField var FIRST_STAGE_TICKS = 0
        @JvmField var SECOND_STAGE_TICKS = 0
        @JvmField var THIRD_STAGE_TICKS = 0
    }

    /**
     * control how far extended the linear slide is based on LinearSlideStage
     */
    var stage = LinearSlideStage.Bottom
        set(value) {
            when (value) {
                LinearSlideStage.Bottom -> motorLinearSlide.targetPosition = 0
                LinearSlideStage.First -> motorLinearSlide.targetPosition = LinearSlideConstants.FIRST_STAGE_TICKS
                LinearSlideStage.Second -> motorLinearSlide.targetPosition = LinearSlideConstants.SECOND_STAGE_TICKS
                LinearSlideStage.Third -> motorLinearSlide.targetPosition = LinearSlideConstants.THIRD_STAGE_TICKS
            }
            field = value
        }

    /**
     * speed at which the linear slide moves to each stage
     */
    var speed = 0.0
        set(value) {
            motorLinearSlide.power = speed
            field = value
        }

    init {
        motorLinearSlide = hwMap.get(DcMotorEx::class.java, "motor2")
        motorLinearSlide.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motorLinearSlide.targetPosition = 0
        motorLinearSlide.mode = DcMotor.RunMode.RUN_TO_POSITION
        motorLinearSlide.power = 0.3
    }
}