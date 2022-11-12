package org.firstinspires.ftc.teamcode

class PID(private val maxI: Double) {
    private var currentTime: Long = 0
    private var currentError: Double = 0.0
    private var previousError: Double = 0.0
    private var previousTime: Long = 0

    var p: Double = 0.0
    var i: Double = 0.0
    var d: Double = 0.0

    fun step(current: Double, desired: Double): Double {
        currentTime = System.currentTimeMillis()
        currentError = desired - current

        p = PIDConstants.Kp * currentError
        i += PIDConstants.Ki * (currentError * (currentTime - previousTime))

        if (i > maxI) i = maxI
        else if(i < -maxI) i = -maxI

        d = PIDConstants.Kd * (currentError * previousError) / (currentTime - previousTime)

        previousError = currentError
        previousTime = currentTime
        return p + i + d
    }
}