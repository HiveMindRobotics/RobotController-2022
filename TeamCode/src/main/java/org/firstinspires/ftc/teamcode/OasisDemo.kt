package org.firstinspires.ftc.teamcode

import com.oasislang.oasis.Interpreter
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.oasislang.oasis.run

@TeleOp(name = "Driver Control", group = "Linear Opmode")
class OasisDemo : LinearOpMode() {
    override fun runOpMode() {
        waitForStart()
        run("while true io:print(\"foo!\")", Interpreter())
    }
}