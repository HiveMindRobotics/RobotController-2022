package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive





@Autonomous(name = "Roadrunner Auto", group = "Linear Opmode")
class RoadrunnerAuto : LinearOpMode() {
    override fun runOpMode() {
        val drive = SampleMecanumDrive(hardwareMap)

        val myTrajectory: Trajectory = drive.trajectoryBuilder(Pose2d(), 0.0)
            .strafeRight(10.0)
            .forward(5.0)
            .build()

        waitForStart()

        if (isStopRequested) return
    }

}