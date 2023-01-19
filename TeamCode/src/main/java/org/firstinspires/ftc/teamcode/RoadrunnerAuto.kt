package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive


@Autonomous(name = "Roadrunner Auto", group = "Linear Opmode")
class RoadrunnerAuto : LinearOpMode() {
    override fun runOpMode() {
        val drive = SampleTankDrive(hardwareMap)

        val myTrajectory: Trajectory = drive.trajectoryBuilder(Pose2d(), 0.0)
            .forward(10.0)
            .back(10.0)
            .strafeLeft(10.0)
            .strafeRight(10.0)
            .build()

        waitForStart()

        drive.followTrajectory(myTrajectory)

        if (isStopRequested) return
    }

}