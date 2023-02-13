package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

@Autonomous(name="EncoderNessieAuto")
public class EncoderNessieAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-36, 60, Math.toRadians(270));

        TrajectorySequence ts = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(0, -24), Math.toRadians(270))
//                .splineTo(new Vector2d(12, -48), Math.toRadians(0))
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectorySequence(ts);
    }
}
