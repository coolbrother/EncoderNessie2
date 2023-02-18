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

        Pose2d startPose = new Pose2d(36, -63.5, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence ts = drive.trajectorySequenceBuilder(startPose)
//                .lineToConstantHeading(new Vector2d(60, -60))
//                .lineToLinearHeading(new Pose2d(60, -12, Math.toRadians(180)))
//                .splineTo(new Vector2d(50, -10), Math.toRadians(150))
//                .splineTo(new Vector2d(60, -12), Math.toRadians(180))
//                .lineToConstantHeading(new Vector2d(36, -12))
//                .lineToConstantHeading(new Vector2d(36, -36))
                .lineTo(new Vector2d(36, -12))
                .waitSeconds(2)
                .turn(Math.toRadians(90))
                .waitSeconds(2)
                .lineTo(new Vector2d(60, -12))
                .waitSeconds(2)
//                .splineTo(new Vector2d(48, -12), Math.toRadians(150))
//                .splineTo(new Vector2d(60, -12), Math.toRadians(180))
                .lineTo(new Vector2d(36, -12))
                .waitSeconds(2)
                .turn(Math.toRadians(90))
                .waitSeconds(2)
                .lineTo(new Vector2d(36, -36))
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectorySequence(ts);
    }
}
