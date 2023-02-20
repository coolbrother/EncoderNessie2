package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Timer;
import java.util.TimerTask;

@Autonomous(name="EncoderNessieAuto")
public class EncoderNessieAuto extends LinearOpMode {
    class lowerArmToLowPosition extends TimerTask {
        public void run() {
            ElbowL.getController().setServoPosition(ElbowL.getPortNumber(), ElbowLBackwardPosition );
            ElbowR.getController().setServoPosition(ElbowR.getPortNumber(), ElbowRBackwardPosition );

            telemetry.addData("AAAAA", 3);
            telemetry.update();
//                sleep(5000);
        }
    }

    class lowerArmToMediumPosition extends TimerTask {
        public void run() {

            ElbowL.getController().setServoPosition(ElbowL.getPortNumber(), ElbowLBackwardPosition - 0.05);
            ElbowR.getController().setServoPosition(ElbowR.getPortNumber(), ElbowRBackwardPosition + 0.05);
        }
    }

    class lowerArmToHighPosition extends TimerTask {
        public void run() {
            ElbowL.getController().setServoPosition(ElbowL.getPortNumber(), ElbowLBackwardPosition - 0.1);
            ElbowR.getController().setServoPosition(ElbowR.getPortNumber(), ElbowRBackwardPosition + 0.1);

            telemetry.addData("AAAAA", 3);
            telemetry.update();
        }
    }

    class closeClaw extends TimerTask {
        public void run() {
            Finger.setPosition(FingerGrabPosition);
//                sleep(5000);
        }
    }

    class openClaw extends TimerTask {
        public void run() {
            Finger.setPosition(FingerReleasePosition);
//                sleep(5000);
        }
    }

    private Servo Finger;
    private CRServo Spinner;
    private CRServo ElbowL;
    private CRServo ElbowR;
    private DcMotor VerticalSlidePackL;
    private DcMotor VerticalSlidePackR;
    private final double SlidePackSpeed = 1.0;
    private final double FingerReleasePosition = 0.61;
    private final double FingerGrabPosition = 0.65;
    private final double SpinnerForwardPosition = .6;//0.9;
    private final double SpinnerBackwardPosition = .05; //0.35;
    private final double SpinnerIntermediatePosition = .78; //0.68;
    //    private final double SpinnerGrabbingPosition = 1.0;
    private final double ElbowLForwardPosition = 0.15;
    private final double ElbowLBackwardPosition = 0.95;
    private final double ElbowLIntermediatePosition = 0.39;
    private final double ElbowRForwardPosition = 0.81;
    private final double ElbowRBackwardPosition = 0.05;
    private final double ElbowRIntermediatePosition = 0.575;
    private NessieTeleop.PoleHeight CurrentPoleHeight = NessieTeleop.PoleHeight.GROUND;
    private NessieTeleop.FingerHeight CurrentFingerHeight = NessieTeleop.FingerHeight.LOW;
    private Timer timer = new Timer();
    private ElapsedTime eTime = new ElapsedTime();
    private final double ANGLE_1 = 0.46;

    @Override
    public void runOpMode() {
        Finger = hardwareMap.servo.get("FG");
        Spinner = hardwareMap.crservo.get("SP");
        ElbowL = hardwareMap.crservo.get("EL");
        ElbowR = hardwareMap.crservo.get("ER");
        VerticalSlidePackL = hardwareMap.dcMotor.get("VSPL");
        VerticalSlidePackR = hardwareMap.dcMotor.get("VSPR");

        // Set Directions
        Finger.setDirection(Servo.Direction.FORWARD);
        Spinner.setDirection(CRServo.Direction.FORWARD);
        VerticalSlidePackL.setDirection(DcMotor.Direction.REVERSE);
        VerticalSlidePackR.setDirection(DcMotor.Direction.FORWARD);

        VerticalSlidePackL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        VerticalSlidePackR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(36, -63.5, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence ts = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    Finger.setPosition(FingerGrabPosition);
                    Spinner.getController().setServoPosition(Spinner.getPortNumber(), SpinnerIntermediatePosition);
                    ElbowL.getController().setServoPosition(ElbowL.getPortNumber(), ElbowLIntermediatePosition);
                    ElbowR.getController().setServoPosition(ElbowR.getPortNumber(), ElbowRIntermediatePosition);
                })
                .lineTo(new Vector2d(36, -14))
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(48, -14))
                .turn(ANGLE_1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    moveSlidePack(NessieTeleop.SlidePackDirection.UP, SlidePackSpeed, 1700);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    Spinner.getController().setServoPosition(Spinner.getPortNumber(), SpinnerForwardPosition);
                    ElbowL.getController().setServoPosition(ElbowL.getPortNumber(), ElbowLForwardPosition);
                    ElbowR.getController().setServoPosition(ElbowR.getPortNumber(), ElbowRForwardPosition);
                })
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    timer.schedule(new openClaw(), 0);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.5, () -> {
                    timer.schedule(new lowerArmToLowPosition(), 0);
                    Spinner.getController().setServoPosition(Spinner.getPortNumber(), SpinnerBackwardPosition);
                    moveSlidePack(NessieTeleop.SlidePackDirection.DOWN, SlidePackSpeed, 1700);
                })
                .waitSeconds(2.5)
                .turn(-ANGLE_1)
                .lineTo(new Vector2d(59.5, -14))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    timer.schedule(new closeClaw(), 0);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    ElbowL.getController().setServoPosition(ElbowL.getPortNumber(), ElbowLIntermediatePosition);
                    ElbowR.getController().setServoPosition(ElbowR.getPortNumber(), ElbowRIntermediatePosition);
                })
                .waitSeconds(1)
                .lineTo(new Vector2d(48, -14))
                .turn(ANGLE_1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    moveSlidePack(NessieTeleop.SlidePackDirection.UP, SlidePackSpeed, 1700);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    Spinner.getController().setServoPosition(Spinner.getPortNumber(), SpinnerForwardPosition);
                    ElbowL.getController().setServoPosition(ElbowL.getPortNumber(), ElbowLForwardPosition);
                    ElbowR.getController().setServoPosition(ElbowR.getPortNumber(), ElbowRForwardPosition);
                })
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    timer.schedule(new openClaw(), 0);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.5, () -> {
                    timer.schedule(new lowerArmToLowPosition(), 0);
                    Spinner.getController().setServoPosition(Spinner.getPortNumber(), SpinnerBackwardPosition);
                    moveSlidePack(NessieTeleop.SlidePackDirection.DOWN, SlidePackSpeed, 1700);
                })
                .waitSeconds(2.5)
                .turn(-ANGLE_1)
                .build();

        waitForStart();

        if(isStopRequested()) return;

        Finger.setPosition(FingerGrabPosition);
        VerticalSlidePackL.setPower(0.05);
        VerticalSlidePackR.setPower(0.05);

        drive.followTrajectorySequence(ts);
    }

    private void moveSlidePack(NessieTeleop.SlidePackDirection spd, double power, double time) {
        eTime.reset();
        switch (spd) {
            case UP:
                VerticalSlidePackL.setPower(power);
                VerticalSlidePackR.setPower(power);
                break;
            case DOWN:
                VerticalSlidePackL.setPower(-power);
                VerticalSlidePackR.setPower(-power);
                break;
        }
        while (opModeIsActive() && eTime.milliseconds() < time) {
            telemetry.addData("Time:", eTime);
            telemetry.update();
        }
        VerticalSlidePackL.setPower(0.05);
        VerticalSlidePackR.setPower(0.05);
    }
}
