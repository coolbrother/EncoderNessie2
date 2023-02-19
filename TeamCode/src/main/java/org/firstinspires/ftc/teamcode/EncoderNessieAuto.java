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
                .lineTo(new Vector2d(36, -14))
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(48, -14))
                .turn(0.433647607)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    Spinner.getController().setServoPosition(Spinner.getPortNumber(), SpinnerForwardPosition);
                    ElbowL.getController().setServoPosition(ElbowL.getPortNumber(), ElbowLForwardPosition);
                    ElbowR.getController().setServoPosition(ElbowR.getPortNumber(), ElbowRForwardPosition);
                    moveSlidePack(NessieTeleop.SlidePackDirection.UP, SlidePackSpeed, 1500);
                })
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    timer.schedule(new openClaw(), 0);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.2, () -> {
                    timer.schedule(new lowerArmToHighPosition(), 0);
                    Spinner.getController().setServoPosition(Spinner.getPortNumber(), SpinnerBackwardPosition);
                    moveSlidePack(NessieTeleop.SlidePackDirection.DOWN, SlidePackSpeed, 1400);
                })
                .waitSeconds(2.2)
                .turn(-0.433647607)
                .lineTo(new Vector2d(57.5, -14))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    timer.schedule(new closeClaw(), 0);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    ElbowL.getController().setServoPosition(ElbowL.getPortNumber(), ElbowLIntermediatePosition);
                    ElbowR.getController().setServoPosition(ElbowR.getPortNumber(), ElbowRIntermediatePosition);
                })
                .waitSeconds(1)
                .lineTo(new Vector2d(48, -14))
                .turn(0.433647607)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    Spinner.getController().setServoPosition(Spinner.getPortNumber(), SpinnerForwardPosition);
                    ElbowL.getController().setServoPosition(ElbowL.getPortNumber(), ElbowLForwardPosition);
                    ElbowR.getController().setServoPosition(ElbowR.getPortNumber(), ElbowRForwardPosition);
                    moveSlidePack(NessieTeleop.SlidePackDirection.UP, SlidePackSpeed, 1100);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.8, () -> {
                    timer.schedule(new openClaw(), 0);
                })
                .UNSTABLE_addTemporalMarkerOffset(3, () -> {
                    timer.schedule(new lowerArmToHighPosition(), 0);
                    Spinner.getController().setServoPosition(Spinner.getPortNumber(), SpinnerBackwardPosition);
                    moveSlidePack(NessieTeleop.SlidePackDirection.DOWN, SlidePackSpeed, 1400);
                })
                .waitSeconds(3)
                .turn(-0.433647607)
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
