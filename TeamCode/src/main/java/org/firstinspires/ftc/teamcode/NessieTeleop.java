/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
 // IM DEPRESSED IRL
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import java.lang.Math;


@TeleOp(name="NessieTeleop")
//@Disabled
public class NessieTeleop extends LinearOpMode {
    
    enum SlidePackDirection {
        UP,
        DOWN
    }

    enum PoleHeight {
        HIGH,
        MEDIUM,
        LOW,
        GROUND
    }

    private DcMotor FLMotor;
    private DcMotor FRMotor;
    private DcMotor BLMotor;
    private DcMotor BRMotor;
    private Servo Finger;
    private CRServo Spinner;
    private CRServo ElbowL;
    private CRServo ElbowR;
    private DcMotor VerticalSlidePackL;
    private DcMotor VerticalSlidePackR;
    private double drive;
    private double turn;
    private final double DriveSpeed = 0.9;
    private final double SlidePackSpeed = 0.7;
    private final double FingerGrabPosition = 0.0;
    private final double FingerReleasePosition = 0.23;
    private final double SpinnerForwardPosition = 0.25;
    private final double SpinnerBackwardPosition = 0.91;
    private final double ElbowForwardPosition = 0.0;
    private final double ElbowBackwardPosition = 0.0;
    private PoleHeight CurrentPoleHeight = PoleHeight.GROUND;
    private final double BATTERY_LEVEL = 1;
    private ElapsedTime eTime = new ElapsedTime();

    @Override
    public void runOpMode () {
        FLMotor = hardwareMap.dcMotor.get("1");
        FRMotor = hardwareMap.dcMotor.get("0");
        BLMotor = hardwareMap.dcMotor.get("2");
        BRMotor = hardwareMap.dcMotor.get("3");
        Finger = hardwareMap.servo.get("FG");
        Spinner = hardwareMap.crservo.get("SP");
        ElbowL = hardwareMap.crservo.get("EL");
        ElbowR = hardwareMap.crservo.get("ER");
        VerticalSlidePackL = hardwareMap.dcMotor.get("VSPL");
        VerticalSlidePackR = hardwareMap.dcMotor.get("VSPR");

        // Set Directions
        FLMotor.setDirection(DcMotor.Direction.REVERSE);
        FRMotor.setDirection(DcMotor.Direction.FORWARD);
        BLMotor.setDirection(DcMotor.Direction.REVERSE);
        BRMotor.setDirection(DcMotor.Direction.FORWARD);
        Finger.setDirection(Servo.Direction.FORWARD);
        Spinner.setDirection(CRServo.Direction.FORWARD);
        VerticalSlidePackL.setDirection(DcMotor.Direction.FORWARD);
        VerticalSlidePackR.setDirection(DcMotor.Direction.REVERSE);

        VerticalSlidePackL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        VerticalSlidePackR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // wait for the coach to press start
        waitForStart();
        telemetry.addData("Status","TeleOp");
        telemetry.update();
        
        boolean OldFingerPushed = false;
        boolean OldSpinnerPushed = false;
        boolean currentDirectionForward = false;

        while(opModeIsActive()) {
            telemetry.addData("currentDirectionForward", currentDirectionForward);

            //Driver 1
            drive = -gamepad1.left_stick_y;
            
            if (!currentDirectionForward) {
                drive *= -1;
            }
            
            turn = gamepad1.right_stick_x;
            
            double LeftDrive = Range.clip(drive + turn, -1.0, 1.0);
            double RightDrive = Range.clip(drive - turn, -1.0, 1.0);

            telemetry.addData("left_stick_y", LeftDrive);
            telemetry.addData("right_stick_y", RightDrive);
            telemetry.addData("x", gamepad1.left_stick_x);
            telemetry.addData("y", gamepad1.left_stick_y);

            if (LeftDrive > 0.9) {
                LeftDrive = DriveSpeed;
            } else if (LeftDrive > 0) {
                LeftDrive = 0.3 * DriveSpeed;
            } else if (LeftDrive < -0.9) {
                LeftDrive = -DriveSpeed;
            } else if (LeftDrive < 0) {
                LeftDrive = 0.3 * -DriveSpeed;
            } else {
                LeftDrive = 0;
            }

            if (RightDrive > 0.9) {
                RightDrive = DriveSpeed;
            } else if (RightDrive > 0) {
                RightDrive = 0.3 * DriveSpeed;
            } else if (RightDrive < -0.9) {
                RightDrive = -DriveSpeed;
            } else if (RightDrive < 0) {
                RightDrive = 0.3 * -DriveSpeed;
            } else {
                RightDrive = 0;
            }
            
            double LeftStrafe = gamepad1.left_trigger;
            double RightStrafe = gamepad1.right_trigger;
            
            if (!currentDirectionForward) {
                LeftStrafe = gamepad1.right_trigger;
                RightStrafe = gamepad1.left_trigger;
            }

            // THE CLAW
            double VerticalSlidePackForward = -gamepad2.left_stick_y * SlidePackSpeed;
//            double VerticalSlidePackBackward = gamepad2.dpad_down ? -1 : 0;
            boolean FingerPushed = gamepad2.a;
            // boolean FingerOut = gamepad2.b;
            // double SpinnerForward = -gamepad2.right_stick_y;
            boolean SpinnerPushed = gamepad2.x;
            boolean GroundPoleHeight = gamepad2.dpad_down;
            boolean LowPoleHeight = gamepad2.dpad_left;
            boolean MediumPoleHeight = gamepad2.dpad_right;
            boolean HighPoleHeight = gamepad2.dpad_up;
            
            boolean temp1 = isWithinRange(Finger.getPosition(), FingerGrabPosition, 0.01);

            if (FingerPushed != OldFingerPushed && FingerPushed) {
//                 FingerL.getController().setServoPosition(FingerL.getPortNumber(), FingerIn ? FingerLGrabPosition : FingerLReleasePosition);
                Finger.setPosition(temp1 ? FingerReleasePosition : FingerGrabPosition);
            }
            
            boolean temp2 = isWithinRange(Spinner.getController().getServoPosition(Spinner.getPortNumber()), SpinnerForwardPosition, 0.2);
            
            if (SpinnerPushed != OldSpinnerPushed && SpinnerPushed) {
                currentDirectionForward = !currentDirectionForward;
                Spinner.getController().setServoPosition(Spinner.getPortNumber(), temp2 ? SpinnerBackwardPosition : SpinnerForwardPosition);
            }
            
            if (GroundPoleHeight) {
                moveSlidePackToPosition(CurrentPoleHeight, PoleHeight.GROUND);
            } else if (LowPoleHeight) {
                moveSlidePackToPosition(CurrentPoleHeight, PoleHeight.LOW);
            } else if (MediumPoleHeight) {
                moveSlidePackToPosition(CurrentPoleHeight, PoleHeight.MEDIUM);
            } else if (HighPoleHeight) {
                moveSlidePackToPosition(CurrentPoleHeight, PoleHeight.HIGH);
            }
            
            VerticalSlidePackL.setPower(VerticalSlidePackForward);
            VerticalSlidePackR.setPower(VerticalSlidePackForward);
            
            if (LeftStrafe == 0 && RightStrafe == 0) {
                FLMotor.setPower(LeftDrive);
                BLMotor.setPower(LeftDrive);
                FRMotor.setPower(RightDrive);
                BRMotor.setPower(RightDrive);
            } else if (LeftStrafe > 0) {
                if (LeftStrafe > 0.9) {
                    FLMotor.setPower(1);
                    BLMotor.setPower(-1);
                    FRMotor.setPower(-1);
                    BRMotor.setPower(1);
                } else {
                    FLMotor.setPower(0.3);
                    BLMotor.setPower(-0.3);
                    FRMotor.setPower(-0.3);
                    BRMotor.setPower(0.3);
                }
            } else if (RightStrafe > 0) {
                if (RightStrafe > 0.9) {
                    FLMotor.setPower(-1);
                    BLMotor.setPower(1);
                    FRMotor.setPower(1);
                    BRMotor.setPower(-1);
                } else {
                    FLMotor.setPower(-0.3);
                    BLMotor.setPower(0.3);
                    FRMotor.setPower(0.3);
                    BRMotor.setPower(-0.3);
                }
            }


            // add random telemetry stuff (shows up on driver station app) cuz why not
            telemetry.addData("LeftDrive", LeftDrive);
            telemetry.addData("RightDrive", RightDrive);
//             telemetry.addData("Flywheel", FlywheelCounterClockwise + FlywheelClockwise);
//            telemetry.addData("HorizontalSlidePack", -HorizontalSlidePackBackward + HorizontalSlidePackForward);
            telemetry.addData("VerticalSlidePack", VerticalSlidePackForward);
            telemetry.addData("FingerIn", FingerPushed);
            // telemetry.addData("FingerOut", FingerOut);
            telemetry.addData("CurPolePosition", CurrentPoleHeight);
            telemetry.addData("GroundPoleHeight", GroundPoleHeight);
            telemetry.addData("LowPoleHeight", LowPoleHeight);
            telemetry.addData("MediumPoleHeight", MediumPoleHeight);
            telemetry.addData("HighPoleHeight", HighPoleHeight);
            telemetry.addData("SpinnerPushed", SpinnerPushed);
            telemetry.addData("SpinnerPosition", Spinner.getController().getServoPosition(Spinner.getPortNumber()));
            telemetry.update();
            OldFingerPushed = FingerPushed;
            OldSpinnerPushed = SpinnerPushed;
        }
    }
    
    private boolean isWithinRange(double a, double b, double c) {
        return Math.abs(a - b) <= c;
    }
    
    private void moveSlidePackToPosition(PoleHeight curPoleHeight, PoleHeight targetPoleHeight) {
        int timeToMove = getMoveTimeOfSlidePack(curPoleHeight, targetPoleHeight);
        // VerticalSlidePack.setTargetPosition(getMoveTimeOfSlidePack(curPoleHeight, targetPoleHeight));
        // VerticalSlidePack.setPower(1);
        if (timeToMove >= 0)
            moveSlidePack(SlidePackDirection.UP, getDrivePower(SlidePackSpeed), timeToMove);
        else {
            timeToMove *= 0.8;
            moveSlidePack(SlidePackDirection.DOWN, getDrivePower(SlidePackSpeed), -timeToMove);
        }
        CurrentPoleHeight = targetPoleHeight;
        telemetry.addData("Moving To", targetPoleHeight);
        telemetry.update();
    }
    
    private int getMoveTimeOfSlidePack(PoleHeight curPoleHeight, PoleHeight targetPoleHeight) {
        telemetry.addData("target pole height", convertPoleHeightToMs(targetPoleHeight));
        telemetry.addData("cur pole height", convertPoleHeightToMs(curPoleHeight));
        return convertPoleHeightToMs(targetPoleHeight) - convertPoleHeightToMs(curPoleHeight);
    }

    private int convertPoleHeightToMs(PoleHeight ph) {
        switch (ph) {
            case HIGH:
                return 3600;
            case MEDIUM:
                return 2400;
            case LOW:
                return 1200;
            case GROUND:
                return 0;
            default:
                return 0;
        }
    }
    
    private double getDrivePower(double power) {
        return power * (1 + (0.1 - BATTERY_LEVEL * 0.1));
    }
    
    private void moveSlidePack(SlidePackDirection spd, double power, double time) {
        eTime.reset();
        switch(spd) {
            case UP:
                VerticalSlidePackL.setPower(power);
                VerticalSlidePackR.setPower(power);
                break;
            case DOWN:
                VerticalSlidePackL.setPower(-power);
                VerticalSlidePackR.setPower(-power);
                break;
        }
        while(opModeIsActive() && eTime.milliseconds() < time){
            telemetry.addData("Time:", eTime);
            telemetry.update();
        }
        VerticalSlidePackL.setPower(0);
        VerticalSlidePackR.setPower(0);
    }

}
