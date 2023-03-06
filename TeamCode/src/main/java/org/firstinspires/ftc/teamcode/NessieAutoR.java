package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.vuforia.Image;
import com.vuforia.Vuforia;
import java.nio.ByteBuffer;

import java.util.List;


@Autonomous(name="NessieAutoRIGHT")
@Disabled
public class NessieAutoR extends LinearOpMode {

    enum DriveDirection {
        FORWARD,
        LEFT,
        RIGHT,
        BACKWARD
    }

    enum StartingPositionEnum {
        LEFT,
        RIGHT
    }

    enum SlidePackDirection {
        UP,
        DOWN
    }

    enum ParkingSpace {
        UNO,
        DOS,
        TRES
    }

    enum PoleHeight {
        HIGH,
        MEDIUM,
        LOW,
        GROUND,
    }

    private final StartingPositionEnum STARTING_POSITION = StartingPositionEnum.RIGHT;
    private final int numberOfRowsToScanInImage = 30;
    private final double BATTERY_LEVEL = 1;
    private final double Multiplier = (435.0 / 2) / 312;
    private final double DrivePower = 0.85 * 0.75;
    private final double SlidePackPower = 0.75;
    private final double GrabberLGrabPosition = 0.3;
    private final double GrabberLReleasePosition = 0.64;
    private final double GrabberRGrabPosition = 0.0;
    private final double GrabberRReleasePosition = 0.23;
    private final double SpinnerForwardPosition = 0.25;
    private final double SpinnerBackwardPosition = 0.91;
    private final boolean useRegularFunctions = false;
    private final boolean usePlanB = false;
    private ParkingSpace parkingSpace = ParkingSpace.UNO;
    private PoleHeight CurrentPoleHeight = PoleHeight.GROUND;

    private DcMotor FLMotor;
    private DcMotor FRMotor;
    private DcMotor BLMotor;
    private DcMotor BRMotor;
    private DcMotor Flywheel;
    private CRServo GrabberL;
    private Servo GrabberR;
    private CRServo Spinner;
    private DcMotor VerticalSlidePack;
    // private DcMotor HorizontalSlidePack;
    // private DcMotor VerticalSlidePack;
    // private DcMotor EaterMotor;
    private Blinker expansion_Hub_4;
    DigitalChannel touch;
    BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;
    private ElapsedTime eTime = new ElapsedTime();
    private static final String VUFORIA_KEY = "AVPRW+T/////AAABmYg0Njwhc0n/teI+7Sz8f/Baxyp0o6W48fBEflz8RZs3G/bVjI/5PyebGV6SkXhE1unHTRVzOVCo2cuuePhML8YCeHWm1dHZ2KbshLfc/yne7rfe2VaKPR3rrJXPF5CdMTWj4nTxm6w7KxiqvtvF2p2si1FrculcXUwbHeZ9X3O6VSntXMuNJDxXJEC3O5hT5kb7ZzsSWlot9YfUqJRxttrYYz8Xu1D2IhtOs26a2A9FC8afgGouyHucBDfl+WP59+H6wYaXRbyvcFdytq9Fp7mlSsA9RA6DV70PtJWDmehLO5hhOKq4ihVNCjJcgG38UefDAyDhWWMdjRwsiaVctq6QkmG1oMuTfIF1Dun2lDpZ";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() throws InterruptedException
    {
        //Maps hardware for all parts
        FLMotor = hardwareMap.dcMotor.get("1");
        FRMotor = hardwareMap.dcMotor.get("0");
        BLMotor = hardwareMap.dcMotor.get("2");
        BRMotor = hardwareMap.dcMotor.get("3");
        // Flywheel = hardwareMap.dcMotor.get("Fly");
        // GrabberL = hardwareMap.crservo.get("GL");
//        GrabberR = hardwareMap.servo.get("GR");
//        VerticalSlidePack = hardwareMap.dcMotor.get("VSP");
//        Spinner = hardwareMap.crservo.get("SP");

        FLMotor.setDirection(DcMotor.Direction.FORWARD);
        FRMotor.setDirection(DcMotor.Direction.REVERSE);
        BLMotor.setDirection(DcMotor.Direction.FORWARD);
        BRMotor.setDirection(DcMotor.Direction.REVERSE);
        // Flywheel.setDirection(DcMotor.Direction.FORWARD);
        // GrabberL.setDirection(CRServo.Direction.FORWARD);
//        GrabberR.setDirection(Servo.Direction.REVERSE);
//        VerticalSlidePack.setDirection(DcMotor.Direction.REVERSE);
//        Spinner.setDirection(CRServo.Direction.FORWARD);

        telemetry.addData("Status","Auto");
        telemetry.update();

        //Configures settings for different parts
        FLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        VerticalSlidePack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Flywheel.setMode(DcMotor.RunMode.RUN);
//        VerticalSlidePack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Mode", "waiting for start");
        telemetry.update();

        boolean isCameraReady = getCameraReady();

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        // wait for start button.
        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        sleep(50);
        //The actual program
        eTime.reset();

        if (isCameraReady) {
            parkingSpace = getCameraReading();
        }

        telemetry.addData("parkingSpace", parkingSpace);
        telemetry.update();

        switch (STARTING_POSITION) {
            case LEFT:
                doActions(StartingPositionEnum.LEFT, parkingSpace);
                break;
            case RIGHT:
                doActions(StartingPositionEnum.RIGHT, parkingSpace);
                break;
            default:
                break;
        }
        // sleep(5000);
    }

    private void doActions(StartingPositionEnum position, ParkingSpace ps) {
        // ps = ParkingSpace.UNO;
        boolean needInvert = (position != StartingPositionEnum.LEFT);

        driveTiles(DriveDirection.FORWARD, getDrivePower(DrivePower), 0.4);
        telemetry.addData("Forward", "");
        telemetry.update();
        sleep(200);

        switch (ps) {
            case TRES:
                // // Step 1.2: Strafe RIGHT
                turn(getCorrectDirection(DriveDirection.LEFT, needInvert), 0.5 * getDrivePower(DrivePower), 2);
                telemetry.addData("Strafe RIGHT", "");
                telemetry.update();
                sleep(500);

                driveTiles(DriveDirection.FORWARD, getDrivePower(DrivePower), 1.1);
                telemetry.addData("Forward", "");
                telemetry.update();
                sleep(200);

                turn(getCorrectDirection(DriveDirection.RIGHT, needInvert), 0.5 * getDrivePower(DrivePower), 2);
                telemetry.addData("Strafe RIGHT", "");
                telemetry.update();
                sleep(200);
                break;
            case DOS:
                break;
            case UNO:
                // // Step 1.2: Strafe RIGHT
                turn(getCorrectDirection(DriveDirection.RIGHT, needInvert), 0.5 * getDrivePower(DrivePower), 2);
                telemetry.addData("Strafe RIGHT", "");
                telemetry.update();
                sleep(500);

                driveTiles(DriveDirection.FORWARD, getDrivePower(DrivePower), 1.1);
                telemetry.addData("Forward", "");
                telemetry.update();
                sleep(200);

                turn(getCorrectDirection(DriveDirection.LEFT, needInvert), 0.5 * getDrivePower(DrivePower), 2);
                telemetry.addData("Strafe RIGHT", "");
                telemetry.update();
                sleep(200);
                break;
            default:
                break;
        }

        driveTiles(DriveDirection.FORWARD, getDrivePower(DrivePower), 1.3);
    }

    private DriveDirection getCorrectDirection(DriveDirection direction, boolean needInvert) {
        if (!needInvert)
            return direction;

        DriveDirection invertedDirection = direction;
        switch (direction) {
            case LEFT:
                invertedDirection = DriveDirection.RIGHT;
                break;
            case RIGHT:
                invertedDirection = DriveDirection.LEFT;
                break;
            case FORWARD:
                invertedDirection = DriveDirection.BACKWARD;
                break;
            case BACKWARD:
                invertedDirection = DriveDirection.FORWARD;
                break;
            default:
                break;
        }

        return invertedDirection;
    }


    private void moveSlidePackToPosition(PoleHeight curPoleHeight, PoleHeight targetPoleHeight) {
        int timeToMove = getMoveTimeOfSlidePack(curPoleHeight, targetPoleHeight);
        // VerticalSlidePack.setTargetPosition(getMoveTimeOfSlidePack(curPoleHeight, targetPoleHeight));
        // VerticalSlidePack.setPower(1);
        if (timeToMove >= 0)
            moveSlidePack(SlidePackDirection.UP, getDrivePower(SlidePackPower), timeToMove);
        else {
            timeToMove *= 0.8;
            moveSlidePack(SlidePackDirection.DOWN, getDrivePower(SlidePackPower), -timeToMove);
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
                return 2400;
            case MEDIUM:
                return 2200;
            case LOW:
                return 650;
            case GROUND:
                return 0;
            default:
                return 0;
        }
    }

    private void pickUpCone(PoleHeight ph) {
//         Step 4: Grab Block
        closeClaw();
        sleep(500);

//         Step 5: Move Slide Pack Up
        moveSlidePackToPosition(CurrentPoleHeight, ph);
        telemetry.addData("Picking up Block", "");
        telemetry.update();
    }

    private void scoreCone() {
//         Step 4: Grab Block
        openClaw();
        sleep(500);

//         Step 5: Move Slide Pack DOWN
        telemetry.addData("Scoring Block", "");
        telemetry.update();
    }

    private void openClaw() {
        // GrabberL.getController().setServoPosition(GrabberL.getPortNumber(), GrabberLReleasePosition);
        GrabberR.getController().setServoPosition(GrabberR.getPortNumber(), GrabberRReleasePosition);
    }

    private void closeClaw() {
        // GrabberL.getController().setServoPosition(GrabberL.getPortNumber(), GrabberLGrabPosition);
        GrabberR.getController().setServoPosition(GrabberR.getPortNumber(), GrabberRGrabPosition);
    }

    private int getDriveTime(int time) {
        return (int) (time * (1 + (0.1 - BATTERY_LEVEL * 0.1)));
    }

    private double getDrivePower(double power) {
        return power * (1 + (0.1 - BATTERY_LEVEL * 0.1));
    }

    private void spinFlywheel(double power, double time) {
        eTime.reset();
        Flywheel.setPower(power);
        while(opModeIsActive() && eTime.milliseconds() < time){
            telemetry.addData("Time:", eTime);
            telemetry.update();
        }
        Flywheel.setPower(0);
    }

    private void driveTiles(DriveDirection direction, double power, double tiles) {
        drive(direction, power, tiles * 650);
    }

    private void strafeTiles(DriveDirection direction, double power, double tiles) {
        strafe(direction, power, tiles * 1100);
    }

    private void turn(DriveDirection direction, double power, double tiles) {
        drive(direction, power, tiles * 720);
    }

    private void drive(DriveDirection direction, double power, double time) {
        eTime.reset();
        switch(direction) {
            case FORWARD:
                FLMotor.setPower(power);
                FRMotor.setPower(power);
                BLMotor.setPower(power);
                BRMotor.setPower(power);
                break;
            case RIGHT:
                FLMotor.setPower(-power);
                FRMotor.setPower(power);
                BLMotor.setPower(-power);
                BRMotor.setPower(power);
                break;
            case LEFT:
                FLMotor.setPower(power);
                FRMotor.setPower(-power);
                BLMotor.setPower(power);
                BRMotor.setPower(-power);
                break;
            case BACKWARD:
                FLMotor.setPower(-power);
                FRMotor.setPower(-power);
                BLMotor.setPower(-power);
                BRMotor.setPower(-power);
        }
        while(opModeIsActive() && eTime.milliseconds() < time){
            telemetry.addData("Time:", eTime);
            telemetry.update();
        }
        FLMotor.setPower(0);
        FRMotor.setPower(0);
        BLMotor.setPower(0);
        BRMotor.setPower(0);
    }

    private void moveSlidePack(SlidePackDirection spd, double power, double time) {
        eTime.reset();
        switch(spd) {
            case UP:
                VerticalSlidePack.setPower(power);
                break;
            case DOWN:
                VerticalSlidePack.setPower(-power);
                break;
        }
        while(opModeIsActive() && eTime.milliseconds() < time){
            telemetry.addData("Time:", eTime);
            telemetry.update();
        }
        VerticalSlidePack.setPower(0);
    }

    private void strafe(DriveDirection driveDirection, double power, double time){
        eTime.reset();
        switch (driveDirection) {
            case LEFT:
                while(opModeIsActive() && eTime.milliseconds() < time){
                    FLMotor.setPower(-power);
                    FRMotor.setPower(power);
                    BLMotor.setPower(power);
                    BRMotor.setPower(-power);
                    telemetry.addData("Time:", eTime);
                    telemetry.update();
                }
                break;
            case RIGHT:
                while(opModeIsActive() && eTime.milliseconds() < time){
                    FLMotor.setPower(power);
                    FRMotor.setPower(-power);
                    BLMotor.setPower(-power);
                    BRMotor.setPower(power);
                    telemetry.addData("Time:", eTime);
                    telemetry.update();
                }
                break;
        }
        FLMotor.setPower(0);
        FRMotor.setPower(0);
        BLMotor.setPower(0);
        BRMotor.setPower(0);
    }
//     private void resetAngle()
//     {
//         lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

//         globalAngle = 0;
//     }

//     //Get Angle Function
//     private double getAngle()
//     {
//         Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

//         double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

//         if (deltaAngle < -180)
//             deltaAngle += 360;
//         else if (deltaAngle > 180)
//             deltaAngle -= 360;

//         globalAngle += deltaAngle;

//         lastAngles = angles;

//         return globalAngle;
//     }

//     //    Check Direction Function
//     private double checkDirection()
//     {
//         double correction, angle, gain = .10;

//         angle = getAngle();

//         if (angle == 0)
//             correction = 0;
//         else
//             correction = -angle;

//         correction = correction * gain;

//         return correction;
//     }

//     //Rotate Function
//     private void rotate(double power, int degrees)
//     {
//         if(opModeIsActive())
//         {
//             double currentAng = 0;
//             resetAngle();
//             if (power > 0)
//             {
//                 while(opModeIsActive() && (-currentAng) < degrees)
//                 {
//                     currentAng  =   getAngle();
//                     FLMotor.setPower(power);
//                     FRMotor.setPower(-power);
//                     BLMotor.setPower(power);
//                     BRMotor.setPower(-power);

//                     telemetry.addData("Angle: ", currentAng);
//                     telemetry.addData("Target: ", degrees);
//                 }
//             }

//             else
//             {
//                 degrees = -degrees;
//                 while(opModeIsActive() && (currentAng) < degrees)
//                 {
//                     currentAng  =   getAngle();
//                     FLMotor.setPower(power);
//                     FRMotor.setPower(-power);
//                     BLMotor.setPower(power);
//                     BRMotor.setPower(-power);

//                     telemetry.addData("Angle: ", currentAng);
//                     telemetry.addData("Target: ", degrees);
//                 }
//             }

//             FLMotor.setPower(0);
//             FRMotor.setPower(0);
//             BLMotor.setPower(0);
//             BRMotor.setPower(0);

//         }
//     }

    //Encoder Drive Function
    // public void drive(double power, int centimeters)
    // {

    //     int currentPosFR    = 0;
    //     int currentPosFL    = 0;
    //     int currentPosBR    = 0;
    //     int currentPosBL    = 0;
    //     int target = centimeters * 45;

    //     // Ensure that the opmode is still active
    //     if (opModeIsActive())
    //     {
    //         FRMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
    //         FLMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
    //         BLMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
    //         BRMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);

    //         // Turn On RUN_TO_POSITION
    //         FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    //         BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    //         FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    //         BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    //         // Determine new target position, and pass to motor controller
    //         BRMotor.setTargetPosition(target);
    //         FLMotor.setTargetPosition(target);
    //         BLMotor.setTargetPosition(target);
    //         FRMotor.setTargetPosition(target);

    //         FLMotor.setPower(power);
    //         BRMotor.setPower(power);
    //         FRMotor.setPower(power);
    //         BLMotor.setPower(power);

    //         // keep looping while we are still active, and there is time left, and both motors are running.
    //         // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
    //         // its target position, the motion will stop.  This is "safer" in the event that the robot will
    //         // always end the motion as soon as possible.
    //         // However, if you require that BOTH motors have finished their moves before the robot continues
    //         // onto the next step, use (isBusy() || isBusy()) in the loop test.
    //         if (power > 0)
    //         {
    //             while (opModeIsActive() && (currentPosFL < target || currentPosBR < target ||
    //             currentPosFR < target || currentPosBL < target))
    //             {
    //                 currentPosBR    = BRMotor.getCurrentPosition();
    //                 currentPosFL    = FLMotor.getCurrentPosition();
    //                 currentPosFR    = FRMotor.getCurrentPosition();
    //                 currentPosBL    = BLMotor.getCurrentPosition();

    //                 // Display it for the driver.
    //                 telemetry.addData("Running to ", target);
    //                 telemetry.addData("Running at front left: ",  currentPosFL);
    //                 telemetry.addData("Running at back right: ", currentPosBR);
    //                 telemetry.addData("Running at front right: ",  currentPosFR);
    //                 telemetry.addData("Running at back left: ", currentPosBL);
    //                 telemetry.update();
    //             }
    //         }

    //         else
    //         {
    //             while (opModeIsActive() && (currentPosFL > target || currentPosBR > target ||
    //             currentPosFR > target || currentPosBL > target))
    //             {
    //                 currentPosBR    = BRMotor.getCurrentPosition();
    //                 currentPosFL    = FLMotor.getCurrentPosition();
    //                 currentPosFR    = FRMotor.getCurrentPosition();
    //                 currentPosBL    = BLMotor.getCurrentPosition();

    //                 // Display it for the driver.
    //                 telemetry.addData("Running to ", target);
    //                 telemetry.addData("Running at front left: ",  currentPosFL);
    //                 telemetry.addData("Running at back right: ", currentPosBR);
    //                 telemetry.addData("Running at front right: ",  currentPosFR);
    //                 telemetry.addData("Running at back left: ", currentPosBL);
    //                 telemetry.update();
    //             }
    //         }
    //     }

//        // Stop all motion;
//        BRMotor.setPower(0);
//        FRMotor.setPower(0);
//        FLMotor.setPower(0);
//        BLMotor.setPower(0);
//
//        // Turn off RUN_TO_POSITION
//        FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    //  sleep(250);   // optional pause after each move


//Land Function
    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB888, true);
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        vuforia.setFrameQueueCapacity(10);
        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow ObjThe arguments TFOD_MODEL_ASSET, LABELS are defined earlier in the op mode and are season specific.ect Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
    }

    private boolean getCameraReady() {
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
            telemetry.addData("TFOD is Activated", "");

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1, 16.0/9.0);
        }

        if (tfod == null) {
            telemetry.addData("TFOD is Null", "");
            telemetry.update();
            return false;
        }
        return true;
    }

    private ParkingSpace getCameraReading() {
        VuforiaLocalizer.CloseableFrame frame = null;
        try {
            frame = vuforia.getFrameQueue().take(); //takes the frame at the head of the queue
        } catch(Exception e) {
            telemetry.addData("e", "E");
            telemetry.update();
        }
        if (frame == null) return ParkingSpace.UNO;
        long numImages = frame.getNumImages();
        Image image = null;
        for (int i = 0; i < numImages; i++) {
            if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                image = frame.getImage(i);
            }
        }

        int[] colors = {0, 0, 0, 0};

        if (image != null) {
            ByteBuffer pixels = image.getPixels();
            byte[] pixelArray = new byte[pixels.remaining()];
            pixels.get(pixelArray, 0, pixelArray.length);
            int imgWidth = image.getWidth();
            int imgHeight = image.getHeight();
            int[] startingIndexes = getRowStartingIndexes(imgHeight, imgWidth, numberOfRowsToScanInImage);
            for (int i = numberOfRowsToScanInImage / 3; i < numberOfRowsToScanInImage * 2 / 3; i++) {
                for (int j = startingIndexes[i] + imgWidth * 2 / 3; j < startingIndexes[i] + imgWidth * 2 * 2/3; j += 2) {
                    colors[getColor(pixelArray[j], pixelArray[j+1])]++;
                    telemetry.addData("width", imgWidth);
                    telemetry.addData("height", imgHeight);
                    telemetry.addData("startingIndexes[i]", startingIndexes[i]);
                    telemetry.addData("yellow", colors[0]);
                    telemetry.addData("green", colors[1]);
                    telemetry.addData("black", colors[2]);
                    telemetry.update();
                    // sleep(30000);
                }
            }
            // sleep(5000);
            // telemetry.addData("b1", pixelArray[0]);
            // telemetry.addData("b2", pixelArray[1]);
            telemetry.addData("yellow", colors[0]);
            telemetry.addData("green", colors[1]);
            telemetry.addData("black", colors[2]);
            telemetry.update();
            for (int i = 0; i < startingIndexes.length; i++)
                telemetry.addData("startingIndexes[i]", startingIndexes[i]);
            telemetry.update();
            // sleep(30000);
        }

        frame.close();
        int max_index = 0;
        for (int i = 0; i < 3; i++) {
            if (colors[i] > colors[max_index])
                max_index = i;
        }

        if (max_index == 0)
            return ParkingSpace.UNO;
        if (max_index == 1)
            return ParkingSpace.DOS;
        return ParkingSpace.TRES;
    }

    private int[] getRowStartingIndexes(int height, int width, int numRows) {
        int[] newArr = new int[numRows];
        int stepSize = 2 * height / numRows * width;
        for (int i = 1; i < numRows; i++) {
            newArr[i] = i * stepSize;
        }
        return newArr;
    }

    private int getColor(byte b1, byte b2) {
        // GGGBBBBB RRRRRGGG;
        String s1 = String.format("%8s", Integer.toBinaryString(b2 & 0xFF)).replace(' ', '0');
        String s2 = String.format("%8s", Integer.toBinaryString(b1 & 0xFF)).replace(' ', '0');
        // RRRRRGGG GGGBBBBB;
        int[] color = new int[3];
        String r = s1.substring(0, 5);
        String g = s1.substring(5) + s2.substring(0, 3);
        String b = s2.substring(3);
        color[0] = convertBitStringToInt(r);
        color[1] = convertBitStringToInt(g);
        color[2] = convertBitStringToInt(b);
        double[] hsv = convertRGBtoHSV(color);
        telemetry.addData("hsv", hsv[0]);
        telemetry.addData("hsv", hsv[1]);
        telemetry.addData("hsv", hsv[2]);
        telemetry.addData("b1", b1);
        telemetry.addData("b2", b2);
        telemetry.addData("hsv[2]", hsv[2]);
        if (hsv[2] < 0.3)
            return 2;
        if (hsv[0] >= 70 && hsv[0] <= 155 && hsv[1] > 0.15 && hsv[2] > 0.2)
            return 1;
        if (hsv[0] >= 45 && hsv[0] <= 70 && hsv[1] > 0.15 && hsv[2] > 0.5)
            return 0;
        return 3;
    }

    private double[] convertRGBtoHSV(int[] rgb) {
        double rPrime = (double) rgb[0]/31;
        double gPrime = (double) rgb[1]/63;
        double bPrime = (double) rgb[2]/31;
        double cMax = Math.max(rPrime, Math.max(gPrime, bPrime));
        double cMin = Math.min(rPrime, Math.min(gPrime, bPrime));
        double delta = cMax - cMin;
        double[] hsv = new double[3];

        // calculate hue
        if (delta == 0)
            hsv[0] = 0;
        else if (cMax == rPrime) {
            double temp = ((gPrime - bPrime) / delta) % 6;
            if (temp < 0)
                temp += 6;
            hsv[0] = 60 * temp;
        }
        else if (cMax == gPrime)
            hsv[0] = 60 * (((bPrime - rPrime) / delta) + 2);
        else
            hsv[0] = 60 * (((rPrime - gPrime) / delta) + 4);

        // calculate saturation
        if (cMax == 0)
            hsv[1] = 0;
        else
            hsv[1] = delta / cMax;

        // calculate value
        hsv[2] = cMax;

        return hsv;
    }

    private int convertBitStringToInt(String s) {
        int sum = 0;
        // Little Endian
        // int digit = 0;
        // for (char c : s.toCharArray()) {
        //     if (c == '1') {
        //         sum += Math.pow(2, digit);
        //     }
        //     digit++;
        // }
        // Big Endian
        int digit = s.length() - 1;
        for (char c : s.toCharArray()) {
            if (c == '1') {
                sum += Math.pow(2, digit);
            }
            digit--;
        }
        return sum;
    }
}
