package org.firstinspires.ftc.teamcode.Skystone_Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Skystone_Hardware;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@Autonomous(name= "Blue_Test", group="Sky autonomous")
//@Disabled//comment out this line before using
public class Blue_Full extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;

    private static float rectHeight = .6f/8f;
    private static float rectWidth = 1.5f/8f;

    private static float offsetX = 0f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = {4f/8f+offsetX, 6.2f/8f+offsetY};//0 = col, 1 = row
    private static float[] leftPos = {1.5f/8f+offsetX, 6.2f/8f+offsetY};
    private static float[] rightPos = {6.5f/8f+offsetX, 6.2f/8f+offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;

    OpenCvCamera phoneCam;

    Skystone_Hardware robot = new Skystone_Hardware();

    // Declare OpMode members.

    //Declare Sensors
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    //USER GENERATED VALUES//
    int zAccumulated;  //Total rotation left/right
    double headingResetValue;
    int detv;
    int heading;

    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        //P.S. if you're using the latest version of easyopencv, you might need to change the next line to the following:
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);

        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.

        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.front_right.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.back_right.setDirection(DcMotorSimple.Direction.FORWARD);

        //drivetrain = new MecanumDrivetrain(new DcMotor[] {robot.front_left, robot.front_right, robot.back_left, robot.back_right});

        //CODE FOR SETTING UP AND INITIALIZING IMU
        BNO055IMU.Parameters parameters2 = new BNO055IMU.Parameters();
        parameters2.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters2.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters2.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters2.loggingEnabled = true;
        parameters2.loggingTag = "IMU";
        parameters2.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        //Reset Encoders
        robot.front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        //Set the Run Mode For The Motors
        robot.back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  //Controls the speed of the motors to be consistent even at different battery levels
        robot.front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //DEFINE SENSORS
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters2);

        //Setup The Telemetry Dashboard
        composeTelemetry();

        //Initilization
        int opState = 0;
        int stoneState = 0;

        robot.foundation_New1.setPosition(robot.foundationNew1Up);
        robot.foundation_New2.setPosition(robot.foundationNew2Up);

        robot.intakeServo.setPosition(robot.intakeUp);

        robot.depositGrabber.setPosition(robot.dGrabberOpen);

        robot.grabber.setPosition(robot.aGrabberOpenFull);

        // Wait for the game to start (driver presses PLAY)
        this.headingResetValue = this.getAbsoluteHeading();
        //waitForStart();
        runtime.reset();

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status:", "Waiting for start command.");
            telemetry.update();
        };
        runtime.reset();
        while (opModeIsActive()) {
            telemetry.addData("Values", valLeft+"   "+valMid+"   "+valRight);

            if (valRight == 0 && opState == 0) {
                phoneCam.closeCameraDevice();
                robot.depositGrabber.setPosition(robot.dGrabberOpen);
                robot.intakeServo.setPosition(robot.intakeDown);
                robot.grabber.setPosition(robot.aGrabberOpenFull);
                robot.arm.setPosition(robot.armStore);
                opState++;
                if (opState == 1 && opModeIsActive()) { //drive over to the first stone
                    driveLRTest(-975, 0.35, 5, 0, 3); //0.4 //-960
                    sleep(10);
                    opState++;
                }
                if (opState == 2 && opModeIsActive()) { //grab the block in the 3rd position
                    robot.arm.setPosition(robot.armDown);
                    sleep(350);
                    robot.grabber.setPosition(robot.aGrabberClosed);
                    sleep(350);
                    robot.arm.setPosition(robot.armUp);
                    sleep(100);
                    opState++;
                }
                if (opState == 3 && opModeIsActive()) { //back up to foundation
                    driveSBTest(-2700, 0.75, 10, 0, 10);
                    sleep(100);
                    opState++;
                }
                if (opState == 4 && opModeIsActive()) { //move over to foundation
                    driveLRTest(-105, 0.3, 5, 0, 1.5);
                    sleep(100);
                    opState++;
                }
                if (opState == 5 && opModeIsActive()) { //drop block
                    robot.arm.setPosition(robot.armDown);
                    sleep(350);
                    robot.grabber.setPosition(robot.aGrabberOpenFull);
                    sleep(350);
                    robot.arm.setPosition(robot.armUp);
                    sleep(100);
                    opState++;
                }
                if (opState == 6 && opModeIsActive()) { //line up foundation grabbers with foundation
                    turnTestTest(-90, 0.4, 2);
                    sleep(100);
                    opState++;
                }
                if (opState == 7 && opModeIsActive()) { //drive to foundation
                    driveSBTest(-120, 0.2, 5, -90, 1.5);
                    sleep(100);
                    opState++;
                }
                if (opState == 8 && opModeIsActive()) {
                    robot.foundation_New1.setPosition(robot.foundationNew1Down);
                    robot.foundation_New2.setPosition(robot.foundationNew2Down);
                    sleep(100);
                    opState++;
                }
                if (opState == 9 && opModeIsActive()) {
                    turnTestTest(-70, 0.3, 2);
                    sleep(10);
                    driveSBTest(700, 0.4, 20, -70, 2.5);
                    sleep(100);
                    opState++;
                }
                if (opState == 10 && opModeIsActive()) {
                    turnTestTest(0, 0.5, 3);
                    sleep(100);
                    opState++;
                }
                if (opState == 11 && opModeIsActive()) {
                    driveSBTest(-200, 0.3, 20, 0, 2);
                    sleep(100);
                    robot.foundation_New1.setPosition(robot.foundationNew1Up);
                    robot.foundation_New2.setPosition(robot.foundationNew2Up);
                    opState++;
                }
                if (opState == 12 && opModeIsActive()) {
                    driveLRTest(-450, 0.4,  20, 0, 1.5);
                    sleep(100);
                    opState++;
                }
                if (opState == 13 && opModeIsActive()) { //drive to lz to the 6th position
                    robot.arm.setPosition(robot.armStore);
                    robot.grabber.setPosition(robot.aGrabberClosedFull);
                    driveSBTest(3000, 0.7, 10, 0, 5); //slower
                    sleep(100);
                    opState++;
                }
                if (opState == 14 && opModeIsActive()) { //scoot to  block
                    driveLRTest(-475, 0.2, 5, 0, 3.5);
                    sleep(10);
                    robot.intakeMotors.setPower(-0.6);
                    robot.intakeCollector.setPower(-0.8);
                    driveSBTest(200, 0.4, 5, 0, 2);
                    sleep(10);
                    driveLRTest(575, 0.3, 5, 0, 3.5);
                    sleep(10);
                    opState++;
                }
                if (opState == 15 && opModeIsActive()) { //drive to foundation
                    driveSBTest(-2700, 0.8, 10, 0, 3.5); //slower
                    sleep(100);
                    robot.intakeMotors.setPower(0);
                    robot.intakeCollector.setPower(0);
                    opState++;
                }
                if (opState == 16 && opModeIsActive()) { //park
                    driveSBTest(600, 0.4, 10, 0, 10);
                    stop();
                }

            } else if (valMid == 0 && opState == 0) {
                phoneCam.closeCameraDevice();
                robot.depositGrabber.setPosition(robot.dGrabberOpen);
                robot.intakeServo.setPosition(robot.intakeDown);
                robot.grabber.setPosition(robot.aGrabberOpenFull);
                robot.arm.setPosition(robot.armStore);
                opState++;
                if (opState == 1 && opModeIsActive()) { //drive to the first stone
                    driveLRTest(-975, 0.35, 5, 0, 3); //0.4 //-960
                    sleep(100);
                    driveSBTest(-200, 0.3, 5, 0,  2);
                    sleep(10);
                    opState++;
                }
                if (opState == 2 && opModeIsActive()) { //grab the block in the 3rd position
                    robot.arm.setPosition(robot.armDown);
                    sleep(350);
                    robot.grabber.setPosition(robot.aGrabberClosed);
                    sleep(350);
                    robot.arm.setPosition(robot.armUp);
                    sleep(100);
                    opState++;
                }
                if (opState == 3 && opModeIsActive()) { //back up to foundation
                    driveSBTest(-2500, 0.75, 10, 0, 10);
                    sleep(100);
                    opState++;
                }
                if (opState == 4 && opModeIsActive()) { //move over to foundation
                    driveLRTest(-105, 0.3, 5, 0, 1.5);
                    sleep(100);
                    opState++;
                }
                if (opState == 5 && opModeIsActive()) { //drop block
                    robot.arm.setPosition(robot.armDown);
                    sleep(350);
                    robot.grabber.setPosition(robot.aGrabberOpenFull);
                    sleep(350);
                    robot.arm.setPosition(robot.armUp);
                    sleep(100);
                    opState++;
                }
                if (opState == 6 && opModeIsActive()) { //line up foundation grabbers with foundation
                    turnTestTest(-90, 0.4, 2);
                    sleep(100);
                    opState++;
                }
                if (opState == 7 && opModeIsActive()) { //drive to foundation
                    driveSBTest(-120, 0.2, 5, -90, 1.5);
                    sleep(100);
                    opState++;
                }
                if (opState == 8 && opModeIsActive()) {
                    robot.foundation_New1.setPosition(robot.foundationNew1Down);
                    robot.foundation_New2.setPosition(robot.foundationNew2Down);
                    sleep(100);
                    opState++;
                }
                if (opState == 9 && opModeIsActive()) {
                    turnTestTest(-70, 0.3, 2);
                    sleep(10);
                    driveSBTest(700, 0.4, 20, -70, 2.5);
                    sleep(100);
                    opState++;
                }
                if (opState == 10 && opModeIsActive()) {
                    turnTestTest(0, 0.5, 3);
                    sleep(100);
                    opState++;
                }
                if (opState == 11 && opModeIsActive()) {
                    driveSBTest(-200, 0.3, 20, 0, 2);
                    sleep(100);
                    robot.foundation_New1.setPosition(robot.foundationNew1Up);
                    robot.foundation_New2.setPosition(robot.foundationNew2Up);
                    opState++;
                }
                if (opState == 12 && opModeIsActive()) {
                    driveLRTest(-450, 0.4,  20, 0, 1.5);
                    sleep(100);
                    opState++;
                }
                if (opState == 13 && opModeIsActive()) { //drive to lz to the 6th position
                    robot.arm.setPosition(robot.armStore);
                    robot.grabber.setPosition(robot.aGrabberClosedFull);
                    driveSBTest(2800, 0.7, 10, 0, 5); //slower
                    sleep(100);
                    opState++;
                }
                if (opState == 14 && opModeIsActive()) { //scoot to  block
                    driveLRTest(-475, 0.2, 5, 0, 3.5);
                    sleep(10);
                    robot.intakeMotors.setPower(-0.6);
                    robot.intakeCollector.setPower(-0.8);
                    driveSBTest(200, 0.4, 5, 0, 2);
                    sleep(10);
                    driveLRTest(575, 0.3, 5, 0, 3.5);
                    sleep(10);
                    opState++;
                }
                if (opState == 15 && opModeIsActive()) { //drive to foundation
                    driveSBTest(-2500, 0.8, 10, 0, 3.5); //slower
                    sleep(100);
                    robot.intakeMotors.setPower(0);
                    robot.intakeCollector.setPower(0);
                    opState++;
                }
                if (opState == 16 && opModeIsActive()) { //park
                    driveSBTest(700, 0.4, 10, 0, 10);
                    stop();
                }
            } else if (valLeft == 0 && opState == 0) {
                phoneCam.closeCameraDevice();
                robot.depositGrabber.setPosition(robot.dGrabberOpen);
                robot.intakeServo.setPosition(robot.intakeDown);
                robot.grabber.setPosition(robot.aGrabberOpenFull);
                robot.arm.setPosition(robot.armStore);
                opState++;
                if (opState == 1 && opModeIsActive()) { //drive to the first stone
                    driveLRTest(-975, 0.35, 5, 0, 3); //0.4 //-960
                    sleep(100);
                    driveSBTest(-450, 0.3, 5, 0,  2);
                    sleep(10);
                    opState++;
                }
                if (opState == 2 && opModeIsActive()) { //grab the block in the 3rd position
                    robot.arm.setPosition(robot.armDown);
                    sleep(350);
                    robot.grabber.setPosition(robot.aGrabberClosed);
                    sleep(350);
                    robot.arm.setPosition(robot.armUp);
                    sleep(100);
                    opState++;
                }
                if (opState == 3 && opModeIsActive()) { //back up to foundation
                    driveSBTest(-2300, 0.75, 10, 0, 10);
                    sleep(100);
                    opState++;
                }
                if (opState == 4 && opModeIsActive()) { //move over to foundation
                    driveLRTest(-105, 0.3, 5, 0, 1.5);
                    sleep(100);
                    opState++;
                }
                if (opState == 5 && opModeIsActive()) { //drop block
                    robot.arm.setPosition(robot.armDown);
                    sleep(350);
                    robot.grabber.setPosition(robot.aGrabberOpenFull);
                    sleep(350);
                    robot.arm.setPosition(robot.armUp);
                    sleep(100);
                    opState++;
                }
                if (opState == 6 && opModeIsActive()) { //line up foundation grabbers with foundation
                    turnTestTest(-90, 0.4, 2);
                    sleep(100);
                    opState++;
                }
                if (opState == 7 && opModeIsActive()) { //drive to foundation
                    driveSBTest(-120, 0.2, 5, -90, 1.5);
                    sleep(100);
                    opState++;
                }
                if (opState == 8 && opModeIsActive()) {
                    robot.foundation_New1.setPosition(robot.foundationNew1Down);
                    robot.foundation_New2.setPosition(robot.foundationNew2Down);
                    sleep(100);
                    opState++;
                }
                if (opState == 9 && opModeIsActive()) {
                    turnTestTest(-70, 0.3, 2);
                    sleep(10);
                    driveSBTest(700, 0.4, 20, -70, 2.5);
                    sleep(100);
                    opState++;
                }
                if (opState == 10 && opModeIsActive()) {
                    turnTestTest(0, 0.5, 3);
                    sleep(100);
                    opState++;
                }
                if (opState == 11 && opModeIsActive()) {
                    driveSBTest(-200, 0.3, 20, 0, 2);
                    sleep(100);
                    robot.foundation_New1.setPosition(robot.foundationNew1Up);
                    robot.foundation_New2.setPosition(robot.foundationNew2Up);
                    opState++;
                }
                if (opState == 12 && opModeIsActive()) {
                    driveLRTest(-450, 0.4,  20, 0, 1.5);
                    sleep(100);
                    opState++;
                }
                if (opState == 13 && opModeIsActive()) { //drive to lz to the 6th position
                    robot.arm.setPosition(robot.armStore);
                    robot.grabber.setPosition(robot.aGrabberClosedFull);
                    driveSBTest(2600, 0.7, 10, 0, 5); //slower
                    sleep(100);
                    opState++;
                }
                if (opState == 14 && opModeIsActive()) { //scoot to  block
                    driveLRTest(-500, 0.2, 5, 0, 3.5);
                    sleep(10);
                    robot.intakeMotors.setPower(-0.6);
                    robot.intakeCollector.setPower(-0.8);
                    driveSBTest(200, 0.4, 5, 0, 2);
                    sleep(10);
                    driveLRTest(575, 0.3, 5, 0, 3.5);
                    sleep(10);
                    opState++;
                }
                if (opState == 15 && opModeIsActive()) { //drive to foundation
                    driveSBTest(-2300, 0.8, 10, 0, 3.5); //slower
                    sleep(100);
                    robot.intakeMotors.setPower(0);
                    robot.intakeCollector.setPower(0);
                    opState++;
                }
                if (opState == 16 && opModeIsActive()) { //park
                    driveSBTest(700, 0.4, 10, 0, 10);
                    stop();
                }
            }
        }
    }

    //FUNCTIONS
    public void driveSBTest (double duration, double speedPercent, double error, double heading, double time) {
        double position = robot.back_right.getCurrentPosition();
        double target = position + duration;
        double distanceToTargetStart = Math.abs(target - position);
        double distanceToTarget = target - position;
        double percentToTarget = distanceToTarget/distanceToTargetStart;
        double speed = 0;
        double[] wheelSpeed = new double[4]; //fl, fr, bl, br
        double turnSpeed = ((heading - getAbsoluteHeading())/20);
        double startTime = getRuntime();

        while (Math.abs(distanceToTarget) > error && !isStopRequested() && getRuntime() < startTime + time) {
            position = robot.back_right.getCurrentPosition();
            distanceToTarget = target - position;
            percentToTarget = distanceToTarget/distanceToTargetStart;
            if (percentToTarget >= 0) {
                speed = ((-((percentToTarget-1)*(percentToTarget-1)*(percentToTarget-1)*(percentToTarget-1))+1)*speedPercent)*0.9;
                if (Math.abs(speed) < 0.05) {speed = 0.05;}
            }else if (percentToTarget < 0) {
                speed = -((-((percentToTarget+1)*(percentToTarget+1)*(percentToTarget+1)*(percentToTarget+1))+1)*speedPercent)*0.9;
                if (Math.abs(speed) < 0.05) {speed = -0.05;}
            }

            turnSpeed = -((heading - getAbsoluteHeading())/40);

            telemetry.addData("DaS:", distanceToTargetStart);
            telemetry.addData("Target:", target);
            telemetry.addData("Position:", position);
            telemetry.addData("Distance:", distanceToTarget);
            telemetry.addData("Percent:", percentToTarget);
            telemetry.addData("Speed:", speed);
            telemetry.addData("Turn Speed:", turnSpeed);
            telemetry.update();

            wheelSpeed[0] = Range.clip(speed, -0.9, 0.9) + Range.clip(turnSpeed, -0.1, 0.1);
            wheelSpeed[1] = Range.clip(speed, -0.9, 0.9) - Range.clip(turnSpeed, -0.1, 0.1);
            wheelSpeed[2] = Range.clip(speed, -0.9, 0.9) + Range.clip(turnSpeed, -0.1, 0.1);
            wheelSpeed[3] = Range.clip(speed, -0.9, 0.9) - Range.clip(turnSpeed, -0.1, 0.1);

            motorSetSpeed(wheelSpeed[0], wheelSpeed[1], wheelSpeed[2], wheelSpeed[3]);
        };

        wheelSpeed[0] = 0;
        wheelSpeed[1] = 0;
        wheelSpeed[2] = 0;
        wheelSpeed[3] = 0;

        motorSetSpeed(wheelSpeed[0], wheelSpeed[1], wheelSpeed[2], wheelSpeed[3]);
    };
    public void driveSBSlowerTest (double duration, double speedPercent, double error, double heading) {
        double position = robot.back_right.getCurrentPosition();
        double target = position + duration;
        double distanceToTargetStart = Math.abs(target - position);
        double distanceToTarget = target - position;
        double percentToTarget = distanceToTarget/distanceToTargetStart;
        double speed = 0;
        double[] wheelSpeed = new double[4]; //fl, fr, bl, br
        double turnSpeed = ((heading - getAbsoluteHeading())/20);

        while (Math.abs(distanceToTarget) > error && !isStopRequested()) {
            position = robot.back_right.getCurrentPosition();
            distanceToTarget = target - position;
            percentToTarget = distanceToTarget/distanceToTargetStart;
            if (percentToTarget >= 0) {
                speed = ((-((percentToTarget-1)*(percentToTarget-1))+1)*speedPercent)*0.9;
                if (Math.abs(speed) < 0.05) {speed = 0.05;}
            }else if (percentToTarget < 0) {
                speed = -((-((percentToTarget+1)*(percentToTarget+1))+1)*speedPercent)*0.9;
                if (Math.abs(speed) < 0.05) {speed = -0.05;}
            }

            turnSpeed = -((heading - getAbsoluteHeading())/38);

            telemetry.addData("DaS:", distanceToTargetStart);
            telemetry.addData("Target:", target);
            telemetry.addData("Position:", position);
            telemetry.addData("Distance:", distanceToTarget);
            telemetry.addData("Percent:", percentToTarget);
            telemetry.addData("Speed:", speed);
            telemetry.addData("Turn Speed:", turnSpeed);
            telemetry.update();

            wheelSpeed[0] = Range.clip(speed, -0.9, 0.9) + Range.clip(turnSpeed, -0.1, 0.1);
            wheelSpeed[1] = Range.clip(speed, -0.9, 0.9) - Range.clip(turnSpeed, -0.1, 0.1);
            wheelSpeed[2] = Range.clip(speed, -0.9, 0.9) + Range.clip(turnSpeed, -0.1, 0.1);
            wheelSpeed[3] = Range.clip(speed, -0.9, 0.9) - Range.clip(turnSpeed, -0.1, 0.1);

            motorSetSpeed(wheelSpeed[0], wheelSpeed[1], wheelSpeed[2], wheelSpeed[3]);
        };

        wheelSpeed[0] = 0;
        wheelSpeed[1] = 0;
        wheelSpeed[2] = 0;
        wheelSpeed[3] = 0;

        motorSetSpeed(wheelSpeed[0], wheelSpeed[1], wheelSpeed[2], wheelSpeed[3]);
    };

    public void driveLRFasterTest (double duration, double speedPercent, double error, double heading) {
        double position = robot.back_right.getCurrentPosition();
        double target = position + duration;
        double distanceToTargetStart = Math.abs(target - position);
        double distanceToTarget = target - position;
        double percentToTarget = distanceToTarget/distanceToTargetStart;
        double speed = 0;
        double[] wheelSpeed = new double[4]; //fl, fr, bl, br
        double turnSpeed = ((heading - getAbsoluteHeading())/20);

        while (Math.abs(distanceToTarget) > error && !isStopRequested()) {
            position = robot.back_right.getCurrentPosition();
            distanceToTarget = target - position;
            percentToTarget = distanceToTarget/distanceToTargetStart;
            if (percentToTarget >= 0) {
                speed = ((-((percentToTarget-1)*(percentToTarget-1)*(percentToTarget-1)*(percentToTarget-1)*(percentToTarget-1)*(percentToTarget-1)*(percentToTarget-1)*(percentToTarget-1))+1)*speedPercent)*0.9;
            }else if (percentToTarget < 0) {
                speed = -((-((percentToTarget+1)*(percentToTarget+1)*(percentToTarget+1)*(percentToTarget+1)*(percentToTarget+1)*(percentToTarget+1)*(percentToTarget+1)*(percentToTarget+1))+1)*speedPercent)*0.9;
            }

            turnSpeed = -((heading - getAbsoluteHeading())/35);

            telemetry.addData("DaS:", distanceToTargetStart);
            telemetry.addData("Target:", target);
            telemetry.addData("Position:", position);
            telemetry.addData("Distance:", distanceToTarget);
            telemetry.addData("Percent:", percentToTarget);
            telemetry.addData("Speed:", speed);
            telemetry.addData("Turn Speed:", turnSpeed);
            telemetry.update();

            wheelSpeed[0] = Range.clip(speed, -0.9, 0.9) + Range.clip(turnSpeed, -0.1, 0.1);
            wheelSpeed[1] = -Range.clip(speed, -0.9, 0.9) - Range.clip(turnSpeed, -0.1, 0.1);
            wheelSpeed[2] = -Range.clip(speed, -0.9, 0.9) + Range.clip(turnSpeed, -0.1, 0.1);
            wheelSpeed[3] = Range.clip(speed, -0.9, 0.9) - Range.clip(turnSpeed, -0.1, 0.1);

            motorSetSpeed(wheelSpeed[0], wheelSpeed[1], wheelSpeed[2], wheelSpeed[3]);
        };

        wheelSpeed[0] = 0;
        wheelSpeed[1] = 0;
        wheelSpeed[2] = 0;
        wheelSpeed[3] = 0;

        motorSetSpeed(wheelSpeed[0], wheelSpeed[1], wheelSpeed[2], wheelSpeed[3]);
    };

    public void driveLRTest (double duration, double speedPercent, double error, double heading, double time) {
        double position = robot.back_right.getCurrentPosition();
        double target = position + duration;
        double distanceToTargetStart = Math.abs(target - position);
        double distanceToTarget = target - position;
        double percentToTarget = distanceToTarget/distanceToTargetStart;
        double speed = 0;
        double[] wheelSpeed = new double[4]; //fl, fr, bl, br
        double turnSpeed = ((heading - getAbsoluteHeading())/20);
        double startTime = getRuntime();

        while (Math.abs(distanceToTarget) > error && !isStopRequested() && getRuntime() < startTime + time) {
            position = robot.back_right.getCurrentPosition();
            distanceToTarget = target - position;
            percentToTarget = distanceToTarget/distanceToTargetStart;
            if (percentToTarget >= 0) {
                speed = (((-((percentToTarget-1)*(percentToTarget-1)*(percentToTarget-1)*(percentToTarget-1))+1)*speedPercent)*0.9)+0.025;
            }else if (percentToTarget < 0) {
                speed = (-((-((percentToTarget+1)*(percentToTarget+1)*(percentToTarget+1)*(percentToTarget+1))+1)*speedPercent)*0.9)-0.025;

            }

            turnSpeed = -((heading - getAbsoluteHeading())/35);

            telemetry.addData("DaS:", distanceToTargetStart);
            telemetry.addData("Target:", target);
            telemetry.addData("Position:", position);
            telemetry.addData("Distance:", distanceToTarget);
            telemetry.addData("Percent:", percentToTarget);
            telemetry.addData("Speed:", speed);
            telemetry.addData("Turn Speed:", turnSpeed);
            telemetry.update();

            wheelSpeed[0] = Range.clip(speed, -0.9, 0.9) + Range.clip(turnSpeed, -0.1, 0.1);
            wheelSpeed[1] = -Range.clip(speed, -0.9, 0.9) - Range.clip(turnSpeed, -0.1, 0.1);
            wheelSpeed[2] = -Range.clip(speed, -0.9, 0.9) + Range.clip(turnSpeed, -0.1, 0.1);
            wheelSpeed[3] = Range.clip(speed, -0.9, 0.9) - Range.clip(turnSpeed, -0.1, 0.1);

            motorSetSpeed(wheelSpeed[0], wheelSpeed[1], wheelSpeed[2], wheelSpeed[3]);
        };

        wheelSpeed[0] = 0;
        wheelSpeed[1] = 0;
        wheelSpeed[2] = 0;
        wheelSpeed[3] = 0;

        motorSetSpeed(wheelSpeed[0], wheelSpeed[1], wheelSpeed[2], wheelSpeed[3]);
    };

    public void turnTestTest (double target, double speedPercent, double error) {
        double DaS = Math.abs(target - getAbsoluteHeading());
        double distance = target - getAbsoluteHeading();
        double percent = distance/DaS;

        double speed = 0;
        double[] wheelSpeed = new double[4]; //fl, fr, bl, br

        while (Math.abs(distance) > error && !isStopRequested()) {
            distance = target - getAbsoluteHeading();
            percent = distance/DaS;

            speed = speedPercent*(0.9*(Math.cbrt(percent)));

            wheelSpeed[0] = -speed;
            wheelSpeed[1] = speed;
            wheelSpeed[2] = -speed;
            wheelSpeed[3] = speed;

            motorSetSpeed(wheelSpeed[0], wheelSpeed[1], wheelSpeed[2], wheelSpeed[3]);
        }
        wheelSpeed[0] = 0;
        wheelSpeed[1] = 0;
        wheelSpeed[2] = 0;
        wheelSpeed[3] = 0;

        motorSetSpeed(wheelSpeed[0], wheelSpeed[1], wheelSpeed[2], wheelSpeed[3]);
    };

    public void motorSetSpeed (double fl, double fr, double bl, double br) {
        robot.front_left.setPower(fl);
        robot.front_right.setPower(fr);
        robot.back_left.setPower(bl);
        robot.back_right.setPower(br);
    };

    //FUNCTIONS NEEDED BY THE GYRO
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    private double getAbsoluteHeading(){
        return this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    private double getRelativeHeading(){
        return this.getAbsoluteHeading() - this.headingResetValue;
    }

    //VISION/////////////////
    //detection pipeline
    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage
        {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private Stage stageToRenderToViewport = Stage.detection;
        private Stage[] stages = Stage.values();

        @Override
        public void onViewportTapped()
        {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input)
        {
            contoursList.clear();
            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame
            double[] pixMid = thresholdMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
            valMid = (int)pixMid[0];

            double[] pixLeft = thresholdMat.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
            valLeft = (int)pixLeft[0];

            double[] pixRight = thresholdMat.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
            valRight = (int)pixRight[0];

            //create three points
            Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
            Point pointLeft = new Point((int)(input.cols()* leftPos[0]), (int)(input.rows()* leftPos[1]));
            Point pointRight = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()* rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointLeft,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointRight,5, new Scalar( 255, 0, 0 ),1 );//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols()*(leftPos[0]-rectWidth/2),
                            input.rows()*(leftPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(leftPos[0]+rectWidth/2),
                            input.rows()*(leftPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols()*(midPos[0]-rectWidth/2),
                            input.rows()*(midPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(midPos[0]+rectWidth/2),
                            input.rows()*(midPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols()*(rightPos[0]-rectWidth/2),
                            input.rows()*(rightPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(rightPos[0]+rectWidth/2),
                            input.rows()*(rightPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);

            switch (stageToRenderToViewport)
            {
                case THRESHOLD:
                {
                    return thresholdMat;
                }

                case detection:
                {
                    return all;
                }

                case RAW_IMAGE:
                {
                    return input;
                }

                default:
                {
                    return input;
                }
            }
        }

    }

    public void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the n    ecessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }
}
