package org.firstinspires.ftc.teamcode.Skystone_Auto;

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

//import edu.spa.ftclib.internal.drivetrain.MecanumDrivetrain;

import java.util.Locale;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Auto Test", group="14174")
@Disabled
public class SkystoneAuto14174 extends LinearOpMode {

    Skystone_Hardware robot = new Skystone_Hardware();

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //Declare Sensors
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    //USER GENERATED VALUES//
    int zAccumulated;  //Total rotation left/right
    double headingResetValue;
    int detv;
    int heading;
    //public MecanumDrivetrain drivetrain;

    @Override
    public void runOpMode() {

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

        robot.grabber.setPosition(robot.armStore);

        // Wait for the game to start (driver presses PLAY)
        this.headingResetValue = this.getAbsoluteHeading();
        //waitForStart();
        runtime.reset();

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status:", "Waiting for start command.");
            telemetry.update();
        };

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.update();

            //Blue FULL AUTO FRAMEWORK
            if(opState == 0 && opModeIsActive()) { //scan
                //scan
                sleep(1000);
                opState++;
            };
            // 1 is closest to the bridges
            if(stoneState == 1 && opModeIsActive()) { //if the stones are in the 1st and 4th positions
                if (opState == 1 && opModeIsActive()) { //drive over to the first stone
                    driveLRTest(-1200, 0.5, 5);
                    sleep(500);
                    opState++;
                }
                if (opState == 2 && opModeIsActive()) { //grab the block in the 1st position
                    robot.arm.setPosition(robot.armDown);
                    sleep(500);
                    robot.grabber.setPosition(robot.aGrabberClosed);
                    sleep(500);
                    robot.arm.setPosition(robot.armUp);
                    sleep(500);
                    opState++;
                }
                if (opState == 3 && opModeIsActive()) { //line up with inner path
                    driveLRTest(150,  0.2, 5);
                    sleep(500);
                    opState++;
                }
                if (opState == 4 && opModeIsActive()) { //back up to foundation
                    driveSBSlowerTest(-2500, 0.8, 10);
                    sleep(500);
                    opState++;
                }
                if (opState == 5 && opModeIsActive()) { //move over to foundation
                    driveLRTest(-200, 0.2, 5);
                    sleep(500);
                    opState++;
                }
                if (opState == 6 && opModeIsActive()) { //drop block
                    robot.arm.setPosition(robot.armDown);
                    sleep(500);
                    robot.grabber.setPosition(robot.aGrabberOpenFull);
                    sleep(500);
                    robot.arm.setPosition(robot.armUp);
                    sleep(500);
                    opState++;
                }
                if (opState == 7 && opModeIsActive()) { //line up with inner path
                    driveLRTest(200, 0.2, 5);
                    sleep(500);
                    opState++;
                }
                if (opState == 8 && opModeIsActive()) { //drive to lz to the 4th position
                    driveSBSlowerTest(2700, 0.8, 10);
                    sleep(500);
                    opState++;
                }
                if (opState == 9 && opModeIsActive()) { //scoot over to block
                    driveLRTest(-150, 0.2, 5);
                    sleep(500);
                    opState++;
                }
                if (opState == 10 && opModeIsActive()) { //pick up block in 4th position
                    robot.arm.setPosition(robot.armDown);
                    sleep(500);
                    robot.grabber.setPosition(robot.aGrabberClosed);
                    sleep(500);
                    robot.arm.setPosition(robot.armUp);
                    sleep(500);
                    opState++;
                }
                if (opState == 11 && opModeIsActive()) { //line up with inner path
                    driveLRTest(150, 0.2, 5);
                    sleep(500);
                    opState++;
                }
                if (opState == 12 && opModeIsActive()) { //drive to foundation
                    driveSBSlowerTest(-2700, 0.8, 10);
                    sleep(500);
                    opState++;
                }
                if (opState == 13 && opModeIsActive()) { //move over to foundation
                    driveLRTest(-100, 0.2, 2); //this is smaller as to place this block in front of the previous
                    sleep(500);
                    opState++;
                }
                if (opState == 14 && opModeIsActive()) { //place second block
                    robot.arm.setPosition(robot.armDrop);
                    sleep(500);
                    robot.grabber.setPosition(robot.aGrabberOpenDrop);
                    sleep(500);
                    robot.arm.setPosition(robot.armUp);
                    robot.grabber.setPosition(robot.aGrabberOpenFull);
                    sleep(500);
                    opState++;
                }
                if (opState == 15 && opModeIsActive()) { //line up with inner path
                    driveLRTest(100, 0.2, 5);
                    sleep(500);
                    opState++;
                }
                if (opState == 16 && opModeIsActive()) { //drive to lz to the 2nd position
                    driveSBSlowerTest(2600, 0.8, 10);
                    sleep(500);
                    opState++;
                }
                if (opState == 17 && opModeIsActive()) { //scoot over to block
                    driveLRTest(-150, 0.2, 5);
                    sleep(500);
                    opState++;
                }
                if (opState == 18 && opModeIsActive()) { //pick up block in 2nd position (3rd block)
                    robot.arm.setPosition(robot.armDown);
                    sleep(500);
                    robot.grabber.setPosition(robot.aGrabberClosed);
                    sleep(500);
                    robot.arm.setPosition(robot.armUp);
                    sleep(500);
                    opState++;
                }
                if (opState == 19 && opModeIsActive()) { //line up with inner path
                    driveLRTest(150, 0.2, 5);
                    sleep(500);
                    opState++;
                }
                if (opState == 20 && opModeIsActive()) { //drive to foundation
                    driveSBSlowerTest(2700, 0.8, 10);//this should put it past the two already placed blocks
                    sleep(500);
                    opState++;
                }
                if (opState == 21 && opModeIsActive()) { //scoot over to foundation
                    driveLRTest(-200, 0.2, 5);
                    sleep(500);
                    opState++;
                }
                if (opState == 22 && opModeIsActive()) { //place block
                    robot.arm.setPosition(robot.armDown);
                    sleep(500);
                    robot.grabber.setPosition(robot.aGrabberOpenFull);
                    sleep(500);
                    robot.arm.setPosition(robot.armUp);
                    sleep(500);
                    opState++;
                }
                if (opState == 23 && opModeIsActive()) { //line up foundation grabbers with foundation
                    turnTestTest(-90, 0.4, 2);
                    sleep(500);
                    opState++;
                }
                if (opState == 24 && opModeIsActive()) { //drive to foundation
                    driveSBTest(50, 0.2, 5);
                    sleep(500);
                    opState++;
                }
                if (opState == 25 && opModeIsActive()) {
                    driveSBTest(-900, 0.4, 20);
                    sleep(1000);
                    robot.foundation_New1.setPosition(robot.foundationNew1Down);
                    robot.foundation_New2.setPosition(robot.foundationNew2Down);
                    sleep(100);
                    opState++;
                }
                if (opState == 26 && opModeIsActive()) {
                    driveSBTest(600, 0.4, 20);
                    sleep(100);
                    opState++;
                }
                if (opState == 27 && opModeIsActive()) {
                    turnTestTest(92, 0.4, 3);
                    sleep(100);
                    opState++;
                }
                if (opState == 28 && opModeIsActive()) {
                    driveSBTest(-70, 0.3, 20);
                    sleep(100);
                    robot.foundation_New1.setPosition(robot.foundationNew1Up);
                    robot.foundation_New2.setPosition(robot.foundationNew2Up);
                    opState++;
                }
                if (opState == 29 && opModeIsActive()) {
                    driveLRTest(-500, 0.4,  20);
                    sleep(1000);
                    opState++;
                }
                if (opState == 30 && opModeIsActive()) {
                    driveSBTest(1300,  0.5, 20);
                    sleep(1000);
                    stop();
                }
            }
        }
    }

    //FUNCTIONS
    public void driveSBTest (double duration, double speedPercent, double error) {
        double position = robot.back_right.getCurrentPosition();
        double target = position + duration;
        double distanceToTargetStart = Math.abs(target - position);
        double distanceToTarget = target - position;
        double percentToTarget = distanceToTarget/distanceToTargetStart;
        double speed = 0;
        double[] wheelSpeed = new double[4]; //fl, fr, bl, br
        double heading = getAbsoluteHeading();
        double turnSpeed = ((heading - getAbsoluteHeading())/20);

        while (Math.abs(distanceToTarget) > error && !isStopRequested()) {
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
    public void driveSBSlowerTest (double duration, double speedPercent, double error) {
        double position = robot.back_right.getCurrentPosition();
        double target = position + duration;
        double distanceToTargetStart = Math.abs(target - position);
        double distanceToTarget = target - position;
        double percentToTarget = distanceToTarget/distanceToTargetStart;
        double speed = 0;
        double[] wheelSpeed = new double[4]; //fl, fr, bl, br
        double heading = getAbsoluteHeading();
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

    public void driveLRTest (double duration, double speedPercent, double error) {
        double position = robot.back_right.getCurrentPosition();
        double target = position + duration;
        double distanceToTargetStart = Math.abs(target - position);
        double distanceToTarget = target - position;
        double percentToTarget = distanceToTarget/distanceToTargetStart;
        double speed = 0;
        double[] wheelSpeed = new double[4]; //fl, fr, bl, br
        double heading = getAbsoluteHeading();
        double turnSpeed = ((heading - getAbsoluteHeading())/20);

        while (Math.abs(distanceToTarget) > error && !isStopRequested()) {
            position = robot.back_right.getCurrentPosition();
            distanceToTarget = target - position;
            percentToTarget = distanceToTarget/distanceToTargetStart;
            if (percentToTarget >= 0) {
                speed = ((-((percentToTarget-1)*(percentToTarget-1)*(percentToTarget-1)*(percentToTarget-1))+1)*speedPercent)*0.9;
            }else if (percentToTarget < 0) {
                speed = -((-((percentToTarget+1)*(percentToTarget+1)*(percentToTarget+1)*(percentToTarget+1))+1)*speedPercent)*0.9;

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

    public void turnTest (double target, double error, double speedPercent, double direction) {
        double speed = 0;
        double angle = 0;
        if (direction >= 0) {
            angle = (getRelativeHeading()+360)%360;
        } else if (direction < 0) {
            angle = (-getRelativeHeading()+360)%360;
        }

        double DaS = target - angle;
        double distance = target - angle;
        double percent = distance/DaS;

        double[] wheelSpeed = new double[4]; //fl, fr, bl, br

        while (Math.abs(distance) > error && !isStopRequested()) {
            telemetry.addData("heading", heading);
            telemetry.addData("angle", angle);
            telemetry.addData("speed", speed);
            telemetry.addData("distance", distance);
            telemetry.addData("percent", percent);
            telemetry.update();

            if (direction >= 0) {
                angle = (getRelativeHeading()+360)%360;
            } else if (direction < 0) {
                angle = (-getRelativeHeading()+360)%360;
            }

            distance = target - angle;
            percent = distance/DaS;

            speed = speedPercent*(0.9*(Math.cbrt(percent)));

            wheelSpeed[0] = Range.clip(1, -1, -speed);
            wheelSpeed[1] = Range.clip(1, -1, speed);
            wheelSpeed[2] = Range.clip(1, -1, -speed);
            wheelSpeed[3] = Range.clip(1, -1, speed);

            motorSetSpeed(wheelSpeed[0], wheelSpeed[1],  wheelSpeed[2],  wheelSpeed[3]);
        }
        wheelSpeed[0] = 0;
        wheelSpeed[1] = 0;
        wheelSpeed[2] = 0;
        wheelSpeed[3] = 0;

        motorSetSpeed(wheelSpeed[0], wheelSpeed[1],  wheelSpeed[2],  wheelSpeed[3]);
    }

    public void motorSetSpeed (double fl, double fr, double bl, double br) {
        robot.front_left.setPower(fl);
        robot.front_right.setPower(fr);
        robot.back_left.setPower(bl);
        robot.back_right.setPower(br);
    };
    /*
    //Drive Straight TeleOp (TEST HEADING CORRECTION)
    public void driveStraightTeleOp(double duration, double allowableError, double minSpeed) {
        double startPosition = -robot.back_right.getCurrentPosition();
        double startAngle = angles.firstAngle;
        double target = startPosition + duration;
        double distanceToGo = target - -robot.back_right.getCurrentPosition();
        double progressToFinish =  distanceToGo / duration;

        double progressToAngle = Range.clip((angles.firstAngle - startAngle)/100, -1, 1);

        while (distanceToGo > allowableError) {
            distanceToGo = target - -robot.back_right.getCurrentPosition();
            if(distanceToGo / duration > 0.2) {
                progressToFinish = distanceToGo / duration;
            } else {
                progressToFinish = minSpeed;
            }
            double course = Math.atan2(-progressToFinish, 0) - Math.PI/2;
            double velocity = Math.hypot(0, progressToFinish);
            double rotation = -progressToAngle;

            drivetrain.setCourse(course);
            drivetrain.setVelocity(velocity);
            drivetrain.setRotation(rotation);

            telemetry.addData("pos:", -robot.back_right.getCurrentPosition());
            telemetry.addData("toGo:", distanceToGo);
            telemetry.update();
        }

        robot.front_left.setPower(0);
        robot.front_right.setPower(0);
        robot.back_left.setPower(0);
        robot.back_right.setPower(0);
    };

    //drive backward teleop (TEST HEADING CORRECTION)
    public void driveBackwardTeleOp(double duration, double allowableError, double minSpeed) {

        double startPosition = -robot.back_right.getCurrentPosition();
        double startAngle = angles.firstAngle;
        double target = startPosition + -(Math.abs(duration));
        double distanceToGo = target - -robot.back_right.getCurrentPosition();
        double progressToFinish =  distanceToGo / duration;

        double progressToAngle = Range.clip((angles.firstAngle - startAngle)/100, -1, 1);

        while (Math.abs(distanceToGo) > allowableError) {
            distanceToGo = target - -robot.back_right.getCurrentPosition();
            if(distanceToGo / Math.abs(duration) > 0.2) {
                progressToFinish = distanceToGo / duration;
            } else {
                progressToFinish = minSpeed;
            }
            double course = Math.atan2(progressToFinish, 0) - Math.PI/2;
            double velocity = Math.hypot(0, -progressToFinish);
            double rotation = -progressToAngle;

            drivetrain.setCourse(course);
            drivetrain.setVelocity(velocity);
            drivetrain.setRotation(rotation);

            telemetry.addData("pos:", -robot.back_right.getCurrentPosition());
            telemetry.addData("toGo:", distanceToGo);
            telemetry.update();
        }

        robot.front_left.setPower(0);
        robot.front_right.setPower(0);
        robot.back_left.setPower(0);
        robot.back_right.setPower(0);
    };
    //WIP
    //Encoder goes -
    public void driveLeftTeleOp(double duration, double allowableError, double minSpeed) {

        double startPosition = -robot.back_right.getCurrentPosition();
        double startAngle = angles.firstAngle;
        double target = startPosition + -(Math.abs(duration));
        double distanceToGo = target - -robot.back_right.getCurrentPosition();
        double progressToFinish =  distanceToGo / duration;

        double progressToAngle = Range.clip((angles.firstAngle - startAngle)/100, -1, 1);

        while (Math.abs(distanceToGo) > allowableError) {
            distanceToGo = target - -robot.back_right.getCurrentPosition();
            if(distanceToGo / Math.abs(duration) > 0.2) {
                progressToFinish = distanceToGo / duration;
            } else {
                progressToFinish = minSpeed;
            }
            double course = Math.atan2(0, progressToFinish) - Math.PI/2;
            double velocity = Math.hypot(progressToFinish, 0);
            double rotation = -progressToAngle;

            drivetrain.setCourse(course);
            drivetrain.setVelocity(velocity);
            drivetrain.setRotation(rotation);

            telemetry.addData("pos:", -robot.back_right.getCurrentPosition());
            telemetry.addData("toGo:", distanceToGo);
            telemetry.update();
        }

        robot.front_left.setPower(0);
        robot.front_right.setPower(0);
        robot.back_left.setPower(0);
        robot.back_right.setPower(0);
    };
    //WIP
    //Encoder goes +
    public void driveRightTeleOp(double duration, double allowableError, double minSpeed) {
        double startPosition = -robot.back_right.getCurrentPosition();
        double startAngle = angles.firstAngle;
        double target = startPosition + duration;
        double distanceToGo = target - -robot.back_right.getCurrentPosition();
        double progressToFinish =  distanceToGo / duration;

        double progressToAngle = Range.clip((angles.firstAngle - startAngle)/100, -1, 1);

        while (distanceToGo > allowableError) {
            distanceToGo = target - -robot.back_right.getCurrentPosition();
            if(distanceToGo / duration > 0.2) {
                progressToFinish = distanceToGo / duration;
            } else {
                progressToFinish = minSpeed;
            }
            double course = Math.atan2(0, progressToFinish) - Math.PI/2;
            double velocity = Math.hypot(progressToFinish, 0);
            double rotation = -progressToAngle;

            huge_penis_in_my_butthole;
            drivetrain.setCourse(course);
            drivetrain.setVelocity(velocity);
            drivetrain.setRotation(rotation);

            telemetry.addData("pos:", -robot.back_right.getCurrentPosition());
            telemetry.addData("toGo:", distanceToGo);
            telemetry.update();
        }

        robot.front_left.setPower(0);
        robot.front_right.setPower(0);
        robot.back_left.setPower(0);
        robot.back_right.setPower(0);
    };

    //up right positive
    public void driveUpRightTeleOp(double duration, double allowableError, double minSpeed) {
        double startPosition = -robot.back_right.getCurrentPosition();
        double startAngle = angles.firstAngle;
        double target = startPosition + duration;
        double distanceToGo = target - -robot.back_right.getCurrentPosition();
        double progressToFinish =  distanceToGo / duration;

        double progressToAngle = Range.clip((angles.firstAngle - startAngle)/100, -1, 1);

        while (distanceToGo > allowableError) {
            distanceToGo = target - -robot.back_right.getCurrentPosition();
            if(distanceToGo / duration > 0.2) {
                progressToFinish = distanceToGo / duration;
            } else {
                progressToFinish = minSpeed;
            }
            double course = Math.atan2(-progressToFinish, progressToFinish) - Math.PI/2;
            double velocity = Math.hypot(progressToFinish, progressToFinish);
            double rotation = -progressToAngle;

            drivetrain.setCourse(course);
            drivetrain.setVelocity(velocity);
            drivetrain.setRotation(rotation);

            telemetry.addData("pos:", -robot.back_right.getCurrentPosition());
            telemetry.addData("toGo:", distanceToGo);
            telemetry.update();
        }

        robot.front_left.setPower(0);
        robot.front_right.setPower(0);
        robot.back_left.setPower(0);
        robot.back_right.setPower(0);
    };
    //ENCODER GOES NEGATIVE
    public void driveDownLeftTeleOp(double duration, double allowableError, double minSpeed) {

        double startPosition = -robot.back_right.getCurrentPosition();
        double startAngle = angles.firstAngle;
        double target = startPosition + -(Math.abs(duration));
        double distanceToGo = target - -robot.back_right.getCurrentPosition();
        double progressToFinish =  distanceToGo / duration;

        double progressToAngle = Range.clip((angles.firstAngle - startAngle)/100, -1, 1);

        while (Math.abs(distanceToGo) > allowableError) {
            distanceToGo = target - -robot.back_right.getCurrentPosition();
            if(distanceToGo / Math.abs(duration) > 0.2) {
                progressToFinish = distanceToGo / duration;
            } else {
                progressToFinish = minSpeed;
            }
            double course = Math.atan2(-progressToFinish, progressToFinish) - Math.PI/2;
            double velocity = Math.hypot(progressToFinish, progressToFinish);
            double rotation = -progressToAngle;

            drivetrain.setCourse(course);
            drivetrain.setVelocity(velocity);
            drivetrain.setRotation(rotation);

            telemetry.addData("pos:", -robot.back_right.getCurrentPosition());
            telemetry.addData("toGo:", distanceToGo);
            telemetry.update();
        }
        huge_penis_in_my_butthole;
        robot.front_left.setPower(0);
        robot.front_right.setPower(0);
        robot.back_left.setPower(0);
        robot.back_right.setPower(0);
    };

    public void driveUpLeftTeleOp(double duration, double allowableError, double minSpeed) {
        double startPosition = robot.back_left.getCurrentPosition();
        double startAngle = angles.firstAngle;
        double target = startPosition + duration;
        double distanceToGo = target - robot.back_left.getCurrentPosition();
        double progressToFinish =  distanceToGo / duration;

        double progressToAngle = Range.clip((angles.firstAngle - startAngle)/100, -1, 1);

        while (distanceToGo > allowableError) {
            distanceToGo = target - robot.back_left.getCurrentPosition();
            if(distanceToGo / duration > 0.2) {
                progressToFinish = distanceToGo / duration;
            } else {
                progressToFinish = minSpeed;
            }
            double course = Math.atan2(-progressToFinish, 0) - Math.PI/2;
            double velocity = Math.hypot(0, progressToFinish);
            double rotation = -progressToAngle;

            drivetrain.setCourse(course);
            drivetrain.setVelocity(velocity);
            drivetrain.setRotation(rotation);

            telemetry.addData("pos:", robot.back_left.getCurrentPosition());
            telemetry.addData("toGo:", distanceToGo);
            telemetry.update();
        }

        robot.front_left.setPower(0);
        robot.front_right.setPower(0);
        robot.back_left.setPower(0);
        robot.back_right.setPower(0);
    };
    */
    //DRIVE STRAIGHT USING ENCODERS
    public void driveStraight(double duration, double power) {
        double leftSpeed; //Power to feed the motors
        double rightSpeed;
        double currentdirection;

        double target = angles.firstAngle;  //Starting direction
        double startPosition = robot.back_right.getCurrentPosition();//Starting position


        while (robot.back_right.getCurrentPosition() < duration + startPosition && opModeIsActive()) {  //While we have not passed out intended distance
            currentdirection = angles.firstAngle;  //Current direction

            leftSpeed = power + (angles.firstAngle - target) / 100;  //Calculate speed for each side
            rightSpeed = power - (angles.firstAngle - target) / 100;  //See Gyro Straight video for detailed explanation

            leftSpeed = Range.clip(leftSpeed, -1, 1);
            rightSpeed = Range.clip(rightSpeed, -1, 1);

            robot.back_left.setPower(leftSpeed);
            robot.front_left.setPower(leftSpeed);
            robot.back_right.setPower(rightSpeed);
            robot.front_right.setPower(rightSpeed);

            telemetry.addData("1. Back Left", robot.back_left.getCurrentPosition());
            telemetry.addData("2. Front Left", robot.front_left.getCurrentPosition());
            telemetry.addData("3. Back Right", robot.back_right.getCurrentPosition());
            telemetry.addData("4. Front Right", robot.front_right.getCurrentPosition());
            telemetry.addData("5. Distance to go", duration + startPosition - robot.back_right.getCurrentPosition());
            telemetry.update();
        }

        robot.back_left.setPower(0);
        robot.front_left.setPower(0);
        robot.back_right.setPower(0);
        robot.front_right.setPower(0);
    }

    //DRIVE BACKWARDS USING ENCODERS
    public void drivebackwards(int duration, double power) {
        double leftSpeed; //Power to feed the motors
        double rightSpeed;
        double currentdirection;

        double target = angles.firstAngle;  //Starting direction
        double startPosition = robot.back_right.getCurrentPosition();  //Starting position

        while (robot.back_right.getCurrentPosition() > duration + startPosition && opModeIsActive()) {  //While we have not passed out intended distance
            currentdirection = angles.firstAngle;  //Current direction

            //leftSpeed = power + (zAccumulated - target) / 100;  //Calculate speed for each side
            // rightSpeed = power - (zAccumulated - target) / 100;  //See Gyro Straight video for detailed explanation
            //huge_penis_in_my_butthole;
            leftSpeed = 0.2;
            rightSpeed = 0.2;

            leftSpeed = Range.clip(leftSpeed, -1, 1);
            rightSpeed = Range.clip(rightSpeed, -1, 1);

            robot.back_left.setPower(-leftSpeed);
            robot.front_left.setPower(-leftSpeed);
            robot.back_right.setPower(-rightSpeed);
            robot.front_right.setPower(-rightSpeed);

            telemetry.addData("1. Back Left", robot.back_left.getCurrentPosition());
            telemetry.addData("2. Front Left", robot.front_left.getCurrentPosition());
            telemetry.addData("3. Back Right", robot.back_right.getCurrentPosition());
            telemetry.addData("4. Front Right", robot.front_right.getCurrentPosition());
            telemetry.addData("5. Distance to go", duration + startPosition - robot.back_right.getCurrentPosition());
            telemetry.update();
        }

        robot.back_left.setPower(0);
        robot.front_left.setPower(0);
        robot.back_right.setPower(0);
        robot.front_right.setPower(0);
    }

    //TURN LEFT USING ENCODERS
    public void TurnLeft(double duration, double power) {
        double leftSpeed; //Power to feed the motors
        double rightSpeed;
        double currentdirection;

        double target = angles.firstAngle;  //Starting direction
        double startPosition = robot.back_right.getCurrentPosition();//Starting position


        while (robot.back_right.getCurrentPosition() < duration + startPosition && opModeIsActive()) {  //While we have not passed out intended distance
            currentdirection = angles.firstAngle;  //Current direction

            //leftSpeed = power + (angles.firstAngle - target) / 100;  //Calculate speed for each side
            //rightSpeed = power - (angles.firstAngle - target) / 100;  //See Gyro Straight video for detailed explanation
            leftSpeed=0.15;
            rightSpeed=0.15;

            leftSpeed = Range.clip(leftSpeed, .1, 1);
            rightSpeed = Range.clip(rightSpeed, -1, 1);

            robot.back_left.setPower(-leftSpeed);
            robot.front_left.setPower(-leftSpeed);
            robot.back_right.setPower(rightSpeed);
            robot.front_right.setPower(rightSpeed);

            telemetry.addData("1. Back Left", robot.back_left.getCurrentPosition());
            telemetry.addData("2. Front Left", robot.front_left.getCurrentPosition());
            telemetry.addData("3. Back Right", robot.back_right.getCurrentPosition());
            telemetry.addData("4. Front Right", robot.front_right.getCurrentPosition());
            telemetry.addData("5. Distance to go", duration + startPosition - robot.back_right.getCurrentPosition());
            telemetry.update();
        }

        robot.back_left.setPower(0);
        robot.front_left.setPower(0);
        robot.back_right.setPower(0);
        robot.front_right.setPower(0);
    }

    //TURN RIGHT USING ENCODERS
    public void TurnRight(int duration, double power) {
        double leftSpeed; //Power to feed the motors
        double rightSpeed;
        double currentdirection;

        double target = angles.firstAngle;  //Starting direction
        double startPosition = robot.back_right.getCurrentPosition();  //Starting position

        while (robot.back_right.getCurrentPosition() > duration + startPosition && opModeIsActive()) {  //While we have not passed out intended distance
            currentdirection = angles.firstAngle;  //Current direction

            //leftSpeed = power + (zAccumulated - target) / 100;  //Calculate speed for each side
            //rightSpeed = power - (zAccumulated - target) / 100;  //See Gyro Straight video for detailed explanation
            leftSpeed= 0.15;
            rightSpeed= 0.15;

            leftSpeed = Range.clip(leftSpeed, -1, 1);
            rightSpeed = Range.clip(rightSpeed, -1, 1);

            robot.back_left.setPower(leftSpeed);
            robot.front_left.setPower(leftSpeed);
            robot.back_right.setPower(-rightSpeed);
            robot.front_right.setPower(-rightSpeed);

            telemetry.addData("1. Back Left", robot.back_left.getCurrentPosition());
            telemetry.addData("2. Front Left", robot.front_left.getCurrentPosition());
            telemetry.addData("3. Back Right", robot.back_right.getCurrentPosition());
            telemetry.addData("4. Front Right", robot.front_right.getCurrentPosition());
            telemetry.addData("5. Distance to go", duration + startPosition - robot.back_right.getCurrentPosition());
            telemetry.update();
        }

        robot.back_left.setPower(0);
        robot.front_left.setPower(0);
        robot.back_right.setPower(0);
        robot.front_right.setPower(0);
    }

    //TURN ABSOLUTE USING GYRO
    public void turnAbsolute(int target) {
        double direction = angles.firstAngle;  //Set variables to gyro readings
        double minspeed = .1;
        double maxspeed = .2;
        double errorDegs = Math.abs(direction-target);
        double turnSpeed = maxspeed * (errorDegs/180) + minspeed;

        while (errorDegs > 3 && opModeIsActive()) {  //Continue while the robot direction is further than three degrees from the target
            if (direction > target) {  //if gyro is positive, we will turn right
                robot.back_left.setPower(-turnSpeed);
                robot.front_left.setPower(-turnSpeed);
                robot.back_right.setPower(turnSpeed);
                robot.front_right.setPower(turnSpeed);
            }

            if (direction < target) {  //if gyro is positive, we will turn left
                robot.back_left.setPower(turnSpeed);
                robot.front_left.setPower(turnSpeed);
                robot.back_right.setPower(-turnSpeed);
                robot.front_right.setPower(-turnSpeed);
            }

            direction = angles.firstAngle;  //Set variables to gyro readings
            errorDegs = Math.abs(direction-target);
            turnSpeed = maxspeed * (errorDegs/180) + minspeed;
            telemetry.addData("accu", String.format("%03d", zAccumulated));
            telemetry.update();
        }


        robot.back_left.setPower(0);
        robot.front_left.setPower(0);
        robot.back_right.setPower(0);
        robot.front_right.setPower(0);
    }

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

    private int correctCount = 0;
    /*
     * @param gyroTarget The target heading in degrees, between 0 and 360
     * @param gyroRange The acceptable range off target in degrees, usually 1 or 2
     * @param gyroActual The current heading in degrees, between 0 and 360
     * @param minSpeed The minimum power to apply in order to turn (e.g. 0.05 when moving or 0.15 when stopped)
     * @param addSpeed The maximum additional speed to apply in order to turn (proportional component), e.g. 0.3
     * @return The number of times in a row the heading has been in the range
     */
    public int gyroCorrect(double gyroTarget, double gyroRange, double gyroActual, double minSpeed, double addSpeed) {
        double delta = (gyroTarget - gyroActual + 360.0) % 360.0; //the difference between target and actual mod 360
        if (delta > 180.0) delta -= 360.0; //makes delta between -180 and 180
        if (Math.abs(delta) > gyroRange) { //checks if delta is out of range
            this.correctCount = 0;
            double gyroMod = delta / 45.0; //scale from -1 to 1 if delta is less than 45 degrees
            if (Math.abs(gyroMod) > 4.0) gyroMod = Math.signum(gyroMod); //set gyromod to 1 or -1 if the error is more than 45 degrees
            this.turn(minSpeed * Math.signum(gyroMod) + addSpeed * gyroMod);
        }
        else {
            this.correctCount++;
            this.turn(0.0);
        }
        return this.correctCount;
    }

    //BASIC TURN
    public void turn(double sPower) {
        robot.front_left.setPower(- sPower);
        robot.back_left.setPower(- sPower);
        robot.front_right.setPower(+ sPower);
        robot.back_right.setPower(+ sPower);
    }
    //COMPOSE TELEMETRY
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

