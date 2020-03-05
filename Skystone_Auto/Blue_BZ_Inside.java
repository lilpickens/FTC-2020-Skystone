package org.firstinspires.ftc.teamcode.Skystone_Auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

@Autonomous(name="Blue_Foundation_Inside", group="14174")
//@Disabled
public class Blue_BZ_Inside extends LinearOpMode {

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
    
    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.init(hardwareMap);

        robot.front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.front_right.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.back_right.setDirection(DcMotorSimple.Direction.FORWARD);
        
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

        //DEFINE SENSORS
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters2);

        //Setup The Telemetry Dashboard
        composeTelemetry();

        //Initilization
        int opState = 0;

        robot.foundation_New1.setPosition(robot.foundationNew1Up);
        robot.foundation_New2.setPosition(robot.foundationNew2Up);

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
            //blue bz DONE

            if (opState == 0 && opModeIsActive()) {
                driveLRTest(1200, 1, 20);
                sleep(100);
                opState++;
            }

            if (opState == 1 && opModeIsActive()) {
                driveSBTest(-900, 0.4, 20);
                sleep(1000);
                robot.foundation_New1.setPosition(robot.foundationNew1Down);
                robot.foundation_New2.setPosition(robot.foundationNew2Down);
                sleep(100);
                opState++;
            }

            if (opState == 2 && opModeIsActive()) {
                driveSBTest(600, 0.4, 20);
                sleep(100);
                opState++;
            }

            if (opState == 3 && opModeIsActive()) {
                turnTestTest(92, 0.4, 3);
                sleep(100);
                opState++;
            }

            if (opState == 4 && opModeIsActive()) {
                driveSBTest(-70, 0.3, 20);
                sleep(100);
                robot.foundation_New1.setPosition(robot.foundationNew1Up);
                robot.foundation_New2.setPosition(robot.foundationNew2Up);
                opState++;
            }

            if (opState == 5 && opModeIsActive()) {
                driveLRTest(-500, 0.4,  20);
                sleep(1000);
                opState++;
            }

            if (opState == 6 && opModeIsActive()) {
                driveSBTest(1300,  0.5, 20);
                sleep(1000);
                stop();
            }

            /*
            //blue lz in

            //line up with block
            if (opState == 0 && opModeIsActive()) {
                driveSBTest(300, 0.2, 20);
                sleep(1000);
                opState++;
            }

            //move over to the blocks
            if (opState == 1 && opModeIsActive()) {
                driveLRTest(-1575, 0.6, 20);
                sleep(1000);
                opState++;
            }

            // collect a block
            if (opState == 2 && opModeIsActive()) {
                //Spin Motors
                sleep(1000);
                driveSBTest(200, 0.1, 10);
                //Stop Motors
                sleep(1000);
                opState++;
            }

            //line up with inside path
            if (opState == 3 && opModeIsActive()) {
                driveLRTest(600, 0.4, 20);
                sleep(1000);
                opState++;
            }

            //back to other side of field
            if (opState == 4 && opModeIsActive()) {
                driveSBTest(-1500, 0.6, 20);
                sleep(1000);
                opState++;
            }

            //turn and spit out block
            if (opState == 5 && opModeIsActive()) {
                turnTestTest(90, 0.5, 1);
                sleep(1000);
                //reverse motor speed
                sleep(1000);
                //motor off
                opState++;
            }

            //turn back facing lz
            if (opState == 6 && opModeIsActive()) {
                turnTestTest(0, 0.4,  1); //////////////////////////////////////////////////////////////////////
                sleep(1000);
                stop();
                //opState++;
            }

            //drive back to lz
            if (opState == 7 && opModeIsActive()) {
                driveSBTest(850, 0.5, 20);
                sleep(1000);
                opState++;
            }
            //dive sideways to line up with blocks
            if (opState == 8 && opModeIsActive()) {
                driveLRTest(-400, 0.4, 20);
                sleep(1000);
                opState++;
            }

            //drive forward to collect block
            if (opState == 9 && opModeIsActive()) {
                //motors on
                driveSBTest(100, 0.2, 10);
                sleep(1000);
                //motors off
                opState++;
            }

            //drive sideways to line up with inside path
            if (opState == 10 && opModeIsActive()) {
                driveLRTest(400, 0.4, 20);
                sleep(1000);
                opState++;
            }

            //drive to bz
            if (opState == 11 && opModeIsActive()) {
                driveSBTest(-850, 0.5, 20);
                sleep(1000);
                opState++;
            }

            //turn and spit out block
            if (opState == 12 && opModeIsActive()) {
                turnTestTest(100, 0.5, 2);
                sleep(1000);
                stop();
            }
            */

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

