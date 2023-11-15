package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utility.ConfigurationName;

//@Disabled
@Config
@TeleOp(name="Rijden, Slides, Nout Grijper", group="linop")
public class TeleOp_GrijpNoutSlidesRijden extends LinearOpMode {

    public static double turnfactor;
    public static double maxspeed;

    public static double twistpickup = 0.13;
    public static double twistdrop = 0.81;
    public static double linksdrop = 0.88;
    public static double linkspickup = 0.62;
    public static double links1drop = 0.7;
    public static double rechtsdrop = 0.14;
    public static double rechtspickup = 0.45;
    public static double rechts1drop = 0.3;


    private DcMotor leftRear;
    
    private DcMotor rightRear;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor slides;
    private Servo twist;
    private Servo linksklauw;
    private Servo rechtsklauw;

    BNO055IMU imu;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftFront = hardwareMap.dcMotor.get(ConfigurationName.leftFront);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);

        leftRear = hardwareMap.dcMotor.get(ConfigurationName.leftRear);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);

        rightFront = hardwareMap.dcMotor.get(ConfigurationName.rightFront);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        rightRear = hardwareMap.dcMotor.get(ConfigurationName.rightRear);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        slides = hardwareMap.dcMotor.get(ConfigurationName.slides);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setDirection(DcMotor.Direction.REVERSE);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        twist = hardwareMap.servo.get(ConfigurationName.twist);

        rechtsklauw = hardwareMap.servo.get(ConfigurationName.rechtsklauw);

        linksklauw = hardwareMap.servo.get(ConfigurationName.linksklauw);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        waitForStart();


        while (opModeIsActive()) {
            if (gamepad2.left_trigger>0.02){
                linksklauw.setPosition(linkspickup);
            } else if (gamepad2.right_trigger>0.02){
                rechtsklauw.setPosition(rechtspickup);
            }

            if (gamepad2.right_stick_button){
                twist.setPosition(twistdrop);
            }
            if (gamepad2.left_stick_button && slides.getCurrentPosition()>250){
                twist.setPosition(twistpickup);
            }
            if(gamepad2.left_bumper){
                slides.setTargetPosition(slides.getCurrentPosition()+15);
                slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if(gamepad2.a){
                linksklauw.setPosition(links1drop);
                rechtsklauw.setPosition(rechts1drop);
            }
            if (gamepad2.x){
                rechtsklauw.setPosition(rechtsdrop);
            }
            if (gamepad2.b){
                linksklauw.setPosition(linksdrop);
            }
            if (gamepad2.y){
                slides.setTargetPosition(580);
                slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }
            telemetry.addData("Links: ",linksklauw.getPosition());
            telemetry.addData("Rechts: ",rechtsklauw.getPosition());
            telemetry.addData("Pols: ",twist.getPosition());
            telemetry.addData("Slides: ",slides.getCurrentPosition());
            telemetry.addData("SlidesTarget: ",slides.getTargetPosition());
            telemetry.update();




            double cl = -gamepad2.left_stick_y; // Remember, this is reversed!
            double cr = -gamepad2.right_stick_y; // Counteract imperfect strafing

            double slidespower = (cl / 1.5) + (cr / 2.5);

            if (slides.isBusy() && slidespower == 0) {
                slides.setPower(0.5);
            } else if (slides.isBusy() && slidespower != 0) {
                slides.setPower(0.5);
            } else if (slides.isBusy() == false && slidespower != 0) {
                slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slides.setPower(slidespower);
            } else if (slides.isBusy() == false && slidespower == 0) {
                slides.setPower(0);
            }


            if (Math.abs(gamepad1.right_trigger) < 0.02) {
                   turnfactor = 0.9;
                   maxspeed = 1.2;

               } else if (Math.abs(gamepad1.right_trigger) > 0.02) {
                   turnfactor = (-0.3*gamepad1.right_trigger)+0.9;
                   maxspeed = (-0.9*gamepad1.right_trigger)+1.2;

               }else if (Math.abs(gamepad1.left_trigger) > 0.02) {
                   turnfactor = (0.3*gamepad1.left_trigger)+0.9;
                   maxspeed = (0.4*gamepad1.left_trigger)+1.2;

               }

            if (gamepad1.a) {
                turnfactor = 0.9;
                maxspeed = 1.2;

                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);

            }

               double y = -gamepad1.left_stick_y; // Remember, this is reversed!
               double x = gamepad1.left_stick_x; // Counteract imperfect strafing
               double rx = gamepad1.right_stick_x;

               double botHeading = -imu.getAngularOrientation().firstAngle;

               if (gamepad1.right_bumper){
                   imu.initialize(parameters);
               }

               double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
               double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

               double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
               double frontLeftPower = ((rotY + rotX + (rx * turnfactor)) / denominator) * maxspeed;
               double backLeftPower = ((rotY - rotX + (rx * turnfactor)) / denominator) * maxspeed;
               double frontRightPower = ((rotY - rotX - (rx * turnfactor)) / denominator) * maxspeed;
               double backRightPower = ((rotY + rotX - (rx * turnfactor)) / denominator) * maxspeed;

               leftFront.setPower(frontLeftPower);
               leftRear.setPower(backLeftPower);
               rightFront.setPower(frontRightPower);
               rightRear.setPower(backRightPower);
            }

        }

    }

