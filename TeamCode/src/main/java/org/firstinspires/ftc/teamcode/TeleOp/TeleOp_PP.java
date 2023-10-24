package org.firstinspires.ftc.teamcode.TeleOp;


import static org.firstinspires.ftc.teamcode.Utility.GlobalValues.down;
import static org.firstinspires.ftc.teamcode.Utility.GlobalValues.high;
import static org.firstinspires.ftc.teamcode.Utility.GlobalValues.low;
import static org.firstinspires.ftc.teamcode.Utility.GlobalValues.middle;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.commands.core.LynxResetMotorEncoderCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utility.ConfigurationName;
import org.firstinspires.ftc.teamcode.Utility.GlobalValues;


//@Disabled
@Config
@TeleOp(name="TeleOp_Full", group="linop")
public class TeleOp_PP extends LinearOpMode {


    boolean Drivetrain = true;
    boolean Slides = true;
    boolean pickup = true;


    boolean AlwaysTrue = true;

    public static double turnfactor;
    public static double maxspeed;

    private DcMotor leftRear;
    
    private DcMotor rightRear;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor cranearm;

    private Servo grijper;
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

        cranearm = hardwareMap.dcMotor.get(ConfigurationName.craneArm);
        cranearm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cranearm.setDirection(DcMotor.Direction.REVERSE);
        cranearm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        grijper = hardwareMap.servo.get(ConfigurationName.grijper);

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

           if (Slides) {
               if (gamepad2.b) {
                   cranearm.setTargetPosition(down);
                   cranearm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
               } else if (gamepad2.y) {
                   cranearm.setTargetPosition(high);
                   cranearm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
               } else if (gamepad2.x) {
                   cranearm.setTargetPosition(middle);
                   cranearm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
               } else if (gamepad2.a) {
                   cranearm.setTargetPosition(low);
                   cranearm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
               }

               double cl = -gamepad2.left_stick_y; // Remember, this is reversed!
               double cr = -gamepad2.right_stick_y; // Counteract imperfect strafing

               double cranearmpower = (cl / 1.5) + (cr / 2.5);


               if (cranearm.isBusy() && cranearmpower == 0) {
                   cranearm.setPower(0.5);
               } else if (cranearm.isBusy() && cranearmpower != 0) {
                   cranearm.setPower(0.5);
               } else if (cranearm.isBusy() == false && cranearmpower != 0) {
                   cranearm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                   cranearm.setPower(cranearmpower);
               } else if (cranearm.isBusy() == false && cranearmpower == 0) {
                   cranearm.setPower(0);
               }

               FtcDashboard dashboard = FtcDashboard.getInstance();
               Telemetry dashboardTelemetry = dashboard.getTelemetry();

               dashboardTelemetry.addData("Current Pos:", cranearm.getCurrentPosition());
               dashboardTelemetry.addData("Target Pos:", cranearm.getTargetPosition());

               dashboardTelemetry.update();
           }


           if (pickup) {
               if (Math.abs(gamepad2.left_trigger) > 0.02) {
                   grijper.setPosition(GlobalValues.servoopen);
               } else if (Math.abs(gamepad2.right_trigger) > 0.02) {
                   grijper.setPosition(GlobalValues.servoclosed);
               }
           }


           if (Drivetrain) {
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

               double y = -gamepad1.left_stick_y; // Remember, this is reversed!
               double x = gamepad1.left_stick_x; // Counteract imperfect strafing
               double rx = gamepad1.right_stick_x;

               double botHeading = -imu.getAngularOrientation().firstAngle;

               if (gamepad1.left_bumper&&gamepad1.right_bumper){
                   imu.initialize(parameters);
               }
               if (gamepad2.left_bumper&&gamepad2.right_bumper){
                   cranearm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

    }}

