package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.Utility.GlobalValues.links1drop;
import static org.firstinspires.ftc.teamcode.Utility.GlobalValues.linksdrop;
import static org.firstinspires.ftc.teamcode.Utility.GlobalValues.linkspickup;
import static org.firstinspires.ftc.teamcode.Utility.GlobalValues.rechts1drop;
import static org.firstinspires.ftc.teamcode.Utility.GlobalValues.rechtsdrop;
import static org.firstinspires.ftc.teamcode.Utility.GlobalValues.rechtspickup;
import static org.firstinspires.ftc.teamcode.Utility.GlobalValues.twistdrop;
import static org.firstinspires.ftc.teamcode.Utility.GlobalValues.twistpickup;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Utility.ConfigurationName;
import org.firstinspires.ftc.teamcode.Utility.GlobalValues;

//@Disabled
@Config
@TeleOp(name="TeleOp_Full", group="linop")
public class TeleOp_Full extends LinearOpMode {

    public static double turnfactor;
    public static double maxspeed;



    public static double leftlow = 0;
    public static double lefthigh = 0.85;
    public static double leftmid = 0.51;
    public static double leftbabymid = 0.4;


    public static double rightmid = 0.52;
    public static double rightbabymid = 0.6;
    public static double rightlow = 1;
    public static double righthigh = 0.18;
    public static double planeinit = 0.265;
    public static double planeshoot = 0.4;


    public enum Slideys {
        GROUND,
        TWISTDROP,
        Up,
        Down,
        TWISTPICKUP,
        TWISTEXTRA
    }
    Slideys slideystate = Slideys.GROUND;

    public enum controls {
        Normal,
        Hangplane,
        Hangensmooth
    }
    controls controlstate = controls.Normal;

    public enum rumble {
        left,
        right,
        nothing
    }
    rumble rumblestate = rumble.nothing;

    private DcMotor leftRear;
    
    private DcMotor rightRear;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor slides;
    private Servo twist;
    private Servo plane;

    private Servo linksklauw;
    private Servo rechtsklauw;
    private DistanceSensor linkssensor;
    private DistanceSensor rechtsssensor;



    private Servo lefthang;
    private Servo righthang;
    public boolean lefttwisted;
    public boolean bothclosed;
    public boolean sensor = false;

    public boolean rumbled = false;
    public boolean leftrumbled = false;
    double TwistWait = 1.0;

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

        righthang = hardwareMap.servo.get(ConfigurationName.righthang);

        lefthang = hardwareMap.servo.get(ConfigurationName.lefthang);
        plane = hardwareMap.servo.get(ConfigurationName.plane);

        linkssensor = hardwareMap.get(DistanceSensor.class, ConfigurationName.linkssensor);

        rechtsssensor = hardwareMap.get(DistanceSensor.class, ConfigurationName.rechtssensor);



        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        lefthang.setPosition(leftmid);
        righthang.setPosition(rightmid);
        plane.setPosition(planeinit);

        ElapsedTime WaitTimer = new ElapsedTime();

        waitForStart();


        while (opModeIsActive()) {

//            if (gamepad2.y){
//                slides.setTargetPosition(300);
//                slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                twist.setPosition(twistdrop);
//            }
            switch (slideystate) {
                case GROUND:
                    if (gamepad2.triangle && controlstate.equals(controls.Normal)) {
                        slides.setTargetPosition(400);
                        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        slideystate = Slideys.TWISTDROP;
                    }
                    if (gamepad2.square && controlstate.equals(controls.Normal)) {
                        if (twist.getPosition() == twistpickup) {
                            slides.setTargetPosition(0);
                            slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            slideystate = Slideys.GROUND;

                        } else if (Math.abs(slides.getCurrentPosition()) > 250) {
                            twist.setPosition(twistpickup);
                            WaitTimer.reset();
                            slideystate = Slideys.TWISTPICKUP;
                        } else {
                            slides.setTargetPosition(300);
                            slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            slideystate = Slideys.TWISTEXTRA;
                        }
                    }
                    break;
                case TWISTDROP:
                    if (Math.abs(slides.getCurrentPosition() - 400) < 30) {
                        twist.setPosition(twistdrop);
                        slideystate = Slideys.GROUND;
                    }
                    break;
                case TWISTPICKUP:
                    if (WaitTimer.seconds() >= TwistWait) {
                        slides.setTargetPosition(0);
                        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        slideystate = Slideys.GROUND;
                    }
                    break;
                case TWISTEXTRA:
                    if (Math.abs(slides.getCurrentPosition()) > 250) {
                        twist.setPosition(twistpickup);
                        WaitTimer.reset();
                        slideystate = Slideys.TWISTPICKUP;
                    }
                    break;
            }
            switch (controlstate) {
                case Normal:
                    gamepad2.setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
                    if (twist.getPosition() > 0.5) {
                        if (gamepad2.left_trigger > 0.02) {
                            linksklauw.setPosition(linkspickup);
                        }
                        if (gamepad2.right_trigger > 0.02) {
                            rechtsklauw.setPosition(rechtspickup);
                        }
                        if (gamepad2.left_bumper) {
                            linksklauw.setPosition(linksdrop);
                        }
                        if (gamepad2.right_bumper) {
                            rechtsklauw.setPosition(rechtsdrop);
                        }

                    }
                    if (twist.getPosition() < 0.5) {
                        if (gamepad2.left_trigger > 0.02) {
                            rechtsklauw.setPosition(rechtspickup);
                        }
                        if (gamepad2.right_trigger > 0.02) {
                            linksklauw.setPosition(linkspickup);
                        }
                        if (gamepad2.left_bumper) {
                            rechtsklauw.setPosition(rechtsdrop);
                        }
                        if (gamepad2.right_bumper) {
                            linksklauw.setPosition(linksdrop);
                        }

                    }

                    if (gamepad2.right_stick_button) {
                        twist.setPosition(twistpickup);
                    }
                    if (gamepad2.left_stick_button && slides.getCurrentPosition() > 250) {
                        twist.setPosition(twistdrop);
                    }

                    if (gamepad2.cross) {
                        linksklauw.setPosition(links1drop);
                        rechtsklauw.setPosition(rechts1drop);
                    }
                    if (gamepad2.dpad_up) {
                        slides.setTargetPosition(slides.getCurrentPosition() + 15);
                        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }
                    if (gamepad2.touchpad){
                        sensor = true;
                    } else {
                        sensor = false;
                    }

//                    if (gamepad2.share && slideystate != Slideys.GROUND) {
//                        slideystate = Slideys.GROUND;
//                    }
                    if (gamepad2.share){
                        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    }
                    if (gamepad2.options&&gamepad2.share){
                        controlstate = controls.Hangensmooth;
                        gamepad2.rumbleBlips(3);
                    }
                    telemetry.addLine("NORMAALSLET");
                break;
                case Hangensmooth:
                    gamepad2.setLedColor(1,0,0.91, Gamepad.LED_DURATION_CONTINUOUS);
                    if (gamepad2.right_trigger>0.02){
                        lefthang.setPosition(leftlow);
                        righthang.setPosition(rightlow);
                    } if (gamepad2.left_trigger>0.02) {
                        lefthang.setPosition(lefthigh);
                        righthang.setPosition(righthigh);}
                    if (gamepad2.a){
                        lefthang.setPosition(leftmid);
                        righthang.setPosition(rightmid);
                    }
                    if(gamepad2.touchpad){
                        controlstate = controls.Normal;
                    }
                    if(gamepad2.dpad_up){
                        controlstate = controls.Hangplane;
                    }
                    if (gamepad2.left_stick_button&&gamepad2.right_stick_button){
                        plane.setPosition(planeshoot);
                    }
                    telemetry.addLine("HANGENBITCH");
                case Hangplane:
                    gamepad2.setLedColor(0,1,0, Gamepad.LED_DURATION_CONTINUOUS);
                    if (gamepad2.dpad_down){
                        lefthang.setPosition(leftlow);
                    } if (gamepad2.dpad_left) {
                        lefthang.setPosition(leftmid);
                    } if (gamepad2.dpad_right) {
                        lefthang.setPosition(leftbabymid);
                    } if (gamepad2.dpad_up){
                        lefthang.setPosition(lefthigh);
                    } if (gamepad2.triangle){
                        righthang.setPosition(righthigh);
                    } if (gamepad2.circle){
                        righthang.setPosition(rightmid);
                    } if (gamepad2.square) {
                        righthang.setPosition(rightbabymid);
                    } if (gamepad2.cross){
                        righthang.setPosition(rightlow);
                    }

                    if(gamepad2.touchpad){
                        controlstate = controls.Normal;
                    }
                    if (gamepad2.share) {
                        controlstate = controls.Hangensmooth;
                    }
                    telemetry.addLine("Hangenalles");
                    break;


            }

//            if (gamepad2.right_bumper && gamepad2.left_bumper){
//                gamepad2.setLedColor(1,0,0.91, Gamepad.LED_DURATION_CONTINUOUS);
//                if (gamepad2.dpad_down){
//                    lefthang.setPosition(leftlow);
//                } if (gamepad2.dpad_left){
//                    lefthang.setPosition(leftmid);
//                } if (gamepad2.dpad_up){
//                    lefthang.setPosition(lefthigh);
//                }
//
//                if (gamepad2.triangle){
//                    lefthang.setPosition(righthigh);
//                } if (gamepad2.circle){
//                    lefthang.setPosition(rightmid);
//                } if (gamepad2.cross){
//                    lefthang.setPosition(rightlow);
//                }
//            }

            telemetry.addData("Links: ", linksklauw.getPosition());
            telemetry.addData("Rechts: ", rechtsklauw.getPosition());
            telemetry.addData("Pols: ", twist.getPosition());
            telemetry.addData("Slides: ", slides.getCurrentPosition());
            telemetry.addData("SlidesTarget: ", slides.getTargetPosition());
            if (sensor) {
                if (rechtsssensor.getDistance(DistanceUnit.MM)<70) {
                    rechtsklauw.setPosition(rechtspickup);
                }
                if (linkssensor.getDistance(DistanceUnit.MM)<70) {
                    linksklauw.setPosition(linkspickup);
                }
            }


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
            if (rechtsklauw.getPosition() > 0.4 && linksklauw.getPosition() < 0.7 && linkssensor.getDistance(DistanceUnit.MM)<70 && rechtsssensor.getDistance(DistanceUnit.MM)<70) {
                bothclosed = true;
            } else {
                bothclosed = false;
            }
            if (bothclosed && !rumbled) {
                gamepad1.rumble(1.0, 1.0, 200);
                gamepad2.rumble(1.0, 1.0, 200);
            }


//            if (linksklauw.getPosition() < 0.7) {
//                lefttwisted = true;
//            } else {
//                lefttwisted = false;
//            }
//            if (lefttwisted && !leftrumbled) {
//                gamepad1.rumble(1.0, 0.0, 250);
//                gamepad2.rumble(1.0, 0.0, 250);
//
//            }

//            rightrumbled = righttwisted;
            rumbled = bothclosed;


            if (Math.abs(gamepad1.right_trigger) < 0.02) {
                turnfactor = 0.9;
                maxspeed = 1.2;

            } else if (Math.abs(gamepad1.right_trigger) > 0.02) {
                turnfactor = (-0.3 * gamepad1.right_trigger) + 0.9;
                maxspeed = (-0.9 * gamepad1.right_trigger) + 1.2;

            } else if (Math.abs(gamepad1.left_trigger) > 0.02) {
                turnfactor = (0.3 * gamepad1.left_trigger) + 0.9;
                maxspeed = (0.4 * gamepad1.left_trigger) + 1.2;

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

            if (gamepad1.options) {
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


            telemetry.update();
        }

        }

    }

