package org.firstinspires.ftc.teamcode.Autonomouse;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOp_GrijpNoutSlidesRijden;
import org.firstinspires.ftc.teamcode.Utility.ConfigurationName;
import org.firstinspires.ftc.teamcode.Utility.GlobalValues;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous
public class RedRight extends LinearOpMode {
    private DcMotor slides;
    private Servo twist;
    private Servo linksklauw;
    private Servo rechtsklauw;
    private OpenCvCamera webcam;

    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    public static double borderLeftX = 0.0;   //fraction of pixels from the left side of the cam to skip
    public static double borderRightX = 0.0;   //fraction of pixels from the right of the cam to skip
    public static double borderTopY = 0.0;   //fraction of pixels from the top of the cam to skip
    public static double borderBottomY = 0.0;   //fraction of pixels from the bottom of the cam to skip

    // Pink Range                                      Y      Cr     Cb
    //public static Scalar scalarLowerYCrCb = new Scalar(  0.0, 160.0, 100.0);
    //public static Scalar scalarUpperYCrCb = new Scalar(255.0, 255.0, 255.0);

    // Red                                              Y      Cr     Cb    (Do not change Y)
    public static Scalar scalarLowerYCrCb = new Scalar(0.0, 160.0, 0.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 255.0, 120.0);

    // Blue                                              Y      Cr     Cb    (Do not change Y)
//    public Scalar scalarLowerYCrCb = new Scalar(0.0, 0.0, 150.0);
//    public Scalar scalarUpperYCrCb = new Scalar(255.0, 140.0, 255.0);

    // Yellow Range
//    public static Scalar scalarLowerYCrCb = new Scalar(0.0, 100.0, 0.0);
//    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 170.0, 120.0);

    boolean LEFT = false;
    boolean MIDDLE = false;
    boolean RIGHT = false;
    boolean TOUP = false;

    boolean PushWait = true;
    boolean ScoreWait = false;
    boolean ParkWait = false;
    boolean ParkCorner = false;
    boolean ParkMiddle = true;
    double waitstick;
    double WaitBeforePush = 1.5;
    double WaitBeforeScore = 1.5;
    double WaitBeforePark = 1.5;


    public enum TradWifeState {
        idol,
        WaitBeforePush,
        ToPush,
        WaitBeforeScore,
        ToScore,
        WaitBeforePark,
        ToPark,
        FinishedSLAY,
    }

    TradWifeState currentstate = TradWifeState.idol;

    public enum Slideys {
        GROUND,
        OPEN,
        UP
    }

    Slideys slideystate = Slideys.GROUND;


    Pose2d startPose = (new Pose2d(15, -64, Math.toRadians(90)));


    @Override
    public void runOpMode() throws InterruptedException {
        slides = hardwareMap.dcMotor.get(ConfigurationName.slides);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setDirection(DcMotor.Direction.REVERSE);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        twist = hardwareMap.servo.get(ConfigurationName.twist);

        rechtsklauw = hardwareMap.servo.get(ConfigurationName.rechtsklauw);

        linksklauw = hardwareMap.servo.get(ConfigurationName.linksklauw);

        // OpenCV webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //OpenCV Pipeline
        ContourPipeline myPipeline;
        webcam.setPipeline(myPipeline = new ContourPipeline(borderLeftX, borderRightX, borderTopY, borderBottomY));
        // Configuration of Pipeline
        myPipeline.configureScalarLower(scalarLowerYCrCb.val[0], scalarLowerYCrCb.val[1], scalarLowerYCrCb.val[2]);
        myPipeline.configureScalarUpper(scalarUpperYCrCb.val[0], scalarUpperYCrCb.val[1], scalarUpperYCrCb.val[2]);
        // Webcam Streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) { /** This will be called if the camera could not be opened**/}

        });
        // Only if you are using ftcdashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(webcam, 10);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose);

        TrajectorySequence StartToLeft = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(15, -35))
                .lineTo(new Vector2d(1, -35))
                .build();
        TrajectorySequence StartToMiddle = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(16.5, -31.5))
                .build();
        TrajectorySequence StartToRight = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(22, -41, Math.toRadians(90)))
                .build();
        TrajectorySequence RightTo3 = drive.trajectorySequenceBuilder(StartToRight.end())
                .lineToLinearHeading(new Pose2d(22, -48, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(45, -41, Math.toRadians(0)))
                .build();
        TrajectorySequence LeftTo1 = drive.trajectorySequenceBuilder(StartToLeft.end())
                .lineToLinearHeading(new Pose2d(50, -29, Math.toRadians(0)))
                .build();
        TrajectorySequence MiddleTo2 = drive.trajectorySequenceBuilder(StartToMiddle.end())
                .lineToLinearHeading(new Pose2d(14, -40, Math.toRadians(45)))
                .splineToLinearHeading(new Pose2d(50, -35.5, Math.toRadians(0)), Math.toRadians(0))
                .build();
        TrajectorySequence FuckOffToCorner = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-36, -12, Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(-12, -12))
                .build();
        TrajectorySequence FuckOffToMiddle = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-36, -12, Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(-12, -12))
                .build();


        ElapsedTime WaitTimer = new ElapsedTime();

        while (!isStarted() && !isStopRequested()) {
            myPipeline.configureBorders(borderLeftX, borderRightX, borderTopY, borderBottomY);
            if (myPipeline.error) {
                telemetry.addData("Exception: ", myPipeline.debug);
            }
            // Only use this line of the code when you want to find the lower and upper values

            telemetry.addData("RectArea: ", myPipeline.getRectArea());
            telemetry.addData("MidPointX: ", myPipeline.getRectMidpointX());

            if (myPipeline.getRectArea() > 2000) {
                if (myPipeline.getRectMidpointX() > 400) {
                    RIGHT = true;
                    LEFT = false;
                    MIDDLE = false;
                    telemetry.addLine("RECHTSSLET");
                } else if (myPipeline.getRectMidpointX() > 200) {
                    RIGHT = false;
                    LEFT = false;
                    MIDDLE = true;
                    telemetry.addLine("MIDDENHOER");
                }
//                if (200 > myPipeline.getRectX())
                else {
                    RIGHT = false;
                    LEFT = true;
                    MIDDLE = false;
                    telemetry.addLine("LINKSBITCH");
                }
            }

            waitstick = -(gamepad1.right_stick_y / 50000);
            if (PushWait) {
                telemetry.addLine("Changing Wait Before Push");
                WaitBeforePush += waitstick;
            }
            if (ScoreWait) {
                telemetry.addLine("Changing Wait Before Score ");
                WaitBeforeScore += waitstick;
            }
            if (ParkWait) {
                telemetry.addLine("Changing Wait Before Park");
                WaitBeforePark += waitstick;
            }
            telemetry.addLine();
            telemetry.addData("a: Wait Before Push ", "%.2f", WaitBeforePush);
            telemetry.addData("b: Wait Before Score", "%.2f", WaitBeforeScore);
            telemetry.addData("y: Wait Before Park", "%.2f", WaitBeforePark);
            telemetry.addLine();
            if (ParkCorner) {
                telemetry.addLine("Park in the Corner:}");
            }
            if (ParkMiddle) {
                telemetry.addLine("Park in the Middle:}");
            }
            telemetry.addLine();
            telemetry.addLine("Right Trigger = Park in Corner");
            telemetry.addLine("Left Trigger = Park in Middle");
            if (gamepad1.a) {
                PushWait = true;
                ScoreWait = false;
                ParkWait = false;
            }
            if (gamepad1.b) {
                PushWait = false;
                ScoreWait = true;
                ParkWait = false;
            }
            if (gamepad1.y) {
                PushWait = false;
                ScoreWait = false;
                ParkWait = true;
            }
            if (gamepad1.right_trigger > 0.02) {
                ParkCorner = true;
                ParkMiddle = false;
            }
            if (gamepad1.left_trigger > 0.02) {
                ParkCorner = false;
                ParkMiddle = true;
            }
            linksklauw.setPosition(GlobalValues.linkspickup);
            rechtsklauw.setPosition(GlobalValues.rechtspickup);


            telemetry.update();
            while (opModeIsActive()) {
                switch (currentstate) {
                    case idol:
                        if (!drive.isBusy()) {
                            currentstate = TradWifeState.WaitBeforePush;
                        }
                        break;

                    case WaitBeforePush:
                        if (!drive.isBusy()) {
                            WaitTimer.reset();
                            currentstate = TradWifeState.ToPush;
                        }
                        break;

                    case ToPush:
                        if (WaitTimer.seconds() >= WaitBeforePush) {
                            if (LEFT) {
                                drive.followTrajectorySequenceAsync(StartToLeft);
                            } else if (MIDDLE) {
                                drive.followTrajectorySequenceAsync(StartToMiddle);
                            } else if (RIGHT) {
                                drive.followTrajectorySequenceAsync(StartToRight);
                            } else {
                                drive.followTrajectorySequenceAsync(StartToMiddle);
                            }

                        }
                        break;

                    case WaitBeforeScore:
                        if (!drive.isBusy()) {
                            TOUP = true;
                            WaitTimer.reset();
                        }
                        break;

                    case ToScore:
                        if (WaitTimer.seconds() >= WaitBeforeScore && slides.getCurrentPosition() > 15) {
                            if (LEFT) {
                                drive.followTrajectorySequenceAsync(LeftTo1);
                            } else if (MIDDLE) {
                                drive.followTrajectorySequenceAsync(MiddleTo2);
                            } else if (RIGHT) {
                                drive.followTrajectorySequenceAsync(RightTo3);
                            } else {
                                drive.followTrajectorySequenceAsync(MiddleTo2);
                            }
                            twist.setPosition(GlobalValues.twistdrop);
                            currentstate = TradWifeState.WaitBeforePark;
                        }
                        break;

                    case WaitBeforePark:
                        if (!drive.isBusy()) {
                            WaitTimer.reset();
                            currentstate = TradWifeState.ToPark;
                        }
                        break;

                    case ToPark:
                        if (WaitTimer.seconds() >= WaitBeforePark) {
                            currentstate = TradWifeState.FinishedSLAY;
                            if (ParkCorner) {
                                drive.followTrajectorySequenceAsync(FuckOffToCorner);
                            } else if (ParkMiddle) {
                                drive.followTrajectorySequenceAsync(FuckOffToCorner);
                            } else {
                                drive.followTrajectorySequenceAsync(FuckOffToMiddle);
                            }
                        }
                        break;

                    case FinishedSLAY:
                        break;
                }
                switch (slideystate) {
                    case GROUND:
                        if (TOUP) {
                            slideystate = Slideys.OPEN;
                        }
                        break;

                    case OPEN:
                        linksklauw.setPosition(GlobalValues.linksdrop);
                        slideystate = Slideys.UP;

                    case UP:
                        if (linksklauw.getPosition() == GlobalValues.linksdrop) {
                            slides.setTargetPosition(100);
                            slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            TOUP = false;
                            slideystate = Slideys.GROUND;
                        }
                        break;
                }


                // We update drive continuously in the background, regardless of state
                drive.update();
                // We update our lift PID continuously in the background, regardless of state

                // Read pose
                Pose2d poseEstimate = drive.getPoseEstimate();

                PoseStorage.currentPose = poseEstimate;

                // Print pose to telemetry
                telemetry.addData("x", poseEstimate.getX());
                telemetry.addData("y", poseEstimate.getY());
                telemetry.addData("heading", poseEstimate.getHeading());
                telemetry.update();
            }
        }
    }
}
