package org.firstinspires.ftc.teamcode.Autonomouse;

import static org.firstinspires.ftc.teamcode.TeleOp.TeleOp_GrijpNoutSlidesRijden.leftmid;
import static org.firstinspires.ftc.teamcode.TeleOp.TeleOp_GrijpNoutSlidesRijden.planeinit;
import static org.firstinspires.ftc.teamcode.TeleOp.TeleOp_GrijpNoutSlidesRijden.rightmid;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Utility.ConfigurationName;
import org.firstinspires.ftc.teamcode.Utility.GlobalValues;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous
public class LEFTPUSHPARK extends LinearOpMode {
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;
    private OpenCvCamera webcam;
    private DcMotor slides;
    private Servo twist;
    private Servo plane;

    private Servo linksklauw;
    private Servo rechtsklauw;

    private Servo lefthang;
    private Servo righthang;
    private TrajectoryFollower follower;

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
//    public static Scalar scalarLowerYCrCb = new Scalar(0.0, 160.0, 0.0);
//    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 255.0, 120.0);


//     Blue                                              Y      Cr     Cb    (Do not change Y)
    public Scalar scalarLowerYCrCb = new Scalar(0.0, 0.0, 150.0);
    public Scalar scalarUpperYCrCb = new Scalar(255.0, 140.0, 255.0);

    // Yellow Range
//    public static Scalar scalarLowerYCrCb = new Scalar(0.0, 100.0, 0.0);
//    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 170.0, 120.0);

    boolean LEFT = false;
    boolean MIDDLE = false;
    boolean RIGHT = false;

    boolean PushWait = true;
    boolean ScoreWait = false;
    boolean ParkWait = false;
    boolean SlidesWait = false;
    boolean ParkLeft = false;
    boolean ParkMiddle = true;
    boolean ParkRight = false;

    public boolean Sensitive = false;

    double waitstick;
    double WaitBeforePush = 1.5;
    double WaitBeforeScore = 1.5;
    double WaitBeforePark = 1.5;
    double Chill = 1.5;


    public enum TradWifeState {
        idol,
        WaitBeforePush,
        ToPush,
        WaitBeforeScore,
        ToBabyScore,
        ToScore,
        WaitBeforePark,
        ToBabyPark,
        ToPark,
        FinishedSLAY,
    }

    TradWifeState currentstate = TradWifeState.idol;
    Pose2d startPose = (new Pose2d(15, 64, Math.toRadians(270)));

    @Override
    public void runOpMode() throws InterruptedException {
        twist = hardwareMap.servo.get(ConfigurationName.twist);

        rechtsklauw = hardwareMap.servo.get(ConfigurationName.rechtsklauw);

        linksklauw = hardwareMap.servo.get(ConfigurationName.linksklauw);

        slides = hardwareMap.dcMotor.get(ConfigurationName.slides);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setDirection(DcMotor.Direction.REVERSE);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        righthang = hardwareMap.servo.get(ConfigurationName.righthang);

        lefthang = hardwareMap.servo.get(ConfigurationName.lefthang);
        plane = hardwareMap.servo.get(ConfigurationName.plane);
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();


        // OpenCV webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //OpenCV Pipeline
        ContourPipeline myPipeline;
        webcam.setPipeline(myPipeline = new ContourPipeline(borderLeftX,borderRightX,borderTopY,borderBottomY));
        // Configuration of Pipeline
        myPipeline.configureScalarLower(scalarLowerYCrCb.val[0],scalarLowerYCrCb.val[1],scalarLowerYCrCb.val[2]);
        myPipeline.configureScalarUpper(scalarUpperYCrCb.val[0],scalarUpperYCrCb.val[1],scalarUpperYCrCb.val[2]);
        // Webcam Streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            { webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);}
            @Override
            public void onError(int errorCode)
            { /** This will be called if the camera could not be opened**/}

        });
        // Only if you are using ftcdashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(webcam, 10);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose);

        TrajectorySequence StartToRight = drive.trajectorySequenceBuilder(startPose)
//                .splineToLinearHeading(new Pose2d(12,-31,Math.toRadians(180)),Math.toRadians(180))
                .lineToConstantHeading(new Vector2d(15, 38.5))
                .lineToConstantHeading(new Vector2d(1, 38.5))
                .build();
        TrajectorySequence StartToMiddle = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(16.5,31.5))
                .build();
        TrajectorySequence StartToLeft = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(22,42,Math.toRadians(270)))
                .build();
        TrajectorySequence LeftToBaby = drive.trajectorySequenceBuilder(StartToLeft.end())
                .lineToLinearHeading(new Pose2d(22, 48, Math.toRadians(270)))
                .build();
        TrajectorySequence RightToBaby = drive.trajectorySequenceBuilder(StartToRight.end())
                .lineToConstantHeading(new Vector2d(13,39.5))
                .build();
        TrajectorySequence MiddleToBaby = drive.trajectorySequenceBuilder(StartToMiddle.end())
                .lineToLinearHeading(new Pose2d(14, 40, Math.toRadians(305)))
                .build();



        ElapsedTime WaitTimer = new ElapsedTime();

        while (!isStarted() && !isStopRequested()) {

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            myPipeline.configureBorders(borderLeftX, borderRightX, borderTopY, borderBottomY);
            if (myPipeline.error) {
                telemetry.addData("Exception: ", myPipeline.debug);
            }
            // Only use this line of the code when you want to find the lower and upper values

            telemetry.addData("RectArea: ", myPipeline.getRectArea());
            telemetry.addData("MidPointX: ", myPipeline.getRectMidpointX());

            if(myPipeline.getRectArea() > 2000) {
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


            waitstick = -(gamepad1.right_stick_y / 10000);
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
            }if (SlidesWait) {
                telemetry.addLine("Changing Chill");
                Chill += waitstick;
            }
            telemetry.addLine();
            telemetry.addData("a: Wait Before Push ", "%.2f", WaitBeforePush);
            telemetry.addData("b: Wait Before Score", "%.2f", WaitBeforeScore);
            telemetry.addData("y: Wait Before Park", "%.2f", WaitBeforePark);
            telemetry.addData("x: Chill", "%.2f", Chill);

            telemetry.addLine();
            if (ParkLeft) {
                telemetry.addLine("Park Left:}");
            }
            if (ParkMiddle) {
                telemetry.addLine("Park Middle:}");
            }
            if (ParkRight) {
                telemetry.addLine("Park Right:}");
            }
            telemetry.addLine();
            telemetry.addLine("Right Trigger = Park More Right");
            telemetry.addLine("Left Trigger = Park More Left");
            if (gamepad1.a) {
                PushWait = true;
                ScoreWait = false;
                ParkWait = false;
                SlidesWait = false;
            }
            if (gamepad1.b) {
                PushWait = false;
                ScoreWait = true;
                ParkWait = false;
                SlidesWait = false;
            }
            if (gamepad1.y) {
                PushWait = false;
                ScoreWait = false;
                ParkWait = true;
                SlidesWait = false;
            }
            if (gamepad1.x) {
                PushWait = false;
                ScoreWait = false;
                ParkWait = false;
                SlidesWait = true;
            }
            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                if (ParkMiddle) {
                    ParkRight = true;
                    ParkMiddle = false;
                    ParkLeft = false;
                }
                if (ParkLeft) {
                    ParkRight = false;
                    ParkMiddle = true;
                    ParkLeft = false;
                }
            }
            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                if (ParkMiddle) {
                    ParkRight = false;
                    ParkMiddle = false;
                    ParkLeft = true;
                }
                if (ParkRight) {
                    ParkRight = false;
                    ParkMiddle = true;
                    ParkLeft = false;
                }
            }
            linksklauw.setPosition(GlobalValues.linkspickup);
            lefthang.setPosition(leftmid);
            righthang.setPosition(rightmid);
            plane.setPosition(planeinit);
            rechtsklauw.setPosition(GlobalValues.rechtspickup);


            telemetry.update();
            while (opModeIsActive()) {
                if (slides.isBusy()){
                    slides.setPower(0.5);
                } else {
                    slides.setPower(0);
                }
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
                            if (LEFT){
                                drive.followTrajectorySequenceAsync(StartToLeft);
                            } else if (MIDDLE){
                                drive.followTrajectorySequenceAsync(StartToMiddle);
                            } else if (RIGHT){
                                drive.followTrajectorySequenceAsync(StartToRight);
                            }else {
                                drive.followTrajectorySequenceAsync(StartToMiddle);
                            }
                            currentstate = TradWifeState.WaitBeforeScore;
                        }
                        break;

                    case WaitBeforeScore:
                        if (!drive.isBusy()) {
                            WaitTimer.reset();
                            rechtsklauw.setPosition(GlobalValues.rechtsdrop);
                            currentstate = TradWifeState.FinishedSLAY;
                        }
                        break;

                    case FinishedSLAY:
                        if(WaitTimer.seconds() >= WaitBeforePush) {
                            slides.setTargetPosition(50);
                            slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        }
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
