package org.firstinspires.ftc.teamcode.Autonomouse;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive.FollowTrajectoryAction;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

import com.acmerobotics.roadrunner.Trajectory;

import org.firstinspires.ftc.teamcode.RoadRunner.PoseMessage;
import org.firstinspires.ftc.teamcode.Utility.ConfigurationName;
import org.firstinspires.ftc.teamcode.Utility.GlobalValues;
import org.opencv.core.Scalar;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Config
@Autonomous
public class RR10 extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;

    private DcMotor cranearm;

    private Servo servoclaw;

    boolean drivetrain = true;

    boolean LEFT = false;
    boolean MIDDLE = false;
    boolean RIGHT = false;
    boolean AlwaysTrue = true;


    private OpenCvCamera webcam;

    private static final int CAMERA_WIDTH  = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    private double CrLowerUpdate = 160;
    private double CbLowerUpdate = 100;
    private double CrUpperUpdate = 255;
    private double CbUpperUpdate = 255;

    public static double borderLeftX    = 0.0;   //fraction of pixels from the left side of the cam to skip
    public static double borderRightX   = 0.0;   //fraction of pixels from the right of the cam to skip
    public static double borderTopY     = 0.0;   //fraction of pixels from the top of the cam to skip
    public static double borderBottomY  = 0.0;   //fraction of pixels from the bottom of the cam to skip

    private double lowerruntime = 0;
    private double upperruntime = 0;

    // Pink Range                                      Y      Cr     Cb
    public static Scalar scalarLowerYCrCb = new Scalar(  0.0, 160.0, 100.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 255.0, 255.0);

    // Yellow Range
//    public static Scalar scalarLowerYCrCb = new Scalar(0.0, 100.0, 0.0);
//    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 170.0, 120.0);


    static final double FEET_PER_METER = 3.28084;

    int conestack = 5;

    public Pose2d pose;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;


    public enum TradWifeState{
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
    TradWifeState laststate = currentstate;

    MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
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
            {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        // Only if you are using ftcdashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(webcam, 10);

        telemetry.setMsTransmissionInterval(50);

        drive = new MecanumDrive(hardwareMap,new Pose2d(30,15,10));


        Pose2d startPose = new Pose2d(-40, -63.5, Math.toRadians(90));

        TrajectoryBuilder StartToLeft = drive.actionBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-33, -55), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-33, -3), Math.toRadians(90))
                .lineToYConstantHeading((-3)
                .lineToLinearHeading(new Pose2d(-26.5, -4.5,Math.toRadians(45)))
                .build();


        TrajectoryBuilder StartToMiddle = drive.actionBuilder(drive.updatePose2dEstimate())
//                .splineToLinearHeading(new Pose2d(-34, -12, Math.toRadians(180)), Math.toRadians(180))
//                .splineToLinearHeading(new Pose2d(-61.5, -11, Math.toRadians(180)), Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(-33,-11,Math.toRadians(45)))
                .lineToLinearHeading(new Pose2d(-62,-12,Math.toRadians(180)))
                .build();

        TrajectoryBuilder StartToRight = drive.actionBuilder(drive.updatePose2dEstimate())
                .lineToLinearHeading(new Pose2d(-33,-11,Math.toRadians(45)))
                .lineToLinearHeading(new Pose2d(-27,-5,Math.toRadians(45)))
                .build();


        TrajectoryBuilder RightTo1 = drive.actionBuilder(drive.updatePose2dEstimate())
                .lineToLinearHeading(new Pose2d(-36,-12,Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(-58,-12))
                .build();

        TrajectoryBuilder RightTo2 = drive.actionBuilder(drive.updatePose2dEstimate())
                .lineToLinearHeading(new Pose2d(-36,-12,Math.toRadians(90)))
                .build();

        TrajectoryBuilder RightTo3 = drive.actionBuilder(drive.updatePose2dEstimate())
                .lineToLinearHeading(new Pose2d(-36,-12,Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(-12,-12))
                .build();

        TrajectoryBuilder LeftTo1 = drive.actionBuilder(drive.updatePose2dEstimate())
                .lineToLinearHeading(new Pose2d(-36,-12,Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(-12,-12))
                .build();

        TrajectoryBuilder LeftTo2 = drive.actionBuilder(drive.updatePose2dEstimate())
                .lineToLinearHeading(new Pose2d(-36,-12,Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(-12,-12))
                .build();

        TrajectoryBuilder LeftTo3 = drive.actionBuilder(drive.updatePose2dEstimate())
                .lineToLinearHeading(new Pose2d(-36,-12,Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(-12,-12))
                .build();

        TrajectoryBuilder MiddleTo1 = drive.actionBuilder(drive.updatePose2dEstimate())
                .lineToLinearHeading(new Pose2d(-36,-12,Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(-12,-12))
                .build();

        TrajectoryBuilder MiddleTo2 = drive.actionBuilder(drive.updatePose2dEstimate())
                .lineToLinearHeading(new Pose2d(-36,-12,Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(-12,-12))
                .build();

        TrajectoryBuilder MiddleTo3 = drive.actionBuilder(drive.updatePose2dEstimate())
                .lineToLinearHeading(new Pose2d(-36,-12,Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(-12,-12))
                .build();

        TrajectoryBuilder FuckOffToCorner = drive.actionBuilder(drive.updatePose2dEstimate())
                .lineToLinearHeading(new Pose2d(-36,-12,Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(-12,-12))
                .build();

        TrajectoryBuilder FuckOffToMiddle = drive.actionBuilder(drive.updatePose2dEstimate())
                .lineToLinearHeading(new Pose2d(-36,-12,Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(-12,-12))
                .build();



        if (drivetrain){
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
        }


        while (!isStarted() && !isStopRequested()) {


            myPipeline.configureBorders(borderLeftX, borderRightX, borderTopY, borderBottomY);
            if(myPipeline.error){
                telemetry.addData("Exception: ", myPipeline.debug);
            }

            telemetry.addData("RectArea: ", myPipeline.getRectArea());
            telemetry.update();

            if(myPipeline.getRectArea() > 2000){
                if(myPipeline.getRectMidpointX() > 400){
                    telemetry.addData("RECHTS", myPipeline.getRectArea());
                    RIGHT = true;
                    LEFT = false;
                    MIDDLE = false;
                }
                else if(myPipeline.getRectMidpointX() > 200){
                    telemetry.addData("MIDDENNNN", myPipeline.getRectArea());
                    RIGHT = false;
                    LEFT = true;
                    MIDDLE = false;
                }
                else {
                    telemetry.addData("LINKSSSS: ", myPipeline.getRectArea());
                    RIGHT = false;
                    LEFT = true;
                    MIDDLE = false;
                }
            }
                telemetry.update();
                sleep(20);
            }

        while (opModeIsActive()) {

            telemetry.update();

            switch (currentstate) {
                case idol:
                    if (FollowTrajectoryAction.run()) {
                        currentstate = TradWifeState.openservo;
                        cranearm.setTargetPosition(GlobalValues.high);
                        cranearm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        drive.followTrajectorySequenceAsync(TrajStartToDropoff);
                    }
                    break;

                case WaitBeforePush:
                    if (!drive.isBusy()) {
                        servoclaw.setPosition(GlobalValues.servoopen);
                        currentstate = TradWifeState.wait2;

                    }
                    break;

                case ToPush:
                    if (!drive.isBusy()) {
                        drive.followTrajectorySequenceAsync(Wait);
                        if(conestack == 5) {
                            currentstate = TradWifeState.TrajDropoffToPickup;
                        }
                        if(conestack == 4) {
                            cranearm.setTargetPosition(0);
                            if (LEFT) {
                                currentstate = TradWifeState.TrajTo1;
                            } else if (MIDDLE) {
                                currentstate = TradWifeState.TrajTo2;
                            } else if (RIGHT) {
                                currentstate = TradWifeState.TrajTo3;
                            } else {
                                currentstate = TradWifeState.TrajTo2;
                            };
                        }

                    }

                case WaitBeforeScore:
                    if (!drive.isBusy()&& servoclaw.getPosition()==GlobalValues.servoopen) {
                        if(conestack == 5){
                            cranearm.setTargetPosition(GlobalValues.fivecones);
                        }
                        else if(conestack == 4){
                            cranearm.setTargetPosition(GlobalValues.fourcones);
                        }
                        else if(conestack == 3){
                            cranearm.setTargetPosition(GlobalValues.threecones);
                        }
                        else if(conestack == 2){
                            cranearm.setTargetPosition(GlobalValues.twocones);
                        }
                        else if(conestack == 1){
                            cranearm.setTargetPosition(0);
                        }
                        coneStackChanged = true;
                        if (coneStackChanged) {
                            conestack -= 1;
                            coneStackChanged = false;
                        }
                        cranearm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        drive.followTrajectorySequenceAsync(TrajDropoffToPickup);
                        currentstate = TradWifeState.closeservo;

                    }
                    break;

                case ToScore:
                    if (!drive.isBusy()) {
                        servoclaw.setPosition(GlobalValues.servoclosed);
                        currentstate = TradWifeState.wait;
                    }
                case WaitBeforePark:
                    if (!drive.isBusy()) {
                        drive.followTrajectorySequenceAsync(Wait);
                        if (conestack == 4) {
                            currentstate = TradWifeState.TrajPickupToDropoff;
                        }
                        if (conestack == 3) {
                            currentstate = TradWifeState.TrajPickupToDropoff2;
                        }
                    }

                case ToPark:
                    if(!drive.isBusy()) {
                        cranearm.setTargetPosition(GlobalValues.high);
                        cranearm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        currentstate = TradWifeState.openservo;
                        drive.followTrajectorySequenceAsync(TrajPickupToDropoff);
                    }
                    break;

                case FinishedSLAY:
                    if (!drive.isBusy()) {
                        cranearm.setTargetPosition(0);
                        currentstate = TradWifeState.wait3;
                    }
                    break;

                case FinishedSLAY:
                    if (!drive.isBusy()) {
                        currentstate = TradWifeState.WaitForTELEOP;
                        cranearm.setTargetPosition(0);
                        cranearm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }
                    break;


            }
        }
    }
}