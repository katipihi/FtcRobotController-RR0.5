package org.firstinspires.ftc.teamcode.Autonomouse;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import org.firstinspires.ftc.teamcode.Utility.GlobalValues;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous
public class RedRight extends LinearOpMode {
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;
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
    MecanumDrive drive;

    public enum State {
        Start,
        ToShove1,
        ToShove2,
        ToShove3,
        ToBoard1,
        ToBoard2,
        ToBoard3,
    }

    State currentstate = State.Start;


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

        drive = new MecanumDrive(hardwareMap, (new Pose2d(-39, -64, Math.toRadians(90))));

        TrajectoryActionBuilder TrajStartToDropoff = drive.TrajectoryActionBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-33, -55), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-33, -3), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-33, -10), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(-26.5, -4.5, Math.toRadians(45)))
                .build();


        while (!isStarted() && !isStopRequested()) {
            myPipeline.configureBorders(borderLeftX, borderRightX, borderTopY, borderBottomY);
            if (myPipeline.error) {
                telemetry.addData("Exception: ", myPipeline.debug);
            }
            // Only use this line of the code when you want to find the lower and upper values

            telemetry.addData("RectArea: ", myPipeline.getRectArea());
            telemetry.update();

            if (myPipeline.getRectArea() > 2000) {
                if (myPipeline.getRectMidpointX() > 400) {
                    LEFT = true;
                    MIDDLE = false;
                    RIGHT = false;
                    telemetry.addLine("Tag 1 found :(");
                } else if (myPipeline.getRectMidpointX() > 200) {
                    MIDDLE = true;
                    LEFT = false;
                    RIGHT = false;
                    telemetry.addLine("Tag 2 found :(");
                } else {
                    RIGHT = true;
                    LEFT = false;
                    MIDDLE = false;
                    telemetry.addLine("Tag 3 found :(");
                }


            }
            while (opModeIsActive()) {
                switch (currentstate) {
                    case Start:
                        if (!drive.isBusy()) {
                            if (LEFT) {
                                currentstate = State.ToShove1;
                            } else if (MIDDLE) {
                                currentstate = State.ToShove2;
                            } else if (RIGHT) {
                                currentstate = State.ToShove3;
                            } else {
                                currentstate = State.ToShove1;
                            }
                            ;
                            drive.FollowTrajectoryAction(TrajStartToDropoff);
                        }
                        break;
                }
            }
        }
    }
}
