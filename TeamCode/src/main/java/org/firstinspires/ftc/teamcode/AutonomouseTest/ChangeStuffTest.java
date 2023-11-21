package org.firstinspires.ftc.teamcode.AutonomouseTest;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous(name="ChangeDoublesTest", group="Test")
public class ChangeStuffTest extends LinearOpMode {

    boolean PushWait = true;
    boolean ScoreWait = false;
    boolean ParkWait = false;
    boolean ParkCorner = false;

    boolean ParkMiddle = true;

    boolean start = true;

    @Override
    public void runOpMode() throws InterruptedException {



        double WaitBeforePush = 1.5;
        double WaitBeforeScore = 1.5;
        double WaitBeforePark = 1.5;

        double waitstick;

        ElapsedTime WaitTimer = new ElapsedTime();

        while (!isStarted() && !isStopRequested()) {
            if (start){
                telemetry.addLine("Press the Button Before Values to Select Value to Change");
                telemetry.addLine("Use the Right Joystick to Change values");
                telemetry.addLine("Press x to Get Rid of This Message");
            } if (gamepad1.x){
                start = false;
            }
            waitstick = -(gamepad1.right_stick_y/50000);
            if (PushWait){
                telemetry.addLine("Changing Wait Before Push");
                WaitBeforePush += waitstick;
            }
            if (ScoreWait){
                telemetry.addLine("Changing Wait Before Score ");
                WaitBeforeScore += waitstick;
            }
            if (ParkWait){
                telemetry.addLine("Changing Wait Before Park");
                WaitBeforePark += waitstick;
            }
            telemetry.addLine();
            telemetry.addData("a: Wait Before Push ","%.2f",WaitBeforePush);
            telemetry.addData("b: Wait Before Score","%.2f",WaitBeforeScore);
            telemetry.addData("y: Wait Before Park","%.2f",WaitBeforePark);
            telemetry.addLine();
            if (ParkCorner){
                telemetry.addLine("Park in the Corner:}");

            }
            if (ParkMiddle){
                telemetry.addLine("Park in the Middle:}");

            }
            telemetry.addLine();
            telemetry.addLine("Right Trigger = Park in Corner");
            telemetry.addLine("Left Trigger = Park in Middle");
            if (gamepad1.a) {
                PushWait = true;
                ScoreWait = false;
                ParkWait = false;
            } if (gamepad1.b) {

                PushWait = false;
                ScoreWait = true;
                ParkWait = false;
            } if (gamepad1.y) {
                PushWait = false;
                ScoreWait = false;
                ParkWait = true;
            }
            if (gamepad1.right_trigger>0.02){
                ParkCorner = true;
                ParkMiddle = false;
            }
            if (gamepad1.left_trigger>0.02){
                ParkCorner = false;
                ParkMiddle = true;
            }


            telemetry.update();

            while (opModeIsActive()) {

            }
        }
    }
}
