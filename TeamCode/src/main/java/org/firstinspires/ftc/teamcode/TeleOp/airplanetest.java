//package org.firstinspires.ftc.teamcode.TeleOp;
//
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//
//import org.firstinspires.ftc.teamcode.Utility.ConfigurationName;
//
//
//@Config
//@TeleOp(name="Planetest", group="linop")
//public class airplanetest extends LinearOpMode{
//
//    private Servo plane;
//    private double servospan = 0.5;
//    private double servolos = 0.9;
//
//    @Override
//    public void runOpMode() {
//        plane = hardwareMap.servo.get(ConfigurationName.grijper);
//
//        waitForStart();
//
//
//        while (opModeIsActive()) {
//            if (gamepad1.a){
//                plane.setPosition(servospan);
//            }
//            if(gamepad1.b){
//                plane.setPosition(servolos);
//            }
//
//        }
//        }
//
//
//}
//
//
