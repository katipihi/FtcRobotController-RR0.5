package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(50, -41, Math.toRadians(0)))
                                .splineToLinearHeading(new Pose2d(41,-52,Math.toRadians(0)),Math.toRadians(90))
                                .splineToLinearHeading(new Pose2d(60,-60,Math.toRadians(90)),Math.toRadians(0))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
//.splineToLinearHeading(new Pose2d(50,-29,Math.toRadians(0)),Math.toRadians(0))
//LEFT
//                                .lineToLinearHeading(new Pose2d(22,-43,Math.toRadians(90)))
//                                        .lineToLinearHeading(new Pose2d(13,-43,Math.toRadians(0)))
//                                        .lineToLinearHeading(new Pose2d(50,-41,Math.toRadians(0)))

//MIDDLE
//                                .lineTo(new Vector2d(12,-33))
//                                .lineToLinearHeading(new Pose2d(20,-40,Math.toRadians(45)))
//                                .splineToLinearHeading(new Pose2d(50,-35.5,Math.toRadians(0)),Math.toRadians(0))
//RIGHT
//                                .splineToLinearHeading(new Pose2d(12,-31,Math.toRadians(0)),Math.toRadians(0))
//                                .lineToLinearHeading(new Pose2d(13,-43,Math.toRadians(0)))
//                                .lineToLinearHeading(new Pose2d(50,-41,Math.toRadians(0)))
//MIDDLEINUSE
//                                  .lineTo(new Vector2d(12,-33))
//                                  .lineToLinearHeading(new Pose2d(14 ,-40,Math.toRadians(45)))
//                                  .splineToLinearHeading(new Pose2d(50,-35.5,Math.toRadians(0)),Math.toRadians(0))
