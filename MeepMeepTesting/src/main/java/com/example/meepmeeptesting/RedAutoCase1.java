// go to close junction to drop off preloaded; score at 2 high junctions

package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RedAutoCase1 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(16, 16)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(36, -62, 0))
                                .lineTo(new Vector2d(36, 0))
                                .waitSeconds(0.5)
                                .splineToConstantHeading(new Vector2d(42,-12), Math.toRadians(0))
                                .lineTo(new Vector2d(60, -12))
                                .lineTo(new Vector2d(24, -12))
                                .lineTo(new Vector2d(60, -12))
                                .build()
//                        drive.trajectorySequenceBuilder(new Pose2d(36, -62, Math.toRadians(90)))
//                                .lineTo(new Vector2d(36, 0))
//                                .waitSeconds(0.5)
//                                .splineToLinearHeading(new Pose2d(42,-12), Math.toRadians(0))
//                                .lineTo(new Vector2d(60, -12))
//                                .lineTo(new Vector2d(24, -12))
//                                .lineTo(new Vector2d(60, -12))
//                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
