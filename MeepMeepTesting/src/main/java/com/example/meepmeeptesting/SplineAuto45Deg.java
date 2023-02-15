package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class SplineAuto45Deg {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot =
                new DefaultBotBuilder(meepMeep)
                        .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                        .setDimensions(16, 16)
                        .followTrajectorySequence(
                                drive ->
                                        drive.trajectorySequenceBuilder(new Pose2d(41.94, -66.45, Math.toRadians(90.00)))
                                                .splineTo(new Vector2d(38.30, -54.96), Math.toRadians(107.57))
                                                .splineTo(new Vector2d(35.43, -34.28), Math.toRadians(98.27))
                                                .splineTo(new Vector2d(34.85, -18.77), Math.toRadians(90.80))
                                                .splineTo(new Vector2d(28.34, -6.32), Math.toRadians(117.61))
                                                .splineToLinearHeading(new Pose2d(34.47, -13.02, Math.toRadians(180)), Math.toRadians(-9.46))
                                                .lineTo(new Vector2d(48.06, -12.26))
                                                .lineTo(new Vector2d(60, -12.26))
                                                .lineTo(new Vector2d(-60, -12.26))
                                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
