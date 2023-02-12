package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class SplineAuto {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot =
                new DefaultBotBuilder(meepMeep)
                        .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                        .setDimensions(16, 16)
                        .followTrajectorySequence(
                                drive ->
                                        drive.trajectorySequenceBuilder(
                                                        new Pose2d(40, -62, Math.toRadians(90)))
                                                .splineToSplineHeading(
                                                        new Pose2d(36, -48, Math.toRadians(90)),
                                                        Math.toRadians(90))
                                                .splineToSplineHeading(
                                                        new Pose2d(36, -24, Math.toRadians(90)),
                                                        Math.toRadians(90))
                                                .splineToSplineHeading(
                                                        new Pose2d(32, -8, Math.toRadians(135)),
                                                        Math.toRadians(90))
                                                .waitSeconds(0.5)
                                                .lineToSplineHeading(
                                                        new Pose2d(40, -13, Math.toRadians(180)))
                                                .lineTo(new Vector2d(50, -12))
                                                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
