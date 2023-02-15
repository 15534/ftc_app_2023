package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class SplineAutoNoHitWall {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot =
                new DefaultBotBuilder(meepMeep)
                        .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                        .setDimensions(16, 16)
                        .followTrajectorySequence(
                                drive ->
                                        drive.trajectorySequenceBuilder(new Pose2d(40, -62, Math.toRadians(90.00)))
                                                .splineToConstantHeading(new Vector2d(34.56, -53.17), Math.toRadians(90.00))
                                                .splineToConstantHeading(new Vector2d(35.94, -29.97), Math.toRadians(90.00))
                                                .splineTo(new Vector2d(28.13, -5.63), Math.toRadians(129.47))
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
