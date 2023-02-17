package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class SplineAutoNoHitWall_Left {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d startingPos = new Pose2d(-40, -62, Math.toRadians(90));
        Vector2d mediumJunction = new Vector2d(-24.2, -12);

        RoadRunnerBotEntity myBot =
                new DefaultBotBuilder(meepMeep)
                        .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                        .setDimensions(16, 16)
                        .followTrajectorySequence(
                                drive ->
                                        drive.trajectorySequenceBuilder(new Pose2d(-40, -62, Math.toRadians(90.00)))
                                                .splineToConstantHeading(
                                                        new Vector2d(-34.56, -53.17), Math.toRadians(90.00)
                                                )
                                                .splineToConstantHeading(new Vector2d(-35.94, -29.97), Math.toRadians(90.00))
                                                .splineTo(new Vector2d(-30.5, -10.75), Math.toRadians(180 - 125.00))
                                                .splineToLinearHeading(new Pose2d(-34.47, -13.02, Math.toRadians(0)), Math.toRadians(-9.46))
                                                .lineToSplineHeading(new Pose2d(-56.2, -14.1, Math.toRadians(0)))
                                                .lineTo(new Vector2d(-54.70, -10.70))
                                                .lineTo(new Vector2d(-55.80, -13.8))
                                                .lineTo(mediumJunction)
                                                .lineToLinearHeading(new Pose2d(new Vector2d(-36, -12), Math.toRadians(90)))
                                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
