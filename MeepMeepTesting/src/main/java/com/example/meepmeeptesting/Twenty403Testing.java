package com.example.meepmeeptesting;

import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class Twenty403Testing {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(750);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(14, 14)
                //Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 9.5)
                .followTrajectorySequence(drive ->
                        drive
                                .trajectorySequenceBuilder(AutoConstantsRed.Away.START)
                                .addTrajectory(AutoConstantsRed.Away.START_TO_LEFT_LOW.get())
                                //.addTrajectory(AutoConstantsRed.Away.START_TO_RIGHT_LOW.get())
                                .addTrajectory(AutoConstantsRed.Away.LEFT_LOW_TO_BETWEEN_LEFT.get())
                                .addTrajectory(AutoConstantsRed.Away.BETWEEN_TO_PARK_LEFT.get())
                                .build()
                );

        meepMeep.setBackgroundAlpha(0.95f).addEntity(myBot);
        try {
            BufferedImage centerStage = ImageIO.read(new File("Field.jpg"));
            meepMeep.setBackground(centerStage);
        } catch (IOException io) {
            meepMeep
                    .setBackground(MeepMeep.Background.GRID_BLUE);
        }
        meepMeep.start();
    }
}
