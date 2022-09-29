package org.team5557.util;

import java.io.IOException;
import java.util.Arrays;

import org.frcteam2910.library.control.*;
import org.frcteam2910.library.math.Rotation2;
import org.frcteam2910.library.math.Vector2;


//import edu.wpi.first.math.trajectory.Trajectory;
import org.frcteam2910.library.control.Trajectory;

public class AutonomousTrajectories {
    private static final double SAMPLE_DISTANCE = 0.1;

    private Trajectory twoBallPartOne;
    private Trajectory twoBallPartTwo;
    private Trajectory twoPlusTwoBallPartThree;
    private Trajectory twoPlusTwoBallPartFour;
    private Trajectory twoPlusTwoBallPartFive;
    private Trajectory twoPlusTwoBallPartSeven;
    private Trajectory twoPlusTwoBallPartSix;
    private Trajectory fourBallPartFive;
    private Trajectory fourBallPartFour;
    private Trajectory fourBallPartThree;
    private Trajectory fourBallPartTwo;
    private Trajectory fourBallPartOne;
    private Trajectory scootBack;

    private Trajectory twoBallPartOneSlow;
    private Trajectory twoBallPartTwoSlow;


    public AutonomousTrajectories(TrajectoryConstraint[] trajectoryConstraints) throws IOException {
        TrajectoryConstraint[] slowConstraints = Arrays.copyOf(trajectoryConstraints, trajectoryConstraints.length + 1);
        slowConstraints[slowConstraints.length - 1] = new MaxVelocityConstraint(6.0 * 12.0);
        slowConstraints[slowConstraints.length - 2] = new MaxAccelerationConstraint(4.0 * 12.0);

        //our balls
        final Vector2 topLeftABall = new Vector2(194.04, 243.14);
        final Vector2 middleABall = new Vector2(198.49, 73.7);
        final Vector2 bottomRightABall = new Vector2(73.7, 198.49);
        final Vector2 terminalABall = new Vector2(56.49, 57.7);

        //their balls
        final Vector2 middleOBall = new Vector2(175.84, 128.23);
        final Vector2 topOBall = new Vector2(235.13, 286.45);


        final Vector2 bottomRightABallIntakePose = new Vector2(bottomRightABall.x + 0, bottomRightABall.y + 18.5);

        final Vector2 twoBallStartingPose = new Vector2(topLeftABall.x + 46, topLeftABall.y - 39);
        final Vector2 twoBallShootingPose = new Vector2(topLeftABall.x + 80.5-13, topLeftABall.y - 64.5+13);


        final Vector2 dumpPose = new Vector2(88, 298.45);
        final Vector2 preDumpPose = new Vector2(88, dumpPose.y - 30);

        final Vector2 fourBallStartingPose = new Vector2(bottomRightABall.x + 0, bottomRightABall.y + 57.5);
        final Vector2 fourBallShootingPose = new Vector2(bottomRightABall.x + 0, bottomRightABall.y + 103);


        twoBallPartOneSlow = new Trajectory(
            new SimplePathBuilder(twoBallStartingPose, Rotation2.fromDegrees(135))   //start position inline with opp. ball and 19 inches off midline
                    .lineTo(topLeftABall, Rotation2.fromDegrees(135.0))  //goes to first blue ball
                    .build(),
            slowConstraints, SAMPLE_DISTANCE);

        twoBallPartTwoSlow = new Trajectory(    //blue ball to fender
            new SimplePathBuilder(topLeftABall, Rotation2.fromDegrees(135.0))
                    .lineTo(twoBallShootingPose, Rotation2.fromDegrees(159.0))
                    .build(),
            slowConstraints, SAMPLE_DISTANCE);



        twoBallPartOne = new Trajectory(
            new SimplePathBuilder(twoBallStartingPose, Rotation2.fromDegrees(135))   //start position inline with opp. ball and 19 inches off midline
                    .lineTo(topLeftABall, Rotation2.fromDegrees(135.0))  //goes to first blue ball
                    .build(),
            trajectoryConstraints, SAMPLE_DISTANCE);

        twoBallPartTwo = new Trajectory(    //blue ball to fender
            new SimplePathBuilder(topLeftABall, Rotation2.fromDegrees(135.0))
                    .lineTo(twoBallShootingPose, Rotation2.fromDegrees(159.0))
                    .build(),
            trajectoryConstraints, SAMPLE_DISTANCE);
        

        twoPlusTwoBallPartThree = new Trajectory(    //blue ball to fender
            new SimplePathBuilder(twoBallShootingPose, Rotation2.fromDegrees(159.0))
                .lineTo(new Vector2(215, 174), Rotation2.fromDegrees(180.0))
                .build(),
            trajectoryConstraints, SAMPLE_DISTANCE);

        twoPlusTwoBallPartFour = new Trajectory(    //blue ball to fender
            new SimplePathBuilder(new Vector2(215, 174), Rotation2.fromDegrees(180.0))
                .lineTo(middleOBall, Rotation2.fromDegrees(225.0))
                .build(),
            trajectoryConstraints, SAMPLE_DISTANCE);

        twoPlusTwoBallPartFive = new Trajectory(    //blue ball to fender
            new SimplePathBuilder(middleOBall, Rotation2.fromDegrees(225.0))
                .lineTo(topOBall, Rotation2.fromDegrees(70.0))
                .build(),
            trajectoryConstraints, SAMPLE_DISTANCE);

        twoPlusTwoBallPartSix = new Trajectory(    //blue ball to fender
            new SimplePathBuilder(topOBall, Rotation2.fromDegrees(70.0))
                .lineTo(preDumpPose, Rotation2.fromDegrees(90.0))
                .build(),
            trajectoryConstraints, SAMPLE_DISTANCE);

        twoPlusTwoBallPartSeven = new Trajectory(    //blue ball to fender
            new SimplePathBuilder(preDumpPose, Rotation2.fromDegrees(90.0))
                .lineTo(dumpPose, Rotation2.fromDegrees(90.0))
                .build(),
            trajectoryConstraints, SAMPLE_DISTANCE);




        fourBallPartOne = new Trajectory(    //starting pose to bottom blue ball
            new SimplePathBuilder(fourBallStartingPose, Rotation2.fromDegrees(270.0))
                .lineTo(bottomRightABallIntakePose, Rotation2.fromDegrees(270.0))
                .build(),
            trajectoryConstraints, SAMPLE_DISTANCE);

        fourBallPartTwo = new Trajectory(    //bottom blue ball to fender
            new SimplePathBuilder(bottomRightABallIntakePose, Rotation2.fromDegrees(270.0))
                .lineTo(fourBallShootingPose, Rotation2.fromDegrees(249.0))
                .build(),
            trajectoryConstraints, SAMPLE_DISTANCE);

        fourBallPartThree = new Trajectory(    //fender to middle blue ball
            new SimplePathBuilder(fourBallShootingPose, Rotation2.fromDegrees(249.0))
                .lineTo(middleABall, Rotation2.fromDegrees(200.0))
                .build(),
            trajectoryConstraints, SAMPLE_DISTANCE);

        fourBallPartFour = new Trajectory(    //middle blue ball to terminal
            new SimplePathBuilder(middleABall, Rotation2.fromDegrees(200.0))
                .lineTo(terminalABall, Rotation2.fromDegrees(225.0))
                .build(),
            trajectoryConstraints, SAMPLE_DISTANCE);

        fourBallPartFive = new Trajectory(    //terminal to fender
            new SimplePathBuilder(middleABall, Rotation2.fromDegrees(225.0))
                .lineTo(terminalABall, Rotation2.fromDegrees(249.0))
                .build(),
            trajectoryConstraints, SAMPLE_DISTANCE);


        scootBack = new Trajectory(    //terminal to fender
            new SimplePathBuilder(new Vector2(0, 0), Rotation2.fromDegrees(0))
                .lineTo(new Vector2(100, 0), Rotation2.fromDegrees(0))
                .build(),
            trajectoryConstraints, SAMPLE_DISTANCE);
    }

    public Trajectory getScootBack() {return scootBack;}

    public Trajectory getTwoBallPartOneSlow() {return twoBallPartOneSlow;}
    public Trajectory getTwoBallPartTwoSlow() {return twoBallPartTwoSlow;}

    public Trajectory getTwoBallPartOne() {return twoBallPartOne;}
    public Trajectory getTwoBallPartTwo() {return twoBallPartTwo;}

    public Trajectory getTwoPlusTwoBallPartThree() {return twoPlusTwoBallPartThree;}
    public Trajectory getTwoPlusTwoBallPartFour() {return twoPlusTwoBallPartFour;}
    public Trajectory getTwoPlusTwoBallPartFive() {return twoPlusTwoBallPartFive;}
    public Trajectory getTwoPlusTwoBallPartSix() {return twoPlusTwoBallPartSix;}
    public Trajectory getTwoPlusTwoBallPartSeven() {return twoPlusTwoBallPartSeven;}


    public Trajectory getFourBallPartOne() {return fourBallPartOne;}
    public Trajectory getFourBallPartTwo() {return fourBallPartTwo;}
    public Trajectory getFourBallPartThree() {return fourBallPartThree;}
    public Trajectory getFourBallPartFour() {return fourBallPartFour;}
    public Trajectory getFourBallPartFive() {return fourBallPartFive;}
}