package org.team5557.commands.drive;

import org.frcteam2910.library.control.PidConstants;
import org.frcteam2910.library.control.PidController;
import org.frcteam2910.library.math.MathUtils;
import org.frcteam2910.library.math.RigidTransform2;
import org.frcteam2910.library.math.Vector2;
import org.team5557.subsystems.SwerveSubsystemMK2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.lang.Math;
import java.util.function.DoubleSupplier;

public class SmartLock extends CommandBase {
    SwerveSubsystemMK2 drivetrain;
    RigidTransform2 pose;
    Vector2 target;
    private Vector2 distanceR2T;
    private double angleR2T;
    PIDController lockController;
    private double angle;
    private Vector2 fieldToTarget;

    public SmartLock(SwerveSubsystemMK2 drivetrainSubsystem, Vector2 target, boolean updatePoseWithLimelight) {
        drivetrain = drivetrainSubsystem;
        pose = drivetrain.getPose();
        this.target = target;
    }

    @Override
    public void initialize() {
        drivetrain.setToSnapping();
    }

    @Override
    public void execute() {
        //distanceR2T = target.subtract(pose.translation);
        //angleR2T = Math.atan2(-distanceR2T.y, -distanceR2T.x);  //take the negatives and flip cause jack in the bot is wack
        //angle = angleR2T - pose.rotation.toRadians();
        //angle = MathUtils.toUnitCircAngle(angle);
        //angle = -drivetrain.getGyroAngle() - angle;
        //System.out.println(pose.translation);
        //lockController.setSetpoint(angle);
        //drivetrain.drive(translationalVelocityRaw, MathUtil.clamp(lockController.calculate(pose.rotation.toRadians()), -0.05, 0.05), true);
        //double output = snapRotationController.calculate(drivetrain.getPose().rotation.toRadians(), dt);
        //drivetrain.drive(translationalVelocityRaw,
                    //-output,
                    //true);

        //System.out.println("Output: " + -output);
        //drivetrain.setSnapRotation(angle);

                //Vector2 translationalVelocityRaw = new Vector2(xAxis.getAsDouble(), yAxis.getAsDouble());
        pose = drivetrain.getPose();
        fieldToTarget = target;
        angle = getAngleToTarget(pose.translation, fieldToTarget).getRadians();
        drivetrain.setSnapRotation(angle);
    }

    public static Rotation2d getAngleToTarget(Vector2 fieldToRobot, Vector2 fieldToTarget) {
        Vector2 targetToRobot = fieldToTarget.subtract(fieldToRobot);
        return new Rotation2d(targetToRobot.x, targetToRobot.y);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(Vector2.ZERO, 0, false);
        drivetrain.setSnapRotation(Double.NaN);
        drivetrain.stopSnap();
    }
}
