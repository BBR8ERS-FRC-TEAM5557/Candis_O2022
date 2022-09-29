package org.team5557.subsystems;

import org.frcteam2910.library.control.*;
import org.frcteam2910.library.drivers.Gyroscope;
import org.frcteam2910.library.drivers.MK2SwerveModule;
import org.frcteam2910.library.drivers.Mk2SwerveModuleBuilder;
import org.frcteam2910.library.drivers.NavX;
import org.frcteam2910.library.drivers.SwerveModule;
import org.frcteam2910.library.kinematics.ChassisVelocity;
import org.frcteam2910.library.kinematics.SwerveKinematics;
import org.frcteam2910.library.kinematics.SwerveOdometry;
import org.frcteam2910.library.math.RigidTransform2;
import org.frcteam2910.library.math.Rotation2;
import org.frcteam2910.library.math.Vector2;
import org.frcteam2910.library.robot.UpdateManager;
import org.frcteam2910.library.util.*;
import org.google.GuardedBy;

import java.util.Optional;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel;
//import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.*;
import org.team5557.Constants;
import org.team5557.commands.drive.SmartLock;
import org.frcteam2910.library.robot.subsystems.*;

public class SwerveSubsystemMK2 implements Subsystem, UpdateManager.Updatable {
    public static final double TRACKWIDTH = 21.5;
    public static final double WHEELBASE = 21.5;

    //constants for a feedforward controller --> learn about implementation?
    public static final DrivetrainFeedforwardConstants FEEDFORWARD_CONSTANTS = new DrivetrainFeedforwardConstants(
        0.042746,
        0.0032181,
        0.30764
    );
    //constraints for a trajectories max acceleration/deceleration and velocity
    public static final TrajectoryConstraint[] TRAJECTORY_CONSTRAINTS = {
        new FeedforwardConstraint(11.0, FEEDFORWARD_CONSTANTS.getVelocityConstant(), FEEDFORWARD_CONSTANTS.getAccelerationConstant(), false),
        new MaxAccelerationConstraint(12.5 * 12.0),
        new CentripetalAccelerationConstraint(15 * 12.0)
    };

    private static final int MAX_LATENCY_COMPENSATION_MAP_ENTRIES = 25;

    private final HolonomicMotionProfiledTrajectoryFollower follower = new HolonomicMotionProfiledTrajectoryFollower(
            new PidConstants(0.4, 0.0, 0.025),
            new PidConstants(1.0, 0.0, 0.0),
            new HolonomicFeedforward(FEEDFORWARD_CONSTANTS)
    );

    private final SwerveKinematics swerveKinematics = new SwerveKinematics(
            new Vector2(TRACKWIDTH / 2.0, WHEELBASE / 2.0),         //front left
            new Vector2(TRACKWIDTH / 2.0, -WHEELBASE / 2.0),        //front right
            new Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0),       //back left
            new Vector2(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0)        //back right
    );
    private final SwerveDriveKinematics wpi_driveKinematics = new SwerveDriveKinematics(
            new Translation2d(-TRACKWIDTH / 2.0, WHEELBASE / 2.0), //front left
            new Translation2d(TRACKWIDTH / 2.0, WHEELBASE / 2.0), //front right
            new Translation2d(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0), // back left
            new Translation2d(TRACKWIDTH / 2.0, -WHEELBASE / 2.0) // back right
    );
    
    private final SwerveModule[] modules;
    
    private final Object sensorLock = new Object();
    @GuardedBy("sensorLock")
    private final Gyroscope gyroscope = new NavX(SPI.Port.kMXP);


    private final Object kinematicsLock = new Object();
    @GuardedBy("kinematicsLock")
    private final SwerveOdometry swerveOdometry = new SwerveOdometry(swerveKinematics, RigidTransform2.ZERO);
    @GuardedBy("kinematicsLock")
    private RigidTransform2 pose = RigidTransform2.ZERO;
    @GuardedBy("kinematicsLock")
    private final InterpolatingTreeMap<InterpolatingDouble, RigidTransform2> latencyCompensationMap = new InterpolatingTreeMap<>();
    @GuardedBy("kinematicsLock")
    private Vector2 velocity = Vector2.ZERO;
    @GuardedBy("kinematicsLock")
    private double angularVelocity = 0.0;

    private final Object stateLock = new Object();
    @GuardedBy("stateLock")

    private HolonomicDriveSignal driveSignal = null;
    private double snapRotation = Double.NaN;
    private boolean m_isSnapping = false;
    private PidController snapRotationController;
    private static final PidConstants SNAP_ROTATION_CONSTANTS = new PidConstants(5.0, 0.0, 0.0);//(0.045, 0.065, 0.005);
    
    //logging
    private final NetworkTableEntry odometryXEntry;
    private final NetworkTableEntry odometryYEntry;
    private final NetworkTableEntry odometryAngleEntry;
    private final NetworkTableEntry[] moduleAngleEntries;

    private NetworkTableEntry snapP;

    final SwerveModule frontLeftModule = new Mk2SwerveModuleBuilder(
        new Vector2(TRACKWIDTH / 2.0, WHEELBASE / 2.0))
        .angleEncoder(new AnalogInput(Constants.DRIVETRAIN_FRONT_LEFT_ENCODER_PORT), Constants.DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET)
        .angleMotor(new CANSparkMax(Constants.DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
                Mk2SwerveModuleBuilder.MotorType.NEO)
        .driveMotor(new CANSparkMax(Constants.DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
                Mk2SwerveModuleBuilder.MotorType.NEO)
        .build();
    final SwerveModule frontRightModule = new Mk2SwerveModuleBuilder(
        new Vector2(TRACKWIDTH / 2.0, -WHEELBASE / 2.0))
        .angleEncoder(new AnalogInput(Constants.DRIVETRAIN_FRONT_RIGHT_ENCODER_PORT), Constants.DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET)
        .angleMotor(new CANSparkMax(Constants.DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
                Mk2SwerveModuleBuilder.MotorType.NEO)
        .driveMotor(new CANSparkMax(Constants.DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
                Mk2SwerveModuleBuilder.MotorType.NEO)
        .build();
    final SwerveModule backLeftModule = new Mk2SwerveModuleBuilder(
        new Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0))
        .angleEncoder(new AnalogInput(Constants.DRIVETRAIN_BACK_LEFT_ENCODER_PORT), Constants.DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET)
        .angleMotor(new CANSparkMax(Constants.DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
                Mk2SwerveModuleBuilder.MotorType.NEO)
        .driveMotor(new CANSparkMax(Constants.DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
                Mk2SwerveModuleBuilder.MotorType.NEO)
        .build();
    final SwerveModule backRightModule = new Mk2SwerveModuleBuilder(
        new Vector2(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0))
        .angleEncoder(new AnalogInput(Constants.DRIVETRAIN_BACK_RIGHT_ENCODER_PORT), Constants.DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET)
        .angleMotor(new CANSparkMax(Constants.DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
                Mk2SwerveModuleBuilder.MotorType.NEO)
        .driveMotor(new CANSparkMax(Constants.DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
                Mk2SwerveModuleBuilder.MotorType.NEO)
        .build();

    public SwerveSubsystemMK2() {
        snapRotationController = new PidController(SNAP_ROTATION_CONSTANTS);
        snapRotationController.setInputRange(0.0, 2.0 * Math.PI);
        snapRotationController.setContinuous(true);
        snapRotationController.setOutputRange(-0.5, 0.5);

        resetPose(new RigidTransform2(new Vector2(13, 12), Rotation2.ZERO));
        synchronized (sensorLock) {
            gyroscope.setInverted(true);
        }
        gyroscope.calibrate();
        modules = new SwerveModule[]{ 
            frontLeftModule,
            frontRightModule,
            backLeftModule,
            backRightModule
        };
        
        moduleAngleEntries = new NetworkTableEntry[modules.length];

        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        odometryXEntry = tab.add("X", 0.0)
                .withPosition(0, 0)
                .withSize(1, 1)
                .getEntry();
        odometryYEntry = tab.add("Y", 0.0)
                .withPosition(0, 1)
                .withSize(1, 1)
                .getEntry();
        odometryAngleEntry = tab.add("Angle", 0.0)
                .withPosition(0, 2)
                .withSize(1, 1)
                .getEntry();
        snapP = tab.add("Setpoint", 0.0)
                .withPosition(0, 3)
                .withSize(1, 1)
                .getEntry();


        ShuffleboardLayout[] moduleLayouts = {
                tab.getLayout("Front Left Module", BuiltInLayouts.kList),
                tab.getLayout("Front Right Module", BuiltInLayouts.kList),
                tab.getLayout("Back Left Module", BuiltInLayouts.kList),
                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
        };
        for (int i = 0; i < modules.length; i++) {
            ShuffleboardLayout layout = moduleLayouts[i]
                    .withPosition(2 + i * 2, 0)
                    .withSize(2, 4);
            moduleAngleEntries[i] = layout.add("Angle", 0.0).getEntry();
        }
        tab.addNumber("Rotation Voltage", () -> {
            HolonomicDriveSignal signal;
            synchronized (stateLock) {
                signal = driveSignal;
            }

            if (signal == null) {
                return 0.0;
            }

            return signal.getRotation() * RobotController.getBatteryVoltage();
        });
    }

    public void drive(Vector2 translationalVelocityRaw, double rotationalVelocityRaw, boolean isFieldOriented) {
        synchronized (stateLock) {
            driveSignal = new HolonomicDriveSignal(translationalVelocityRaw, rotationalVelocityRaw, isFieldOriented);
            //We are getting drive signal from the XBOX Controller
        }
    }

    @Override
    public void update(double time, double dt) {
        updateOdometry(time, dt);
        
        //HolonomicDriveSignal driveSignalProcessed;
        Optional<HolonomicDriveSignal> trajectorySignal = follower.update(getPose(), getVelocity(), getAngularVelocity(), time, dt);
        //NO SIGNAL COMING IN TO UPDATE driveSignal
        if (trajectorySignal.isPresent()) {
            driveSignal = trajectorySignal.get();
            driveSignal = new HolonomicDriveSignal(
                driveSignal.getTranslation().scale(1.0 / RobotController.getBatteryVoltage()),
                driveSignal.getRotation() / RobotController.getBatteryVoltage(),
                driveSignal.isFieldOriented()
            );
        } 
        else if (!m_isSnapping){
            synchronized (stateLock) {
                driveSignal = this.driveSignal;
            } 
        }
        else {
            double localSnapRotation;
            synchronized (stateLock) {
                localSnapRotation = snapRotation;
            }
            snapRotationController.setSetpoint(localSnapRotation);
    
            driveSignal = new  HolonomicDriveSignal(
                                driveSignal.getTranslation(),
                                snapRotationController.calculate(getGyroAngle(), dt),
                                driveSignal.isFieldOriented());
                                
            System.out.println("Setpoint: " + localSnapRotation);
            System.out.println("Angle: " + getGyroAngle());
        }

        updateModules(driveSignal, dt);
    }

    private void updateModules(HolonomicDriveSignal driveSignal, double dt) {
        ChassisVelocity chassisVelocity;
        if (driveSignal == null) {
            chassisVelocity = new ChassisVelocity(Vector2.ZERO, 0.0);
        } else if (driveSignal.isFieldOriented()) {
            chassisVelocity = new ChassisVelocity(
                driveSignal.getTranslation().rotateBy(gyroscope.getAngle().inverse()),//getPose().rotation.inverse()),
                driveSignal.getRotation()
            );
        } else {
            chassisVelocity = new ChassisVelocity(
                driveSignal.getTranslation(),
                driveSignal.getRotation()
            );
        }

        Vector2[] moduleOutputs = swerveKinematics.toModuleVelocities(chassisVelocity);
        SwerveKinematics.normalizeModuleVelocities(moduleOutputs, 1);

        for (int i = 0; i < moduleOutputs.length; i++) {
            var module = modules[i];
            //moduleOutputs[i].length = moduleOutputs[i].length/4;
            module.setTargetVelocity(moduleOutputs[i]); 
            module.updateState(dt);
            //System.out.println("Module " + i + ": " + moduleOutputs[i]);
        }
    }

    private void updateOdometry(double time, double dt) {
        Vector2[] moduleVelocities = new Vector2[modules.length];
        for (int i = 0; i < modules.length; i++) {
            var module = modules[i];
            module.updateSensors();

            moduleVelocities[i] = Vector2.fromAngle(Rotation2.fromRadians(module.getCurrentAngle())).scale(module.getCurrentVelocity());
            //a vector of the modules current velocity in terms of inches
        }

        Rotation2 angle;
        double angularVelocity;
        synchronized (sensorLock) {
            angle = gyroscope.getAngle();
            angularVelocity = gyroscope.getRate();
        }

        ChassisVelocity velocity = swerveKinematics.toChassisVelocity(moduleVelocities);

        synchronized (kinematicsLock) {
            this.pose = swerveOdometry.update(angle, dt, moduleVelocities);
          

            if (latencyCompensationMap.size() > MAX_LATENCY_COMPENSATION_MAP_ENTRIES) {
                latencyCompensationMap.remove(latencyCompensationMap.firstKey());
            }
            latencyCompensationMap.put(new InterpolatingDouble(time), pose);

            this.velocity = velocity.getTranslationalVelocity();
            this.angularVelocity = angularVelocity;
        }
    }

    @Override
    public void periodic() {
        RigidTransform2 pose = getPose();

        odometryXEntry.setDouble(pose.translation.x);
        odometryYEntry.setDouble(pose.translation.y);
        odometryAngleEntry.setDouble(getPose().rotation.toDegrees());
        snapP.setDouble(Math.toDegrees(snapRotationController.getSetpoint()));

        for (int i = 0; i < modules.length; i++) {
            moduleAngleEntries[i].setDouble(Math.toDegrees(modules[i].getCurrentAngle()));
        }
    }


    //THIS CODE IS RESETTING METHODS
    public void resetPose(RigidTransform2 pose) {
        synchronized (kinematicsLock) {
            this.pose = pose;
            swerveOdometry.resetPose(pose);
        }
        System.out.print("Pose Reset");
    }

    public void resetGyroAngle(Rotation2 angle) {
        synchronized (sensorLock) {
            gyroscope.setAdjustmentAngle(gyroscope.getUnadjustedAngle().rotateBy(angle.inverse()));
        }
    }

    public void setSnapRotation(double snapRotation) {
        synchronized (stateLock) {
            this.snapRotation = snapRotation;
        }
    }

    public void setToSnapping() {
        synchronized (stateLock) {
            m_isSnapping = true;
        }
    }

    public void stopSnap() {
        synchronized (stateLock) {
            this.snapRotation = Double.NaN;
            m_isSnapping = false;
        }
    }

    //THIS CODE IS ALL GETTER METHODS
    public HolonomicMotionProfiledTrajectoryFollower getFollower() {
        return follower;
    }

    public RigidTransform2 getPoseAtTime(double timestamp) {
        synchronized (kinematicsLock) {
            if (latencyCompensationMap.isEmpty()) {
                return RigidTransform2.ZERO;
            }
            return latencyCompensationMap.getInterpolated(new InterpolatingDouble(timestamp));
        }
    }

    public double getAverageAbsoluteValueVelocity() {
        double averageVelocity = 0;
        for (var module : modules) {
            averageVelocity += Math.abs(module.getCurrentVelocity());
        }
        return averageVelocity / 4;
    }

    public RigidTransform2 getPose() {
        synchronized (kinematicsLock) {
            return pose;
        }
    }

    public Vector2 getVelocity() {
        synchronized (kinematicsLock) {
            return velocity;
        }
    }

    public double getAngularVelocity() {
        synchronized (kinematicsLock) {
            return angularVelocity;
        }
    }

    public double getGyroAngle(){
        return gyroscope.getAngle().toRadians();
    }

}