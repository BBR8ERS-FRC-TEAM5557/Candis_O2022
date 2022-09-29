package org.team5557.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.frcteam2910.library.math.MathUtils;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.team5557.Constants;
import org.team5557.Robot;
import org.team5557.RobotContainer;
import org.team5557.util.ShooterConstantTuner;

public class ShooterSubsystem extends SubsystemBase{
    private final CANSparkMax flywheelMotor, flywheelSlave;
    final SparkMaxPIDController flywheelMotorPIDController;
    final RelativeEncoder flywheelMotorEncoder;

    //private static final double FLYWHEEL_POSITION_SENSOR_COEFFICIENT = 1.0 / 2048.0 * Constants.BOTTOM_FLYWHEEL_GEAR_RATIO;
    //private static final double FLYWHEEL_VELOCITY_SENSOR_COEFFICIENT = FLYWHEEL_POSITION_SENSOR_COEFFICIENT * (1000.0 / 100.0) * (60.0);//in terms of talonfx counts
    private static final double FLYWHEEL_FEEDFORWARD_COEFFICIENT = 0.0012148;
    private static final double FLYWHEEL_STATIC_FRICTION_CONSTANT = 0.5445;

    private static final double FLYWHEEL_ALLOWABLE_ERROR = 300.0;

    private static final double FLYWHEEL_P = 0.001;
    private static final double FLYWHEEL_I = 0.0;
    private static final double FLYWHEEL_D = 0.0;
    private static final double FLYWHEEL_F = 0.00025;

    private static final double FLYWHEEL_CURRENT_LIMIT = 10.0;

    private double m_flywheelTargetSpeed;
    private RobotContainer container;


    public ShooterSubsystem(RobotContainer container) {

        this.container = container;

        ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
        tab.addNumber("Flywheel Velocity", this::getFlywheelVelocity);
        tab.addNumber("Flywheel Setpoint", this::getFlywheelTargetVelocity);

        flywheelMotor = new CANSparkMax(Constants.SHOOTER_FLYWHEEL_MOTOR, MotorType.kBrushless);
        flywheelSlave = new CANSparkMax(Constants.SHOOTER_FLYWHEEL_MOTORSLAVE, MotorType.kBrushless);

        flywheelMotor.setInverted(true);
        //flywheelMotor.restoreFactoryDefaults();

        flywheelMotorEncoder = flywheelMotor.getEncoder();
        flywheelMotorPIDController = flywheelMotor.getPIDController();
        configurePID();
        flywheelSlave.follow(flywheelMotor, true);
    }



    public void shootFlywheel(double speed) {
        m_flywheelTargetSpeed = speed;
    }

    public void stopFlywheel() {
        m_flywheelTargetSpeed = 0;
    }

    public void idleFlywheel() {
        m_flywheelTargetSpeed = 1500;//container.getTunableNumbers().getIdleRPM();
    }

    public double getFlywheelVelocity() {
        return flywheelMotorEncoder.getVelocity();
    }

    public double getFlywheelTargetVelocity() {
        return m_flywheelTargetSpeed;
    }

    public boolean isFlywheelAtTargetVelocity() {
        return MathUtils.epsilonEquals(
                getFlywheelVelocity(),
                getFlywheelTargetVelocity(),
                FLYWHEEL_ALLOWABLE_ERROR
        );
    }

    public void configurePID() {
        flywheelMotorPIDController.setP(FLYWHEEL_P, 0);
        flywheelMotorPIDController.setI(FLYWHEEL_I, 0);
        flywheelMotorPIDController.setD(FLYWHEEL_D, 0);
        flywheelMotorPIDController.setFF(FLYWHEEL_F, 0);
        flywheelMotorPIDController.setOutputRange(0.0, 1.0);
        flywheelMotor.enableVoltageCompensation(FLYWHEEL_CURRENT_LIMIT);
        flywheelMotor.setSmartCurrentLimit(40);
        flywheelSlave.enableVoltageCompensation(FLYWHEEL_CURRENT_LIMIT);
        flywheelSlave.setSmartCurrentLimit(40);
    }

    @Override
    public void periodic() {
        flywheelMotorPIDController.setReference(m_flywheelTargetSpeed, ControlType.kVelocity, 0);
    }

}
