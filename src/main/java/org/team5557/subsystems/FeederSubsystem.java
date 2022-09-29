package org.team5557.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.frcteam2910.library.math.MathUtils;
import org.team5557.Constants;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSubsystem extends SubsystemBase{

    CANSparkMax feederMotor;
    double m_feederMotorSpeed;
    SparkMaxPIDController feederController;
    private RelativeEncoder feederEncoder;

    private static final double FLYWHEEL_P = 0.0003;
    private static final double FLYWHEEL_I = 0.0;
    private static final double FLYWHEEL_D = 0.0;
    private static final double FLYWHEEL_F = 0.0004;

    private static final double FEEDER_ALLOWABLE_ERROR = 100;

    public FeederSubsystem() {
        feederMotor = new CANSparkMax(Constants.UPLIFT_MOTOR_CONTROLLER, MotorType.kBrushless);
        feederMotor.setIdleMode(IdleMode.kBrake);
        feederController = feederMotor.getPIDController();
        feederEncoder = feederMotor.getEncoder();
        configurePID();

        ShuffleboardTab tab = Shuffleboard.getTab("Feeder");
        tab.addNumber("Flywheel Velocity", this::getFeederVelocity);
        tab.addNumber("Flywheel Setpoint", this::getFeederTargetVelocity);
    }

    public void setFeederMotorOutput(double motorOutput) {
        m_feederMotorSpeed = motorOutput;
    }

    public void stopFeeder() {
        m_feederMotorSpeed = 0.0;
    }

    public double getFeederVelocity() {
        return feederEncoder.getVelocity();
    }

    public boolean isFeederAtTargetVelocity() {
        return MathUtils.epsilonEquals(
            getFeederVelocity(),
            m_feederMotorSpeed,
            FEEDER_ALLOWABLE_ERROR
    );
    }

    public double getFeederTargetVelocity() {
        return m_feederMotorSpeed;
    }

    @Override
    public void periodic() {
        feederController.setReference(m_feederMotorSpeed, ControlType.kVelocity, 0);
    }

    public void configurePID() {
        feederController.setP(FLYWHEEL_P, 0);
        feederController.setI(FLYWHEEL_I, 0);
        feederController.setD(FLYWHEEL_D, 0);
        feederController.setFF(FLYWHEEL_F, 0);
    }
}
