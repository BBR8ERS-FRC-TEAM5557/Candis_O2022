package org.team5557.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.team5557.Constants;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
    private final CANSparkMax m_leftMotorMaster, m_leftMotorInverse, m_rightMotorMaster, m_rightMotorInverse;
    final RelativeEncoder m_leftMotorEncoder, m_rightMotorEncoder;

    final SparkMaxPIDController m_leftMotorPIDController, m_rightMotorPIDController;

    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, allowedErr;
    public boolean isZeroing;
    private boolean climbEnabled;
    private int m_desiredHeight = 0;
    private int lowerLimit = 0;
    private int upperLimit = 185;

    public ClimberSubsystem() {
        m_leftMotorMaster= new CANSparkMax(Constants.CLIMBER_LEFT_MOTOR_INVERSE, MotorType.kBrushless);
        m_leftMotorInverse = new CANSparkMax(Constants.CLIMBER_LEFT_MOTOR_MASTER, MotorType.kBrushless);
        m_rightMotorMaster = new CANSparkMax(Constants.CLIMBER_RIGHT_MOTOR_MASTER, MotorType.kBrushless);
        m_rightMotorInverse = new CANSparkMax(Constants.CLIMBER_RIGHT_MOTOR_INVERSE, MotorType.kBrushless);

        m_leftMotorEncoder = m_leftMotorMaster.getEncoder();
        m_rightMotorEncoder = m_rightMotorMaster.getEncoder();

        m_leftMotorPIDController = m_leftMotorMaster.getPIDController();
        m_rightMotorPIDController = m_rightMotorMaster.getPIDController();

        m_leftMotorMaster.setInverted(true);
        m_rightMotorMaster.setInverted(true);

        m_leftMotorInverse.follow(m_leftMotorMaster, true);
        m_rightMotorInverse.follow(m_rightMotorMaster, true);


        ShuffleboardTab tab = Shuffleboard.getTab("Climber");
        tab.addNumber("Left Height", this::getLeftHeight);
        tab.addNumber("Right Height", this::getRightHeight);
        tab.addNumber("Desired Height", this::getDesiredHeight);
        tab.addBoolean("Climb Enabled", this::getClimbEnabled);

        // PID coefficients
        kP = 2; 
        kI = 0;
        kD = 0; 
        kIz = 0; 
        kFF = 0;//0.000156; 
        kMaxOutput = 1; 
        kMinOutput = -1;

        //
        climbEnabled = false;
        isZeroing = false;

        this.configurePID();
        this.configureSoftLimits();
    }

    @Override
    public void periodic() {
        if (!isZeroing && climbEnabled) {
            if (Math.abs(m_leftMotorEncoder.getPosition() - m_desiredHeight) > 0.2 && Math.abs(m_rightMotorEncoder.getPosition() - m_desiredHeight) > 0.2) {
                followHeight();
            }
        }
    }

    public void zeroHeight() {
        m_leftMotorMaster.getEncoder().setPosition(0.0);
        m_rightMotorMaster.getEncoder().setPosition(0.0);
    }

    public double getHeight() {
        return (m_leftMotorEncoder.getPosition() + m_rightMotorEncoder.getPosition())/2.0;
    }

    public void followHeight() {
        System.out.println("following height");
        m_leftMotorPIDController.setReference(m_desiredHeight, ControlType.kPosition);
        m_rightMotorPIDController.setReference(m_desiredHeight, ControlType.kPosition);
    }

    public void setHeight(int height) {
        if (height < lowerLimit) {
            m_desiredHeight = lowerLimit;
        }
        else if (height > upperLimit) {
            m_desiredHeight = upperLimit;
        }
        else {
            m_desiredHeight = height;
        }
    }

    public int getDesiredHeight() {
        return m_desiredHeight;
    }

    public double getLeftHeight() {
        return m_leftMotorEncoder.getPosition();
    }

    public double getRightHeight() {
        return m_rightMotorEncoder.getPosition();
    }

    public void setMotorOutput(double output) {
        //m_leftMotorPIDController.setReference(output, ControlType.kVoltage);
        m_rightMotorMaster.set(output);
        m_leftMotorMaster.set(output);
        //m_rightMotorPIDController.setReference(output, ControlType.kVoltage);
    }

    public void riseToBar() {
        setHeight(20);
    }

    public void releaseFromBar() {
        setHeight(30);
    }

    public void extendToMidBar() {
        setHeight(185);
    }

    private void configurePID() {
        m_leftMotorPIDController.setP(kP);
        m_leftMotorPIDController.setI(kI);
        m_leftMotorPIDController.setD(kD);
        m_leftMotorPIDController.setIZone(kIz);
        m_leftMotorPIDController.setFF(kFF);
        m_leftMotorPIDController.setOutputRange(kMinOutput, kMaxOutput);

        m_rightMotorPIDController.setP(kP);
        m_rightMotorPIDController.setI(kI);
        m_rightMotorPIDController.setD(kD);
        m_rightMotorPIDController.setIZone(kIz);
        m_rightMotorPIDController.setFF(kFF);
        m_rightMotorPIDController.setOutputRange(kMinOutput, kMaxOutput);
    }

    public void configureSoftLimits() {
        m_leftMotorMaster.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, upperLimit+5);
        m_leftMotorMaster.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, lowerLimit+2);
    
        m_rightMotorMaster.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, upperLimit+5);
        m_rightMotorMaster.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, lowerLimit+2);
    }

    public void enableSoftLimits() {
        m_leftMotorMaster.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        m_leftMotorMaster.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        m_rightMotorMaster.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        m_rightMotorMaster.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    }
    public void disableSoftLimits() {
        m_leftMotorMaster.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
        m_leftMotorMaster.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
        m_rightMotorMaster.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
        m_rightMotorMaster.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
    }

    public void changeClimbEnabled() {
        climbEnabled = !climbEnabled;
    }

    public boolean getClimbEnabled() {
        return climbEnabled;
    }

    public boolean isAtHeight() {
        return Math.abs(m_desiredHeight - getHeight()) < 3.0 ;
    }

    public void setMinandMaxOutput(double number) {
        m_leftMotorPIDController.setOutputRange(-number, number);
        m_rightMotorPIDController.setOutputRange(-number, number);
    }
}

