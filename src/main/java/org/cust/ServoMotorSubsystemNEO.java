package org.cust;

import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.InputMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;
import com.swervedrivespecialties.swervelib.rev.RevUtils;

public abstract class ServoMotorSubsystemNEO extends SubsystemFramework {
    private static final int kSmartMotionProfileSlot = 0;
    private static final int kPositionPIDSlot = 1;

    public static class NeoMotorConstants {
        public int id = -1;
        public MotorType kMotorType = MotorType.kBrushless;
        public boolean invert_motor = false;
        public boolean invert_sensor_phase = false;
    }

    public static class ServoMotorSubsystemNEOConstants {
        public String kName = "ERROR _ASSIGN_A_NAME";

        
        public InputMode kInputMode = InputMode.kCAN;


        public NeoMotorConstants kMasterConstants = new NeoMotorConstants();
        public NeoMotorConstants[] kSlaveConstants = new NeoMotorConstants[0];

        public double kHomePosition = 0.0; //units
        public double kTicksPerUnitDistance = 1.0;

        public double kKP = 0.0;
        public double kKI = 0.0;
        public double kKD = 0.0;
        public double kKFF = 0.0;
        public double kIZone = 0.0;
        public double kMaxIntegralAccumulator = 0.0;

        public double kSmartMotionKP = 0.0;
        public double kSmartMotionKI = 0.0;
        public double kSmartMotionKD = 0.0;
        public double kSmartMotionKFF = 0.0;
        public double kSmartMotionKIZone = 0.0;
        public double kSmartMotionMaxIntegralAccumulator = 0.0;

        public int kCruiseVelovity = 0;
        public int kAcceleration = 0;

        public double kMaxUnitsLimit = Double.POSITIVE_INFINITY;
        public double kMinUnitsLimit = Double.NEGATIVE_INFINITY;
        
    }

    protected final ServoMotorSubsystemNEOConstants mConstants;
    protected final CANSparkMax mMaster;
    protected final CANSparkMax[] mSlaves;

    protected final int mForwardSoftLimitTicks;
    protected final int mReverseSoftLimitTicks;

    public int CURRENT_LIMIT = 200; //TODO: IDEK what number to put

    protected ServoMotorSubsystemNEO(final ServoMotorSubsystemNEOConstants constants) {
        mConstants = constants;
        mMaster = new CANSparkMax(mConstants.kMasterConstants.id, mConstants.kMasterConstants.kMotorType);
        SparkMaxPIDController mMaster_PID = mMaster.getPIDController();
        mSlaves = new CANSparkMax[mConstants.kSlaveConstants.length];
        
        mMaster_PID.setFeedbackDevice(mMaster.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4096));

        mForwardSoftLimitTicks = (int) ((mConstants.kMaxUnitsLimit - mConstants.kHomePosition) * mConstants.kTicksPerUnitDistance);
        mMaster.setSoftLimit(SoftLimitDirection.kForward, mForwardSoftLimitTicks);
        mMaster.enableSoftLimit(SoftLimitDirection.kForward, true);

        mReverseSoftLimitTicks = (int) ((mConstants.kMinUnitsLimit - mConstants.kHomePosition) * mConstants.kTicksPerUnitDistance);
        mMaster.setSoftLimit(SoftLimitDirection.kReverse, mReverseSoftLimitTicks);
        mMaster.enableSoftLimit(SoftLimitDirection.kReverse, true);

        mMaster_PID.setP(mConstants.kKP, kPositionPIDSlot);
        mMaster_PID.setI(mConstants.kKI, kPositionPIDSlot);
        mMaster_PID.setD(mConstants.kKD, kPositionPIDSlot);
        mMaster_PID.setFF(mConstants.kKFF, kPositionPIDSlot);
        mMaster_PID.setIMaxAccum(mConstants.kMaxIntegralAccumulator, kPositionPIDSlot);
        mMaster_PID.setIZone(mConstants.kIZone, kPositionPIDSlot);

        mMaster_PID.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, kPositionPIDSlot);
        mMaster_PID.setSmartMotionMaxAccel(mConstants.kAcceleration, kPositionPIDSlot);
        mMaster_PID.setSmartMotionMaxVelocity(mConstants.kCruiseVelovity, kPositionPIDSlot);

        mMaster.setSecondaryCurrentLimit(CURRENT_LIMIT);
        mMaster.setIdleMode(IdleMode.kBrake);

        mMaster.setInverted(mConstants.kMasterConstants.invert_motor);
    }

    protected enum ControlState {
        OPEN_LOOP, SMART_MOTION, POSITION_PID, SMART_VELOCITY
    }

    protected ControlState mControlState = ControlState.OPEN_LOOP;
    protected boolean mHasBeenZeroed = false;

    public synchronized void setIdleMode(IdleMode mode) {
        mMaster.setIdleMode(mode);
        for (int i = 0; i < mSlaves.length; i++) {
            mSlaves[i].setIdleMode(mode);
        }
    }

    
}
