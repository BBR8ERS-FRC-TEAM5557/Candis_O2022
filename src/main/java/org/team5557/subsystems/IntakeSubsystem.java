package org.team5557.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.team5557.Constants;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SynchronousInterrupt.WaitResult;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.PneumaticHub;



/**
 * Add your docs here.
 */
public class IntakeSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public final Solenoid intakeCylinders;
  public PneumaticHub pneumaticHub;
  private CANSparkMax intakeMotor, storeMotor;
  private double m_intakeMotorOutput, m_storeMotorOutput;


  //NetworkTableEntry pneumaticsTab;
  
    public IntakeSubsystem() {
	    pneumaticHub = new PneumaticHub(1);
        intakeCylinders = new Solenoid(PneumaticsModuleType.REVPH, 8);
        intakeMotor = new CANSparkMax(Constants.INTAKE_MOTOR_CONTROLLER, MotorType.kBrushless);
        storeMotor = new CANSparkMax(Constants.STORE_MOTOR_CONTROLLER, MotorType.kBrushless);
        intakeCylinders.setPulseDuration(0.15);
        //leftIntake.set(true);        
        //pneumaticsTab = Shuffleboard.getTab("pneumatics").add("psi", pneumaticHub.getPressure(0)).getEntry();
	}

    public void extendIntake() {
        intakeCylinders.set(true);
    }

    public void retractIntakeDelicately() throws InterruptedException {
        intakeCylinders.set(false);
        Thread.sleep(125);
        intakeCylinders.startPulse();
    }

    public void retractIntakeViolently() {
        intakeCylinders.set(false);
    }

    public void setIntakeMotorOutput(double motorOutput) {
        this.m_intakeMotorOutput = motorOutput;
    }

    public void setStoreMotorOutput(double motorOutput) {
        this.m_storeMotorOutput = motorOutput;
    }

    public double getPressure() {
        return pneumaticHub.getPressure(0);
    }

    @Override
    public void periodic() {
        intakeMotor.set(m_intakeMotorOutput);
        storeMotor.set(m_storeMotorOutput);
    }
}
