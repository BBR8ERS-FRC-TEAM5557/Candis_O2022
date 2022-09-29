package org.team5557.util;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class ShooterConstantTuner {

    private double highRPM, lowRPM;
    private double idleRPM;
    private double highFeederRPM;

    private double highRPMDefault = 3750;
    private double lowRPMDefault = 2000;
    private double idleRPMDefault = 1500;
    private double highFeederRPMDefault = 5000;

    private NetworkTableEntry highRPMEntry;
    private NetworkTableEntry lowRPMEntry;
    private NetworkTableEntry idleRPMEntry;
    private NetworkTableEntry highFeederRPMEntry;


    public ShooterConstantTuner() {

        ShuffleboardTab tab = Shuffleboard.getTab("Tunable Numbers");

        highRPMEntry = tab.add("High RPM", highRPMDefault).getEntry();
        lowRPMEntry = tab.add("Low RPM", lowRPMDefault).getEntry();
        idleRPMEntry = tab.add("Idle RPM", idleRPMDefault).getEntry();

        highFeederRPMEntry = tab.add("High Feeder RPM", highFeederRPMDefault).getEntry();

        tab.add("Update Values", new Update(this));

    }


    public void updateValues() {
        highRPM = highRPMEntry.getDouble(highRPMDefault);
        lowRPM = lowRPMEntry.getDouble(lowRPMDefault);
        idleRPM = idleRPMEntry.getDouble(idleRPMDefault);
        highFeederRPM = highFeederRPMEntry.getDouble(highFeederRPMDefault);

        System.out.println(getHighRPM());
    }

    private static class Update extends CommandBase {
        private ShooterConstantTuner tunableNumbers;

        public Update(ShooterConstantTuner tuner) {
            tunableNumbers = tuner;
            setName("Update Values");
        }

        @Override
        public void initialize() {
            tunableNumbers.updateValues();
        }

        @Override
        public boolean runsWhenDisabled() {
            return true;
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    public double getHighRPM() {return highRPM;}

    public double getLowRPM() {return lowRPM;}

    public double getIdleRPM() {return idleRPM;}

    public double getHighFeederRPM() {return highFeederRPM;}

}
