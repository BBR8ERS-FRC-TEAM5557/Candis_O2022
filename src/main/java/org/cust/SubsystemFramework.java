package org.cust;

public abstract class SubsystemFramework {
    public void writeToLog(){}
    public void readPeriodicInputs(){}
    public void writePeriodicInputs(){}
    public abstract void stop();
    public void zeroSensors(){}
    public abstract boolean checkSystem();
    public boolean hasEmergency = false;
}
