// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
  private final TurretStats ioStats = new TurretStats();
  private final ShuffleboardTab turretShuffleboard;
  private final TurretKraken m_TurretKraken = TurretKraken.getInstance();

  /* Shuffleboard entrys */
  private GenericEntry turretPosition;
  private GenericEntry turretVelocity;
  private GenericEntry turretAppliedVolts;
  private GenericEntry turretSupplyCurrent;
  private GenericEntry turretTorqueCurrent;
  private GenericEntry turretTemp;
  private GenericEntry turretState;
  private GenericEntry turretGoal; 
  
  /* Define the Valid States for the Turret Subsystem  */
  public enum State {
    IDLE, 
    GOTOANGLE, 
    CHARACTERIZE, 
  }

  /* Variable to hold the current state of the state machine  */
  private State currentState = State.IDLE;

  private double requestedRotations = 0.0;
  private double leftAngle = -20.0;
  private double rightAngle = 20.0;  

  /* a Generic class for handling the IO stats generated in the Kraken module  */  
  class TurretStats {
    //Stats for the turret/Expel Motor  
    public boolean MotorConnected = true;    
    public double turretPositionRads = 0.0;
    public double turretVelocityRpm = 0.0;
    public double turretAppliedVolts = 0.0;
    public double turretSupplyCurrentAmps = 0.0;
    public double turretTorqueCurrentAmps = 0.0;
    public double turretTempCelsius = 0.0;      
}

  /** Constructor - Creates a new turretSubsystem. */
  public Turret() {

    //Define Suffleboard Tab for this Subsystem 
    this.turretShuffleboard = Shuffleboard.getTab("Turret");
    turretVelocity = this.turretShuffleboard.add("Velocity (rot per s)", 0.0).getEntry();
    turretPosition = this.turretShuffleboard.add("Position (rot)", 0.0).getEntry();;
    turretSupplyCurrent = this.turretShuffleboard.add("Supply Current", 0.0).getEntry();
    turretTorqueCurrent = this.turretShuffleboard.add("Torque Current", 0.0).getEntry();
    turretTemp = this.turretShuffleboard.add("Motor Temp", 0.0).getEntry();
    turretAppliedVolts = this.turretShuffleboard.add("Applied Volts",0.0).getEntry();
    turretState = this.turretShuffleboard.add("State","").getEntry();
    turretGoal = this.turretShuffleboard.add("Goal",0.0).getEntry();

  }

  private void flipState(State inState, double reqRotations ) {
    System.out.println("Setting state...." + inState.name() + " Rotations-"+ reqRotations);
    requestedRotations = reqRotations;
    currentState = inState; 
  }

  @Override  
  /* This method will be called once per scheduler run */
  public void periodic() {
    //go update the signal data 
    m_TurretKraken.updateStats(ioStats);
     
    switch (currentState) {

      case IDLE -> {
        // m_TurretKraken.stop();
      }

      case GOTOANGLE -> {
        //move turret to requested position 
        m_TurretKraken.runPosition(requestedRotations);
        turretGoal.setDouble(requestedRotations); 
        flipState(State.IDLE,0.0);
       
      }

      case CHARACTERIZE -> {}

    } // End of Switch 
    
    //Update Dashboard with Telementry Data 
    UpdateTelemetry();
  
  }

  /************************************************
   * Update the Shuffleboard with Motor Statistics 
   ************************************************/
  private void UpdateTelemetry() {
    turretVelocity.setDouble(ioStats.turretVelocityRpm);
    turretPosition.setDouble(ioStats.turretPositionRads);
    turretSupplyCurrent.setDouble(ioStats.turretSupplyCurrentAmps);
    turretTorqueCurrent.setDouble(ioStats.turretTorqueCurrentAmps);
    turretAppliedVolts.setDouble(ioStats.turretAppliedVolts);
    turretTemp.setDouble(ioStats.turretTempCelsius);
    turretState.setValue(currentState.name());

  }


  /**************************************************************
  * Commands 
  **************************************************************/
  /**
   * Set command to turret
   *
   * @return a command
   */
  public Command LeftAngleCommand() {
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
          flipState(State.GOTOANGLE,leftAngle);
        });
  }
   /**
   * Set command to turret
   *
   * @return a command
   */
  public Command RightAngleCommand() {
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
          flipState(State.GOTOANGLE,rightAngle);
        });
  }

  /**
   * Set command to Stow
   *
   * @return a command
   */
  public Command IdleCommand() {
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
          flipState(State.IDLE,0.0);
        });
  }


}