// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final ShooterStats ioStats = new ShooterStats();
  private final ShuffleboardTab shooterShuffleboard;
  private final ShooterKraken mShooterKraken = ShooterKraken.getInstance();

  /* Shuffleboard entrys */
  private GenericEntry shooterPosition;
  private GenericEntry shooterVelocity;
  private GenericEntry shooterAppliedVolts;
  private GenericEntry shooterSupplyCurrent;
  private GenericEntry shooterTorqueCurrent;
  private GenericEntry shooterTemp;
  private GenericEntry shooterState;
  
  /* Define the Valid States for the shooter Subsystem  */
  public enum State {
    IDLE, 
    SHOOT, 
    RUNNING, 
    CHARACTERIZE;
  }

  /* Variable to hold the current state of the state machine  */
  private State currentState = State.IDLE;

  /* a Generic class for handling the IO stats generated in the Kraken module  */  
  class ShooterStats {
    //Stats for the Shooter Motor  
    public boolean MotorConnected = true;    
    public double shooterPositionRads = 0.0;
    public double shooterVelocityRpm = 0.0;
    public double shooterAppliedVolts = 0.0;
    public double shooterSupplyCurrentAmps = 0.0;
    public double shooterTorqueCurrentAmps = 0.0;
    public double shooterTempCelsius = 0.0;      
}

  /** Constructor - Creates a new shooterSubsystem. */
  public Shooter() {

    //Define Suffleboard Tab for this Subsystem 
    this.shooterShuffleboard = Shuffleboard.getTab("Shooter");
    shooterVelocity = this.shooterShuffleboard.add("Velocity (rot per s)", 0.0).getEntry();
    shooterPosition = this.shooterShuffleboard.add("Position (rot)", 0.0).getEntry();;
    shooterSupplyCurrent = this.shooterShuffleboard.add("Supply Current", 0.0).getEntry();
    shooterTorqueCurrent = this.shooterShuffleboard.add("Torque Current", 0.0).getEntry();
    shooterTemp = this.shooterShuffleboard.add("Motor Temp", 0.0).getEntry();
    shooterAppliedVolts = this.shooterShuffleboard.add("Applied Volts",0.0).getEntry();
    shooterState = this.shooterShuffleboard.add("State","").getEntry();

  }

  private void flipState(State inState ) {
    System.out.println("Setting state...." + inState.name());
    currentState = inState; 
  }

  @Override  
  /* This method will be called once per scheduler run */
  public void periodic() {
    //go update the signal data 
    mShooterKraken.updateStats(ioStats);
     
    switch (currentState) {

      case IDLE -> {
        mShooterKraken.stop();
      }

      case SHOOT -> {
        //run intaking motoring forward
        mShooterKraken.runVelocity(ShooterConstants.kShooterSpeed, 0.0);
        flipState(State.RUNNING);
      }

      case RUNNING -> {
        //mShooterKraken.runVelocity(shooterConstants.kshooterSpeed, 0.0);
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
    shooterVelocity.setDouble(ioStats.shooterVelocityRpm);
    shooterPosition.setDouble(ioStats.shooterPositionRads);
    shooterSupplyCurrent.setDouble(ioStats.shooterSupplyCurrentAmps);
    shooterTorqueCurrent.setDouble(ioStats.shooterTorqueCurrentAmps);
    shooterAppliedVolts.setDouble(ioStats.shooterAppliedVolts);
    shooterTemp.setDouble(ioStats.shooterTempCelsius);
    shooterState.setValue(currentState.name());

  }

  /************************************************
   * External functions for Old Command Style 
   ************************************************/
  public void runVelocity(double speed) {
    mShooterKraken.runVelocity(speed,0.0); 
  }

  public void stop() {
    mShooterKraken.stop(); 
  }



  /**************************************************************
  * Commands 
  **************************************************************/
  /**
   * Set command to shooter
   *
   * @return a command
   */
  public Command shootCommand() {
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
          flipState(State.SHOOT);
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
          flipState(State.IDLE);
        });
  }

}