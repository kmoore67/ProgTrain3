// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Intake extends SubsystemBase {
  private final IntakeStats ioStats = new IntakeStats();
  private final ShuffleboardTab intakeShuffleboard;
  private final IntakeKraken m_IntakeKraken = IntakeKraken.getInstance();

  /* Shuffleboard entrys */
  private GenericEntry intakePosition;
  private GenericEntry intakeVelocity;
  private GenericEntry intakeAppliedVolts;
  private GenericEntry intakeSupplyCurrent;
  private GenericEntry intakeTorqueCurrent;
  private GenericEntry intakeTemp;
  private GenericEntry intakeState;
  
  /* Define the Valid States for the Intake Subsystem  */
  public enum State {
    IDLE, 
    INTAKE, 
    RUNNING,  
    EJECT;
  }

  /* Variable to hold the current state of the state machine  */
  private State currentState = State.IDLE;

  /* a Generic class for handling the IO stats generated in the Kraken module  */  
  class IntakeStats {
    //Stats for the Intake/Expel Motor  
    public boolean MotorConnected = true;    
    public double rollerPositionRads = 0.0;
    public double rollerVelocityRpm = 0.0;
    public double rollerAppliedVolts = 0.0;
    public double rollerSupplyCurrentAmps = 0.0;
    public double rollerTorqueCurrentAmps = 0.0;
    public double rollerTempCelsius = 0.0;      
}

  /** Constructor - Creates a new IntakeSubsystem. */
  public Intake() {

    //Define Suffleboard Tab for this Subsystem 
    this.intakeShuffleboard = Shuffleboard.getTab("Intake");
    
    //Add Widgets to Shuffleboard Tab 
    intakeVelocity = this.intakeShuffleboard.add("Velocity (rot per s)", 0.0).getEntry();
    intakePosition = this.intakeShuffleboard.add("Position (rot)", 0.0).getEntry();;
    intakeSupplyCurrent = this.intakeShuffleboard.add("Supply Current", 0.0).getEntry();
    intakeTorqueCurrent = this.intakeShuffleboard.add("Torque Current", 0.0).getEntry();
    intakeTemp = this.intakeShuffleboard.add("Motor Temp", 0.0).getEntry();
    intakeAppliedVolts = this.intakeShuffleboard.add("Applied Volts",0.0).getEntry();
    intakeState = this.intakeShuffleboard.add("State","").getEntry();

  }

  private void flipState(State inState ) {
    System.out.println("Setting state...." + inState.name());
    currentState = inState; 
  }


  @Override  
  /* This method will be called once per scheduler run */
  public void periodic() {

    //go update the signal data from the motors 
    m_IntakeKraken.updateStats(ioStats);
     
    switch (currentState) {

      case IDLE -> {
        m_IntakeKraken.stop();
      }

      case INTAKE -> {
        //run intaking motoring forward
        m_IntakeKraken.runVelocity(IntakeConstants.kIntakeSpeed, 0.0);
        flipState(State.RUNNING);
      }

      case RUNNING -> {
        //m_IntakeKraken.runVelocity(IntakeConstants.kIntakeSpeed, 0.0);
      }

      case EJECT -> {
        //Run the intake rollers backwards 
        m_IntakeKraken.runVelocity(-IntakeConstants.kEjectSpeed,0.0);  
        flipState(State.RUNNING);
      }

    } // End of Switch 
    
    //Update Dashboard with Telementry Data 
    UpdateTelemetry();
  
  }

  /************************************************
  * Update the Shuffleboard with Motor Statistics 
  ************************************************/
  private void UpdateTelemetry() {
    intakeVelocity.setDouble(ioStats.rollerVelocityRpm);
    intakePosition.setDouble(ioStats.rollerPositionRads);
    intakeSupplyCurrent.setDouble(ioStats.rollerSupplyCurrentAmps);
    intakeTorqueCurrent.setDouble(ioStats.rollerTorqueCurrentAmps);
    intakeAppliedVolts.setDouble(ioStats.rollerAppliedVolts);
    intakeTemp.setDouble(ioStats.rollerTempCelsius);
    intakeState.setValue(currentState.name());

  }

  /************************************************
  * External functions for Old Command Style 
  ************************************************/
  public void runVelocity(double speed) {
    m_IntakeKraken.runVelocity(speed,0.0); 
  }

  public void stop() {
    m_IntakeKraken.stop(); 
  }

  /* characterType = true, run  Quasistatic */
  /* characterType = false, run  Dynamic */
  private void runCharacterization(boolean characterType, SysIdRoutine.Direction direction ) {
    //go update the signal data from the motors 
    m_IntakeKraken.updateStats(ioStats);
    m_IntakeKraken.runCharacterization(characterType, direction);
    //Update Dashboard with Telementry Data 
    UpdateTelemetry();
  }


  /**************************************************************
  * Commands 
  **************************************************************/
  /**
   * Set command to Intake
   * @return a command
   */
  public Command IntakeCommand() {
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          flipState(State.INTAKE);
        });
  }

  /**
   * Set command to Stow
   * @return a command
   */
  public Command IdleCommand() {
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          flipState(State.IDLE);
        });
  }

  /**
   * Set Command to Eject
   * @return a command
   */
  public Command EjectCommand() {
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(() -> {flipState(State.EJECT); });
  }

  /**
  * Set Command to Run SysID Quasistatic test
  * @return a command
  */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
      () -> {
        runCharacterization(true, direction);
      });
  }

  /**
   * Set Command to Run SysID Dynamic Test
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
      () -> {
        runCharacterization(false, direction);
      });
  }

  /**
  * Set Command to Start Logging to a Hoot File 
  * @return a command
  */
  public Command enableLogging() {
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(SignalLogger::start);
  }

  /**
  * Set Command to Stop Logging to a Hoot File 
  * @return a command
  */
  public Command disableLogging() {
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(SignalLogger::stop);
  }

}