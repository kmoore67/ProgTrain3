package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;


import edu.wpi.first.math.MathUtil;


import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.sim.PhysicsSim;
import frc.robot.subsystems.Shooter.Shooter.ShooterStats;

public class ShooterKraken {
  private static ShooterKraken instance = null;
  private double appliedVolts = 0.0; 
  private boolean isSimulation = false; 

   /**************************************************************** 
    * Change this variable to select which motor control feature runs 
    *     0-means run Velocity Voltage Control Mode  
    *     1-means run Velocity Torque Control Mode  
    *     2-means run Motion Magic Voltage Control Mode  
    *     3-means run Motion Magic Torque Control mode  
    *****************************************************************/           
  private int controlMode = 3; 
  
  // Hardware
  private TalonFX m_talonFX = new TalonFX(ShooterConstants.kShooterMotorID, ShooterConstants.kBusName);

  //Control 
  private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);    /* Start at velocity 0, use slot 0 */
  private final VelocityTorqueCurrentFOC m_velocityTorque = new VelocityTorqueCurrentFOC(0).withSlot(1);  /* Start at velocity 0, use slot 1 */
  private final MotionMagicVoltage m_mmVelocityVoltage = new MotionMagicVoltage(0).withSlot(2);
  private final MotionMagicTorqueCurrentFOC m_mmVelocityTorque = new MotionMagicTorqueCurrentFOC(0.0).withSlot(1);

  private final NeutralOut m_brake = new NeutralOut();   /* Keep a neutral out so we can disable the motor */
 
  // Status Signals
  private final StatusSignal<Angle> shooterPosition;
  private final StatusSignal<AngularVelocity> shooterVelocity;
  private final StatusSignal<Voltage> shooterAppliedVolts;
  private final StatusSignal<Current> shooterSupplyCurrent;
  private final StatusSignal<Current> shooterTorqueCurrent;
  private final StatusSignal<Temperature> shooterTempCelsius;
  
  //Constructor  
  public ShooterKraken() {

    /**********************************
    * Confiure the shooter Motor 
    ***********************************/
    // Apply Configuration from Intake Constants to motor 
    StatusCode statuscode = m_talonFX.getConfigurator().apply(ShooterConstants.ShooterConfigs.shooterWheelConfigs(), 1.0); 
    if(!statuscode.isOK()) { 
      System.out.println("Configuration Failed on Intake shooter Motor ");
    } 

    // Set signals
    shooterPosition = m_talonFX.getPosition();
    shooterVelocity = m_talonFX.getVelocity();
    shooterAppliedVolts = m_talonFX.getMotorVoltage();
    shooterSupplyCurrent = m_talonFX.getSupplyCurrent();
    shooterTorqueCurrent = m_talonFX.getTorqueCurrent();
    shooterTempCelsius = m_talonFX.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        shooterPosition,
        shooterVelocity,
        shooterAppliedVolts,
        shooterSupplyCurrent,
        shooterTorqueCurrent,
        shooterTempCelsius
    );

    /* Optimize out the other signals, since they're not useful */
    m_talonFX.optimizeBusUtilization();

    /* Defines the motor to the Physics Simulation-only in Simulation */
    if(Utils.isSimulation()){
      isSimulation = true; 
      PhysicsSim.getInstance().addTalonFX(m_talonFX, 0.001);
    }

    } // End Constructor 


  /*****************************************************************************
   * Return an instance of this class - Ensures that there is a single instance   
   ****************************************************************************/
  public static ShooterKraken getInstance() {
    return instance = instance == null ? new ShooterKraken() : instance;
  }

  /****************************************************************************************
  * Get the IO statictics from the motor and put them into the passed in IntakeStats class 
  *****************************************************************************************/ 
  public void updateStats(ShooterStats stats) {

    /* Only run the Physics Simulator in Simulation */
    if(isSimulation){
      PhysicsSim.getInstance().run();
    }

    stats.MotorConnected =
        BaseStatusSignal.refreshAll(
                shooterPosition,
                shooterVelocity,
                shooterAppliedVolts,
                shooterSupplyCurrent,
                shooterTorqueCurrent,
                shooterTempCelsius)
            .isOK();

    stats.shooterPositionRads = shooterPosition.getValueAsDouble();
    stats.shooterVelocityRpm = shooterVelocity.getValueAsDouble();
    stats.shooterAppliedVolts = shooterAppliedVolts.getValueAsDouble();
    stats.shooterSupplyCurrentAmps = shooterSupplyCurrent.getValueAsDouble();
    stats.shooterTorqueCurrentAmps = shooterTorqueCurrent.getValueAsDouble();
    stats.shooterTempCelsius = shooterTempCelsius.getValueAsDouble();
  }
  


  public void runVolts(double Voltage) {
      appliedVolts = MathUtil.clamp(Voltage,-12.0, 12.0);
      m_talonFX.setControl(new VoltageOut(appliedVolts));
  }
  
  /*********************
  * The runVelocity function can run (based on the controlMode Variable either:
  *     0-means run Velocity Voltage Control Mode  
  *     1-means run Velocity Torque Control Mode  
  *     2-means run Motion Magic Voltage Control Mode  
  *     3-means run Motion Magic Torque Control mode  
  **********************/
  public void runVelocity(double RPM, double Feedforward) {      
    //Convert rotations per minute into rotations per second 
    double desiredRotationsPerSecond = (RPM/60.0); 

    switch (controlMode) {
      case 0:      /* Use velocity voltage */
        m_talonFX.setControl(m_velocityVoltage.withVelocity(desiredRotationsPerSecond));
      case 1:      /* Use velocity torque */
        m_talonFX.setControl(m_velocityTorque.withVelocity(desiredRotationsPerSecond));
      case 2:      /*Use Motion Magic Voltage */
        m_talonFX.setControl(m_mmVelocityVoltage.withPosition(desiredRotationsPerSecond));
      case 3:      /*Use Motion Magic Torque Current FOC */
        m_talonFX.setControl(m_mmVelocityTorque.withPosition(desiredRotationsPerSecond));
      }
  }

  public void stop() {
    /* tell the motor to brake */
    m_talonFX.setControl(m_brake);
  }

  public void runCharacterization(double input) {
  }
}