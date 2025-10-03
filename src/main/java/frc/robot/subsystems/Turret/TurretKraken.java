package frc.robot.subsystems.Turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.sim.PhysicsSim;
import frc.robot.subsystems.Turret.Turret.TurretStats;


public class TurretKraken {
  private static TurretKraken instance = null;
  private double appliedVolts = 0.0; 
  private boolean isSimulation = false; 
   
  /**************************************************************** 
  * Change this variable to select which motor control feature runs 
  *     0-means run Position Voltage Control Mode  
  *     1-means run Position Torque Control Mode  
  *     2-means run Motion Magic Expo Position Voltage Control Mode  
  *     3-means run Motion Magic Expo Position Torque Control mode  
  *****************************************************************/           
  private int controlMode = 3; /* 0-PositionVoltage, 1-PositionTorque, 2-MotionMagic Position Voltage, 3-MotionMagic Position Torque */ 
 
    
  // Hardware
  private TalonFX m_talonFX = new TalonFX(TurretConstants.kTurretMotorID, TurretConstants.kBusName);

  //Control 
  private final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);
  private final PositionTorqueCurrentFOC m_positionTorque = new PositionTorqueCurrentFOC(0).withSlot(1);
  private final MotionMagicExpoVoltage m_mmExpoVoltage = new MotionMagicExpoVoltage(0).withSlot(2);
  private final MotionMagicExpoTorqueCurrentFOC m_mmExpoTorque = new MotionMagicExpoTorqueCurrentFOC(0.0).withSlot(1);

  private final NeutralOut m_brake = new NeutralOut();   /* Keep a neutral out so we can disable the motor */

  // Status Signals
  private final StatusSignal<Angle> turretPosition;
  private final StatusSignal<AngularVelocity> turretVelocity;
  private final StatusSignal<Voltage> turretAppliedVolts;
  private final StatusSignal<Current> turretSupplyCurrent;
  private final StatusSignal<Current> turretTorqueCurrent;
  private final StatusSignal<Temperature> turretTempCelsius;
  


  //Constructor  
  public TurretKraken() {

    /**********************************
    * Confiure the turret Motor 
    ***********************************/
    // Apply Configuration from Intake Constants to motor 
    StatusCode statuscode = m_talonFX.getConfigurator().apply(TurretConstants.TurretConfigs.turretMotorConfigs(), 1.0); 
  
    if(!statuscode.isOK()) { 
      System.out.println("Configuration Failed on Turret Motor ");
    } 

    /* Make sure we start at 0
     * This is only for testing - normally you would want to use the absolute encoder to get the current 
     * position of where the mechanism currently is - not force it to start at zero 
     */
     m_talonFX.setPosition(0);

    // Set signals
    turretPosition = m_talonFX.getPosition();
    turretVelocity = m_talonFX.getVelocity();
    turretAppliedVolts = m_talonFX.getMotorVoltage();
    turretSupplyCurrent = m_talonFX.getSupplyCurrent();
    turretTorqueCurrent = m_talonFX.getTorqueCurrent();
    turretTempCelsius = m_talonFX.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        turretPosition,
        turretVelocity,
        turretAppliedVolts,
        turretSupplyCurrent,
        turretTorqueCurrent,
        turretTempCelsius
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
  public static TurretKraken getInstance() {
    return instance = instance == null ? new TurretKraken() : instance;
  }

  /****************************************************************************************
  * Get the IO statictics from the motor and put them into the passed in IntakeStats class 
  *****************************************************************************************/ 
  public void updateStats(TurretStats stats) {

    /* Only run the Physics Simulator in Simulation */
    if(isSimulation){
      PhysicsSim.getInstance().run();
    }

    stats.MotorConnected =
        BaseStatusSignal.refreshAll(
                turretPosition,
                turretVelocity,
                turretAppliedVolts,
                turretSupplyCurrent,
                turretTorqueCurrent,
                turretTempCelsius)
            .isOK();

    stats.turretPositionRads = turretPosition.getValueAsDouble();
    stats.turretVelocityRpm = turretVelocity.getValueAsDouble();
    stats.turretAppliedVolts = turretAppliedVolts.getValueAsDouble();
    stats.turretSupplyCurrentAmps = turretSupplyCurrent.getValueAsDouble();
    stats.turretTorqueCurrentAmps = turretTorqueCurrent.getValueAsDouble();
    stats.turretTempCelsius = turretTempCelsius.getValueAsDouble();
  }
  


  public void runVolts(double Voltage) {
      appliedVolts = MathUtil.clamp(Voltage,-12.0, 12.0);
      m_talonFX.setControl(new VoltageOut(appliedVolts));
  }
  
  /***************************************************************************
  * The runVelocity function can run (based on the controlMode Variable either:
  *     0-means run Position Voltage Control Mode  
  *     1-means run Position Torque Control Mode  
  *     2-means run Motion Magic Expo Voltage Control Mode  
  *     3-means run Motion Magic Expo Torque Control mode  
  ***************************************************************************/
  public void runPosition(double desiredRotations) {      
    switch (controlMode) {
      case 0:      /* Use Position Voltage */
        m_talonFX.setControl(m_positionVoltage.withPosition(desiredRotations));
      case 1:      /* Use Position Torque */
        m_talonFX.setControl(m_positionTorque.withPosition(desiredRotations));
      case 2:      /* Use Motion Magic Position Voltage */
        m_talonFX.setControl(m_mmExpoVoltage.withPosition(desiredRotations));
      case 3:      /* Use Motion Magic Position Torque */
        m_talonFX.setControl(m_mmExpoTorque.withPosition(desiredRotations));
    }
  }

  public void stop() {
    /* tell the motor to brake */
    m_talonFX.setControl(m_brake);
  }

  public void runCharacterization(double input) {
  }
}