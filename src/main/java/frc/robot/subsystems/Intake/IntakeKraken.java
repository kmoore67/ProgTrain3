package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.sim.PhysicsSim;
import frc.robot.subsystems.Intake.Intake.IntakeStats;



public class IntakeKraken implements Subsystem {
  private static IntakeKraken instance = null;
  private double appliedVolts = 0.0; 
  private boolean isSimulation = false; 
  /**************************************************************** 
  * Change this variable to select which motor control feature runs 
  *     0-means run Velocity Voltage Control Mode  
  *     1-means run Velocity Torque Control Mode  
  *     2-means run Motion Magic Voltage Control Mode  
  *     3-means run Motion Magic Torque Control mode  
  *****************************************************************/           
  private int controlMode = 0; 
    
  // Hardware
  private TalonFX m_talonFX = new TalonFX(IntakeConstants.kRollerMotorID, IntakeConstants.kBusName);

  // Control Commands 
  private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);    /* Start at velocity 0, use slot 0 */
  private final VelocityTorqueCurrentFOC m_velocityTorque = new VelocityTorqueCurrentFOC(0).withSlot(1);  /* Start at velocity 0, use slot 1 */
  private final MotionMagicVoltage m_mmVelocityVoltage = new MotionMagicVoltage(0).withSlot(2);
  private final MotionMagicTorqueCurrentFOC m_mmVelocityTorque = new MotionMagicTorqueCurrentFOC(0.0).withSlot(1);
  private final NeutralOut m_brake = new NeutralOut();   /* Keep a neutral out so we can disable the motor */
  private final VoltageOut m_sysIdControl = new VoltageOut(0);

  // Status Signals
  private final StatusSignal<Angle> rollerPosition;
  private final StatusSignal<AngularVelocity> rollerVelocity;
  private final StatusSignal<Voltage> rollerAppliedVolts;
  private final StatusSignal<Current> rollerSupplyCurrent;
  private final StatusSignal<Current> rollerTorqueCurrent;
  private final StatusSignal<Temperature> rollerTempCelsius;
  
  // Sys ID Control 
  private final SysIdRoutine m_sysIdRoutine =
    new SysIdRoutine(
        new SysIdRoutine.Config(
            null,         // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic voltage to 4 to prevent brownout
            null,          // Use default timeout (10 s)
                                    // Log state with Phoenix SignalLogger class
            state -> SignalLogger.writeString("state", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> m_talonFX.setControl(m_sysIdControl.withOutput(volts)),
            null,
            this
        )
    );



  //Constructor  
  public IntakeKraken() {

    /**********************************
    * Confiure the Roller Motor 
    ***********************************/
    // Apply Configuration from Intake Constants to motor 
    StatusCode statuscode = m_talonFX.getConfigurator().apply(IntakeConstants.IntakeConfigs.intakeRollerConfigs(), 1.0); 
    if(!statuscode.isOK()) { 
      System.out.println("Configuration Failed on Intake Roller Motor ");
    } 

    // Set signals
    rollerPosition = m_talonFX.getPosition();
    rollerVelocity = m_talonFX.getVelocity();
    rollerAppliedVolts = m_talonFX.getMotorVoltage();
    rollerSupplyCurrent = m_talonFX.getSupplyCurrent();
    rollerTorqueCurrent = m_talonFX.getTorqueCurrent();
    rollerTempCelsius = m_talonFX.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        rollerPosition,
        rollerVelocity,
        rollerAppliedVolts,
        rollerSupplyCurrent,
        rollerTorqueCurrent,
        rollerTempCelsius
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
  public static IntakeKraken getInstance() {
    return instance = instance == null ? new IntakeKraken() : instance;
  }

  /****************************************************************************************
  * Get the IO statictics from the motor and put them into the passed in IntakeStats class 
  *****************************************************************************************/ 
  public void updateStats(IntakeStats stats) {

    /* Only run the Physics Simulator in Simulation */
    if(isSimulation){
      PhysicsSim.getInstance().run();
    }

    stats.MotorConnected =
        BaseStatusSignal.refreshAll(
                rollerPosition,
                rollerVelocity,
                rollerAppliedVolts,
                rollerSupplyCurrent,
                rollerTorqueCurrent,
                rollerTempCelsius)
            .isOK();

    stats.rollerPositionRads = rollerPosition.getValueAsDouble();
    stats.rollerVelocityRpm = rollerVelocity.getValueAsDouble();
    stats.rollerAppliedVolts = rollerAppliedVolts.getValueAsDouble();
    stats.rollerSupplyCurrentAmps = rollerSupplyCurrent.getValueAsDouble();
    stats.rollerTorqueCurrentAmps = rollerTorqueCurrent.getValueAsDouble();
    stats.rollerTempCelsius = rollerTempCelsius.getValueAsDouble();
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

  public void runCharacterization(boolean characterType, SysIdRoutine.Direction direction) {

    if(characterType) {
      m_sysIdRoutine.quasistatic(direction);
    } else {
      m_sysIdRoutine.dynamic(direction);
    }

  }

}