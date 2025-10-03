package frc.robot.subsystems.Turret;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;


public class TurretConstants {
    
    public static int kTurretMotorID = 25;
    public static int kTurretEncoderID = 26 ;
    public static double kReduction =(1.0/1.0);
    public static double kTurretRotations = 20.0; 
    public static String kBusName = "sim";

    // Define configuration for Intake Roller Motor 
    public static final TalonFXConfiguration turretMotorConfigs = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(40)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(40)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLowerLimit(30)
                .withSupplyCurrentLowerTime(1)
        ).withTorqueCurrent(
            new TorqueCurrentConfigs()
                .withPeakForwardTorqueCurrent(120)
                .withPeakReverseTorqueCurrent(-120)
        ).withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)    
                .withNeutralMode(NeutralModeValue.Coast)   
        ).withSlot0(
            new Slot0Configs()            /* Voltage-based position */
                .withKP(2.4)        // An error of 1 rotation results in 2.4v output
                .withKI(0.0)        // No output for integrated error
                .withKD(0.1)        // A velocity of 1 rps results in 0.1v output 
        ).withSlot1(
            new Slot1Configs()            /* Torque-based position  */
                .withKP(60.0)       // An error of 1 rotation results in 60 A output
                .withKI(0.0)        // No output for integrated error     
                .withKD(6.0)        // A velocity of 1 rps results in 6 A output 
        ).withSlot2(                  
            new Slot2Configs()            /* Motion Magic Expo  */  
                .withKS(0.25)       // Add 0.25 V output to overcome static friction
                .withKV(.12)        // A velocity target of 1 rps results in 0.12 V output
                .withKA(0.01)       // An acceleration of 1 rps/s requires 0.01 V output
                .withKP(4.8)        // A position error of 2.5 rotations results in 12 V output
                .withKI(0)          // no output for integrated error
                .withKD(0.1)        // A velocity error of 1 rps results in 0.1 V output
        ).withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(0.0)   // Unlimited cruise velocity
                .withMotionMagicExpo_kV(0.12)          // kV is around 0.12 V/rps
                .withMotionMagicExpo_kA(0.1)          // Use a slower kA of 0.1 V/(rps/s)
            );

    /********************************************************
    * Based on the robot selected, load Intake configs with
    *   the selected robot configuration  
    ********************************************************/
    public static final TurretConfigs TurretConfigs =
        switch (Constants.getRobot()) {
            case COMPBOT -> new TurretConfigs(
                turretMotorConfigs 
            );
            case SIMBOT -> new TurretConfigs(
                turretMotorConfigs 
            );
            case DEVBOT -> new TurretConfigs(
                turretMotorConfigs 
            );
            case CAMERABOT -> new TurretConfigs( 
                turretMotorConfigs 
            );
        };


    /*****************
     * Define the Intake Config Class 
     *****************/
    public record TurretConfigs(
        TalonFXConfiguration turretMotorConfigs 
    ) {}

}
