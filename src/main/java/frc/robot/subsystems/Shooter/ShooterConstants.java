package frc.robot.subsystems.Shooter;

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


public class ShooterConstants {
    
    public static int kShooterMotorID = 27;
    public static int kShooterEncoderID = 28 ;
    public static double kReduction =(1.0/1.0);
    public static double kShooterSpeed = 2000.0; 
    public static double kEjectSpeed = 3000.0;
    public static String kBusName = "sim";

    // Define configuration for Shooter Wheel Motor 
    public static final TalonFXConfiguration shooterWheelConfigs = new TalonFXConfiguration()
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
                .withPeakForwardTorqueCurrent(40)
                .withPeakReverseTorqueCurrent(-40)
        ).withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)    
                .withNeutralMode(NeutralModeValue.Coast)   
        ).withMotionMagic(
            new MotionMagicConfigs() 
                .withMotionMagicCruiseVelocity(10.0)
                .withMotionMagicAcceleration(10.0)
        ).withSlot0(
            new Slot0Configs()            /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
                .withKP(0.11)       // An error of 1 rotation per second results in 0.11 V output
                .withKI(0.0)        // No output for integrated error
                .withKD(0.0)        // No output for error derivative
                .withKS(0.1)        // To account for friction, add 0.1 V of static feedforward
                .withKV(0.12)       // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
        ).withSlot1(
            new Slot1Configs()            /* Torque-based velocity does not require a velocity feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
                .withKP(5.0)        // An error of 1 rotation per second results in 5 A output
                .withKI(0.0)        // No output for integrated error
                .withKD(0.0)        // No output for error derivative
                .withKS(2.5)        // To account for friction, add 2.5 A of static feedforward        
        ).withSlot2(                  
            new Slot2Configs()            /* Motion Magic Velocity  */  
                .withKS(0.25)       // Add 0.25 V output to overcome static friction
                .withKV(.12)        // A velocity target of 1 rps results in 0.12 V output
                .withKA(0.01)       // An acceleration of 1 rps/s requires 0.01 V output
                .withKP(4.8)        // A position error of 2.5 rotations results in 12 V output
                .withKI(0)          // no output for integrated error
                .withKD(0.1)        // A velocity error of 1 rps results in 0.1 V output
        ).withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(80.0)   // Target cruise velocity of 80 rps
                .withMotionMagicAcceleration(160)      // Target acceleration of 160 rps/s (0.5 seconds)
                .withMotionMagicJerk(1600)               // Target jerk of 1600 rps/s/s (0.1 seconds))    

            );


    /********************************************************
    * Based on the robot selected, load Intake configs with
    *   the selected robot configuration  
    ********************************************************/
    public static final ShooterConfigs ShooterConfigs =
        switch (Constants.getRobot()) {
            case COMPBOT -> new ShooterConfigs(
                shooterWheelConfigs 
            );
            case SIMBOT -> new ShooterConfigs(
                shooterWheelConfigs 
            );
            case DEVBOT -> new ShooterConfigs(
                shooterWheelConfigs 
            );
            case CAMERABOT -> new ShooterConfigs( 
                shooterWheelConfigs 
            );
        };


    /*****************
     * Define the Intake Config Class 
     *****************/
    public record ShooterConfigs(
        TalonFXConfiguration shooterWheelConfigs 
    ) {}

}
