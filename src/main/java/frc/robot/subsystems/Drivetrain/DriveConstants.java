package frc.robot.subsystems.Drivetrain;

import static edu.wpi.first.units.Units.*;

import java.util.HashMap;
import java.util.Map;


import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;

public class DriveConstants {
    // DEFAULT GAINS PULLED FROM CTRE EXAMPLE CODE
     private static final Slot0Configs steerGainsCameraBot = new Slot0Configs()
        .withKP(100).withKI(0).withKD(0.5)
        .withKS(0.1).withKV(1.91).withKA(0)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs driveGainsCameraBot = new Slot0Configs()
        .withKP(0.1).withKI(0).withKD(0)
        .withKS(0).withKV(0.124);
    
    public static final double slowModeMultiplier = 0.5;

    public static final double OTF_end_tolerance = 0.2;

    public static final double auto_align_theta_disable = .1;
    public static final double auto_align_top_speed_teleop = 2.4;
    public static final double auto_align_slow_speed_teleop = 1.0;
    public static final double auto_align_top_speed_auton = 2.4;

    public static final double auto_align_slow_in_percent = 0.66;

    public static final double auto_align_tolerance = 0.01;
    public static final double auto_align_lights_tolerance = Units.inchesToMeters(1.5);
    public static final double auto_align_command = 0.035;

    public static final double distance_safe_from_reef = 1.1;
    public static final double feild_center_line = 4.025;


    private static final PIDConstants autoPidConstantsTranslation = new PIDConstants(5, 0, 0);
    private static final PIDConstants autoPidConstantsTheta = new PIDConstants (7, 0, 0); 
    private static final PIDConstants teleopPidConstantsTheta = new PIDConstants (10, 0, 0.5); 
    // Theoretical free speed (m/s) at 12 V applied output;
    // This needs to be tuned to your individual robot
    public static final AngularVelocity kMaxAngularVelocity = RotationsPerSecond.of(2);
    public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(4.58);


    public static final TrapezoidProfile.Constraints DEFAULT_XY_CONSTRAINTS = new TrapezoidProfile.Constraints(
        1.6,
        kSpeedAt12Volts.in(MetersPerSecond) * .7);

    public static final Map<Rotation2d, Pose2d> redPoses = new HashMap<>() {
        {
            put(Rotation2d.fromDegrees(0), new Pose2d(12.23, 4.03, Rotation2d.fromDegrees(0)));
            put(Rotation2d.fromDegrees(60), new Pose2d(12.64, 3.31, Rotation2d.fromDegrees(60)));
            put(Rotation2d.fromDegrees(120), new Pose2d(13.47, 3.31, Rotation2d.fromDegrees(120)));
            put(Rotation2d.fromDegrees(180), new Pose2d(13.89, 4.03, Rotation2d.fromDegrees(180)));
            put(Rotation2d.fromDegrees(-120), new Pose2d(13.47, 4.75, Rotation2d.fromDegrees(-120)));
            put(Rotation2d.fromDegrees(-60), new Pose2d(12.64, 4.75, Rotation2d.fromDegrees(-60))); 
        }
    };

    public static final Map<Rotation2d, Pose2d> bluePoses = new HashMap<>() {
        {
            put(Rotation2d.fromDegrees(0), new Pose2d(3.66, 4.03, Rotation2d.fromDegrees(0)));
            put(Rotation2d.fromDegrees(60), new Pose2d(4.07, 3.31, Rotation2d.fromDegrees(60)));
            put(Rotation2d.fromDegrees(120), new Pose2d(4.9, 3.31, Rotation2d.fromDegrees(120)));
            put(Rotation2d.fromDegrees(180), new Pose2d(5.32, 4.03, Rotation2d.fromDegrees(180)));
            put(Rotation2d.fromDegrees(-120), new Pose2d(4.90, 4.75, Rotation2d.fromDegrees(-120)));
            put(Rotation2d.fromDegrees(-60), new Pose2d(4.07,4.75, Rotation2d.fromDegrees(-60))); 
        }
    };
    // //                                  COMP GAINS
    // public static final Map<Rotation2d, Double[]> redPoleShift = new HashMap<>() {
    //     {   //                                                  left center right
    //         put(Rotation2d.fromDegrees(0), new Double[] {6.0, -1.6, -6.0});
    //         put(Rotation2d.fromDegrees(60), new Double[] {6.0, -1.6,  -6.0});
    //         put(Rotation2d.fromDegrees(120), new Double[] {6.0, -1.6,  -6.0});
    //         put(Rotation2d.fromDegrees(180), new Double[] {6.2, -1.6, -6.5});
    //         put(Rotation2d.fromDegrees(-120), new Double[] {6.0, -1.6, -6.0});
    //         put(Rotation2d.fromDegrees(-60), new Double[] {6.0, -1.6,  -6.0});
    //     }
    // };

    // public static final Map<Rotation2d, Double[]> bluePoleShift = new HashMap<>() {
    //     {   //                                                  left center right
    //         put(Rotation2d.fromDegrees(0), new Double[] {6.0, -0.25, -6.0}); // left maybe -
    //         put(Rotation2d.fromDegrees(60), new Double[] {6.0, -0.25,  -6.5});
    //         put(Rotation2d.fromDegrees(120), new Double[] {6.0, -0.25, -6.2});
    //         put(Rotation2d.fromDegrees(180), new Double[] {6.0, -0.25,  -6.0});
    //         put(Rotation2d.fromDegrees(-120), new Double[] {6.0, -0.25, -6.3});
    //         put(Rotation2d.fromDegrees(-60), new Double[] {6.5, -0.25, -6.0});
    //     }
    // };

    public static final double robotToReefTagFace = 0.39;

    //                             GM GAINS
    public static final Map<Rotation2d, Double[]> redPoleShift = new HashMap<>() {
        {   //                                                  left center right
            put(Rotation2d.fromDegrees(0), new Double[] {6.75, -0.25, -6.0});
            put(Rotation2d.fromDegrees(60), new Double[] {8.0, -0.25,  -7.0});
            put(Rotation2d.fromDegrees(120), new Double[] {5.0, -0.25, -8.0});
            put(Rotation2d.fromDegrees(180), new Double[] {6.0, -0.25, -7.5});
            put(Rotation2d.fromDegrees(-120), new Double[] {6.5, -0.25, -7.0});
            put(Rotation2d.fromDegrees(-60), new Double[] {5.5, -1.25, -8.0});
        }
    };

    public static final Map<Rotation2d, Double[]> bluePoleShift = new HashMap<>() {
        {   //                                                  left center right
            put(Rotation2d.fromDegrees(0), new Double[] {6.0, -0.25, -7.5});
            put(Rotation2d.fromDegrees(60), new Double[] {6.25, -0.25, -8.0});
            put(Rotation2d.fromDegrees(120), new Double[] {5.5, -0.25, -8.0});
            put(Rotation2d.fromDegrees(180), new Double[] {7.0, -0.25, -6.0});
            put(Rotation2d.fromDegrees(-120), new Double[] {8.5, -0.25,  -7.0});
            put(Rotation2d.fromDegrees(-60), new Double[] {5.0, -0.25, -8.0});
        }
    };

    public static final Pose2d BLUE_REEF = new Pose2d(4.483, 4.025, Rotation2d.fromDegrees(0));
    public static final Pose2d RED_REEF = new Pose2d(13.066, 4.025, Rotation2d.fromDegrees(0));



    public static final double OFFSET_TO_RED = 8.583;
    public static final Pose2d REEF_CENTER = new Pose2d(4.483, 4.025, Rotation2d.fromDegrees(0));



    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(2250).withKI(0).withKD(20)
        .withKS(2).withKV(0).withKA(0)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
        ;
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(5.3).withKI(0).withKD(0)
        .withKS(5.5).withKV(0.3);

        // private static final Slot0Configs driveGains = new Slot0Configs()
        // .withKP(0.1).withKI(0).withKD(0)
        // .withKS(0).withKV(0.124);

    private static final Slot0Configs driveGainsVoltage = new Slot0Configs()
        .withKP(0.1).withKI(0).withKD(0)
        .withKS(0).withKV(0.124);
    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.TorqueCurrentFOC;
    private static final ClosedLoopOutputType kDriveClosedLoopOutputCambot = ClosedLoopOutputType.Voltage;


    // The type of motor used for the drive motor
    private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
    // The type of motor used for the drive motor
    private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;

    // The remote sensor feedback type to use for the steer motors;
    // When not Pro-licensed, FusedCANcoder/SyncCANcoder automatically fall back to RemoteCANcoder
    private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final Current kSlipCurrent = Amps.of(120.0);

    // Initial configs for the drive and steer motors and the azimuth encoder; these cannot be null.
    // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
    public static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(80)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(90)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLowerLimit(40)
                .withSupplyCurrentLowerTime(1)
        ).withTorqueCurrent(
            new TorqueCurrentConfigs()
                .withPeakForwardTorqueCurrent(70)
                .withPeakReverseTorqueCurrent(-70)
        );

    public static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
    .withCurrentLimits(
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(70)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(70)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLowerLimit(50)
            .withSupplyCurrentLowerTime(1)
    ).withTorqueCurrent(
        new TorqueCurrentConfigs()
            .withPeakForwardTorqueCurrent(120)
            .withPeakReverseTorqueCurrent(-120)
    );

    private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
    // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
    private static final Pigeon2Configuration pigeonConfigs = null;

    // CAN bus that the devices are located on;
    // All swerve devices must share the same CAN bus
    public static final CANBus kCANBus = new CANBus("robot", "./logs/example.hoot");

    


    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatioCompbot = 4.909090909090909;
    private static final double kCoupleRatio = 3.5714285714285716;
    private static final double kCoupleRatioCambot = 3.5714285714285716;

    private static final double kDriveGearRatioCompbot = 6.976076555023923;
    private static final double kDriveGearRatio = 5.01;
    private static final double kDriveGearRatioCambot = 6.746031746031747;

    private static final double kSteerGearRatioCompbot = 12.1;
    private static final double kSteerGearRatio = 13.3714;
    private static final double kSteerGearRatioCambot = 21.428571428571427;

    private static final Distance kWheelRadius = Inches.of(2);

    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

    private static final boolean kInvertLeftSideCambot = false;
    private static final boolean kInvertRightSideCambot = true;

    private static final int kPigeonId = 50;

    // These are only used for simulation
    private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
    private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);
    // Simulated voltage necessary to overcome friction
    private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
    private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

    public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withCANBusName(kCANBus.getName())
            .withPigeon2Id(kPigeonId)
            .withPigeon2Configs(pigeonConfigs);

    private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator =
        new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withCouplingGearRatio(kCoupleRatio)
            .withWheelRadius(kWheelRadius)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
            .withSlipCurrent(kSlipCurrent)
            .withSpeedAt12Volts(kSpeedAt12Volts)
            .withDriveMotorType(kDriveMotorType)
            .withSteerMotorType(kSteerMotorType)
            .withFeedbackSource(kSteerFeedbackType)
            .withDriveMotorInitialConfigs(driveInitialConfigs)
            .withSteerMotorInitialConfigs(steerInitialConfigs)
            .withEncoderInitialConfigs(encoderInitialConfigs)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage);

            private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreatorCompbot =
            new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                .withDriveMotorGearRatio(kDriveGearRatioCompbot)
                .withSteerMotorGearRatio(kSteerGearRatioCompbot)
                .withCouplingGearRatio(kCoupleRatioCompbot)
                .withWheelRadius(kWheelRadius)
                .withSteerMotorGains(steerGains)
                .withDriveMotorGains(driveGains)
                .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
                .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
                .withSlipCurrent(kSlipCurrent)
                .withSpeedAt12Volts(kSpeedAt12Volts)
                .withDriveMotorType(kDriveMotorType)
                .withSteerMotorType(kSteerMotorType)
                .withFeedbackSource(kSteerFeedbackType)
                .withDriveMotorInitialConfigs(driveInitialConfigs)
                .withSteerMotorInitialConfigs(steerInitialConfigs)
                .withEncoderInitialConfigs(encoderInitialConfigs)
                .withSteerInertia(kSteerInertia)
                .withDriveInertia(kDriveInertia)
                .withSteerFrictionVoltage(kSteerFrictionVoltage)
                .withDriveFrictionVoltage(kDriveFrictionVoltage);

    private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreatorCambot =
        new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorGearRatio(kDriveGearRatioCambot)
            .withSteerMotorGearRatio(kSteerGearRatioCambot)
            .withCouplingGearRatio(kCoupleRatioCambot)
            .withWheelRadius(kWheelRadius)
            .withSteerMotorGains(steerGainsCameraBot)
            .withDriveMotorGains(driveGainsVoltage)
            .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutputCambot)
            .withSlipCurrent(kSlipCurrent)
            .withSpeedAt12Volts(kSpeedAt12Volts)
            .withDriveMotorType(kDriveMotorType)
            .withSteerMotorType(kSteerMotorType)
            .withFeedbackSource(kSteerFeedbackType)
            .withDriveMotorInitialConfigs(driveInitialConfigs)
            .withSteerMotorInitialConfigs(steerInitialConfigs)
            .withEncoderInitialConfigs(encoderInitialConfigs)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage);


    // Front Left
    private static final int kFrontLeftDriveMotorId = 8;
    private static final int kFrontLeftSteerMotorId = 7;
    private static final int kFrontLeftEncoderId = 41;
    private static final Angle kFrontLeftEncoderOffset = Rotations.of(-0.12255859375);
    private static final boolean kFrontLeftSteerMotorInverted = true;
    private static final boolean kFrontLeftEncoderInverted = false;

    private static final Distance kFrontLeftXPos = Inches.of(11);
    private static final Distance kFrontLeftYPos = Inches.of(11);

    // Front Right
    private static final int kFrontRightDriveMotorId = 6;
    private static final int kFrontRightSteerMotorId = 5;
    private static final int kFrontRightEncoderId = 40;
    private static final Angle kFrontRightEncoderOffset = Rotations.of(-0.442138671875);
    private static final boolean kFrontRightSteerMotorInverted = true;
    private static final boolean kFrontRightEncoderInverted = false;

    private static final Distance kFrontRightXPos = Inches.of(11);
    private static final Distance kFrontRightYPos = Inches.of(-11);

    // Back Left
    private static final int kBackLeftDriveMotorId = 4;
    private static final int kBackLeftSteerMotorId = 3;
    private static final int kBackLeftEncoderId = 43;
    private static final Angle kBackLeftEncoderOffset = Rotations.of(-0.17626953125);
    private static final boolean kBackLeftSteerMotorInverted = true;
    private static final boolean kBackLeftEncoderInverted = false;

    private static final Distance kBackLeftXPos = Inches.of(-11);
    private static final Distance kBackLeftYPos = Inches.of(11);

    // Back Right
    private static final int kBackRightDriveMotorId = 2;
    private static final int kBackRightSteerMotorId = 1;
    private static final int kBackRightEncoderId = 42;
    private static final Angle kBackRightEncoderOffset = Rotations.of(0.443359375);
    private static final boolean kBackRightSteerMotorInverted = true;
    private static final boolean kBackRightEncoderInverted = false;

    private static final Distance kBackRightXPos = Inches.of(-11);
    private static final Distance kBackRightYPos = Inches.of(-11);
    


    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft =
        ConstantCreator.createModuleConstants(
            kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset,
            kFrontLeftXPos, kFrontLeftYPos, kInvertLeftSide, kFrontLeftSteerMotorInverted, kFrontLeftEncoderInverted
        ).withDriveMotorInverted(false);
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight =
        ConstantCreator.createModuleConstants(
            kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset,
            kFrontRightXPos, kFrontRightYPos, kInvertRightSide, kFrontRightSteerMotorInverted, kFrontRightEncoderInverted
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft =
        ConstantCreator.createModuleConstants(
            kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset,
            kBackLeftXPos, kBackLeftYPos, kInvertLeftSide, kBackLeftSteerMotorInverted, kBackLeftEncoderInverted
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight =
        ConstantCreator.createModuleConstants(
            kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset,
            kBackRightXPos, kBackRightYPos, kInvertRightSide, kBackRightSteerMotorInverted, kBackRightEncoderInverted
        );



    // Front Left
    private static final int kFrontLeftDriveMotorIdCambot = 6;
    private static final int kFrontLeftSteerMotorIdCambot = 8;
    private static final int kFrontLeftEncoderIdCambot = 18;
    private static final Angle kFrontLeftEncoderOffsetCambot = Rotations.of(0.41552734375);
    private static final boolean kFrontLeftSteerMotorInvertedCambot = false;
    private static final boolean kFrontLeftEncoderInvertedCambot = false;

    private static final Distance kFrontLeftXPosCambot = Inches.of(8.75);
    private static final Distance kFrontLeftYPosCambot = Inches.of(8.75);

    // Front Right
    private static final int kFrontRightDriveMotorIdCambot = 2;
    private static final int kFrontRightSteerMotorIdCambot = 4;
    private static final int kFrontRightEncoderIdCambot = 14;
    private static final Angle kFrontRightEncoderOffsetCambot = Rotations.of(-0.361572265625);
    private static final boolean kFrontRightSteerMotorInvertedCambot = false;
    private static final boolean kFrontRightEncoderInvertedCambot = false;

    private static final Distance kFrontRightXPosCambot = Inches.of(8.75);
    private static final Distance kFrontRightYPosCambot = Inches.of(-8.75);

    // Back Left
    private static final int kBackLeftDriveMotorIdCambot = 5;
    private static final int kBackLeftSteerMotorIdCambot = 7;
    private static final int kBackLeftEncoderIdCambot = 17;
    private static final Angle kBackLeftEncoderOffsetCambot = Rotations.of(-0.162841796875);
    private static final boolean kBackLeftSteerMotorInvertedCambot = false;
    private static final boolean kBackLeftEncoderInvertedCambot = false;

    private static final Distance kBackLeftXPosCambot = Inches.of(-8.75);
    private static final Distance kBackLeftYPosCambot = Inches.of(8.75);

    // Back Right
    private static final int kBackRightDriveMotorIdCambot = 1;
    private static final int kBackRightSteerMotorIdCambot = 3;
    private static final int kBackRightEncoderIdCambot = 13;
    private static final Angle kBackRightEncoderOffsetCambot = Rotations.of(0.43896484375);
    private static final boolean kBackRightSteerMotorInvertedCambot = false;
    private static final boolean kBackRightEncoderInvertedCambot = false; 

    private static final Distance kBackRightXPosCambot = Inches.of(-8.75);
    private static final Distance kBackRightYPosCambot = Inches.of(-8.75);
    



    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeftCambot =
        ConstantCreatorCambot.createModuleConstants(
            kFrontLeftSteerMotorIdCambot, kFrontLeftDriveMotorIdCambot, kFrontLeftEncoderIdCambot, kFrontLeftEncoderOffsetCambot,
            kFrontLeftXPosCambot, kFrontLeftYPosCambot, kInvertLeftSideCambot, kFrontLeftSteerMotorInvertedCambot, kFrontLeftEncoderInvertedCambot
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRightCambot =
        ConstantCreatorCambot.createModuleConstants(
            kFrontRightSteerMotorIdCambot, kFrontRightDriveMotorIdCambot, kFrontRightEncoderIdCambot, kFrontRightEncoderOffsetCambot,
            kFrontRightXPosCambot, kFrontRightYPosCambot, kInvertRightSideCambot, kFrontRightSteerMotorInvertedCambot, kFrontRightEncoderInvertedCambot
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeftCambot =
        ConstantCreatorCambot.createModuleConstants(
            kBackLeftSteerMotorIdCambot, kBackLeftDriveMotorIdCambot, kBackLeftEncoderIdCambot, kBackLeftEncoderOffsetCambot,
            kBackLeftXPosCambot, kBackLeftYPosCambot, kInvertLeftSideCambot, kBackLeftSteerMotorInvertedCambot, kBackLeftEncoderInvertedCambot
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRightCambot =
        ConstantCreatorCambot.createModuleConstants(
            kBackRightSteerMotorIdCambot, kBackRightDriveMotorIdCambot, kBackRightEncoderIdCambot, kBackRightEncoderOffsetCambot,
            kBackRightXPosCambot, kBackRightYPosCambot, kInvertRightSideCambot, kBackRightSteerMotorInvertedCambot, kBackRightEncoderInvertedCambot
        );



        
    // Front Left
    private static final int kFrontLeftDriveMotorIdCompbot = 8;
    private static final int kFrontLeftSteerMotorIdCompbot = 7;
    private static final int kFrontLeftEncoderIdCompbot = 41;
    private static final Angle kFrontLeftEncoderOffsetCompbot = Rotations.of(-0.267822);
    private static final boolean kFrontLeftSteerMotorInvertedCompbot = true;
    private static final boolean kFrontLeftEncoderInvertedCompbot = false;

    private static final Distance kFrontLeftXPosCompbot = Inches.of(11.5);
    private static final Distance kFrontLeftYPosCompbot = Inches.of(11.5);

    // Front Right
    private static final int kFrontRightDriveMotorIdCompbot = 6;
    private static final int kFrontRightSteerMotorIdCompbot = 5;
    private static final int kFrontRightEncoderIdCompbot = 40;
    private static final Angle kFrontRightEncoderOffsetCompbot = Rotations.of(0.3642578125);
    private static final boolean kFrontRightSteerMotorInvertedCompbot = true;
    private static final boolean kFrontRightEncoderInvertedCompbot = false;

    private static final Distance kFrontRightXPosCompbot = Inches.of(11.5);
    private static final Distance kFrontRightYPosCompbot = Inches.of(-11.5);

    // Back Left
    private static final int kBackLeftDriveMotorIdCompbot = 4;
    private static final int kBackLeftSteerMotorIdCompbot = 3;
    private static final int kBackLeftEncoderIdCompbot = 43;
    private static final Angle kBackLeftEncoderOffsetCompbot = Rotations.of(-0.1357421875);
    private static final boolean kBackLeftSteerMotorInvertedCompbot = true;
    private static final boolean kBackLeftEncoderInvertedCompbot = false;

    private static final Distance kBackLeftXPosCompbot = Inches.of(-11.5);
    private static final Distance kBackLeftYPosCompbot = Inches.of(11.5);

    // Back Right
    private static final int kBackRightDriveMotorIdCompbot = 2;
    private static final int kBackRightSteerMotorIdCompbot = 1;
    private static final int kBackRightEncoderIdCompbot = 42;
    private static final Angle kBackRightEncoderOffsetCompbot = Rotations.of(-0.0908203125);
    private static final boolean kBackRightSteerMotorInvertedCompbot = true;
    private static final boolean kBackRightEncoderInvertedCompbot = false;

    private static final Distance kBackRightXPosCompbot = Inches.of(-11.5);
    private static final Distance kBackRightYPosCompbot = Inches.of(-11.5);


    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeftCompbot =
        ConstantCreatorCompbot.createModuleConstants(
            kFrontLeftSteerMotorIdCompbot, kFrontLeftDriveMotorIdCompbot, kFrontLeftEncoderIdCompbot, kFrontLeftEncoderOffsetCompbot,
            kFrontLeftXPosCompbot, kFrontLeftYPosCompbot, kInvertLeftSide, kFrontLeftSteerMotorInvertedCompbot, kFrontLeftEncoderInvertedCompbot
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRightCompbot =
        ConstantCreatorCompbot.createModuleConstants(
            kFrontRightSteerMotorIdCompbot, kFrontRightDriveMotorIdCompbot, kFrontRightEncoderIdCompbot, kFrontRightEncoderOffsetCompbot,
            kFrontRightXPosCompbot, kFrontRightYPosCompbot, kInvertRightSide, kFrontRightSteerMotorInvertedCompbot, kFrontRightEncoderInvertedCompbot
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeftCompbot =
        ConstantCreatorCompbot.createModuleConstants(
            kBackLeftSteerMotorIdCompbot, kBackLeftDriveMotorIdCompbot, kBackLeftEncoderIdCompbot, kBackLeftEncoderOffsetCompbot,
            kBackLeftXPosCompbot, kBackLeftYPosCompbot, kInvertLeftSide, kBackLeftSteerMotorInvertedCompbot, kBackLeftEncoderInvertedCompbot
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRightCompbot =
        ConstantCreatorCompbot.createModuleConstants(
            kBackRightSteerMotorIdCompbot, kBackRightDriveMotorIdCompbot, kBackRightEncoderIdCompbot, kBackRightEncoderOffsetCompbot,
            kBackRightXPosCompbot, kBackRightYPosCompbot, kInvertRightSide, kBackRightSteerMotorInvertedCompbot, kBackRightEncoderInvertedCompbot
        );


        public static final DriveConfig DriveConfig =
        switch (Constants.getRobot()) {
            case COMPBOT -> new DriveConfig(
                FrontLeftCompbot, 
                FrontRightCompbot, 
                BackLeftCompbot, 
                BackRightCompbot,
                DrivetrainConstants,
                kSpeedAt12Volts.in(MetersPerSecond),
                kMaxAngularVelocity.in(RadiansPerSecond),
                driveInitialConfigs.CurrentLimits,
                steerInitialConfigs.CurrentLimits,
                autoPidConstantsTranslation,
                autoPidConstantsTheta,
                teleopPidConstantsTheta
                );
            case DEVBOT -> new DriveConfig(
                FrontLeft, 
                FrontRight, 
                BackLeft, 
                BackRight,
                DrivetrainConstants,
                kSpeedAt12Volts.in(MetersPerSecond),
                kMaxAngularVelocity.in(RadiansPerSecond),
                driveInitialConfigs.CurrentLimits,
                steerInitialConfigs.CurrentLimits,
                autoPidConstantsTranslation,
                autoPidConstantsTheta,
                teleopPidConstantsTheta
                );
            case CAMERABOT -> new DriveConfig(
                    FrontLeftCambot, 
                    FrontRightCambot, 
                    BackLeftCambot, 
                    BackRightCambot,
                    DrivetrainConstants,
                    kSpeedAt12Volts.in(MetersPerSecond),
                    kMaxAngularVelocity.in(RadiansPerSecond),
                    driveInitialConfigs.CurrentLimits,
                    steerInitialConfigs.CurrentLimits,
                    autoPidConstantsTranslation,
                    autoPidConstantsTheta,
                    teleopPidConstantsTheta
                    );
            case SIMBOT -> new DriveConfig(
                FrontLeftCambot, 
                FrontRightCambot, 
                BackLeftCambot, 
                BackRightCambot,
                DrivetrainConstants,
                kSpeedAt12Volts.in(MetersPerSecond),
                kMaxAngularVelocity.in(RadiansPerSecond),
                driveInitialConfigs.CurrentLimits,
                steerInitialConfigs.CurrentLimits,
                autoPidConstantsTranslation,
                autoPidConstantsTheta,
                teleopPidConstantsTheta
                );
        };


    public record DriveConfig(
        SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FRONT_LEFT, 
        SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FRONT_RIGHT, 
        SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BACK_LEFT, 
        SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BACK_RIGHT,
        SwerveDrivetrainConstants DRIVETRAIN,
        double MAX_VELOCITY,
        double MAX_ANGULAR_VELOCITY,
        CurrentLimitsConfigs DRIVE_CURRENT,
        CurrentLimitsConfigs AZIMUTH_CURRENT,
        PIDConstants AUTON_TRANSLATION_PID,
        PIDConstants AUTON_ROTATION_PID,
        PIDConstants TELEOP_ROTATION_PID
        ) {}

        //Comp Gains:
        // public static final double bargePosRedFar = 9.925;
        // public static final double bargePosRedClose = 9.75;
        // public static final double bargePosBlueFar = 7.86;
        // public static final double bargePosBlueClose = 7.685;

        //GM Gains:
        public static final double bargePosRedFar = 10.0;
        public static final double bargePosRedClose = 9.75;
        public static final double bargePosBlueFar = 7.86;
        public static final double bargePosBlueClose = 7.61;


}
