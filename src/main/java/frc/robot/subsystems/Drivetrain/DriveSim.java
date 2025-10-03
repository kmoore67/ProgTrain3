package frc.robot.subsystems.Drivetrain;

import static frc.robot.subsystems.Drivetrain.DriveConstants.*;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;


public class DriveSim extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder>  implements DriveIO {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);

    private Pigeon2 m_Pigeon2;


    private SwerveDriveState currentState;

    private DriveIOdata iOdata = new DriveIOdata();

    
    public DriveSim() {
        super(TalonFX::new, TalonFX::new, CANcoder::new,
        DriveConfig.DRIVETRAIN(), 
        DriveConfig.FRONT_LEFT(), 
        DriveConfig.FRONT_RIGHT(), 
        DriveConfig.BACK_LEFT(), 
        DriveConfig.BACK_RIGHT());

        startSimThread();

        m_Pigeon2 = getPigeon2();

        this.iOdata.m_moduleLocations = getModuleLocations();
    }

    public void setCurrentLimits() { // used for setting different current limits for auton/teleop
        SwerveModule<TalonFX, TalonFX, CANcoder>[] m_modules = getModules();
        
        for (int i = 0; i < m_modules.length; i++) {
            m_modules[i].getDriveMotor().getConfigurator().apply(DriveConstants.driveInitialConfigs.CurrentLimits, 1.0);
            m_modules[i].getSteerMotor().getConfigurator().apply(DriveConstants.steerInitialConfigs.CurrentLimits, 1.0);
            // apply these seperatly (not in a TalonFXConfiguration obj) to ensure PID doesn't get overwritten
            m_modules[i].getDriveMotor().getConfigurator().apply(DriveConstants.driveInitialConfigs.TorqueCurrent, 1.0);
            m_modules[i].getSteerMotor().getConfigurator().apply(DriveConstants.steerInitialConfigs.TorqueCurrent, 1.0);
        }
    }

    @Override
    public void setSwerveRequest(SwerveRequest requestToApply) {
        setControl(requestToApply);
    }

    @Override
    public DriveIOdata update() {
        updateSimState(.02, 12);
        this.currentState = getState();

        if (currentState != null) {
            this.iOdata.state = this.currentState;
        }   

        this.iOdata.pigeon = m_Pigeon2.getRotation3d();

        return this.iOdata;
    }

    public void setTeamRotation(DriverStation.Alliance alliance) {
        if (alliance == DriverStation.Alliance.Red) {
            this.setOperatorPerspectiveForward(RedAlliancePerspectiveRotation);
        } else {
            this.setOperatorPerspectiveForward(BlueAlliancePerspectiveRotation);
        }
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void seedFieldRelative(Pose2d seedling) {
        resetPose(seedling);
    }

    @Override
    public void updateVision(Pose2d calculatedPose, double timestamp, Matrix<N3, N1> stDevs) {
        addVisionMeasurement(calculatedPose, timestamp, stDevs);
    }

    @Override
    public void setOperatorPerspective(Rotation2d rotation2d) {
        setOperatorPerspectiveForward(rotation2d);
    }
}
