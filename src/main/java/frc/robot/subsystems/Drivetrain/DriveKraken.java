package frc.robot.subsystems.Drivetrain;

import static frc.robot.subsystems.Drivetrain.DriveConstants.*;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;


import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;


import com.ctre.phoenix6.swerve.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;





public class DriveKraken extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements DriveIO {


    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);

    private Pigeon2 m_Pigeon2;


    private SwerveDriveState currentState;

    private DriveIOdata iOdata = new DriveIOdata();



    public DriveKraken() {
        // drivetrain constants
        
        super(TalonFX::new, TalonFX::new, CANcoder::new,
        DriveConfig.DRIVETRAIN(), 
        DriveConfig.FRONT_LEFT(), 
        DriveConfig.FRONT_RIGHT(), 
        DriveConfig.BACK_LEFT(), 
        DriveConfig.BACK_RIGHT());

        m_Pigeon2 = getPigeon2();

        this.iOdata.m_moduleLocations = getModuleLocations();
    }

    public void setCurrentLimits() { // used for setting different current limits for auton/teleop
        SwerveModule<TalonFX, TalonFX, CANcoder>[] m_modules = getModules();
        
        for (int i = 0; i < m_modules.length; i++) {
            m_modules[i].getDriveMotor().setNeutralMode(NeutralModeValue.Coast, 1.0);
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
        this.currentState = getState();

        if (currentState != null) {
            this.iOdata.state = this.currentState;

            for (int i = 0; i < 4; i++) {
                SmartDashboard.putNumberArray("Module " + i, new double[] {
                    getModule(i).getSteerMotor().getPosition().getValueAsDouble() % 1, 
                    getModule(i).getSteerMotor().getClosedLoopReference().getValueAsDouble() % 1,
                    getModule(i).getSteerMotor().getStatorCurrent().getValueAsDouble(),
                    getModule(i).getSteerMotor().getSupplyCurrent().getValueAsDouble()

                });
                    // getModule(i).getEncoder().getAbsolutePosition().getValue().in(Degrees)});
            }
        }

        this.iOdata.pigeon = m_Pigeon2.getRotation3d();

        // TODO ADD MODULE DATA
        return this.iOdata;
    }

    @Override
    public void setTeamRotation(Alliance alliance) {
        if (alliance == DriverStation.Alliance.Red) {
            this.setOperatorPerspectiveForward(RedAlliancePerspectiveRotation);
        } else {
            this.setOperatorPerspectiveForward(BlueAlliancePerspectiveRotation);
        }
    }

    @Override
    public void resetPidgeon() {
        m_Pigeon2.setYaw(0);
    }

    @Override
    public void seedFieldRelative(Pose2d seedling) {
        if (seedling != null) {
            if (currentState.Pose.getTranslation().getDistance(seedling.getTranslation()) > .5) {
                resetPose(seedling);
            }
        }
        
    }

    @Override
    public void updateVision(Pose2d calculatedPose, double timestamp, Matrix<N3, N1> stDevs) {
        addVisionMeasurement(calculatedPose, Utils.fpgaToCurrentTime(timestamp), stDevs);
    }

    @Override
    public void setOperatorPerspective(Rotation2d rotation2d) {
        setOperatorPerspectiveForward(rotation2d);
    }

    public void setNeutralMode(NeutralModeValue neutralModeValue) {
        SwerveModule<TalonFX, TalonFX, CANcoder>[] m_modules = getModules();

        for (int i = 0; i < m_modules.length; i++) {
            m_modules[i].getDriveMotor().setNeutralMode(neutralModeValue, 1.0);
        }
    }
}
