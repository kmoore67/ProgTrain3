package frc.robot.subsystems.Drivetrain;

import static frc.robot.subsystems.Drivetrain.DriveConstants.*;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.IntStream;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drivetrain.DriveIO.DriveIOdata;



public class Drive extends SubsystemBase {
    private DriveIO driveIO;
    private DriveIOdata iOdata;

    private ShuffleboardTab driveTab;
    private GenericEntry speedEntry;
    private GenericEntry poseEntry;
    private GenericEntry pathXEntry;
    private GenericEntry pathYEntry;
    private GenericEntry pathRotEntry;

    private Rotation2d heading;

    private PathConstraints constraints;
    private List<Waypoint> waypoints;

    public Pose3d tagTransform;
    public boolean seesReefTag;
    public int reefTagID;


    private PathPlannerPath path;
    private Pose2d reefTarget;

    private Alert alert;

    double[][] corals = {
    {320, -1, 321, -1},
    {320, -1, 321, -1},
    {320, -1, 321, -1},
    {320, -1, 321, -1},
    {320, -1, 321, -1},
    {320, -1, 321, -1},
    {320, -1, 321, -1},
    {320, -1, 321, -1},
    {320, -1, 321, -1},
    {320, -1, 321, -1}}; //x1 y1 x2 y2, 1 top left  2 bottom right

    double[] framesLost = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    double frames = 0;

    double pixelX;   // 640 x 480, origin top left
    double pixelY;
    double pixelYmax;

    int bestCoral = 0;

    double targetXPixel = 320.5;
    double targetYPixel = 430;
    double pixelTolerance = 8;

    double chaseVelocity;

    double sideRatio; //.4 long side, 1.2 short side h/w

    Pose2d poseFieldToCoral;

    NetworkTable objectDetection;
    ArrayList<DoubleArraySubscriber> coralSubs = new ArrayList<>();

    private PIDController thetaController = new PIDController(10, 0, 0.2);
    private PIDController translationControllerIn = new PIDController(5, 0, 0);
    private PIDController translationControllerAcross = new PIDController(5, 0, 0);
    private ProfiledPIDController translationControllerY = new ProfiledPIDController(5, 0, 0, DEFAULT_XY_CONSTRAINTS);
    private ProfiledPIDController translationControllerX = new ProfiledPIDController(5, 0, 0, DEFAULT_XY_CONSTRAINTS);
    private PIDController thetaChaseObjectPID = new PIDController(0.0075, 0, 0);

    private final SwerveRequest.SwerveDriveBrake BRAKE = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.FieldCentric FIELD_CENTRIC = new SwerveRequest.FieldCentric()
    .withDeadband(0.2).withRotationalDeadband(0.0)
    .withDriveRequestType(DriveRequestType.Velocity).withDesaturateWheelSpeeds(false);

    private final SwerveRequest.FieldCentric AUTO_ALIGN = new SwerveRequest.FieldCentric()
    .withDeadband(0.0).withRotationalDeadband(0.0);
    private final SwerveRequest.RobotCentric ROBOT_CENTRIC = new SwerveRequest.RobotCentric();
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds().withDriveRequestType(DriveRequestType.Velocity);

    private int currentPathIndex;
    private List<PathPlannerPath> pathGroup;
    private Pose2d currentTarget;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */

    private Pose2d autoStartPose;
    List<Waypoint> chaseWayPoints = PathPlannerPath.waypointsFromPoses(
        new Pose2d(0,0,Rotation2d.fromDegrees(0)),
        new  Pose2d(0,0,Rotation2d.fromDegrees(0)) // pos through object  
    );
    PathConstraints constraintsFetch = new PathConstraints(4.96, 6.0, 540.0, 720.0, 12.0);
    private PathPlannerPath fetchPath = new PathPlannerPath(
        chaseWayPoints ,
        constraintsFetch,
        null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
        new GoalEndState(1.5, Rotation2d.fromDegrees(0))); // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
    private BooleanSubscriber targetSeenSub;

    private double coralRecalcDistance = 0.0;


    public Drive(DriveIO driveIO) { 
        this.driveIO = driveIO;
        this.iOdata = driveIO.update();

        translationControllerX.setTolerance(auto_align_tolerance);
        translationControllerY.setTolerance(auto_align_tolerance);

        translationControllerIn.setTolerance(auto_align_tolerance);
        translationControllerAcross.setTolerance(auto_align_tolerance);

        heading = Rotation2d.fromDegrees(0);

        driveTab = Shuffleboard.getTab("Drive");
        speedEntry = driveTab.add("Speed (RJU)", 0.0).getEntry();
        poseEntry = driveTab.add("Pose", new Double[] {0.0, 0.0, 0.0}).getEntry();
        pathXEntry = driveTab.add("Path X", 0.0).getEntry();
        pathYEntry = driveTab.add("Path Y", 0.0).getEntry();
        pathRotEntry = driveTab.add("Path Rot", 0.0).getEntry();

        objectDetection = NetworkTableInstance.getDefault().getTable("ObjectDetection");
        for (int i=0; i<10; ++i) {
            coralSubs.add(objectDetection.getDoubleArrayTopic("coral").subscribe(new double[] {320, -1, 321, -1}));
        }
        targetSeenSub = objectDetection.getBooleanTopic("detected").subscribe(false);

        double driveBaseRadius = 0;
        for (var moduleLocation : this.iOdata.m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        pathGroup = new ArrayList<PathPlannerPath>();

        //configurePathPlanner();

        constraints = PathConstraints.unlimitedConstraints(12.0);
        reefTarget = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

        try {
            this.pathGroup.addAll(PathPlannerAuto.getPathGroupFromAutoFile("OTF_TESTING"));
        } catch (IOException e) {
        } catch (ParseException e) {
        }

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(13.0, 1.0, Rotation2d.fromDegrees(0)),
            new Pose2d(16.0, 1.0, Rotation2d.fromDegrees(0)),
            new Pose2d(15.0, 3.0, Rotation2d.fromDegrees(90))
        );

        PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
        // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage

        // Create the path using the waypoints created above
        path = new PathPlannerPath(
                waypoints,
                constraints,
                null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
                new GoalEndState(0.0, Rotation2d.fromDegrees(-90)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );

        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping = true;

        alert = new Alert("test alert", AlertType.kWarning);
    }
            
    public boolean drivetrainAtTarget() {
        if (path != null) {
            
            return Math.abs(
                this.iOdata.state.Pose.getTranslation().getDistance(
                    path.getPathPoses().get(path.getPathPoses().size() - 1).getTranslation())) 
                    < OTF_end_tolerance;
        }
        return false;
    }

    private boolean seesReefTag() {
        return seesReefTag;
    }

    private Rotation2d getNearestReefAngle() {
        return Rotation2d.fromDegrees(60*Math.round(Math.toDegrees(Math.atan2(REEF_CENTER.getY() - iOdata.state.Pose.getY(), (DriverStation.getAlliance().get() == Alliance.Blue ? REEF_CENTER.getX(): REEF_CENTER.getX() + OFFSET_TO_RED) - iOdata.state.Pose.getX()))/60));
    }
            
    public Command generateOnTheFly() {
        return runOnce(() -> {
            List<Pose2d> currentPathPoses = pathGroup.get(currentPathIndex).getPathPoses();
            PathPlannerPath nextPath = pathGroup.get(currentPathIndex + 1);
            waypoints = PathPlannerPath.waypointsFromPoses(
                currentPathPoses.get(currentPathPoses.size() - 1),
                new Pose2d(pathXEntry.getDouble(0), pathYEntry.getDouble(0), Rotation2d.fromDegrees(pathRotEntry.getDouble(0))),
    nextPath.getPathPoses().get(0)
            );
            path = new PathPlannerPath(
            waypoints,
            constraints,
            null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
            new GoalEndState(nextPath.getIdealStartingState().velocityMPS(), nextPath.getInitialHeading()) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
            );
            AutoBuilder.followPath(path).unless(this::drivetrainAtTarget).schedule();
        });
    }

    // public Command runOnTheFly() {
    //     return Commands.sequence(
    //         generateOnTheFly(),
    //         AutoBuilder.followPath(path).unless(this::drivetrainAtTarget));
    // }

    private void getObjectMeasurements() {
        // double cameraToCoral = ((pixelX-320) / 320.0) * 35.0 * Math.PI/180.0;
        // double distance = 1.5;
        
        // Pose2d poseRobotToCoral = new Pose2d(distance * Math.sin(cameraToCoral),distance * Math.cos(cameraToCoral)+0.4572,Rotation2d.fromDegrees(0));

        // Pose2d poseFieldToCoral = new Pose2d(iOdata.state.Pose.getX() + (poseRobotToCoral.getY() * iOdata.state.Pose.getRotation().getCos()) + (poseRobotToCoral.getX() *  iOdata.state.Pose.getRotation().getSin()), iOdata.state.Pose.getY() + (poseRobotToCoral.getY() * iOdata.state.Pose.getRotation().getSin()) + (poseRobotToCoral.getX() * iOdata.state.Pose.getRotation().getCos()), Rotation2d.fromDegrees(0));

        // Rotation2d angleFieldToCoral = Rotation2d.fromDegrees(-90).minus(Rotation2d.fromRadians(Math.atan2(iOdata.state.Pose.getX() - poseFieldToCoral.getX(), iOdata.state.Pose.getY() - poseFieldToCoral.getY())));

        // SmartDashboard.putNumberArray("coral Pose",  new double[] {poseFieldToCoral.getX(), poseFieldToCoral.getY(), angleFieldToCoral.getRadians()});
        // SmartDashboard.putNumber("poseCameraToCoral", poseRobotToCoral.getRotation().getDegrees());
        // SmartDashboard.putNumber("poseCameraToCoral_X", poseRobotToCoral.getX());
        // SmartDashboard.putNumber("poseCameraToCoral_Y", poseRobotToCoral.getY());
        // SmartDashboard.putNumber("target Angle", angleRobotToCoral.getDegrees());
        // SmartDashboard.putNumber("CoralX", iOdata.state.Pose.getX() + distance * angleRobotToCoral.getCos());
        // SmartDashboard.putNumber("CoralY", iOdata.state.Pose.getY() + distance * angleRobotToCoral.getSin());
        // SmartDashboard.putNumber("startingPoseX", iOdata.state.Pose.getX());
        // SmartDashboard.putNumber("startingPoseY", iOdata.state.Pose.getY());
        // SmartDashboard.putNumber("startingPoseTheta", iOdata.state.Pose.getRotation().getDegrees());
        // SmartDashboard.putNumber("camera to coral", cameraToCoral);



        for (int i=0; i<10; ++i) {

            if (corals[i].equals(coralSubs.get(i).get())) {
                ++framesLost[i];
            } else {
                framesLost[i] = 0;
            }

            if (framesLost[i] < 15) {
                corals[i] = coralSubs.get(i).get();

                double newX =(corals[i][0] + corals[i][2]) / 2;
                double newY = corals[i][3];
                double bestX = (corals[bestCoral][0] + corals[bestCoral][2]) / 2;
                double bestY = corals[bestCoral][3];

                if (newY > bestY - 15) {  //close or better y
                    if (Math.abs(newX - targetXPixel) < Math.abs(bestX - targetXPixel) + 15) {  //close or better x
                        bestCoral = i;
                    }
                }
            }
        }


        pixelYmax = corals[bestCoral][3];

        pixelX = (corals[bestCoral][0] + corals[bestCoral][2]) / 2; // xmin + xmax
        pixelY = (corals[bestCoral][1] + pixelYmax) / 2;  //ymin + ymax

        // SmartDashboard.putNumber("chase object/pixel error", Math.abs(pixelX - targetXPixel));
        SmartDashboard.putNumber("pixelX", pixelX);
        SmartDashboard.putNumber("pixelY", pixelY);
        SmartDashboard.putNumberArray("chase object/frames lost", framesLost);
    }

    public boolean alignedToObject() {
        return Math.abs(pixelX - targetXPixel) < pixelTolerance;
    }

    public boolean noObjectsSeen() {
        if (corals[0].equals(coralSubs.get(0).get())) {
            return false;
        } else {
            return true;
        }
    }

    public boolean objectClose() {
        return pixelYmax > targetYPixel;
    }

    
    public void calculateCoralPose(double distance) {
        
        double cameraToCoral = -((pixelX-320) / 320.0) * 35.0 * Math.PI/180.0;
        distance -= .4;
        
        Pose2d translationRtoC = new Pose2d(distance * Math.sin(-cameraToCoral) - .1, distance * Math.cos(-cameraToCoral)+0.4572,Rotation2d.fromDegrees(0));

        Transform2d transform = new Transform2d(translationRtoC.getTranslation(), translationRtoC.getRotation());

        Pose2d poseFieldToCoralNoRotation = iOdata.state.Pose.transformBy(transform);

        Rotation2d angleFieldToCoral = Rotation2d.fromDegrees(270).minus(Rotation2d.fromRadians(Math.atan2(iOdata.state.Pose.getX() - poseFieldToCoralNoRotation.getX(), iOdata.state.Pose.getY() - poseFieldToCoralNoRotation.getY())));
        poseFieldToCoral = new Pose2d(poseFieldToCoralNoRotation.getTranslation(), angleFieldToCoral);


        SmartDashboard.putNumberArray("coral Pose no rotation",  new double[] {poseFieldToCoralNoRotation.getX(), poseFieldToCoralNoRotation.getY(), poseFieldToCoralNoRotation.getRotation().getRadians()});
        SmartDashboard.putNumberArray("coral Pose",  new double[] {poseFieldToCoral.getX(), poseFieldToCoral.getY(), poseFieldToCoral.getRotation().getRadians()});
        SmartDashboard.putNumberArray("start Pose",  new double[] {iOdata.state.Pose.getX(), iOdata.state.Pose.getY(), angleFieldToCoral.getRadians()});
        SmartDashboard.putNumberArray("CtoR", new Double[] {translationRtoC.getX(),translationRtoC.getY()});

        SmartDashboard.putNumber("cameraToCoral", Math.toDegrees(cameraToCoral));
    }

    public void recalculateCoralPose() {
        double cameraToCoral = -((pixelX-320) / 320.0) * 35.0 * Math.PI/180.0;

        double distance = Math.abs(coralRecalcDistance);
        
        Pose2d translationRtoC = new Pose2d(distance * Math.sin(-cameraToCoral) - .1, distance * Math.cos(-cameraToCoral),Rotation2d.fromDegrees(0));

        Transform2d transform = new Transform2d(translationRtoC.getTranslation(), translationRtoC.getRotation());

        Pose2d poseFieldToCoralNoRotation = iOdata.state.Pose.transformBy(transform);
        SmartDashboard.putNumberArray("coral Recalculated Pose",  new double[] {poseFieldToCoralNoRotation.getX(), poseFieldToCoralNoRotation.getY(), poseFieldToCoralNoRotation.getRotation().getRadians()});
        Rotation2d angleFieldToCoral = Rotation2d.fromDegrees(270).minus(Rotation2d.fromRadians(Math.atan2(iOdata.state.Pose.getX() - poseFieldToCoralNoRotation.getX(), iOdata.state.Pose.getY() - poseFieldToCoralNoRotation.getY())));
        if (this.poseFieldToCoral.getTranslation().getDistance(poseFieldToCoralNoRotation.getTranslation()) > .1 && distance > .5 && this.poseFieldToCoral.getTranslation().getDistance(poseFieldToCoralNoRotation.getTranslation()) < .35) {
            poseFieldToCoral = new Pose2d(poseFieldToCoralNoRotation.getTranslation(), angleFieldToCoral);
            SmartDashboard.putNumberArray("coral Pose",  new double[] {poseFieldToCoral.getX(), poseFieldToCoral.getY(), poseFieldToCoral.getRotation().getRadians()});

        }
    }

    public void driveToCoral(double slowDownDistance) {
        // if (targetSeenSub.get()) {
        //     calculateCoralPose();
        // }

        Pose2d Error = poseFieldToCoral.relativeTo(new Pose2d(iOdata.state.Pose.getTranslation(), poseFieldToCoral.getRotation()));
        SmartDashboard.putNumberArray("Drive Error", new double[] {Error.getX(), Error.getY(), Error.getRotation().getRadians()});

        Translation2d speeds = new Translation2d(iOdata.state.Speeds.vyMetersPerSecond, iOdata.state.Speeds.vxMetersPerSecond);
        speeds.rotateBy(poseFieldToCoral.getRotation());
        SmartDashboard.putNumberArray("Relative Speeds", new double[] {speeds.getX(), -speeds.getY(), 0.0});

        translationControllerIn.calculate(Error.getX(), 0.0);
        translationControllerAcross.calculate(Error.getY(), 0.0);

        coralRecalcDistance = Error.getX();

        double in = -translationControllerIn.calculate(Error.getX(), 0.0);
        double across = translationControllerAcross.calculate(Error.getY(), 0.0);
        double cosine = Math.cos(poseFieldToCoral.getRotation().getRadians());
        double sine = Math.sin(poseFieldToCoral.getRotation().getRadians());

        if (iOdata.state.Pose.getTranslation().getDistance(poseFieldToCoral.getTranslation()) < slowDownDistance) {
            in = MathUtil.clamp(in, -auto_align_slow_speed_teleop, auto_align_slow_speed_teleop);
            across =  MathUtil.clamp(across, -auto_align_slow_speed_teleop, auto_align_slow_speed_teleop);
        }
        

        if (DriverStation.getAlliance().get() == Alliance.Blue) 
            driveIO.setSwerveRequest(AUTO_ALIGN
                .withVelocityX(((in * cosine) + (across * sine)))
                .withVelocityY(((across * -cosine) + (in * sine)))
                .withRotationalRate(thetaController.calculate(
            iOdata.state.Pose.getRotation().getRadians(), 
            poseFieldToCoral.getRotation().getRadians() - Math.toRadians(90)
        )));
        else 
            driveIO.setSwerveRequest(AUTO_ALIGN
                .withVelocityX(-((in * cosine) + (across * sine)))
                .withVelocityY(-((across * -cosine) + (in * sine)))
                .withRotationalRate(thetaController.calculate(
            iOdata.state.Pose.getRotation().getRadians(), 
            poseFieldToCoral.getRotation().getRadians() - Math.toRadians(90)
        )));

        // double x = translationControllerFetchX.calculate(iOdata.state.Pose.getX(), poseFieldToCoral.getX());
        // double y = translationControllerFetchY.calculate(iOdata.state.Pose.getY(), poseFieldToCoral.getY());

        

        
        

        // driveIO.setSwerveRequest(AUTO_ALIGN
        // .withVelocityX(x)
        // .withVelocityY(y)
        // .withRotationalRate(thetaController.calculate(
        //     iOdata.state.Pose.getRotation().getRadians(), 
        //     poseFieldToCoral.getRotation().getRadians() - Math.toRadians(90)
        // )));
    }

    public boolean atCoral() {
        if (poseFieldToCoral != null) {
            return iOdata.state.Pose.getTranslation().getDistance(poseFieldToCoral.getTranslation()) < 0.1;
        } else {
            return false;
        }
    }

    public Command fetchAuto(double distance, double slowDistance) {
        return new FunctionalCommand(
        () -> calculateCoralPose(distance),
        () -> {
            driveToCoral(slowDistance);
            // if (targetSeenSub.get()) {
            //     recalculateCoralPose();
            // }

        },
        (interrupted) -> {},
        () -> atCoral(), this)
        .onlyIf(() -> targetSeenSub.get());
    }

    public void chaseSlow() {
        pixelTolerance = 20;
        driveIO.setSwerveRequest(ROBOT_CENTRIC
        .withRotationalRate(alignedToObject() ? 0 : thetaChaseObjectPID.calculate(pixelX, targetXPixel)* 0.5)
        .withVelocityY(1.75) //1.5
        );
    }

    public void alignObjectTeleop(double driveX, double driveY, double driveTheta) {

        if (framesLost[bestCoral] < 5) {
        driveIO.setSwerveRequest(ROBOT_CENTRIC
        .withVelocityX((driveX <= 0 ? -(driveX * driveX) : (driveX * driveX)) * DriveConfig.MAX_VELOCITY())
        .withVelocityY((driveY <= 0 ? -(driveY * driveY) : (driveY * driveY)) * DriveConfig.MAX_VELOCITY())
        .withRotationalRate(alignedToObject() ? 0 : thetaChaseObjectPID.calculate(pixelX, targetXPixel))
        );
        } else {
            driveIO.setSwerveRequest(FIELD_CENTRIC
            .withVelocityX((driveX <= 0 ? -(driveX * driveX) : (driveX * driveX)) * DriveConfig.MAX_VELOCITY())
            .withVelocityY((driveY <= 0 ? -(driveY * driveY) : (driveY * driveY)) * DriveConfig.MAX_VELOCITY())
            .withRotationalRate((driveTheta <= 0 ? -(driveTheta * driveTheta) : (driveTheta * driveTheta)) * DriveConfig.MAX_ANGULAR_VELOCITY())
            );
        }
    }

    // public void alignReef(int leftRight) {
    //     double leftRightFromTable = Units.inchesToMeters(DriverStation.getAlliance().get() == Alliance.Blue ? blueShift.get(reefTagID)[leftRight] : redShift.get(reefTagID)[leftRight]);
    //     driveIO.setSwerveRequest(ROBOT_CENTRIC
    //         .withVelocityX(-(-leftRightFromTable - tagTransform.getX()) * 5)
    //         .withVelocityY((-0.44 - tagTransform.getY()) * 7.5)
    //         // .withRotationalRate()
    //     );
    // }

    public void updateReefTarget(int leftRight) {
        Alliance curAlliance = DriverStation.getAlliance().get();

        heading = getNearestReefAngle();
        heading = heading.getDegrees() == -180.0 ? Rotation2d.fromDegrees(180.0) : heading;

        reefTarget = curAlliance == Alliance.Blue ? bluePoses.get(heading) : redPoses.get(heading);

        double poleShift = curAlliance == Alliance.Blue ? bluePoleShift.get(heading)[leftRight] : redPoleShift.get(heading)[leftRight];

        currentTarget = new Pose2d(
            //                                               Pole shift                                                 Bumper shift
            reefTarget.getX() - (reefTarget.getRotation().getSin() * Units.inchesToMeters(poleShift)) - (reefTarget.getRotation().getCos() * robotToReefTagFace),
            reefTarget.getY() + (reefTarget.getRotation().getCos() * Units.inchesToMeters(poleShift)) - (reefTarget.getRotation().getSin() * robotToReefTagFace),
            reefTarget.getRotation()
        );

        SmartDashboard.putNumberArray("Drive target pose", new double[] {currentTarget.getX(), currentTarget.getY(), currentTarget.getRotation().getRadians()});
    }

    public Command resetControllers() {
        return runOnce(() -> {
            translationControllerX.reset(iOdata.state.Pose.getX(), -iOdata.state.Speeds.vxMetersPerSecond);
            translationControllerY.reset(iOdata.state.Pose.getY(), -iOdata.state.Speeds.vyMetersPerSecond);

            Translation2d speeds = new Translation2d(iOdata.state.Speeds.vyMetersPerSecond, iOdata.state.Speeds.vxMetersPerSecond);
            speeds.rotateBy(currentTarget.getRotation());
            SmartDashboard.putNumberArray("Relative Speeds", new double[] {speeds.getX(), -speeds.getY(), 0.0});

            translationControllerIn.reset();
            translationControllerAcross.reset();
        });
    }

    public void alignReefRobotcentric(boolean auton) {        
        Pose2d Error = currentTarget.relativeTo(new Pose2d(iOdata.state.Pose.getTranslation(), currentTarget.getRotation()));
        SmartDashboard.putNumberArray("Drive Error", new double[] {Error.getX(), Error.getY(), Error.getRotation().getRadians()});

        Translation2d speeds = new Translation2d(iOdata.state.Speeds.vyMetersPerSecond, iOdata.state.Speeds.vxMetersPerSecond);
        speeds.rotateBy(currentTarget.getRotation());
        SmartDashboard.putNumberArray("Relative Speeds", new double[] {speeds.getX(), -speeds.getY(), 0.0});

        translationControllerIn.calculate(Error.getX(), 0.0);
        translationControllerAcross.calculate(Error.getY(), 0.0);

        double in = !translationControllerIn.atSetpoint() ? -translationControllerIn.calculate(Error.getX(), 0.0) : 0.0;
        double across = !translationControllerAcross.atSetpoint() ? translationControllerAcross.calculate(Error.getY(), 0.0) : 0.0;
        double cosine = Math.cos(currentTarget.getRotation().getRadians());
        double sine = Math.sin(currentTarget.getRotation().getRadians());

        if (auton) {
            in = MathUtil.clamp(in, -auto_align_top_speed_auton, auto_align_top_speed_auton);
            across =  MathUtil.clamp(across, -auto_align_top_speed_auton, auto_align_top_speed_auton);
        } else {
            in = MathUtil.clamp(in, -auto_align_top_speed_teleop, auto_align_top_speed_teleop);
            across =  MathUtil.clamp(across, -auto_align_top_speed_teleop, auto_align_top_speed_teleop);
        }
        

        boolean disableTheta = Error.getX() < auto_align_theta_disable;

        if (DriverStation.getAlliance().get() == Alliance.Blue) 
            driveIO.setSwerveRequest(AUTO_ALIGN
                .withVelocityX(((in * cosine) + (across * sine)))
                .withVelocityY(((across * -cosine) + (in * sine)))
                .withRotationalRate(!disableTheta ? thetaController.calculate(
                    iOdata.state.Pose.getRotation().getRadians(), 
                    currentTarget.getRotation().getRadians() + Math.toRadians(90)
                ) : 0.0)
            );
        else 
            driveIO.setSwerveRequest(AUTO_ALIGN
                .withVelocityX(-((in * cosine) + (across * sine)))
                .withVelocityY(-((across * -cosine) + (in * sine)))
                .withRotationalRate(!disableTheta ? thetaController.calculate(
                    iOdata.state.Pose.getRotation().getRadians(), 
                    currentTarget.getRotation().getRadians() + Math.toRadians(90)
                ) : 0.0)
            );
    }

    public void alignReefFieldcentric() {
        SmartDashboard.putNumber("x_SpeedC", translationControllerX.calculate(iOdata.state.Pose.getX()));
        SmartDashboard.putNumber("y_SpeedC", translationControllerY.calculate(iOdata.state.Pose.getY()));
        SmartDashboard.putNumber("cSpeedT", Math.sqrt(Math.pow(translationControllerY.calculate(iOdata.state.Pose.getY()), 2) - Math.pow(translationControllerX.calculate(iOdata.state.Pose.getX()) , 2)));
        
        
        if (DriverStation.getAlliance().get() == Alliance.Blue) {
            driveIO.setSwerveRequest(AUTO_ALIGN
            .withVelocityX(!translationControllerX.atGoal() ? translationControllerX.calculate(iOdata.state.Pose.getX(), currentTarget.getX()) : 0.0)
            .withVelocityY(!translationControllerY.atGoal() ? translationControllerY.calculate(iOdata.state.Pose.getY(), currentTarget.getY()) : 0.0)
            .withRotationalRate(thetaController.calculate(
                iOdata.state.Pose.getRotation().getRadians(), 
                currentTarget.getRotation().getRadians() + Math.toRadians(90)
            ))
            );
        } else {
            driveIO.setSwerveRequest(AUTO_ALIGN
            .withVelocityX(!translationControllerX.atGoal() ? -translationControllerX.calculate(iOdata.state.Pose.getX(), currentTarget.getX()) : 0.0)
            .withVelocityY(!translationControllerY.atGoal() ? -translationControllerY.calculate(iOdata.state.Pose.getY(), currentTarget.getY()) : 0.0)
            .withRotationalRate(thetaController.calculate(
                iOdata.state.Pose.getRotation().getRadians(), 
                currentTarget.getRotation().getRadians() + Math.toRadians(90)
            ))
            );
        }
    }

    public FunctionalCommand autonAlignReefCommand(int LR) {
        return new FunctionalCommand(
        () -> updateReefTarget(LR),
        () -> alignReefRobotcentric(true),
        interrupted -> {}, 
        () -> Math.abs(currentTarget.getTranslation().getDistance(iOdata.state.Pose.getTranslation())) < auto_align_command,
        // () -> translationControllerX.atGoal() && translationControllerY.atGoal(),
        this);
    }

    public Command setCoast() {
        return runOnce(() -> driveIO.setNeutralMode(NeutralModeValue.Coast));
    }

    public Command setBrake() {
        return runOnce(() -> driveIO.setNeutralMode(NeutralModeValue.Brake));
    }

    public void teleopDrive(double driveX, double driveY, double driveTheta)  {
       driveIO.setSwerveRequest(FIELD_CENTRIC
            .withVelocityX((driveX <= 0 ? -(driveX * driveX) : (driveX * driveX)) * DriveConfig.MAX_VELOCITY())
            .withVelocityY((driveY <= 0 ? -(driveY * driveY) : (driveY * driveY)) * DriveConfig.MAX_VELOCITY())
            .withRotationalRate((driveTheta <= 0 ? -(driveTheta * driveTheta) : (driveTheta * driveTheta)) * DriveConfig.MAX_ANGULAR_VELOCITY())
        );

        heading = this.iOdata.state.Pose.getRotation();
    }

    public void teleopDriveSlow(double driveX, double driveY, double driveTheta)  {
        driveIO.setSwerveRequest(FIELD_CENTRIC
             .withVelocityX((driveX <= 0 ? -(driveX * driveX) : (driveX * driveX)) * DriveConfig.MAX_VELOCITY() * slowModeMultiplier)
             .withVelocityY((driveY <= 0 ? -(driveY * driveY) : (driveY * driveY)) * DriveConfig.MAX_VELOCITY() * slowModeMultiplier)
             .withRotationalRate((driveTheta <= 0 ? -(driveTheta * driveTheta) : (driveTheta * driveTheta)) * DriveConfig.MAX_ANGULAR_VELOCITY() * slowModeMultiplier)
         );
 
         heading = this.iOdata.state.Pose.getRotation();
     }

    public void robotCentricTeleopDrive(double driveX, double driveY, double driveTheta)  {
        driveIO.setSwerveRequest(ROBOT_CENTRIC
             .withVelocityX((driveX <= 0 ? -(driveX * driveX) : (driveX * driveX)) * DriveConfig.MAX_VELOCITY())
             .withVelocityY((driveY <= 0 ? -(driveY * driveY) : (driveY * driveY)) * DriveConfig.MAX_VELOCITY())
             .withRotationalRate((driveTheta <= 0 ? -(driveTheta * driveTheta) : (driveTheta * driveTheta)) * DriveConfig.MAX_ANGULAR_VELOCITY())
              );
         heading = this.iOdata.state.Pose.getRotation();
     }
 
 
     public void headingControl(double driveX, double driveY) {
         driveIO.setSwerveRequest(FIELD_CENTRIC
             .withVelocityX((driveX <= 0 ? -(driveX * driveX) : (driveX * driveX)) * DriveConfig.MAX_VELOCITY())
             .withVelocityY((driveY <= 0 ? -(driveY * driveY) : (driveY * driveY)) * DriveConfig.MAX_VELOCITY())
             .withRotationalRate(thetaController.calculate(iOdata.state.Pose.getRotation().getRadians(), heading.getRadians()))
         );
     }
 
 
     public void lockRotation(double driveX, double driveY, Rotation2d rtarget) {
         driveIO.setSwerveRequest(FIELD_CENTRIC
             .withVelocityX((driveX <= 0 ? -(driveX * driveX) : (driveX * driveX)) * DriveConfig.MAX_VELOCITY())
             .withVelocityY((driveY <= 0 ? -(driveY * driveY) : (driveY * driveY)) * DriveConfig.MAX_VELOCITY())
             .withRotationalRate(thetaController.calculate(iOdata.state.Pose.getRotation().getRadians(), rtarget.getRadians()))
         );
         heading = rtarget;
     }
 
     public void facePoint(double driveX, double driveY, Pose2d point) {
         driveIO.setSwerveRequest(FIELD_CENTRIC
             .withVelocityX((driveX <= 0 ? -(driveX * driveX) : (driveX * driveX)) * DriveConfig.MAX_VELOCITY())
             .withVelocityY((driveY <= 0 ? -(driveY * driveY) : (driveY * driveY)) * DriveConfig.MAX_VELOCITY())
             .withRotationalRate(thetaController.calculate(iOdata.state.Pose.getRotation().getRadians(),
             Math.atan2(point.getY() - iOdata.state.Pose.getY(), point.getX() - iOdata.state.Pose.getX())))
         );        
         heading = this.iOdata.state.Pose.getRotation();
    }


     public void lockReef(double driveX, double driveY) {
        double radToReef = getNearestReefAngle().getRadians();
        driveIO.setSwerveRequest(FIELD_CENTRIC
            .withVelocityX((driveX <= 0 ? -(driveX * driveX) : (driveX * driveX)) * DriveConfig.MAX_VELOCITY() * 0.69)
            .withVelocityY((driveY <= 0 ? -(driveY * driveY) : (driveY * driveY)) * DriveConfig.MAX_VELOCITY() * 0.69)
            .withRotationalRate(thetaController.calculate(iOdata.state.Pose.getRotation().getRadians(),
            radToReef + Math.toRadians(90)))
        );        
        heading = new Rotation2d (radToReef + Math.toRadians(90));
    }

    public void lockReefManual(double driveX, double driveY, double rightX, double rightY) {
        double joystickDeg = Math.toDegrees(Math.atan2(rightY, -rightX)) - 90;
        driveIO.setSwerveRequest(FIELD_CENTRIC
            .withVelocityX((driveX <= 0 ? -(driveX * driveX) : (driveX * driveX)) * DriveConfig.MAX_VELOCITY() * 0.69)
            .withVelocityY((driveY <= 0 ? -(driveY * driveY) : (driveY * driveY)) * DriveConfig.MAX_VELOCITY() * 0.69)
            .withRotationalRate(thetaController.calculate(iOdata.state.Pose.getRotation().getRadians(),
            Math.toRadians(60*Math.round(joystickDeg/60))))
        );        
        heading = new Rotation2d (Math.toRadians(60*Math.round(joystickDeg/60)));
    }

    public boolean getNearReef() {
        return this.iOdata.state.Pose.getTranslation().getDistance(DriverStation.getAlliance().get() == Alliance.Blue ? BLUE_REEF.getTranslation() : RED_REEF.getTranslation()) < distance_safe_from_reef;
    }

    public boolean getReverseIntake() {
        double robotRotation = this.iOdata.state.Pose.getRotation().getDegrees(); 
        boolean flip = robotRotation <= -35 + 90 && robotRotation >= -35 - 90;
        if (this.iOdata.state.Pose.getY() > feild_center_line) {
            flip = !flip;
        }
        return flip;
    }
    // -35 +- 90 
    public void setSelectedAutoName(String name) {
        try {
            this.pathGroup.addAll(PathPlannerAuto.getPathGroupFromAutoFile("OTF_TESTING"));
        } catch (IOException e) {
        } catch (ParseException e) {
        }
    }

    @Override
    public void periodic() {
        getObjectMeasurements();

        if (!(DriverStation.isTeleopEnabled()) || Math.abs(iOdata.pigeon.getX()) > 0.2 || Math.abs(iOdata.pigeon.getY()) > 0.2) {
            heading = iOdata.state.Pose.getRotation();
        }

        if (DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                this.driveIO.setOperatorPerspective(
                    allianceColor == Alliance.Red ? kRedAlliancePerspectiveRotation : kBlueAlliancePerspectiveRotation
                ); 
            });
        }

        
		
        this.iOdata = driveIO.update();
        if (this.iOdata.state.Speeds != null) {

            speedEntry.setDouble(Math.hypot(
                this.iOdata.state.Speeds.vxMetersPerSecond,
                this.iOdata.state.Speeds.vyMetersPerSecond) / 5.0);

                SmartDashboard.putNumber("act Speed", (Math.hypot(
                    this.iOdata.state.Speeds.vxMetersPerSecond,
                    this.iOdata.state.Speeds.vyMetersPerSecond) ));

                    SmartDashboard.putNumber("req Speed", (Math.hypot(
                        FIELD_CENTRIC.VelocityX,
                        FIELD_CENTRIC.VelocityY) ));
        }
        if (this.iOdata.state.Pose != null) {
            poseEntry.setDoubleArray(new Double[]{
                this.iOdata.state.Pose.getX(), 
                this.iOdata.state.Pose.getY(), 
                this.iOdata.state.Pose.getRotation().getRadians()});

            if (this.currentTarget != null) {
                SmartDashboard.putBoolean("Auto Align On Target", 
                this.iOdata.state.Pose.getTranslation().getDistance(currentTarget.getTranslation()) < auto_align_lights_tolerance);
            }

            // if (targetSeenSub.get()) {
            //     calculateCoralPose();
            // }
        }

        if (pathGroup != null) {
            currentPathIndex = IntStream.range(0, pathGroup.size())
            .filter(i -> pathGroup.get(i).name.equals(PathPlannerAuto.currentPathName))
            .findFirst()
            .orElse(0);
        }

        alert.set(true);
    }

    public boolean getAutoAlignGood() {
        if (this.iOdata.state.Pose != null && currentTarget != null) {
            return this.iOdata.state.Pose.getTranslation().getDistance(currentTarget.getTranslation()) < auto_align_lights_tolerance;
        } else {
            return false;
        }
    }

    public void addVisionMeasurement(Pose2d calculatedPose, double timestamp, Matrix<N3, N1> stDevs) {
        this.driveIO.updateVision(calculatedPose, timestamp, stDevs);
    }

    public Command resetPidgeon() {
        return runOnce(() -> {
            driveIO.resetPidgeon();
            heading = iOdata.state.Pose.getRotation();
            });
    }

    public Command resetHeading() {
        return runOnce(() -> heading = iOdata.state.Pose.getRotation());
    }

    private void configurePathPlanner() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> this.iOdata.state.Pose,   // Supplier of robot pose
                this::setPose,         // Consumer for seeding pose
                () -> this.iOdata.state.Speeds, // Supplier of robot speeds
                // Consumer of ChassisSpeeds and feedforwards
                (speeds, feedforwards) -> driveIO.setSwerveRequest(
                    m_pathApplyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // translation
                    DriveConfig.AUTON_TRANSLATION_PID(),
                    // rotation
                    DriveConfig.AUTON_ROTATION_PID()
                ),
                config,
                // Assume the path doesnt flip (Sep auto files for red and blue side)
                () -> false,
                this
            )   ;
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    private void setPose(Pose2d pose) {
        this.driveIO.seedFieldRelative(pose);
    }

    public double getAutoStartError() {
        if (iOdata.state.Pose != null && autoStartPose != null) {
            return iOdata.state.Pose.getTranslation().getDistance(autoStartPose.getTranslation());
        } else {
            return -1;
        }
    }

    public void setAutonStartPose(Pose2d desiredPose) {
        autoStartPose = desiredPose;
        SmartDashboard.putNumberArray("Auto Start Pose", new double[] {
            autoStartPose.getX(), 
            autoStartPose.getY(), 
            autoStartPose.getRotation().getDegrees()});
    }

    /**
     * @return true if the robot is in position for a barge shot
     */
    public boolean returnAutoBarge(){
        if(this.iOdata.state.Pose.getX() < DriveConstants.bargePosRedFar && this.iOdata.state.Pose.getX() > DriveConstants.bargePosRedClose){
            return true;

        } else if(this.iOdata.state.Pose.getX() < DriveConstants.bargePosBlueFar && this.iOdata.state.Pose.getX() > DriveConstants.bargePosBlueClose){
            return true;

        } else{
            return false;

        }
    }
}

