// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drivetrain.Drive;
import frc.robot.subsystems.Drivetrain.DriveSim;
import frc.robot.subsystems.Drivetrain.DriveKraken;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Turret.Turret;


public class RobotContainer {

  private SendableChooser<String> chooser = new SendableChooser<>();
  private Command autoCommand;
  private String autoString;

  //Define Subsystems 
  private Drive m_drivetrain;
  private Intake m_intake;
  private Shooter m_shooter;
  private Turret m_turret;

  
  private final CommandXboxController driverController = new CommandXboxController(0);

  public RobotContainer() {
    //RobotController.setBrownoutVoltage(Constants.brownoutVoltage); // stops stuttering under high load when the battery is good.

    if(!Utils.isSimulation()){
        m_drivetrain = new Drive(new DriveKraken());
    } else {
        m_drivetrain = new Drive(new DriveSim());
    }

    m_intake = new Intake();
    m_shooter = new Shooter();
    m_turret = new Turret();

    //Configure Auton Chooser 
    chooser.setDefaultOption("Auto", "Auto");
    
    chooser.addOption("RedR4", "RedR4");
    chooser.addOption("RedL4", "RedL4");
    chooser.addOption("BlueR4", "BlueR4"); 
    chooser.addOption("BlueL4", "BlueL4"); 
    chooser.addOption("Funky Check", "Funky Check"); 
    chooser.addOption("RightBlueLolipop", "RightBlueLolipop"); 
    chooser.addOption("RightRedLolipopp", "RightRedLolipop"); 
    chooser.addOption("RedLeftLolipop", "RedLeftLolipop"); 
    chooser.addOption("BlueLeftLolipop", "BlueLeftLolipop"); 
    chooser.addOption("MiniBackshot", "MiniBackshot"); 
    chooser.addOption("MiniBackshot-optimized", "MiniBackshot-optimized"); 
    chooser.addOption("sacrifice", "Backshot-full"); 

    chooser.addOption("Copy of Regular4", "Copy of Regular4"); 
    chooser.addOption("Test of Regular4", "TestofRegular4"); 
    chooser.addOption("BLUE FLOOR TEST", "BLUE FLOOR TEST"); 
    chooser.addOption("BlueRLoliFetch", "BlueRLoliFetch"); 
    chooser.addOption("BlueLeftLoliFetch", "BlueLeftLoliFetch"); 

    NamedCommands.registerCommand("Align Reef Left",  m_drivetrain.autonAlignReefCommand(0));
    NamedCommands.registerCommand("Align Reef Center",  m_drivetrain.autonAlignReefCommand(1));
    NamedCommands.registerCommand("Align Reef Right",  m_drivetrain.autonAlignReefCommand(2));
    NamedCommands.registerCommand("Stop Drive", m_drivetrain.runOnce(() -> m_drivetrain.teleopDrive(0, 0, 0)));
    NamedCommands.registerCommand("Auton Fetch 2M", m_drivetrain.fetchAuto(2.0, 1.0));
    NamedCommands.registerCommand("Auton Fetch 15M", m_drivetrain.fetchAuto(1.2, 0.8));



    SmartDashboard.putData(chooser);

    configureBindings();
  }

  private void configureBindings() {   

    m_drivetrain.setDefaultCommand
      (m_drivetrain.run(() -> {
        m_drivetrain.teleopDrive(
          Math.abs(driverController.getLeftY()) >= 0.0 ? -driverController.getLeftY() : 0,
          Math.abs(driverController.getLeftX()) >= 0.0 ? -driverController.getLeftX() : 0,
          Math.abs(driverController.getRightX()) >= 0.015 ? -driverController.getRightX() : 0);
        }
    ));
   
    NamedCommands.registerCommand("OTF", m_drivetrain.generateOnTheFly());
    // NamedCommands.registerCommand("R_OTF", drivetrain.runOnTheFly());

    new EventTrigger("OTF").onTrue(Commands.runOnce(() -> m_drivetrain.generateOnTheFly()));

    // Schedule Intake command on button press, cancelling on release.
    //driverController.a().toggleOnTrue(m_intake.IntakeCommand()).toggleOnFalse(m_intake.IdleCommand());
    //driverController.b().toggleOnTrue(m_intake.EjectCommand()).toggleOnFalse(m_intake.IdleCommand());
    
    driverController.a().toggleOnTrue(m_shooter.shootCommand()).toggleOnFalse(m_shooter.IdleCommand());
    
    driverController.x().toggleOnTrue(m_turret.LeftAngleCommand());
    driverController.y().toggleOnTrue(m_turret.RightAngleCommand());


  /*
  * Joystick Y = quasistatic forward
  * Joystick A = quasistatic reverse
  * Joystick B = dynamic forward
  * Joystick X = dyanmic reverse
  */
    // driverController.y().whileTrue(m_intake.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // driverController.a().whileTrue(m_intake.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // driverController.b().whileTrue(m_intake.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // driverController.x().whileTrue(m_intake.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    // driverController.leftBumper().whileTrue(m_intake.enableLogging());
    // driverController.rightBumper().whileTrue(m_intake.disableLogging());

  }

  public void updateAutonCommand() {
    if (autoString != null) {
      if (chooser.getSelected() != autoString) {
        autoString = chooser.getSelected();
        autoCommand = new PathPlannerAuto(autoString);
        try {
          m_drivetrain.setAutonStartPose(PathPlannerAuto.getPathGroupFromAutoFile(autoString).get(0).getStartingDifferentialPose());
        } catch (IOException e) {
          System.err.println("No Auto File");
        } catch (ParseException e) {
          System.err.println("idk good luck");
        } catch (IndexOutOfBoundsException e) {
          System.err.println("no auto Paths");
        }
      }
    } else {
      autoString = chooser.getSelected();
    }
  }

  public Command getAutonomousCommand() {
    return autoCommand;
  }
}