  // Copyright (c) FIRST and other WPILib contributors.
  // Open Source Software; you can modify and/or share it under the terms of
  // the WPILib BSD license file in the root directory of this project.

  package frc.robot;
  
  /**
   * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
   * constants. This class should not be used for any other purpose. All constants should be declared
   * globally (i.e. public static). Do not put anything functional in this class.
   *
   * <p>It is advised to statically import this class (or one of its inner classes) wherever the
   * constants are needed, to reduce verbosity.
   */
  public final class Constants {
    public static final double loopPeriodSecs = 0.02;
    private static RobotType robotType = RobotType.CAMERABOT;
    public static final boolean tuningMode = false;
    public static final String canName = "robot";

    public static final double brownoutVoltage = 5.5;

    public static RobotType getRobot() {
      return robotType;
    }

    public enum Mode {
      /** Running on a real robot. */
      REAL,

      /** Running a physics simulator. */
      SIM,

      /** Replaying from a log file. */
      REPLAY
    }

    public enum RobotType {
      SIMBOT,
      DEVBOT,
      CAMERABOT,
      COMPBOT
    }

    public static boolean disableHAL = false;

    public static void disableHAL() {
      disableHAL = true;
    }

    /** Checks whether the correct robot is selected when deploying. */
    public static void main(String... args) {
      if (robotType == RobotType.SIMBOT) {
        System.err.println("Cannot deploy, invalid robot selected: " + robotType);
        System.exit(1);
      }
    }
    public static class OperatorConstants {
      public static final int kDriverControllerPort = 0;
    }

  }
