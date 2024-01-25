package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.swerve.COTSFalconSwerveConstants;
import frc.lib.swerve.SwerveModuleConstants;

public final class Constants {
  public static final class SwerveConstants {
    public static double limit = 0.8;
    public static final int pigeonID = 1;
    public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

    public static final COTSFalconSwerveConstants chosenModule = //TODO: This must be tuned to specific robot
        COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(24.73); //TODO: This must be tuned to specific robot
    public static final double wheelBase = Units.inchesToMeters(24.73); //TODO: This must be tuned to specific robot
    public static final double wheelCircumference = chosenModule.wheelCircumference;

    /* Swerve Kinematics 
     * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        Mod0.position,
        Mod1.position,
        Mod2.position,
        Mod3.position);

    /* Module Gear Ratios */
    public static final double driveGearRatio = chosenModule.driveGearRatio;
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    /* Motor Inverts */
    public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
    public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = chosenModule.canCoderInvert;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 25;
    public static final int anglePeakCurrentLimit = 40;
    public static final double anglePeakCurrentDuration = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveContinuousCurrentLimit = 35;
    public static final int drivePeakCurrentLimit = 60;
    public static final double drivePeakCurrentDuration = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    /* Angle Motor PID Values */
    public static final double angleKP = 20;
    public static final double angleKI = 0.20;
    public static final double angleKD = 0.18;
    public static final double angleKF = 0.0;

    /* Angle Motor SYSID values */
    public static final double angleKS = 0.3;
    public static final double angleKV = 2.0;
    public static final double angleKG = 0.0;
    public static final double angleKA = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 20; //TODO: This must be tuned to specific robot
    public static final double driveKI = 0.20;
    public static final double driveKD = 0.18;
    public static final double driveKF = 0.4;

    /* Drive Motor Characterization Values 
     * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
    public static final double driveKS = (0.1); //TODO: This must be tuned to specific robot
    public static final double driveKV = (0.6);
    public static final double driveKG = (0.0);
    public static final double driveKA = (0.0);

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double maxSpeedMPS = 4.5; //TODO: This must be tuned to specific robot
    public static final double maxModuleSpeedMPS = maxSpeedMPS;

    /** Radians per Second */
    public static final double maxAngularVelocityRPS = 10.0; //TODO: This must be tuned to specific robot

    public static final double slipCurrent = 800;
    public static final double steerInertia = 0.01;
    public static final double driveInertia = 0.01;
    public static final double couplingGearRatio = 0;
    public static final String CANBusName = "BUS";
    public static final double driveBaseRadiusMeter = trackWidth / 2.0;
    public static final Translation2d[] modulePositions = { Mod0.position, Mod1.position, Mod2.position,
        Mod3.position };

    /* PID Constants */
    public static final PIDConstants translationalPID = new PIDConstants(driveKP, driveKI, driveKD);
    public static final PIDConstants rotationalPID = new PIDConstants(angleKP, angleKI, angleKD);

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 { //TODO: This must be tuned to specific robot
      public static final int driveMotorID = 0;
      public static final int angleMotorID = 1;
      public static final int canCoderID = 0;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
      public static final Translation2d position = new Translation2d(trackWidth / 2.0, wheelBase / 2.0); //front left is  +x, +y
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 { //TODO: This must be tuned to specific robot
      public static final int driveMotorID = 30;
      public static final int angleMotorID = 31;
      public static final int canCoderID = 1;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
      public static final Translation2d position = new Translation2d(trackWidth / 2.0, -wheelBase / 2.0); //front right is +x, -y
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 { //TODO: This must be tuned to specific robot
      public static final int driveMotorID = 10;
      public static final int angleMotorID = 11;
      public static final int canCoderID = 2;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
      public static final Translation2d position = new Translation2d(-trackWidth / 2.0, wheelBase / 2.0); //back left is -x, +y
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 { //TODO: This must be tuned to specific robot
      public static final int driveMotorID = 20;
      public static final int angleMotorID = 21;
      public static final int canCoderID = 3;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
      public static final Translation2d position = new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0); //back right is -x, -y
    }

  }

  public static final class DriverConstants {
    public static final double stickDeadband = 0.1;
    public static final int driverPort = 0;
    public static final int operatorPort = 1;
  }

  public static final class ElevatorConstants {

    public static final int lMotor = 0;
    public static final double ks = 0;
    public static final double kg = 0;
    public static final double kv = 0;
    public static final double high = 0;
    public static final double mid = 0;
    public static final double low = 0;
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

  }

  public static final class ArmConstants {

    public static int intakeId = 0;

  }
}
