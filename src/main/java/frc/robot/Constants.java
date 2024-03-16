package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import frc.lib.swerve.COTSFalconSwerveConstants;
import frc.lib.swerve.SwerveModuleConstants;
import frc.lib.utils.AllianceFlipUtil;
import frc.robot.generated.TunerConstants;

public final class Constants {
  public static final class SwerveConstants {
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
    public static final double angleKP = TunerConstants.steerGains.kP;
    public static final double angleKI = TunerConstants.steerGains.kI;
    public static final double angleKD = TunerConstants.steerGains.kD;
    public static final double angleKF = 0.0;

    /* Angle Motor SYSID values */
    public static final double angleKS = TunerConstants.steerGains.kS;
    public static final double angleKV = TunerConstants.steerGains.kV;
    public static final double angleKG = TunerConstants.steerGains.kG;
    public static final double angleKA = TunerConstants.steerGains.kA;

    /* Drive Motor PID Values */
    public static final double driveKP = TunerConstants.driveGains.kP; //TODO: This must be tuned to specific robot
    public static final double driveKI = TunerConstants.driveGains.kI;
    public static final double driveKD = TunerConstants.driveGains.kD;
    public static final double driveKF = 0;

    /* Drive Motor Characterization Values 
     * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
    public static final double driveKS = TunerConstants.driveGains.kS; //TODO: This must be tuned to specific robot
    public static final double driveKV = TunerConstants.driveGains.kV;
    public static final double driveKG = TunerConstants.driveGains.kG;
    public static final double driveKA = TunerConstants.driveGains.kA;

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double maxVelocityMPS = 4.48; //TODO: 48 must be tuned to specific robot
    public static final double maxModuleVelocityMPS = maxVelocityMPS;

    public static final double maxModuleAccelerationMPSSq = 2.0; //TODO: This must be tuned to specific robot

    /** Radians per Second */
    public static final double maxAngularVelocityRPS = 10.0; //TODO: This must be tuned to specific robot

    public static final double maxAngularAccelerationRPSSq = 5.0; //TODO: This must be tuned to specific robot
    
    public static final double slipCurrent = 300;
    public static final double steerInertia = TunerConstants.kSteerInertia;
    public static final double driveInertia = TunerConstants.kDriveInertia;
    public static final double couplingGearRatio = TunerConstants.kCoupleRatio;
    public static final String CANBusName = TunerConstants.kCANbusName;
    public static final double driveBaseRadiusMeter = Units.inchesToMeters(12);
    public static final Translation2d[] modulePositions = { Mod0.position, Mod1.position, Mod2.position,
        Mod3.position };

    /* PID Constants */
    public static final PIDConstants translationalPID = new PIDConstants(driveKP, driveKI, driveKD);
    public static final PIDConstants rotationalPID = new PIDConstants(angleKP, angleKI, angleKD);


    /* Front Left Module - Module 0 */
    public static final class Mod0 { //TODO: This must be tuned to specific robot
      public static final int driveMotorID = 0;
      public static final int angleMotorID = 1;
      public static final int canCoderID = 0;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(TunerConstants.kFrontLeftEncoderOffset);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
      public static final Translation2d position = new Translation2d(driveBaseRadiusMeter / 2.0, wheelBase / 2.0); //front left is  +x, +y
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 { //TODO: This must be tuned to specific robot
      public static final int driveMotorID = 2;
      public static final int angleMotorID = 3;
      public static final int canCoderID = 1;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(TunerConstants.kFrontRightEncoderOffset);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
      public static final Translation2d position = new Translation2d(driveBaseRadiusMeter / 2.0, -wheelBase / 2.0); //front right is +x, -y
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 { //TODO: This must be tuned to specific robot
      public static final int driveMotorID = 4;
      public static final int angleMotorID = 5;
      public static final int canCoderID = 2;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(TunerConstants.kBackLeftEncoderOffset);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
      public static final Translation2d position = new Translation2d(-driveBaseRadiusMeter / 2.0, wheelBase / 2.0); //back left is -x, +y
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 { //TODO: This must be tuned to specific robot
      public static final int driveMotorID = 6;
      public static final int angleMotorID = 7;
      public static final int canCoderID = 3;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(TunerConstants.kBackRightEncoderOffset);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
      public static final Translation2d position = new Translation2d(-driveBaseRadiusMeter / 2.0, -wheelBase / 2.0); //back right is -x, -y
    }


    public static final PathConstraints pathConstraints = new PathConstraints(maxVelocityMPS, maxModuleAccelerationMPSSq, maxAngularVelocityRPS, maxAngularAccelerationRPSSq);
  }

  public static final class DriverConstants {
    public static final double stickDeadband = 0.2;
    public static final int driverPort = 0;
    public static final int operatorPort = 1;
  }

  public static final class ElevatorConstants {

    public static final int lMotor = 0;
    public static final int rMotor = 0;
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

  public static final class IntakeConstants {
      
    public static final int intakeId = 0;
    public static final double intakeInSpeed = 0.25;
    public static final double intakeOutSpeed = -0.5;

    public static final int armId = 0;

    public static final double high = 0;
    public static final double low = 0;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final int tofId = 0;
    public static final double tofThresholdMMS = Units.inchesToMeters(5) * 1000;

  }

  public static final class ShooterConstants {
      
    public static final int shooterLeftId = 0;
    public static final int shooterRightId = 0;

    public static final double high = 0;
    public static final double low = 0;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double ks = 0;
    public static final double kv = 0;
    public static final double kg = 0;
    public static final int shooterArmId = 0;
    public static final double mid = 0;

    public static final double shooterOutSpeed = 0;
    public static final double shooterInSpeed = 0;
  }


  public class FieldConstants {
        public static final Measure<Distance> kFieldLength = Meters.of(16.54);
        public static final Measure<Distance> kFieldWidth = Meters.of(8.21);

        public static final AprilTagFieldLayout kFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

        // taken from 6328. All in blue alliance origin.
        /* speaker constants */
        public static final class SpeakerK {
            private static final Measure<Distance> kTopX = Inches.of(18.055);
            private static final Measure<Distance> kTopZ = Inches.of(83.091);
            public static final Translation3d kTopRight = new Translation3d(
                kTopX, Inches.of(238.815), kTopZ);
            public static final Translation3d kTopLeft = new Translation3d(
                kTopX, Inches.of(197.765), kTopZ);

            private static final Measure<Distance> kBotX = Inches.of(0);
            private static final Measure<Distance> kBotZ = Inches.of(78.324);
            // private static final Translation3d kBotRight = new Translation3d(
            // kBotX, Inches.of(238.815), kBotZ);
            public static final Translation3d kBotLeft = new Translation3d(
                kBotX, Inches.of(197.765), kBotZ);

            public static final Translation3d kBlueCenterOpening = kBotLeft.interpolate(kTopRight, 0.5);
            public static final Pose3d kBlueCenterOpeningPose3d = new Pose3d(
                kBlueCenterOpening, new Rotation3d());
            public static final Translation3d kRedCenterOpening = AllianceFlipUtil.flip(kBlueCenterOpening);
            public static final Pose3d kRedCenterOpeningPose3d = new Pose3d(
                kRedCenterOpening, new Rotation3d());

            public static final Measure<Distance> kAimOffset = Inches.of(25);
        }

        /* amp constants */
        public static final Measure<Distance> kXToAmp = Inches.of(49.5);
        public static final Measure<Distance> kYToAmp = Inches.of(286.765);
        public static final Measure<Distance> kZToAmp = Inches.of(35);

        public static final Translation3d kBlueAmpPose = new Translation3d(
            kXToAmp, kYToAmp, kZToAmp);
        public static final Pose2d kBlueAmpPose2d = new Pose2d(
            kXToAmp, kYToAmp, Rotation2d.fromDegrees(90));
            
        public static final Translation3d kRedAmpPose = new Translation3d(
              kFieldLength.minus(kXToAmp), kYToAmp, kZToAmp);
        public static final Pose2d kRedAmpPose2d = new Pose2d(
            kRedAmpPose.getX(), kRedAmpPose.getY(), Rotation2d.fromDegrees(90));

        /* stage constants */
        public static final double kBlueStageClearanceDs = Units.inchesToMeters(188.5);
        public static final double kBlueStageClearanceRight = Units.inchesToMeters(88.3);
        public static final double kBlueStageClearanceCenter = Units.inchesToMeters(243.2);
        public static final double kBlueStageClearanceLeft = Units.inchesToMeters(234.9);

        public static final double kRedStageClearanceDs = Units.inchesToMeters(542.2);
        public static final double kRedStageClearanceRight = Units.inchesToMeters(88.3);
        public static final double kRedStageClearanceCenter = Units.inchesToMeters(407.9);
        public static final double kRedStageClearanceLeft = Units.inchesToMeters(234.9);
    }

  public static final class Vision {
    public static final AprilTagFieldLayout fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    public static final Transform3d frontLeftCamTransform = new Transform3d();
    public static final Transform3d frontRightCamTransform = new Transform3d();
  }
}
