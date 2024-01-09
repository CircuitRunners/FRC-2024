package frc.lib.swerve;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

public class SwerveConfig {
    //TODO: tune configs
    private static final Slot0Configs driveConfigs = new Slot0Configs() {
        {
            kP = 0.05;
            kI = 0.0;
            kD = 0.0;
            kS = (0.32 / 12);
            kV = (1.51 / 12);
            kG = 0.0;
            kA = (0.27 / 12);
        }
    };
    private static final Slot0Configs turnConfigs = new Slot0Configs() {
        {
            kP = 0.0;
            kI = 0.0;
            kD = 0.0;
            kS = 0.0;
            kV = 0.0;
            kG = 0.0;
            kA = 0.0;
        }
    };

    private static final SwerveModuleConstantsFactory CONSTANTS_FACTORY = new SwerveModuleConstantsFactory()
    .withDriveMotorGearRatio(SwerveConstants.driveGearRatio)
    .withSteerMotorGearRatio(SwerveConstants.angleGearRatio)
    .withWheelRadius(Units.metersToInches(SwerveConstants.wheelCircumference / (2 * Math.PI)))
    .withSlipCurrent(SwerveConstants.slipCurrent)
    .withSteerMotorGains(turnConfigs)
    .withDriveMotorGains(driveConfigs)
    .withSpeedAt12VoltsMps(SwerveConstants.maxSpeedMPS)
    .withSteerInertia(SwerveConstants.steerInertia)
    .withDriveInertia(SwerveConstants.driveInertia)
    .withSteerMotorInverted(false)
    .withCouplingGearRatio(SwerveConstants.couplingGearRatio);

    private static final SwerveModuleConstants generateConstants(int turnID, int driveID, int canCoderID, Translation2d position, double absoluteOffset){
        return CONSTANTS_FACTORY.createModuleConstants(turnID, driveID, canCoderID, absoluteOffset, position.getX(), position.getY(), false);

    }

    public static CommandSwerveDrivetrain getConfiguredDrivetrain(){
        var drivetrain = new SwerveDrivetrainConstants()
        .withPigeon2Id(SwerveConstants.pigeonID)
        .withCANbusName(SwerveConstants.CANBusName);
        var frontLeft = generateConstants(SwerveConstants.Mod0.angleMotorID, SwerveConstants.Mod0.driveMotorID, SwerveConstants.Mod0.canCoderID, SwerveConstants.Mod0.position, SwerveConstants.Mod0.angleOffset.getRadians());
        var frontRight = generateConstants(SwerveConstants.Mod1.angleMotorID, SwerveConstants.Mod1.driveMotorID, SwerveConstants.Mod1.canCoderID, SwerveConstants.Mod1.position, SwerveConstants.Mod1.angleOffset.getRadians());
        var backLeft = generateConstants(SwerveConstants.Mod2.angleMotorID, SwerveConstants.Mod2.driveMotorID, SwerveConstants.Mod2.canCoderID, SwerveConstants.Mod2.position, SwerveConstants.Mod2.angleOffset.getRadians());
        var backRight = generateConstants(SwerveConstants.Mod3.angleMotorID, SwerveConstants.Mod3.driveMotorID, SwerveConstants.Mod3.canCoderID, SwerveConstants.Mod3.position, SwerveConstants.Mod3.angleOffset.getRadians());

        return new CommandSwerveDrivetrain(drivetrain, new SwerveModuleConstants[] {frontLeft, frontRight, backLeft, backRight});
    }

    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDeadband(Constants.SwerveConstants.maxSpeedMPS * 0.1);
    public static final SwerveRequest.RobotCentric m_RobotCentric = new SwerveRequest.RobotCentric();
    public static final SwerveRequest.SwerveDriveBrake m_Brake = new SwerveRequest.SwerveDriveBrake();
    public static final SwerveRequest.PointWheelsAt m_PointWheelsAt = new SwerveRequest.PointWheelsAt();
    public static final SwerveRequest.ApplyChassisSpeeds m_ApplyChassisSpeeds = new SwerveRequest.ApplyChassisSpeeds();
}
