// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants.ShooterConstants.ArmConstants;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private TalonFX armLeader = new TalonFX(ArmConstants.armLeaderId); 
  private TalonFX armFollower = new TalonFX(ArmConstants.armFollowerId); 
  private TrapezoidProfile armProfile = new TrapezoidProfile(new Constraints(Units.degreesToRadians(45), Units.degreesToRadians(90)));
  private PIDController shooterPID = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
  private ArmFeedforward shooterFeedForward = new ArmFeedforward(ArmConstants.kS, ArmConstants.kV, ArmConstants.kG);
  private DutyCycleEncoder throuhBore = new DutyCycleEncoder(ArmConstants.throuhBoreEncoderPort);
  private Rotation2d targetAngle = new Rotation2d(); 
  private final SysIdRoutine routine = new SysIdRoutine(new Config(), new Mechanism(this::armVoltage, null, this));
  
  public Arm() {
    armLeader.getConfigurator()
      .apply(new CurrentLimitsConfigs().withStatorCurrentLimit(30).withSupplyCurrentLimit(30));
    armFollower.setControl(new Follower(ArmConstants.armLeaderId, false));
    armLeader.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
  }

  public void armVoltage(Measure<Voltage> voltageMeasure){
    armLeader.setVoltage(voltageMeasure.magnitude());
  }

  public Rotation2d getArmPosition(){
    return Rotation2d.fromRadians(throuhBore.getAbsolutePosition());
  }

  public double getArmVelocity(){
    return armLeader.getVelocity().getValueAsDouble();
  }
  public void setTargetAngle(Rotation2d targetAngle){ 
    this.targetAngle = targetAngle;
  }
  
  public Command setArmIntake(){
    return runOnce(() -> setTargetAngle(ArmConstants.intakeRotation));
  }

  public Command setArmShootPosition(){
    return runOnce(() -> setTargetAngle(ArmConstants.shootRotation));
  }

  public void runManual(double value){
    setTargetAngle(targetAngle.plus(Rotation2d.fromDegrees(value * ArmConstants.armSpeed)));
  }

  public boolean isArmAtTarget(double threshold){
    return Math.abs(targetAngle.getRadians() - getArmPosition().getRadians()) < threshold;
  }

  public Command setArmShootPositionAndWait(){
    return run(() -> setTargetAngle(ArmConstants.shootRotation)).until(() -> isArmAtTarget(Units.degreesToRadians(0.5)));
  }

  
  public Command sysIdDnamicCommand(Direction direction){
    return routine.dynamic(direction);
  }

  public Command sysIdQuasistaticCommand(Direction direction){
    return routine.quasistatic(direction);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var targetState = armProfile.calculate(0.02, new TrapezoidProfile.State(getArmPosition().getRadians(), 0.0), new TrapezoidProfile.State(targetAngle.getRadians(), 0.0));
    armLeader.setVoltage(shooterPID.calculate(getArmPosition().getDegrees(), targetState.position) + shooterFeedForward.calculate(targetState.position, targetState.velocity));
  }
}
