// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utils.TunableNumber;
import frc.robot.Constants.ShooterConstants.FlywheelConstants;

public class Flywheel extends SubsystemBase {
  private CANSparkMax flywheelRightLeader = new CANSparkMax(FlywheelConstants.shooterLeftId, MotorType.kBrushless);
  private CANSparkMax flywheelLeftFollower = new CANSparkMax(FlywheelConstants.shooterRightId, MotorType.kBrushless);
  private final RelativeEncoder flywheelLeftEncoder;
  // private BangBangController flywheelController = new BangBangController();
  // private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(FlywheelConstants.kS, FlywheelConstants.kV, FlywheelConstants.kA);
  private TunableNumber tunedkS = new TunableNumber("Flywheel/Tuning/kS");

  /** Creates a new Flywheel. */
  public Flywheel() {
    flywheelRightLeader.setIdleMode(IdleMode.kCoast);
    flywheelLeftFollower.setIdleMode(IdleMode.kCoast);

    flywheelLeftEncoder = flywheelRightLeader.getEncoder();
    
    flywheelLeftFollower.setSmartCurrentLimit(30);
    flywheelRightLeader.setSmartCurrentLimit(30);
    flywheelLeftFollower.follow(flywheelRightLeader, true);

  }

  public double getFlywheelRPM(){
    return flywheelLeftEncoder.getVelocity();
  }

  public void runFlywheel(double speed){
    // flywheelLeftLeader.setVoltage(flywheelController.calculate(getFlywheelRPM(), rpm) + feedforward.calculate(rpm));
    flywheelRightLeader.set(speed);
  }

  public Command runFlywheelOut(){
    return run(() -> runFlywheel(0.8));
  }
  public Command stopFlywheelCommand(){
    return run(() -> runFlywheel(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // if (tunedkS.hasChanged()) {
    //   feedforward = new SimpleMotorFeedforward(tunedkS.get(), FlywheelConstants.kV, FlywheelConstants.kA);
    // }
  }
}
