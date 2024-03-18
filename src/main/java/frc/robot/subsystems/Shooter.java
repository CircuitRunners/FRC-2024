
// Most of this code is from before 
package frc.robot.subsystems;


import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private CANSparkMax flywheelLeft = new CANSparkMax(ShooterConstants.shooterLeftId, MotorType.kBrushless);
  private CANSparkMax flywheelRight = new CANSparkMax(ShooterConstants.shooterRightId, MotorType.kBrushless) ;
  private TalonFX shooterArm = new TalonFX(ShooterConstants.shooterArmId); 
  private VictorSP rollers = new VictorSP(ShooterConstants.rollerID);
  private TimeOfFlight tof = new TimeOfFlight(ShooterConstants.tofId);
  private PIDController shooterPID = new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);
  private ArmFeedforward shooterFeedForward = new ArmFeedforward(ShooterConstants.ks, ShooterConstants.kv, ShooterConstants.kg);
  private TrapezoidProfile profile = new TrapezoidProfile(new Constraints(Units.degreesToRadians(45), Units.degreesToRadians(90)));

  private double targetAngle = 0;

  public Shooter() {
    flywheelRight.setInverted(true);
    flywheelRight.setSmartCurrentLimit(30);
    flywheelLeft.setSmartCurrentLimit(30);
    shooterArm.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimit(30).withSupplyCurrentLimit(30));
  }
  public double getArmPosition(){
    return shooterArm.getPosition().getValueAsDouble();
  }

  public double getArmVelocity(){
    return shooterArm.getVelocity().getValueAsDouble();
  }
  public void moveArmToTargetPosition(double targetAngle){ 
    this.targetAngle = targetAngle;
  }
  
  public Command setArmIntake(){
    return runOnce(() -> moveArmToTargetPosition(ShooterConstants.high));
  }

  public Command setArmOuttake(){
    return runOnce(() -> moveArmToTargetPosition(ShooterConstants.low));
  }


  public Command runShooterOutCommand(){
    return run(() -> runFlywheel(ShooterConstants.shooterOutSpeed));
  }

  public Command runRollersInCommand(){
    return run(() -> runRollers(ShooterConstants.rollerInSpeed));
  }

  public Command autoIntake(){
    return run(() -> {
      if(tof.getRange() > ShooterConstants.tofThreshold){
        runRollers(ShooterConstants.rollerInSpeed);
      } else {
        runRollers(0);
      }
    });
  }

  public void runFlywheel(double speed){
    flywheelLeft.set(speed);
    flywheelRight.set(speed);
  }

  public void runRollers(double speed){
    rollers.set(speed);
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    runShooterOutCommand().execute();
    var targetState = profile.calculate(0.02, new TrapezoidProfile.State(getArmPosition(), 0.0), new TrapezoidProfile.State(targetAngle, 0.0));
    shooterArm.setVoltage(shooterPID.calculate(getArmPosition(), targetState.position) + shooterFeedForward.calculate(targetState.position, targetState.velocity));
  }
}
