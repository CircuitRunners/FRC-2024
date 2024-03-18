
// Most of this code is from before 
package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private TalonFX shooterLeft = new TalonFX(ShooterConstants.shooterLeftId);
  private TalonFX shooterRight = new TalonFX(ShooterConstants.shooterRightId);
  private TalonFX shooterArm = new TalonFX(ShooterConstants.shooterArmId); 
  private PIDController shooterPID = new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);
  // private ArmFeedforward shooterFeedForward = new ArmFeedforward(ShooterConstants.ks, ShooterConstants.kv, ShooterConstants.kg);

  private double targetAngle = 0;

  public Shooter() {
    shooterRight.setInverted(true);
  }
  public double getPosition(){
    return shooterArm.getPosition().getValueAsDouble();
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
    return run(() -> spinShooter(ShooterConstants.shooterOutSpeed));
  }

  public Command runShooterInCommand(){
    return run(() -> spinShooter(ShooterConstants.shooterInSpeed));
  }

  public void spinShooter(double speed){
    shooterLeft.set(speed);
    shooterRight.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    shooterArm.set(shooterPID.calculate(getPosition(), targetAngle));
  }
}
