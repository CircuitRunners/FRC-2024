
// Most of this code is from before 
package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private TalonFX shooterLeft = new TalonFX(ShooterConstants.shooterLeftId);
  private TalonFX shooterRight = new TalonFX(ShooterConstants.shooterRightId);
  private TalonFX shooterArm = new TalonFX(ShooterConstants.shooterArmId); 
  PIDController shooterPID = new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);
  private boolean manual = false;
  private double goal = 0;

  public Shooter() {
    shooterRight.setInverted(true);
  }
  public double getPosition(){
    return shooterArm.getPosition().getValueAsDouble();
  }

  public Command toggleManualCommand(){
    return runOnce(this::toggleManual);
  }
  public void toggleManual(){
    manual = !manual;
  }

  public boolean isManual(){
    return manual;
  }

  public Command setHighCommand(){
    return runOnce(this::setHigh);
  }
  public void setHigh(){
    goal = ShooterConstants.high;
  }


  public Command setLowCommand(){
    return runOnce(this::setLow);
  }

  public void setLow(){
    goal = ShooterConstants.low;
  }

  public void runShooterOut(){
    shooterLeft.set(ShooterConstants.shooterOutSpeed);
    shooterRight.set(ShooterConstants.shooterOutSpeed);
  }

  public void runShooterIn(){
    shooterLeft.set(ShooterConstants.shooterOutSpeed);
    shooterRight.set(ShooterConstants.shooterOutSpeed);
  }

  public Command runShooterOutCommand(){
    return run(this::runShooterOut);
  }

  public Command runShooterInCommand(){
    return run(this::runShooterIn);
  }

  public void spinShooter(double speed){
    shooterLeft.set(speed);
    shooterRight.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(!manual){
      shooterArm.set(shooterPID.calculate(getPosition(), goal));
    }
  }
}
