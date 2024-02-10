
// Most of this code is from before 
package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private TalonFX shooterLeft = new TalonFX(ShooterConstants.shooterLeftId);
  private TalonFX shooterRight = new TalonFX(ShooterConstants.shooterRightId);
  PIDController shooterPID = new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);
  private boolean manual = false;
  private double goal = 0;
  private ShooterFeedforward feedforward = new ShooterFeedforward(ShooterConstants.ks, ShooterConstants.kg, ShooterConstants.kv);

  shooterRight.setInverted(true);

  public double getPosition(){
    return shooter.getPosition().getValueAsDouble();
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

  public Command setMidCommand(){
    return runOnce(this::setMid);
  }

  public void setMid(){
    goal = ShooterConstants.mid;
  }

  public Command setLowCommand(){
    return runOnce(this::setLow);
  }

  public void setLow(){
    goal = ShooterConstants.low;
  }

  public void spinShooter(double speed){
    shooter.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(!manual){
      shooter.set(goal);
    }
  }
}
