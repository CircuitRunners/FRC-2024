package.frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX; 
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Intake extends SubsystemBase {
  TalonFX intake = new TalonFX(ArmConstants.intakeId);

  public CommandBase runIntakeIn command(){
    return runOnce this runIntakeIn;
}
public void runIntakeIn(){
  intake.set(ControlMode.PercentOutput, 1.5);
}
public CommandBase runIntakeOut command(){
  return runOnce this runIntakeOut;
}
public void runIntakeOut(){
  intake.set(ControlMode.PercentOutput, -0.5);
}
  public void periodic();
}
