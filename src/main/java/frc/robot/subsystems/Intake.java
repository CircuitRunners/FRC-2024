package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.*;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase { // i mostly updated this to match elevator.java's structure
  private CANSparkMax arm = new CANSparkMax(IntakeConstants.armId, MotorType.kBrushless);
  private CANSparkMax intake = new CANSparkMax(IntakeConstants.intakeId, MotorType.kBrushless);
  private PIDController armPID = new PIDController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD);
  private TimeOfFlight tof = new TimeOfFlight(IntakeConstants.tofId);
  private double targetAngle = 0;
  private final SysIdRoutine routine = new SysIdRoutine(new Config(), new Mechanism(this::armVoltage, null, this));

  private void armVoltage(Measure<Voltage> voltageMeasure){
    arm.setVoltage(voltageMeasure.baseUnitMagnitude());
  }
  public Command runIntakeInCommand(){
    return run(() -> runIntake(IntakeConstants.intakeInSpeed));
  }

  public Command runIntakeOutCommand(){
    return run(() -> runIntake(IntakeConstants.intakeOutSpeed));
  }

  public Command stopIntakeCommand(){
    return run(() -> runIntake(0));
  }

  public void runIntake(double speed){
    intake.set(speed);
  }


  //

  public Command setArmHighCommand(){
    return run(() -> moveArmToTargetPosition(IntakeConstants.high));
  }

  public Command setArmLowCommand(){
    return run(() -> moveArmToTargetPosition(IntakeConstants.low));
  }

  public Command stopArmCommand(){
    return run(() -> moveArmToTargetPosition(targetAngle));
  }

  public double getTOFRange(){
    return tof.getRange();
  }

  public Command moveArmManualUp() {
    return run(() -> moveArmToTargetPosition(targetAngle ++));
  }

  public RepeatCommand moveArmManualDown() {
    return run(() -> moveArmToTargetPosition(targetAngle --)).repeatedly();
  }

  public void moveArmToTargetPosition(double target){
    this.targetAngle = target;
  }


  //
  @Override
  public void periodic(){
    arm.set(armPID.calculate(arm.getEncoder().getPosition(), targetAngle));
  }

  public Command sysIdDnamicCommand(Direction direction){
    return routine.dynamic(direction);
  }

  public Command sysIdQuasistaticCommand(Direction direction){
    return routine.quasistatic(direction);
  }
}
