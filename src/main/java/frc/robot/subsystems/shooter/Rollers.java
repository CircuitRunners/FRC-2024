// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.util.function.Supplier;

import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants.RollerConstants;

public class Rollers extends SubsystemBase {
  /** Creates a new Rollers. */
  private VictorSP rollers = new VictorSP(RollerConstants.rollerID);
  private TimeOfFlight tof = new TimeOfFlight(RollerConstants.tofId);


  public Rollers() {

  }

  public Command setRollersSpeedInCommand(){
    return runOnce(() -> runRollers(RollerConstants.rollerInSpeed));
  }

  public Command runRollersCommand(Supplier<Double> speedSupplier){ 
    return run(() -> runRollers(speedSupplier.get()));
  }


  public Command autoIntake(){
    return runRollersCommand(() -> RollerConstants.rollerInSpeed).until(this::hasNote).finallyDo(this::stopRollers);
  }
  public void stopRollers(){
    runRollers(0);
  }

  public Command stopRollersCommand(){
    return runOnce(this::stopRollers);
  }

  public boolean hasNote(){
    return tof.getRange() < RollerConstants.tofThreshold;
  }

  public void runRollers(double speed){
    rollers.set(speed);
  }

  public Command runRollersOutCommand(){
    return runOnce(() -> runRollers(RollerConstants.rollerOutSpeed));
  }

  public Command runRollersOutCommandSlow(){
    return run(() -> runRollers(RollerConstants.rollerOutSpeedSlow)).withTimeout(0.1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
