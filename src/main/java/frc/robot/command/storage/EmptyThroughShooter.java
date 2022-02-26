// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.storage;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.tools.Timer;

public class EmptyThroughShooter extends CommandBase {

  private Timer forceEnd = new Timer(5000);
  
  private Command cancelWhenFinished;

  private boolean firstUpdate = true;

  public EmptyThroughShooter(Command cancelWhenFinished) {
    this.cancelWhenFinished = cancelWhenFinished;
    
    addRequirements(Robot.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    forceEnd.reset();
    Robot.storage.setUserStorage(false);
    Robot.storage.updateGoal(6);
    Robot.storage.SetDoStorage(true);
    firstUpdate = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.storage.setFeederMotor(0.75);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.storage.setUserStorage(true);
    Robot.storage.setFeederMotor(0);
    
    if (cancelWhenFinished != null && cancelWhenFinished.isScheduled()) {
      cancelWhenFinished.cancel();
    }
    firstUpdate = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean goalEnd = Robot.storage.getGoal() == (int) Math.round(Robot.storage.getEncoderPosition());
    return (goalEnd || forceEnd.isFinished()) && firstUpdate == false;
  }
}
