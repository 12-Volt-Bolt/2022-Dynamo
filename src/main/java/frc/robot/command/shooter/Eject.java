// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.tools.Timer;
import frc.robot.command.storage.EmptyThroughShooter;

public class Eject extends CommandBase {

  private EmptyThroughShooter emptyThroughShooter = new EmptyThroughShooter(this);
  private Timer spinUpDelay = new Timer(200);

  /** Creates a new Eject. */
  public Eject() {
    addRequirements(Robot.shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.shooter.spin(0.55f);
    if (spinUpDelay.isFinished() && !emptyThroughShooter.isScheduled()) {
      emptyThroughShooter.schedule();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.shooter.stop();
    if (emptyThroughShooter.isScheduled())
    {
      emptyThroughShooter.cancel();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
