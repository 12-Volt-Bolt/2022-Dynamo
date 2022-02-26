// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.tools.Equations;
import frc.robot.tools.Timer;
import frc.robot.command.storage.EmptyThroughShooter;
import frc.robot.subsystem.Intake.IntakePosition;

public class ShootWithVision extends CommandBase {
  
  private EmptyThroughShooter emptyThroughShooter = new EmptyThroughShooter(this);
  private PIDController shooterPID = new PIDController(0.00045, 0.001, 0);
  double power = 0;
  
  public boolean atSpeed = false;

  private Timer IntakeDownDelay = new Timer(250);
  private Timer atSpeedTimer = new Timer(100);
  private Timer forceEnd = new Timer(10000);

  /** Creates a new ShootWithVision. */
  public ShootWithVision() {
    addRequirements(Robot.shooter);
    
    shooterPID.setTolerance(5, 5);
    shooterPID.setIntegratorRange(-0.1, 0.1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    IntakeDownDelay.reset();

    forceEnd.reset();
    atSpeedTimer.reset();

    atSpeed = false;
    
    Robot.intake.setUserControl(false);
    Robot.intake.setGoalPosition(IntakePosition.Down);

    Robot.limelight.setUserControl(false);
    Robot.limelight.setLEDs(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (IntakeDownDelay.isFinished()) {
      double yValue = Robot.limelight.getVerOffset();
      double GoalRPM = 0;
      double feedForward = 0;
           if (yValue > 20) { GoalRPM = 2255; feedForward = 0.395; }
      else if (yValue > 15) { GoalRPM = 2175; feedForward = 0.387; }
      else if (yValue > 10) { GoalRPM = 2185; feedForward = 0.390; }
      else if (yValue > 05) { GoalRPM = 2195; feedForward = 0.395; }
      else if (yValue > 00) { GoalRPM = 2260; feedForward = 0.395; }
      
      GoalRPM += -15;
      shooterPID.setSetpoint(GoalRPM);
      
      double t = shooterPID.calculate(Robot.shooter.getVelocity());
      t = Equations.clamp(t, -0.1, 0.1);
      double power = t + feedForward;
      Robot.shooter.spin(power);

      if (shooterPID.atSetpoint() != true) {
        atSpeedTimer.reset();
      }
    }

    if (atSpeedTimer.isFinished() && !emptyThroughShooter.isScheduled()) {
      emptyThroughShooter.schedule();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.intake.setUserControl(true);
    
    Robot.limelight.setUserControl(true);
    Robot.limelight.setLEDs(false);

    if (emptyThroughShooter.isScheduled()) {
      emptyThroughShooter.cancel();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return forceEnd.isFinished();
  }
}
