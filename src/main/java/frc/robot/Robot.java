// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.classes.EasyPov;
import frc.robot.classes.Equations;
import frc.robot.classes.Switch;
import frc.robot.command.shooter.Eject;
import frc.robot.command.shooter.ShootWithVision;
import frc.robot.subsystem.Drivetrain;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.Limelight;
import frc.robot.subsystem.Shooter;
import frc.robot.subsystem.Storage;
import frc.robot.subsystem.Intake.IntakePosition;

public class Robot extends TimedRobot {

  public static Intake intake = new Intake();
  public static Drivetrain drivetrain = new Drivetrain();
  public static Limelight limelight = new Limelight();
  public static Storage storage = new Storage();
  public static Shooter shooter = new Shooter();

  public static Eject ejectStoredBalls = new Eject();
  public static ShootWithVision shootWithVision = new ShootWithVision();

  public static Joystick joy1 = new Joystick(0);
  public static EasyPov storagePov = new EasyPov(new int[] {0, 0, 1, 0, 0, 0, -1, 0}, 0);
  public static Switch storageInputSwitch = new Switch(false);
  public static Switch storageResetSwitch = new Switch(false);

  @Override
  public void robotInit() {}

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // Limelight on/off
    if (joy1.getRawButtonPressed(frc.robot.constant.controllermap.button.Shooter.LIGHTS)) {
      limelight.toggleLEDsUser();
    }
  }

  @Override
  public void autonomousInit() {
    storage.SetDoStorage(true);
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    storage.SetDoStorage(true);
  }

  @Override
  public void teleopPeriodic() {
    // Drive

    double yPow = joy1.getRawAxis(frc.robot.constant.controllermap.axis.Drivetrain.Y_AXIS);
    double zPow = joy1.getRawAxis(frc.robot.constant.controllermap.axis.Drivetrain.Z_AXIS);
    yPow = Equations.exponentialNegativeBelowZero(yPow, 2);
    //zPow = Equations.exponentialNegativeBelowZero(zPow, 2);
    yPow *= 0.7;
    //zPow *= 0.7;
    drivetrain.arcadeDriveTurnThrottle(yPow, zPow);
    
    // Intake up/down
    if (joy1.getRawButtonPressed(frc.robot.constant.controllermap.button.Intake.TOGGLE)) {
      intake.setGoalPositionUser(intake.getGoalPosition() == IntakePosition.Up ? IntakePosition.Down : IntakePosition.Up);
    }

    // Intake conveyor run
    intake.setConveyorPowerPositionLimited(
      Equations.triggersAsJoy(
          frc.robot.constant.controllermap.axis.Intake.CONVEYOR_IN
        , frc.robot.constant.controllermap.axis.Intake.CONVEYOR_OUT
        , joy1)
      );

    // Manual Storage Control
    int povValue = storagePov.getPovValue(joy1.getPOV());
    if (joy1.getRawButton(frc.robot.constant.controllermap.button.Other.ALTERNATE_BUTTON)) {
      if (povValue != 0)
      {
        storage.SetDoStorage(false);
      }
      storage.storagePower(povValue * 3);
      if (storageResetSwitch.flipOnTrue(povValue == 0))
      {
        storage.resetGoal();
      }
    } else {
      if (storageInputSwitch.flipOnTrue(povValue != 0)){
        storage.updateGoalUser(povValue);
      }
    }

    // Toggle Storage
    if (joy1.getRawButtonPressed(frc.robot.constant.controllermap.button.Storage.STOP))
    {
      if (joy1.getRawButton(frc.robot.constant.controllermap.button.Other.ALTERNATE_BUTTON))
      {
        storage.ToggleAutomaticStorage();
      } else {
        storage.ToggleDoStorage();
      }
    }

    // Empty Through Shooter
    if (
         joy1.getRawButtonPressed(frc.robot.constant.controllermap.button.Shooter.START)
      && joy1.getRawButton(frc.robot.constant.controllermap.button.Other.ALTERNATE_BUTTON)
      ) 
    {
      if (!ejectStoredBalls.isScheduled()) {
        ejectStoredBalls.schedule();
      } else {
        ejectStoredBalls.cancel();
      }
    }

    // Shoot With Vision
    if (
         joy1.getRawButtonPressed(frc.robot.constant.controllermap.button.Shooter.START)
      && !joy1.getRawButton(frc.robot.constant.controllermap.button.Other.ALTERNATE_BUTTON)
      ) {
        if (!shootWithVision.isScheduled()) {
          shootWithVision.schedule();
        } else {
          shootWithVision.cancel();
        }
      }
  }

  @Override
  public void disabledInit() {
    storage.SetDoStorage(false);
  }

  @Override
  public void disabledPeriodic() {
    limelight.setLEDs(false);
  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}
