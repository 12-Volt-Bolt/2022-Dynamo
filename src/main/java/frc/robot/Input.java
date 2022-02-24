package frc.robot;

public class Input {
  private edu.wpi.first.wpilibj.Joystick controller;

  // Driving
  public double driveZAxis() { return controller.getRawAxis(frc.robot.constant.controllermap.axis.Drivetrain.Z_AXIS); }
  public double driveYAxis() { return -controller.getRawAxis(frc.robot.constant.controllermap.axis.Drivetrain.Y_AXIS); }
  
  // Intake
  public boolean intakeToggle() { return controller.getRawButton(frc.robot.constant.controllermap.button.Intake.TOGGLE); }
  public double intakePower() { 
    return triggersAsJoy(
      controller.getRawAxis(frc.robot.constant.controllermap.axis.Intake.CONVEYOR_IN), 
      controller.getRawAxis(frc.robot.constant.controllermap.axis.Intake.CONVEYOR_OUT)
      ); 
  }
  
  // Climber
  public boolean deployLifter() { return controller.getRawButton(frc.robot.constant.controllermap.button.Climber.DEPLOY); }
  public double ClimberPower() { 
    return triggersAsJoy(
      controller.getRawAxis(frc.robot.constant.controllermap.axis.Climber.RETRACT), 
      controller.getRawAxis(frc.robot.constant.controllermap.axis.Climber.RELEASE)
      ); 
  }

  // Shooter
  public boolean emptySorageThroughShooter() { return startShooter() && controller.getRawButton(frc.robot.constant.controllermap.button.Other.ALTERNATE_BUTTON); }
  public boolean startShooter() { return controller.getRawButton(frc.robot.constant.controllermap.button.Shooter.START); }
  public boolean toggleLights() { return controller.getRawButton(frc.robot.constant.controllermap.button.Shooter.LIGHTS); }

  // Storage
  public boolean toggleStorageSensor() { return storageCancel() && controller.getRawButton(frc.robot.constant.controllermap.button.Other.ALTERNATE_BUTTON); }
  public boolean storageCancel() { return controller.getRawButton(frc.robot.constant.controllermap.button.Storage.STOP); }
  public int storageManual() {
    int output = 0;
    switch (controller.getPOV(frc.robot.constant.controllermap.POV.Storage)) {
      case 90:
        output = 1;
        break;
      case 270:
        output = -1;
        break;
    
      default:
        break;
    }
    return output;
  }
  
  /**
   * Returns a -1 to 1 range based of the position of the left and right triggers. 
   * 
   * @param controller
   * 
   * @return If the left trigger is down: left trigger value, if the right trigger is down: right trigger value, else 0.
   */
  private static double triggersAsJoy(double value1, double value2) {
    double output = 0;
    if (value1 == 0 && value2 == 0) 
    {
    } else if (value1 > 0) {
        output = -value1;
    } else if (value2 > 0) {
        output = value2;
    }
    return output;
  }
}
