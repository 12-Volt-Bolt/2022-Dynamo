package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.tools.Switch;

public class Storage extends SubsystemBase {
  private final VictorSPX storageMotor = new VictorSPX(frc.robot.constant.robotmap.motor.Storage.WHEEL);
  private final VictorSPX feederMotor = new VictorSPX(frc.robot.constant.robotmap.motor.Storage.FEEDER);
  private final DutyCycleEncoder encoder = new DutyCycleEncoder(frc.robot.constant.robotmap.sensor.Storage.STORAGE_ENCODER);
  private final DigitalInput ballDetector = new DigitalInput(frc.robot.constant.robotmap.sensor.Storage.BALL_DETECTOR);
  private final DigitalInput storageReset = new DigitalInput(frc.robot.constant.robotmap.sensor.Storage.STORAGE_RESET);
  
  private double offSet = 0.7;
  private int goal = 0;
  private boolean automaticStorage = false;
  private boolean doStorage = false;
  private boolean userStorage = true;

  private Switch detectorSwitch = new Switch(false);
  private PIDController storagePid = new PIDController(4, 10, 0.2);
  
  public Storage() {
    encoder.setDistancePerRotation(1);
  }
  
  @Override
  public void periodic() {
    if (storageReset.get() == false) {
      offSet = encoder.get() - Math.round(encoder.get());
    }

    if (automaticStorage == true && detectorSwitch.flipOnTrue(getBallDetected()) == true && doStorage == true) {
      ++goal;
    }

    if (doStorage == true) {
      storagePid.setSetpoint(goal);

      double power = storagePid.calculate(getEncoderPosition());  
      storagePower(power);
    }
  }

  public void resetGoal() {
    goal = (int) Math.round(getEncoderPosition());
  }
  
  /**
   * Set the location (rotation value) for the storage wheel to move to.
   * @param newGoal New rotation value.
   */
  public void setGoal(int newGoal) {
    goal = newGoal;
  }
  
  /**
   * Adds a value to the current storage wheel location goal.
   * 
   * @param amount The amount to add to the goal.
   */
  public void updateGoal(int amount) {
    if (doStorage == true) {
      goal += amount;
    }
  }

  public int getGoal() {
    return goal;
  }
  
  public void updateGoalUser(int amount) {
    if (userStorage){
      updateGoal(amount);
    }
  }
  
  public void updateOffsetUser(int amount) {
    if (userStorage){
      offSet += amount * 0.05;
    }
  }

  public void resetOffest() {

  }

  public void setUserStorage(boolean state) {
    userStorage = state;
  }
  
  public double getEncoderPosition() {
    return encoder.getDistance() - offSet;
  }
  
  public boolean getBallDetected() {
    return !ballDetector.get();
  }

  public void SetAutomaticStorage(boolean state) {
    automaticStorage = state;
  }

  public void ToggleAutomaticStorage() {
    automaticStorage = !automaticStorage;
  }

  public void SetDoStorage(boolean state) {
    doStorage = state;
  }

  public void ToggleDoStorage() {
    doStorage = !doStorage;
  }

  /**
   * Sets the power of the storage wheel's motor.
   * @param power Storage wheel motor's percentage output.
   */
  public void storagePower(double power) {
    storageMotor.set(VictorSPXControlMode.PercentOutput, power * 0.25);
  }
  
  /*
  public void storageStop() {
    storageMotor.set(VictorSPXControlMode.PercentOutput, 0);
  }
  */
  
  /**
   * Sets the power of the feeder wheel.
   * @param power Feeder wheel's percentage output.
   */
  public void setFeederMotor(double power) {
    feederMotor.set(VictorSPXControlMode.PercentOutput, -Math.abs(power));
  }
}
