package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.tools.Equations;

public class Intake extends SubsystemBase {
  public enum IntakePosition {
      Up (3680)
    , Down (-1770);

    int position;

    private IntakePosition(int position) {
      this.position = position;
    }

    public int GetPosition() {
      return position;
    }
  }

  private final VictorSPX conveyorMotor = new VictorSPX(frc.robot.constant.robotmap.motor.Intake.CONVEYOR);
  private final TalonSRX armMotor = new TalonSRX(frc.robot.constant.robotmap.motor.Intake.ARTICULATION);

  private PIDController PID = new PIDController(.075, 0, 0);
  private IntakePosition currentPosition = IntakePosition.Up;
  private boolean userControl = true;
  
  public Intake() {
    armMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    PID.setTolerance(20);
  }

  /**
   * Sets the power of the conveyor. Max value is 1, min value is -1.
   * @param power Power of the conveyor.
   * @author Lucas Brunner
   */
  public void setConveyorPower(double power) {
    conveyorMotor.set(VictorSPXControlMode.PercentOutput, power);
  }

  /**
   * Sets the power of the conveyor. Max value is 1, min value is -1. Sets power to 0 if the intake is up.
   * @param power Power of the conveyor.
   * @author Lucas Brunner
   */
  public void setConveyorPowerPositionLimited(double power) {
    if (currentPosition == IntakePosition.Down) {
      conveyorMotor.set(VictorSPXControlMode.PercentOutput, power);
    } else {
      stopConveyor();
    }
  }

  /**
   * Sets the power of the conveyor's motor to 0.
   * @author Lucas Brunner
   */
  public void stopConveyor() {
    setConveyorPower(0);
  }
  
  /**
   * Returns the value of the intake's encoder.
   * @return The value of the intake's encoder.
   * @author Lucas Brunner
   */
  public double getArmPos() {
    return armMotor.getSelectedSensorPosition();
  }

  /**
   * Sets the power of the arm's movement motor. Max value is 1, min value is -1.
   * @param power Power of the arm's movement motor.
   * @author Lucas Brunner
   */
  public void setArmPower(double power) {
    armMotor.set(TalonSRXControlMode.PercentOutput, power);
  }

  /**
   * Sets the power of the arm's motor to 0.
   * @author Lucas Brunner
   */
  public void stopArm() {
    setArmPower(0);
  }

  /**
   * Sets goal position state of the intake.
   * @param goalPosition Goal intake position state.
   * @author Lucas Brunner
   */
  public void setGoalPosition(IntakePosition goalPosition) {
    currentPosition = goalPosition;
  }

  /**
   * Sets goal position state of the intake. If userControl is set to false the input is ignored.
   * @param goalPosition Goal intake position state.
   * @author Lucas Brunner
   */
  public void setGoalPositionUser(IntakePosition goalPosition) {
    if (userControl == true) {
      currentPosition = goalPosition;
    }
  }

  /**
   * Returns the current intake position state.
   * @author Lucas Brunner
   */
  public IntakePosition getGoalPosition() {
    return currentPosition;
  }

  /**
   * Sets whether the user can control the intake.
   * @param state The state to be set to.
   * @author Lucas Brunner
   */
  public void setUserControl(boolean state) {
    userControl = state;
  }

  @Override
  public void periodic() {
    PID.setSetpoint(currentPosition.GetPosition());

    double moveSpeed = getArmPos();
    moveSpeed = PID.calculate(moveSpeed);
    moveSpeed = Equations.clamp(moveSpeed / 100, -0.5, 0.5);

    setArmPower(moveSpeed);
  }
}
