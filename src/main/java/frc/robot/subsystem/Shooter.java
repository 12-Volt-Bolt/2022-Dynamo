package frc.robot.subsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  
  private final CANSparkMax shooter1 = new CANSparkMax(frc.robot.constant.robotmap.motor.Shooter.RIGHT, MotorType.kBrushless);
  private final CANSparkMax shooter2 = new CANSparkMax(frc.robot.constant.robotmap.motor.Shooter.LEFT, MotorType.kBrushless);

  /**
   * Sets the power of the shooter wheels.
   * @param power The power value the wheels will be set to.
   * @author Lucas Brunner
   */
  public void spin(double power)
  {
    shooter1.set(-power);
    shooter2.set(+power);
  }
  
  /**
   * Sets the power of the shooter wheels to 0.
   * @author Lucas Brunner
   */
  public void stop()
  {
    shooter1.set(0);
    shooter2.set(0);
  }
  
  /**
   * @return The rpm of the shooter wheels.
   * @author Lucas Brunner
   */
  public double getVelocity()
  {
    return shooter1.getEncoder().getVelocity();
  }
}
