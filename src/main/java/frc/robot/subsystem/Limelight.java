package frc.robot.subsystem;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

  private boolean limelightOn = false;
  private boolean userControl = true;
    
  @Override
  public void periodic()
  {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(limelightOn ? 0 : 1);
  }
  
  public double getVerOffset()
  {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  }
  
  public double getHorOffset()
  {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  }
  
  /**
   * Toggles the LEDs in the Limelight.
   * @return True if the LED were turned on, false if they were turned off.
   * @author Lucas Brunner
   */
  public boolean toggleLEDs()
  {
    limelightOn = !limelightOn;
    return limelightOn;
  }
  
  /**
   * Toggles the LEDs in the Limelight. If userControl is set to false the input is ignored.
   * @return True if the LED were turned on, false if they were turned off.
   * @author Lucas Brunner
   */
  public boolean toggleLEDsUser()
  {
    if (userControl == true) {
      limelightOn = !limelightOn;
    }
    return limelightOn;
  }

  /**
   * Sets whether the user can control the intake.
   * @param state The state to be set to.
   * @author Lucas Brunner
   */
  public void setUserControl(boolean state) {
    userControl = state;
  }
  
  /**
   * Sets the state of the LEDs on the Limelight.
   * @param state The state the LEDs will be set to.
   * @author Lucas Brunner
   */
  public void setLEDs(boolean state)
  {
    limelightOn = state;
  }

  /**
   * @return True if the LED were turned on, false if they were turned off.
   * @author Lucas Brunner
   */
  public boolean LEDState()
  {
    return limelightOn;
  }
}
