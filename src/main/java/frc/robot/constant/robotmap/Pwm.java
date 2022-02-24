package frc.robot.constant.robotmap;

public class Pwm {
  public enum Servo {
    LIFTARM_RELEASE(0);

    private int pwm;

    Servo(int pwm) {
      this.pwm = pwm;
    }

    public int pwm() {
      return pwm;
    }
  }
}
