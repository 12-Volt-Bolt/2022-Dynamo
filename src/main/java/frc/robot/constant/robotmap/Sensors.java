package frc.robot.constant.robotmap;

public class Sensors {
  public enum SingleWire {
    BALL_DETECTOR(8),
    STORAGE_ENCODER(9),
    STORAGE_RESET(1),
    DIAGNOSTIC_SWITCH(0);

    private int dio;

    SingleWire(int dio) {
      this.dio = dio;
    }

    public int dio() {
      return dio;
    }
  }
}
