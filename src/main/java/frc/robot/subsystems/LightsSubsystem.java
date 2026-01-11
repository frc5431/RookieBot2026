package frc.robot.subsystems;

import com.lumynlabs.devices.ConnectorX;
import com.lumynlabs.devices.ConnectorXAnimate;
import com.lumynlabs.connection.usb.USBPort;
import com.lumynlabs.domain.led.Animation;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightsSubsystem extends SubsystemBase {
  private static final USBPort USB_PORT = USBPort.kUSB1;
  private static final String MATRIX_ZONE = "matrix-32x8";

  public enum Mode { IDLE, INTAKING, LAUNCHING, SPINUP, ERROR }

  private final ConnectorX device = new ConnectorX();
  private boolean connected = false;

  private Mode wanted = Mode.IDLE;
  private Mode applied = null;

  private double lastReconnectAttempt = 0.0;

  public LightsSubsystem() {
    connected = device.Connect(USB_PORT);
    lastReconnectAttempt = Timer.getFPGATimestamp();
  }

  public void setMode(Mode mode) {
    wanted = mode;
  }

  @Override
  public void periodic() {
    maintainConnection();
    if (!connected) return;

    if (wanted != applied) {
      apply(wanted);
      applied = wanted;
    }
  }

  private void apply(Mode mode) {
    switch (mode) {
      case IDLE:
        device.leds.SetColor(MATRIX_ZONE, new Color(new Color8Bit(0, 0, 40)));
        break;

      case INTAKING:
        device.leds.SetAnimation(Animation.Chase)
          .ForZone(MATRIX_ZONE)
          .WithColor(new Color(new Color8Bit(0, 255, 0)))
          .WithDelay(Units.Milliseconds.of(40))
          .Reverse(false)
          .RunOnce(false);
        break;

      case SPINUP:
        device.leds.SetAnimation(Animation.Chase)
          .ForZone(MATRIX_ZONE)
          .WithColor(new Color(new Color8Bit(255, 180, 0)))
          .WithDelay(Units.Milliseconds.of(35))
          .Reverse(true)
          .RunOnce(false);
        break;

      case LAUNCHING:
        device.leds.SetAnimation(Animation.Chase)
          .ForZone(MATRIX_ZONE)
          .WithColor(new Color(new Color8Bit(255, 0, 255)))
          .WithDelay(Units.Milliseconds.of(25))
          .Reverse(false)
          .RunOnce(false);
        break;

      case ERROR:
        device.leds.SetColor(MATRIX_ZONE, new Color(new Color8Bit(255, 0, 0)));
        break;
    }
  }

  private void maintainConnection() {
    if (connected) return;

    double now = Timer.getFPGATimestamp();
    if (now - lastReconnectAttempt < 2.0) return;

    lastReconnectAttempt = now;
    connected = device.Connect(USB_PORT);
  }
}
