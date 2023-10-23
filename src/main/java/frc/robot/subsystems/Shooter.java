package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

  private CANSparkMax frontNeoShooter =
      new CANSparkMax(ShooterConstants.frontShooterId, MotorType.kBrushless);
  private CANSparkMax backIndex =
      new CANSparkMax(ShooterConstants.backShooterId, MotorType.kBrushless);

  public void runShooter(double speed) {
    frontNeoShooter.set(speed);
  }

  public void runIndex(double speed) {
    backIndex.set(speed);
  }

  public void ShootCube(double outerSpeed, double innerSpeed) {
    frontNeoShooter.set(outerSpeed);
    backIndex.set(innerSpeed);
  }

  public void IntakeCube(double outerSpeedReversed, double innerOuterSpeedReversed) {

    frontNeoShooter.set(innerOuterSpeedReversed);
    backIndex.set(innerOuterSpeedReversed);
  }
}
