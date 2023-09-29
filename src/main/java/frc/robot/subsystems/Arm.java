package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  private CANSparkMax armLeft = new CANSparkMax(ArmConstants.leftArmMotorID, MotorType.kBrushless);
  private CANSparkMax armRight =
      new CANSparkMax(ArmConstants.rightArmMotorID, MotorType.kBrushless);
  private final Encoder m_reletivEncoder =
      new Encoder(ArmConstants.RELETIVE_ENCODER_A, ArmConstants.RELETIVE_ENCODER_B);
  private final DutyCycleEncoder m_absoluteEncoder =
      new DutyCycleEncoder(ArmConstants.ABSOLUTE_ENCODER_PORT);

  public Arm() {
    m_reletivEncoder.setDistancePerPulse(ArmConstants.reletiveEncoderDistancePerPulse);
    m_absoluteEncoder.setDistancePerRotation(ArmConstants.dutyCycleEncoderDistancePerRotation);
  }

  public void moveArm(double armSpeed) {
    armLeft.set(armSpeed);
    armRight.set(-armSpeed);
  }

  public double getRelitiveDistance() {
    double distance = m_reletivEncoder.getDistance();
    SmartDashboard.putNumber("Relitive arm distance", distance);
    return distance;
  }

  public double getAbsoluteDistance() {
    double distance = m_absoluteEncoder.getDistance();
    SmartDashboard.putNumber("Absolute arm distance", distance);
    return distance;
  }

  @Override
  public void periodic() {
    getAbsoluteDistance();
    getRelitiveDistance();
  }
}
