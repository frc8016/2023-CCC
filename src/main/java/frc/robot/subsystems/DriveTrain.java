package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrainConstants;

public class DriveTrain extends SubsystemBase {
  private CANSparkMax m_frontLeft =
      new CANSparkMax(DriveTrainConstants.frontLeftMotorID, MotorType.kBrushless);
  private CANSparkMax m_backLeft =
      new CANSparkMax(DriveTrainConstants.backLeftMotorID, MotorType.kBrushless);
  private CANSparkMax m_frontRight =
      new CANSparkMax(DriveTrainConstants.frontRightMotorID, MotorType.kBrushless);
  private CANSparkMax m_backRight =
      new CANSparkMax(DriveTrainConstants.backRightMotorID, MotorType.kBrushless);

  private MotorControllerGroup m_leftGroup = new MotorControllerGroup(m_frontLeft, m_backLeft);
  private MotorControllerGroup m_rightGroup = new MotorControllerGroup(m_frontRight, m_backRight);

  private DifferentialDrive driveSystem = new DifferentialDrive(m_leftGroup, m_rightGroup);

  public void arcadeDrive(double speed, double rotation) {
    driveSystem.arcadeDrive(-speed, -rotation);

    m_leftGroup.setInverted(true);
    m_rightGroup.setInverted(false);
  }
}
