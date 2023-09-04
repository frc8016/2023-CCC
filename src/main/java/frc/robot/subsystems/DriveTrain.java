// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveBase. */
  public DriveTrain() {
    MotorController m_frontLeft = new WPI_VictorSPX(7);
    MotorController m_backLeft = new WPI_VictorSPX(6);
    MotorController m_frontRight = new WPI_VictorSPX(8);
    MotorController m_backRight = new WPI_VictorSPX(5);

    MotorControllerGroup m_left = new MotorControllerGroup(m_frontLeft, m_backLeft);
    MotorControllerGroup m_right = new MotorControllerGroup(m_frontRight, m_backRight);

    DifferentialDrive m_robotDrive = new DifferentialDrive(m_left, m_right);

    Joystick m_controller = new Joystick(0);
  }

  /* (non-Javadoc)
   * @see edu.wpi.first.wpilibj2.command.Subsystem#periodic()
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    DifferentialDrive.arcadeDriveIK(0, 0, false);
  }
}
