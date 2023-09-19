package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrainContants;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class DriveTrain extends SubsystemBase{
    private MotorController m_frontLeft = new WPI_VictorSPX(DriveTrainContants.frontLeftMotorId);
    private MotorController m_backLeft = new WPI_VictorSPX(DriveTrainContants.backLeftMotorId);
    private MotorController m_frontRight = new WPI_VictorSPX(DriveTrainContants.frontRightMotorId);
    private MotorController m_backRight = new WPI_VictorSPX(DriveTrainContants.backRightMotorId);
    
    private MotorControllerGroup m_leftGroup = new MotorControllerGroup(m_frontLeft, m_backLeft );
    private MotorControllerGroup m_rightGroup = new MotorControllerGroup(m_frontRight, m_backRight);

    private DifferentialDrive driveSystem = new DifferentialDrive(m_leftGroup, m_rightGroup);

    public void arcadeDrive(Joystick driverControllerJoystick) {
        double forwardPower = driverControllerJoystick.getY();
        double turnPower = driverControllerJoystick.getX();
        
        m_leftGroup.setInverted(true);

        driveSystem.arcadeDrive(forwardPower, turnPower);
    }
}
