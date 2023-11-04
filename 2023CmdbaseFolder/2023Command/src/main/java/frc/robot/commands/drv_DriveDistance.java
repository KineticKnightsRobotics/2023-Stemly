// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.sub_Drive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class drv_DriveDistance extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final sub_Drive m_subsystem;
    private final double m_distance;
    private final double m_speed;
    private static double curr_distance;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public drv_DriveDistance(sub_Drive subsystem, double speed, double distance) {
    m_subsystem = subsystem;
    m_speed = speed;
    m_distance = distance;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.ZeroEncoders();
    m_subsystem.ArcadeDrive(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    curr_distance = (Math.abs(m_subsystem.getDriveEncoder()/Constants.GearRatio.lowDrive)) * (Math.PI*8);
    SmartDashboard.putNumber("current Distance: ", curr_distance);

    if(m_distance - curr_distance > 3){
        m_subsystem.ArcadeDrive(-m_speed, 0);
    } else if (m_distance - curr_distance > 0) {
        m_subsystem.ArcadeDrive(-m_speed * 0.75, 0);
    } else {
        m_subsystem.ArcadeDrive(0, 0);
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.ArcadeDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_distance <= curr_distance);
  }
}
