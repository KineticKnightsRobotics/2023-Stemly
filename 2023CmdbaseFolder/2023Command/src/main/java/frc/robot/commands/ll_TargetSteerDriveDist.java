// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.sub_Drive;
import frc.robot.subsystems.sub_LimeLight;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ll_TargetSteerDriveDist extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final sub_LimeLight m_lime;
    private final sub_Drive m_drive;
    private final double m_distance;
    private final double m_speed;
    private static double CurrentDistance;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ll_TargetSteerDriveDist(sub_LimeLight subsystem,sub_Drive subsystem2,double speed, double distance) {
    m_lime = subsystem;
    m_drive = subsystem2;
    m_distance = distance;
    m_speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem,subsystem2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.ZeroEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    CurrentDistance = (Math.abs(m_drive.getDriveEncoder()/Constants.GearRatio.lowDrive)) * (Math.PI*8);
    double ll_steer = m_lime.ll_tx()/70;
    
    // if distance left to move is greater than current distance  then move forward --> current distance goes up and you  move forward
    if (m_distance - CurrentDistance > 0){
        m_drive.ArcadeDrive(-m_speed, ll_steer);
    }
      else {
        m_drive.ArcadeDrive(0, ll_steer);;
    }
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.ArcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (CurrentDistance > m_distance && (m_lime.ll_tx() < 1 || m_lime.ll_tx() < -1));
  }
}
