// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.sub_Drive;
import frc.robot.subsystems.sub_LimeLight;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import javax.lang.model.util.ElementScanner14;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ll_TargetDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final sub_LimeLight m_Limelightsubsystem;
  private final sub_Drive m_Drivesubsystem;
  private final double m_dist;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ll_TargetDrive(sub_LimeLight subsystem, sub_Drive subsystem2, double dist) {
    m_Limelightsubsystem = subsystem;
    m_Drivesubsystem = subsystem2;
    m_dist = dist;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem, subsystem2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_Limelightsubsystem.ll_tv()){
      // Gets the Value for how much the robot should turn based on the center of the target (through limelight)
      //double ll_steer = (m_Limelightsubsystem.ll_ty()/275) * 8;
      double steer = RobotContainer.DriveX();
      double height = 20; //inches


      double angle = Math.toRadians(m_Limelightsubsystem.ll_ty());
      double current = height/(Math.tan(angle));


      SmartDashboard.putNumber("current", current);

      double delta = 0.0;
      //double ll_speed = (m_dist - m_Limelightsubsystem.ll_ta()/100);
      //double ll_
      //SmartDashboard.putNumber("LL_SPEED", ll_speed);
      //SmartDashboard.putNumber("LL_TA", m_Limelightsubsystem.ll_ta());
      if (current > m_dist + 3) {delta = -0.1;}
      if (current < m_dist - 3) {delta = 0.1;}
      //double speed = RobotContainer.DriveY();
      m_Drivesubsystem.ArcadeDrive(delta, steer);

   }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}