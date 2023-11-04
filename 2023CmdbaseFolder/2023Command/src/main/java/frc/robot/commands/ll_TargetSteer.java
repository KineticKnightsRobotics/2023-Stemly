// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
//import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.sub_Drive;
import frc.robot.subsystems.sub_LimeLight;

//import javax.lang.model.util.ElementScanner14;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ll_TargetSteer extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final sub_LimeLight m_Limelightsubsystem;
  private final sub_Drive m_Drivesubsystem;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ll_TargetSteer(sub_LimeLight subsystem, sub_Drive subsystem2) {
    m_Limelightsubsystem = subsystem;
    m_Drivesubsystem = subsystem2;
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
    /*if (m_Limelightsubsystem.ll_tv()){
      // Gets the Value for how much the robot should turn based on the center of the target (through limelight)
      double ll_steer = (m_Limelightsubsystem.ll_ty()/275) * 8;
      //double ll_steer = (m_Limelightsubsystem.ll_ty()/100);
      if (ll_steer > 0.1) {ll_steer = -0.1;}
      if (ll_steer <-0.1) {ll_steer =0.1;}
      double speed = RobotContainer.DriveY();

      m_Drivesubsystem.ArcadeDrive(speed, ll_steer);

   }*/

   // gets tx value and divides by 70 to get a reasonable turning speed for the robot.
   double ll_steer = (m_Limelightsubsystem.ll_tx()/70);
   double speed = RobotContainer.DriveY();


  
   m_Drivesubsystem.ArcadeDrive(speed * 0.5,ll_steer);

  /*if (m_Limelightsubsystem.ll_tx() > 0) { ll_steer = -0.1;}
  else if (m_Limelightsubsystem.ll_tx() < 0){ll_steer = 0.1;} */


    // Gets the Value for how much the robot should turn based on the center of the target (through limelight)


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