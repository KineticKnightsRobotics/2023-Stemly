// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.sub_Arm;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class arm_MoveArmDown extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final sub_Arm m_subsystem;
  private final double m_pos;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public arm_MoveArmDown(sub_Arm subsystem, double desired_pos) {
    m_subsystem = subsystem;
    m_pos = desired_pos;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.liftArm(0.9);
    if (m_subsystem.getExtEnc() > 0) {
      m_subsystem.extendMotor(-1);
    } else {
      m_subsystem.extendMotor(0);
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setliftEncZero();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_pos <= m_subsystem.getliftEnc()); 
  }
}
