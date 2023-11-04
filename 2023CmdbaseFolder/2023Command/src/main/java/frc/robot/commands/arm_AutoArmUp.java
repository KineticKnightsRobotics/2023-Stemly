// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.sub_Arm;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class arm_AutoArmUp extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final sub_Arm m_subsystem;
  private final double m_liftpos, m_extendpos;
  private final double m_extendtimer;
  private static double timer;
  private static double diff_liftpos, diff_extpos;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public arm_AutoArmUp(sub_Arm subsystem, double lift_pos, double extend_pos, double extend_timer) {
    m_subsystem = subsystem;
    m_liftpos = lift_pos;
    m_extendpos = extend_pos;
    m_extendtimer = extend_timer;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    diff_liftpos = Math.abs(m_liftpos) - Math.abs(m_subsystem.getliftEnc());
    diff_extpos = Math.abs(m_extendpos) - Math.abs(m_subsystem.getExtEnc());

    if (diff_liftpos > 5) {
        m_subsystem.liftArm(-0.95);
    } else if (diff_liftpos > 0) {
        m_subsystem.liftArm(-0.25);
    } else {
        m_subsystem.liftArm(0.0);
    }
    // To determine when the arm should extend
    if (timer >= m_extendtimer && Math.abs(m_subsystem.getExtEnc()) < Math.abs(m_extendpos)) {
        if (diff_extpos > 15) {
            m_subsystem.extendMotor(-1);
        } else {
            m_subsystem.extendMotor(-0.3);
        }
    } else if (timer < m_extendtimer) {
        timer = timer + 1; 
    } else {
        m_subsystem.extendMotor(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.liftArm(0.0);
    m_subsystem.extendMotor(0.0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(m_liftpos) <= Math.abs(m_subsystem.getliftEnc()) && Math.abs(m_extendpos) <= Math.abs(m_subsystem.getExtEnc())); 
  }
}
