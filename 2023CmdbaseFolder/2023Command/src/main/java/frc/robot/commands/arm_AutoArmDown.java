// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.sub_Arm;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class arm_AutoArmDown extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final sub_Arm m_subsystem;
  private final double m_liftpos, m_extendpos;
  private final double m_lifttimer;
  private static double timer;
  private static double diff_liftpos, diff_extpos;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public arm_AutoArmDown(sub_Arm subsystem, double lift_pos, double extend_pos, double lift_timer) {
    m_subsystem = subsystem;
    m_liftpos = lift_pos;
    m_extendpos = extend_pos;
    m_lifttimer = lift_timer;
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

    if (timer >= m_lifttimer && m_subsystem.lift_limitSwitch.get() == false) {
        m_subsystem.liftArm(0.95);
    } else if (timer < m_lifttimer) {
        timer = timer + 1; 
    } else {
        m_subsystem.liftArm(0);
    }

    // To determine when the arm should extend
    
    if (m_subsystem.ext_limitSwitch.get() == false) {
        m_subsystem.extendMotor(1);
    } else {
        m_subsystem.extendMotor(0.0);
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
    return (m_subsystem.lift_limitSwitch.get() && m_subsystem.ext_limitSwitch.get()); 
  }
}
