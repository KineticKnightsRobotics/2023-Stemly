// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.sub_Arm;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class arm_AutoClampEject extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final sub_Arm m_subsystem;
  public final double Realspeed;
  public final double m_time;
  public static double t;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public arm_AutoClampEject(sub_Arm subsystem, double speed, double time) {
    m_subsystem = subsystem;
    Realspeed = speed;
    m_time = time;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    t = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.spinClamp(Realspeed);
    t += 1;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.spinClamp(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return t >= m_time;
  }
}
