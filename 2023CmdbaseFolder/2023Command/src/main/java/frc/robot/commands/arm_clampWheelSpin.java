// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.sub_Arm;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class arm_clampWheelSpin extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final sub_Arm m_subsystem;
  public final double Realspeed;
  private static double timer;
  private final double m_mode;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public arm_clampWheelSpin(sub_Arm subsystem, double speed, double mode) {
    m_subsystem = subsystem;
    Realspeed = speed;
    m_mode = mode; // -1 results in timer mode while other doesnt
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
    if (m_mode == -1){
      timer = timer +1;
    }
    //if (m_subsystem.)
    m_subsystem.spinClamp(Realspeed);

    //m_subsystem.leftmoveClamp(RobotContainer.clampSpeed() * 0.2);
    //m_subsystem.rightmoveClamp(RobotContainer.clampSpeed() * -0.2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.spinClamp(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer > 100;
  }
}
