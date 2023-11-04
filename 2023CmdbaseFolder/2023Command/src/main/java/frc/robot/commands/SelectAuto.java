// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class SelectAuto extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final String autoType;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SelectAuto(String type) {
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(subsystem);
    autoType = type;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(autoType == "lowcone_balance")
    RobotContainer.m_chooser.setDefaultOption("DEFAULT, Low cone balance", RobotContainer.c_lowconebalance);
    else if (autoType == "lowcone"){
    //RobotContainer.m_chooser.setDefaultOption("DEFAULT, Low cone", RobotContainer.c_lowcone);

    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
