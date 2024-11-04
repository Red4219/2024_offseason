package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command{
    private IntakeSubsystem intakeSubsystem = RobotContainer.intakeSubsystem;

    public IntakeCommand() {

        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.hasItem();
    }
}
