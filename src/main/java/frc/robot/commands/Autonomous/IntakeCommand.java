package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Tools.Limelight;

public class IntakeCommand extends Command{
    private Limelight limelight = RobotContainer.limelight;

    public IntakeCommand() {
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
        return limelight.test();
    }
}
