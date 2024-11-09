package frc.robot;

import java.util.OptionalLong;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Tools.JoystickUtils;
import frc.robot.Tools.Limelight;
import frc.robot.Tools.PhotonVision;
import frc.robot.Tools.Parts.PathBuilder;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autonomous.AimCommand;
import frc.robot.commands.Autonomous.DelayCommand;
import frc.robot.commands.Autonomous.DummyCommand;
import frc.robot.commands.Autonomous.IntakeCommand;
import frc.robot.commands.Autonomous.LockWheelsCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {

	public static final PhotonVision photonVision = new PhotonVision();
	public static final DriveSubsystem driveSubsystem = new DriveSubsystem();
	public static final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
	public static final Limelight limelight = new Limelight();
	//private static final CommandXboxController operatorController = new CommandXboxController(1);

	
	// This is required by pathplanner
	public final static PathBuilder autoBuilder = new PathBuilder();

	private final CommandXboxController driverController = new CommandXboxController(0);
	private final CommandXboxController programmerController = new CommandXboxController(
			OperatorConstants.kProgrammerControllerPort);

	private SendableChooser<Command> autoChooser = new SendableChooser<>();;

	public SendableChooser<Command> getAutoChooser() {
		return autoChooser;
	}

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {

		// Configure the trigger bindings
		configureBindings();

		
		// Register commands to be used in Auto
		NamedCommands.registerCommand("LockWheels", new LockWheelsCommand(true));
		NamedCommands.registerCommand("DisableLockWheels", new LockWheelsCommand(false));
		NamedCommands.registerCommand("Intake", new IntakeCommand());
		NamedCommands.registerCommand("Delay500", new DelayCommand(OptionalLong.of(500)));

		if(Constants.kEnablePhotonVision) {
			NamedCommands.registerCommand("Aim", new AimCommand(photonVision));
		} else {
			NamedCommands.registerCommand("Aim", new DummyCommand());
		}
		
		// This creates the chooser from the autos built in Autonomous
		autoChooser = AutoBuilder.buildAutoChooser();

		// Add the chooser to the Shuffleboard to select which Auo to run
		Shuffleboard.getTab("Autonomous").add("Auto", autoChooser);

	}

	private void configureBindings() {

		// Always point the robot at the target
		//operatorController.button(2).onTrue(
		/*operatorController.leftTrigger().onTrue(
			Commands.parallel(new ChassisAimCommand(), new ArmAimCommand())			
		);*/

		//operatorController.button(3).whileTrue(new RunCommand(() -> driveSubsystem.goToPose(Constants.PoseDefinitions.kFieldPoses.AMPLIFIER)));
		//operatorController.button(4).whileTrue(new RunCommand(() -> driveSubsystem.goToPose(Constants.PoseDefinitions.kFieldPoses.SOURCE)));

		// Swerve Drive method is set as default for drive subsystem
		driveSubsystem.setDefaultCommand(

				new RunCommand(() -> driveSubsystem.drive(
					-JoystickUtils.processJoystickInput(driverController.getLeftY()),
					-JoystickUtils.processJoystickInput(driverController.getLeftX()),
					JoystickUtils.processJoystickInput(driverController.getRightX())
				),
				driveSubsystem
			)
		);
	}

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}
}