package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.revrobotics.REVPhysicsSim;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
	private Command autonomousCommand;

	private RobotContainer robotContainer;
	private REVPhysicsSim simulator;

	@Override
	public void robotInit() {

		Pathfinding.setPathfinder(new LocalADStarAK());
		Logger.addDataReceiver(new NT4Publisher());

		//PathPlannerServer.startServer(5811);

		robotContainer = new RobotContainer();
		DriverStation.silenceJoystickConnectionWarning(true);

		Logger.start();
	}

	@Override
	public void robotPeriodic() {

		CommandScheduler.getInstance().run();

		Logger.recordOutput("Power/BatteryVoltage", RobotController.getBatteryVoltage());
		Logger.recordOutput("Power/IsBrownedOut", RobotController.isBrownedOut());
		Logger.recordOutput("CAN/ReceiveErrorCount", RobotController.getCANStatus().receiveErrorCount);
		Logger.recordOutput("CAN/TransmitErrorCount", RobotController.getCANStatus().transmitErrorCount);
		Logger.recordOutput("CAN/PercentBusUtilization", RobotController.getCANStatus().percentBusUtilization);
	}

	@Override
	public void disabledInit() {
		
	}

	@Override
	public void disabledPeriodic() {
		
	}

	@Override
	public void autonomousInit() {

		autonomousCommand = robotContainer.getAutonomousCommand();

		if (autonomousCommand != null) {
			autonomousCommand.schedule();
		}
	}

	@Override
	public void autonomousPeriodic() {

	}

	@Override
	public void teleopInit() {
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
	}

	@Override
	public void teleopPeriodic() {
		
	}

	@Override
	public void testInit() {
		// Cancels all running commands at the start of test mode.
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testPeriodic() {
	}

	@Override
	public void simulationInit() {
		simulator = REVPhysicsSim.getInstance();
		simulator.run();
	}

	@Override
	public void simulationPeriodic() {
		REVPhysicsSim.getInstance().run();
	}
}
