// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Mechanisms;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderSimCollection;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Tools.Parts.PIDGains;
import edu.wpi.first.wpilibj.RobotBase;

public class SwerveModule {
	/** Creates a new SwerveModule. */

	private final  CANSparkFlex driveMotor;
	private final CANSparkMax turningMotor;

	private final CANCoder absoluteEncoder;
	CANCoderSimCollection simCollection;
	private final RelativeEncoder driveEncoder;

	private final SparkPIDController drivePID;
	private final ProfiledPIDController m_turningPIDController;

	public final double angleZero;

	private final String moduleName;
	private Rotation2d _simulatedAbsoluteEncoderRotation2d = new Rotation2d();

	private double m_moduleAngleRadians;
	private SwerveModuleState optimizedState;
	private double angularPIDOutput;
	private double angularFFOutput;
	private double turnOutput;
	private boolean isSim = false;

	SimpleMotorFeedforward turnFeedForward = new SimpleMotorFeedforward(
			ModuleConstants.ksTurning, ModuleConstants.kvTurning);

	public SwerveModule(
			String moduleName,
			int driveMotorChannel,
			int turningMotorChannel,
			int absoluteEncoderPort,
			double angleZero,
			PIDGains angularPID,
			PIDGains drivePID
			) {

		this.moduleName = moduleName;
		this.angleZero = angleZero;

		if(RobotBase.isReal()) {
			isSim = false;
		} else {
			isSim = true;
		}

		// Initialize the motors
		driveMotor = new CANSparkFlex(driveMotorChannel, MotorType.kBrushless);

		if(isSim) {
			//REVPhysicsSim.getInstance().addSparkMax(driveMotor, 2.6f, 5676);
		}
		turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

		if(isSim) {
			REVPhysicsSim.getInstance().addSparkMax(turningMotor, 2.6f, 5676);
		}

		driveMotor.setInverted(true);
		turningMotor.setInverted(false);

		turningMotor.restoreFactoryDefaults();
		driveMotor.restoreFactoryDefaults();

		absoluteEncoder = new CANCoder(absoluteEncoderPort, Constants.kCanivoreCANBusName);
		if(isSim) {
			simCollection = absoluteEncoder.getSimCollection();
			//simCollection = absoluteEncoder.getSimState();
		}
		Timer.delay(1);
		
		MagnetSensorConfigs magnetSensorConfigs = 
		new MagnetSensorConfigs().withMagnetOffset(-1 * angleZero).withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf);

		//absoluteEncoder.getConfigurator().apply(new CANcoderConfiguration().withMagnetSensor(magnetSensorConfigs));

		absoluteEncoder.configFactoryDefault();
		absoluteEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
		absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
		absoluteEncoder.configMagnetOffset(-1 * angleZero);
		absoluteEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10, 100);
		absoluteEncoder.clearStickyFaults();

		driveEncoder = driveMotor.getEncoder();
		driveEncoder.setPositionConversionFactor(ModuleConstants.kdriveGearRatioL3 * ModuleConstants.kwheelCircumference); // meters
		driveMotor.getEncoder().setVelocityConversionFactor(
				ModuleConstants.kdriveGearRatioL3
						* ModuleConstants.kwheelCircumference
						* (1d / 60d)); // meters per second

		// Initialize PID's
		this.drivePID = driveMotor.getPIDController();
		this.drivePID.setP(drivePID.kP);
		this.drivePID.setI(drivePID.kI);
		this.drivePID.setD(drivePID.kD);

		m_turningPIDController = new ProfiledPIDController(
			angularPID.kP,
			angularPID.kI,
			angularPID.kD,
			new TrapezoidProfile.Constraints( // radians/s?
					2 * Math.PI * 600, // theoretical is 5676 RPM -> 94*2pi
					2 * Math.PI * 1200));

		this.drivePID.setFF(ModuleConstants.kDriveFeedForward);

		this.drivePID.setFeedbackDevice(driveMotor.getEncoder());

		this.drivePID.setOutputRange(-1, 1);

		// Configure current limits for motors
		driveMotor.setIdleMode(IdleMode.kBrake);
		turningMotor.setIdleMode(IdleMode.kBrake);
		turningMotor.setSmartCurrentLimit(ModuleConstants.kTurnMotorCurrentLimit);
		driveMotor.setSmartCurrentLimit(ModuleConstants.kDriveMotorCurrentLimit);

		m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

		//SmartDashboard.putNumber(this.moduleName + " Offset", angleZero);
		//SmartDashboard.putString(this.moduleName + " Abs. Status", absoluteEncoder.getLastError().toString());

		ShuffleboardTab swerveTab = Shuffleboard.getTab("Swerve");
		swerveTab.addDouble(moduleName + " Offset", this::getAngleZero);
		swerveTab.addString(moduleName + " Abs. Status", this::getStatus);
	}

	// Returns headings of the module
	public double getAbsoluteHeading() {
		return absoluteEncoder.getAbsolutePosition();
		//return absoluteEncoder.getAbsolutePosition().refresh().getValue();
	}

	public double getDistanceMeters() {
		return driveEncoder.getPosition();
	}

	// Returns current position of the modules
	public SwerveModulePosition getPosition() {

		if(isSim){
			return new SwerveModulePosition(driveEncoder.getPosition(), _simulatedAbsoluteEncoderRotation2d);
		}

		return new SwerveModulePosition(driveEncoder.getPosition(), new Rotation2d(Math.toRadians(absoluteEncoder.getAbsolutePosition())));
	}

	public void setDesiredState(SwerveModuleState desiredState) {

		m_moduleAngleRadians = Math.toRadians(absoluteEncoder.getAbsolutePosition());

		if(isSim) {
			m_moduleAngleRadians = Math.toRadians(desiredState.angle.getDegrees());
			_simulatedAbsoluteEncoderRotation2d = desiredState.angle;
		}

		// Optimize the reference state to avoid spinning further than 90 degrees to
		// desired state
		optimizedState = SwerveModuleState.optimize(
				desiredState,
				new Rotation2d(m_moduleAngleRadians));

		angularPIDOutput = m_turningPIDController.calculate(m_moduleAngleRadians,
				optimizedState.angle.getRadians());

		angularFFOutput = turnFeedForward.calculate(m_turningPIDController.getSetpoint().velocity);

		turnOutput = angularPIDOutput + angularFFOutput;

		turningMotor.setVoltage(turnOutput);		

		if(isSim) {
			drivePID.setReference(
				optimizedState.speedMetersPerSecond,
				CANSparkMax.ControlType.kVoltage
			);
		} else {
			drivePID.setReference(
				optimizedState.speedMetersPerSecond,
				ControlType.kVelocity
			);
		}

		//SmartDashboard.putNumber(this.moduleName + " Optimized Angle", optimizedState.angle.getDegrees());
		//SmartDashboard.putNumber(this.moduleName + " PID", angularPIDOutput);
		//SmartDashboard.putNumber(this.moduleName + " Turn Output", turnOutput);

		/*Logger.getInstance().recordOutput("Motors/DriveMotorCurrentOutput_" + moduleName, driveMotor.getOutputCurrent());
		Logger.getInstance().recordOutput("Motors/DriveMotorTemp_" + moduleName, driveMotor.getMotorTemperature());
		Logger.getInstance().recordOutput("Motors/TurnMotorCurrentOutput_" + moduleName, turningMotor.getOutputCurrent());
		Logger.getInstance().recordOutput("Motors/TurnMotorTemp_" + moduleName, turningMotor.getMotorTemperature());*/

		Logger.recordOutput("Motors/DriveMotorCurrentOutput_" + moduleName, driveMotor.getOutputCurrent());
		Logger.recordOutput("Motors/DriveMotorTemp_" + moduleName, driveMotor.getMotorTemperature());
		Logger.recordOutput("Motors/TurnMotorCurrentOutput_" + moduleName, turningMotor.getOutputCurrent());
		Logger.recordOutput("Motors/TurnMotorTemp_" + moduleName, turningMotor.getMotorTemperature());
	}

	public void resetEncoders() {
		Timer.delay(.1);
		absoluteEncoder.configFactoryDefault();
		Timer.delay(.1);
		absoluteEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
		Timer.delay(.1);
		absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
		Timer.delay(.1);
		absoluteEncoder.configMagnetOffset(-1 * angleZero);
		Timer.delay(.1);
		absoluteEncoder.clearStickyFaults();
	}

	public void stopMotors() {
		driveMotor.stopMotor();
		turningMotor.stopMotor();
	}

	double getAngleZero() {
		return this.angleZero;
	}

	String getStatus() {
		return absoluteEncoder.getLastError().toString();
		//return absoluteEncoder.getMagnetHealth().getValue().name();
	}
}
