// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Mechanisms;

import java.util.EnumSet;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.hardware.CANcoder;
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
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Tools.Parts.PIDGains;
import edu.wpi.first.wpilibj.RobotBase;

public class SwerveModule {
	/** Creates a new SwerveModule. */
	//private final CANBus kCANBus = new CANBus();

	private final CANSparkFlex driveMotor;
	//private final  CANSparkMax driveMotor;
	private final CANSparkMax turningMotor;
	private final CANcoder cancoder;
	private final RelativeEncoder driveEncoder;

	private final SparkPIDController drivePID;
	private final ProfiledPIDController m_turningPIDController;

	public final double angleZero;

	private final String moduleName;
	private Rotation2d _simulatedAbsoluteEncoderRotation2d = null;

	private double m_moduleAngleRadians;
	private SwerveModuleState optimizedState;
	private double angularPIDOutput;
	private double angularFFOutput;
	private double turnOutput;
	private boolean isSim = false;
	private ShuffleboardTab swerveTab = null;
	private NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
	

	SimpleMotorFeedforward turnFeedForward = new SimpleMotorFeedforward(
			ModuleConstants.ksTurning, ModuleConstants.kvTurning);

	public SwerveModule(
			String moduleName,
			int driveMotorChannel,
			int turningMotorChannel,
			int absoluteEncoderPort,
			double angleZero,
			PIDGains angularPID,
			PIDGains drivePID,
			boolean invertTurningMotor,
			boolean invertDriveMotor
			) {

		this.moduleName = moduleName;
		this.angleZero = angleZero;

		if(RobotBase.isReal()) {
			isSim = false;
		} else {
			isSim = true;
			_simulatedAbsoluteEncoderRotation2d = new Rotation2d(0.0);
		}

		// Initialize the motors
		driveMotor = new CANSparkFlex(driveMotorChannel, MotorType.kBrushless);
		//driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);

		if(isSim) {
			//REVPhysicsSim.getInstance().addSparkMax(driveMotor, 2.6f, 5676);
		}
		turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

		if(isSim) {
			REVPhysicsSim.getInstance().addSparkMax(turningMotor, 2.6f, 5676);
		}

		turningMotor.restoreFactoryDefaults();
		driveMotor.restoreFactoryDefaults();

		driveMotor.setInverted(invertDriveMotor);
		turningMotor.setInverted(invertTurningMotor);
		cancoder = new CANcoder(absoluteEncoderPort, Constants.kCanivoreCANBusName);
		
		Timer.delay(1);
		
		
		cancoder.clearStickyFaults();

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
		driveMotor.setIdleMode(IdleMode.kCoast);
		turningMotor.setIdleMode(IdleMode.kBrake);
		turningMotor.setSmartCurrentLimit(ModuleConstants.kTurnMotorCurrentLimit);
		driveMotor.setSmartCurrentLimit(ModuleConstants.kDriveMotorCurrentLimit);

		//m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
		//m_turningPIDController.enableContinuousInput(0, 1);
		m_turningPIDController.enableContinuousInput(0, Math.toRadians(360));

		if(Constants.debugDriveTrain == true) {
			// Swerve tab stuff
			swerveTab = Shuffleboard.getTab("Swerve");
			swerveTab.addDouble(moduleName + " Absolute", this::getAbsoluteHeading);
			//swerveTab.addDouble("FR Absolute", frontRight::getAbsoluteHeading);
			//swerveTab.addDouble("RL Absolute", rearLeft::getAbsoluteHeading);
			//swerveTab.addDouble("RR Absolute", rearRight::getAbsoluteHeading);
			//swerveTab.addDouble("FL Meters", frontLeft::getDistanceMeters);
			//swerveTab.addDouble("FR Meters", frontRight::getDistanceMeters);
			//swerveTab.addDouble("RL Meters", rearLeft::getDistanceMeters);
			//swerveTab.addDouble("RR Meters", rearRight::getDistanceMeters);
			
			swerveTab.addDouble(moduleName + " Offset", this::getAngleZero);
			swerveTab.addString(moduleName + " Abs. Status", this::getStatus);
		}

		networkTableInstance.getEntry(Constants.ModuleConstants.kTurningPID_P).setDouble(angularPID.kP);
		networkTableInstance.getEntry(Constants.ModuleConstants.kTurningPID_I).setDouble(angularPID.kI);
		networkTableInstance.getEntry(Constants.ModuleConstants.kTurningPID_D).setDouble(angularPID.kD);

		networkTableInstance.getEntry(Constants.ModuleConstants.kDrivePID_P).setDouble(drivePID.kP);
		networkTableInstance.getEntry(Constants.ModuleConstants.kDrivePID_I).setDouble(drivePID.kI);
		networkTableInstance.getEntry(Constants.ModuleConstants.kDrivePID_D).setDouble(drivePID.kD);

		// Setup listeners to listen for changes to the values

		networkTableInstance.addListener(
			networkTableInstance.getEntry(Constants.ModuleConstants.kTurningPID_P),
			EnumSet.of(NetworkTableEvent.Kind.kValueAll),
			event -> {
				m_turningPIDController.setP(event.valueData.value.getDouble());
			}
		);

		networkTableInstance.addListener(
			networkTableInstance.getEntry(Constants.ModuleConstants.kTurningPID_I),
			EnumSet.of(NetworkTableEvent.Kind.kValueAll),
			event -> {
				m_turningPIDController.setI(event.valueData.value.getDouble());
			}
		);

		networkTableInstance.addListener(
			networkTableInstance.getEntry(Constants.ModuleConstants.kTurningPID_D),
			EnumSet.of(NetworkTableEvent.Kind.kValueAll),
			event -> {
				m_turningPIDController.setD(event.valueData.value.getDouble());
			}
		);

		networkTableInstance.addListener(
			networkTableInstance.getEntry(Constants.ModuleConstants.kDrivePID_P),
			EnumSet.of(NetworkTableEvent.Kind.kValueAll),
			event -> {
				this.drivePID.setP(event.valueData.value.getDouble());
			}
		);

		networkTableInstance.addListener(
			networkTableInstance.getEntry(Constants.ModuleConstants.kDrivePID_I),
			EnumSet.of(NetworkTableEvent.Kind.kValueAll),
			event -> {
				this.drivePID.setI(event.valueData.value.getDouble());
			}
		);

		networkTableInstance.addListener(
			networkTableInstance.getEntry(Constants.ModuleConstants.kDrivePID_D),
			EnumSet.of(NetworkTableEvent.Kind.kValueAll),
			event -> {
				this.drivePID.setD(event.valueData.value.getDouble());
			}
		);
	}

	// Returns headings of the module
	public double getAbsoluteHeading() {
		return (cancoder.getAbsolutePosition().refresh().getValue() * 360);
	}

	public double getDistanceMeters() {
		return driveEncoder.getPosition();
	}

	// Returns current position of the modules
	public SwerveModulePosition getPosition() {

		if(isSim){
			return new SwerveModulePosition(driveEncoder.getPosition(), _simulatedAbsoluteEncoderRotation2d);
		}

		return new SwerveModulePosition(driveEncoder.getPosition(), new Rotation2d(Math.toRadians(cancoder.getAbsolutePosition().refresh().getValue() * 360)));
	}

	// Sets the position of the swerve module
	public void setDesiredState(SwerveModuleState desiredState) {

		m_moduleAngleRadians = Math.toRadians(cancoder.getAbsolutePosition().refresh().getValue() * 360);

		if(isSim) {
			m_moduleAngleRadians = Math.toRadians(desiredState.angle.getDegrees());
			_simulatedAbsoluteEncoderRotation2d = desiredState.angle;
		}

		// Optimize the reference state to avoid spinning further than 90 degrees to
		// desired state
		optimizedState = SwerveModuleState.optimize(
				desiredState,
				Rotation2d.fromRadians(m_moduleAngleRadians));

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

		Logger.recordOutput("Motors/DriveMotorCurrentOutput_" + moduleName, driveMotor.getOutputCurrent());
		Logger.recordOutput("Motors/DriveMotorTemp_" + moduleName, driveMotor.getMotorTemperature());
		Logger.recordOutput("Motors/TurnMotorCurrentOutput_" + moduleName, turningMotor.getOutputCurrent());
		Logger.recordOutput("Motors/TurnMotorTemp_" + moduleName, turningMotor.getMotorTemperature());
	}

	public void resetEncoders() {
		/*Timer.delay(.1);
		absoluteEncoder.configFactoryDefault();
		Timer.delay(.1);
		absoluteEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
		Timer.delay(.1);
		absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
		Timer.delay(.1);
		absoluteEncoder.configMagnetOffset(-1 * angleZero);
		Timer.delay(.1);
		absoluteEncoder.clearStickyFaults();*/
	}

	public void stopMotors() {
		driveMotor.stopMotor();
		turningMotor.stopMotor();
	}

	double getAngleZero() {
		return this.angleZero;
	}

	String getStatus() {
		return cancoder.getMagnetHealth().getValue().name();
	}

	public void setTurningPID(double p, double i, double d) {
		m_turningPIDController.setPID(p, i, d);
	}

	public void setDrivePID(double p, double i, double d) {
		drivePID.setP(p);
		drivePID.setI(i);
		drivePID.setD(d);
	}
}
