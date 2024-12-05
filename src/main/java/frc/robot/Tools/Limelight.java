package frc.robot.Tools;

import org.littletonrobotics.junction.Logger;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawDetection;
import frc.robot.LimelightHelpers.RawFiducial;

public class Limelight {
    private PoseEstimate limelightMeasurement = null;
    private boolean isSim = false;
	private String name = "";
	private RawDetection[] rawDetections;
	private RawFiducial[] rawFiducials;
	private int[] detections;
	private int counter = 0;
	//private boolean isRed = false;
	//private boolean colorSet = false;
	//private boolean rejectUpdate = false;
	private LimelightHelpers.PoseEstimate mt2;
	private Pose2d robotPose2d = null;

    public Limelight() {
		if (Constants.kEnableLimelight) {
			robotPose2d = new Pose2d();
			Thread thread = new Thread() {
				public void run() {

					while (true) {
							LimelightHelpers.SetRobotOrientation(
									"limelight",
									robotPose2d.getRotation().getDegrees(),
									0,
									0,
									0,
									0,
									0);

							//mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
							mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
							

							if (mt2 != null) {

								/*
								 * if(mt2 == null) {
								 * return null;
								 * }
								 */

								/*if (Math.abs(gyro.getRate()) > 720) // if our angular velocity is greater than 720
																	// degrees
																	// per
																	// second, ignore vision updates
								{
									// rejectUpdate = true;
									// return null;
								}
								if (mt2.tagCount == 0) {
									// rejectUpdate = true;
									// return null;
								}*/
								/*
								 * if(!rejectUpdate) {
								 * m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
								 * m_poseEstimator.addVisionMeasurement(
								 * mt2.pose,
								 * mt2.timestampSeconds
								 * );
								 */

								//Logger.recordOutput("Limelight/Pose", mt2.pose);
							} 
					}
				}
			};

			thread.start();
		}
	}

	public synchronized void setPose(Pose2d pose2d) {
		robotPose2d = pose2d;
	}

	public PoseEstimate getPoseEstimate() {
		return mt2;
	}

    // public PoseEstimate getPoseEstimate(Pose2d pose, AHRS gyro) {
    //     if (Constants.kEnableLimelight) {

	// 		LimelightHelpers.SetRobotOrientation(
	// 			"limelight",
	// 			pose.getRotation().getDegrees(),
	// 			0,
	// 			0,
	// 			0,
	// 			0,
	// 			0
	// 		);

    //   		mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

	// 		if(mt2 == null) {
	// 			return null;
	// 		}

    //   		if(Math.abs(gyro.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
    //   		{
    //     		//rejectUpdate = true;
	// 			return null;
    //   		}
    //   		if(mt2.tagCount == 0) {
    //     		//rejectUpdate = true;
	// 			return null;
    //   		}
    //   		/*if(!rejectUpdate) {
    //     		m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
    //     		m_poseEstimator.addVisionMeasurement(
    //         	mt2.pose,
    //         	mt2.timestampSeconds
	// 		);*/

	// 		Logger.recordOutput("Limelight/Pose", mt2.pose);

	// 		return mt2;
	// 	}

    //     return null;
    // }

	public Pose2d getPose2d() {
		return LimelightHelpers.getBotPose2d(Constants.LimelightConstants.name);
	}

	public boolean hasTarget() {
		return LimelightHelpers.getTV(Constants.LimelightConstants.name);
	}

	public void setPipelineIndex(int index) {
		LimelightHelpers.setPipelineIndex(Constants.LimelightConstants.name, index);
	}

	public double getDistancToTargetFromRobot(int target) {
		rawFiducials = LimelightHelpers.getRawFiducials(Constants.LimelightConstants.name);

		return rawFiducials[target].distToRobot;
	}

	public int[] getDetections() {
		rawDetections = LimelightHelpers.getRawDetections(Constants.LimelightConstants.name);

		counter = 0;
		detections = new int[rawDetections.length];
		for (RawDetection detection : rawDetections) {
			detections[counter] = detection.classId;
			counter++;
		}

		return detections;
	}
}
