package frc.robot.Tools;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
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
	private boolean isRed = false;
	private boolean colorSet = false;

    public Limelight() {
		if(DriverStation.getAlliance().isPresent()) {
			if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
				isRed = false;
			} else {
				isRed = true;	
			}
		}
	}

	public void setIsRed(boolean isRed) {
		this.isRed = isRed;
	}

    public PoseEstimate getPoseEstimate() {
        if (Constants.kEnableLimelight) {

			/*if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
				limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.LimelightConstants.name);	
			} else {
				limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiRed(Constants.LimelightConstants.name);	
			}*/

			if(colorSet == false) {
				if(DriverStation.getAlliance().isPresent()) {
					if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
						isRed = false;
					} else {
						isRed = true;	
					}
				}
			}

			if(isRed) {
				limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.LimelightConstants.name);	
			} else {
				limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiRed(Constants.LimelightConstants.name);	
			}

			// Check for null and handle it if necessary
			if(limelightMeasurement == null) {
				return null;
			}

			// Get the list of tags being detected to be logged
			counter = 0;
			detections = new int[limelightMeasurement.tagCount];
			for (RawFiducial rawFiducial : limelightMeasurement.rawFiducials) {
				detections[counter] = rawFiducial.id;
				counter++;
			}

			Logger.recordOutput("Limelight/Pose", limelightMeasurement.pose);
			Logger.recordOutput("Limelight/Tags", detections);

            return limelightMeasurement;
		}

        return null;
    }

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
