package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawDetection;
import frc.robot.LimelightHelpers.RawFiducial;

public class LimelightSubsystem extends SubsystemBase {

    private PoseEstimate limelightMeasurement = null;
    private boolean isSim = false;
	private RawDetection[] rawDetections;
	private RawFiducial[] rawFiducials;
	private int[] detections;
	private int counter = 0;
	private LimelightHelpers.PoseEstimate mt2;

    private AHRS gyro = null;
    private Pose2d previouPose2d = null;

    public LimelightSubsystem() {
    }

    public void setPreviousPoseEstimate(Pose2d pose) {
        previouPose2d = pose;
    }

    public  PoseEstimate getPoseEstimate() {
        return this.limelightMeasurement;
    }

    public void setGyro(AHRS gyro) {
        this.gyro = gyro;
    }

    @Override
    public void simulationPeriodic() {
        
    }

    @Override
	public void periodic() {
        if (Constants.kEnableLimelight && gyro != null && previouPose2d != null) {
            limelightMeasurement = _getPoseEstimate(previouPose2d, gyro);

            Logger.recordOutput("Limelight/Pose", limelightMeasurement.pose);
			Logger.recordOutput("Limelight/Tags", detections);
        }
    }

    private PoseEstimate _getPoseEstimate(Pose2d pose, AHRS gyro) {
        if (Constants.kEnableLimelight) {

			LimelightHelpers.SetRobotOrientation(
				"limelight",
				pose.getRotation().getDegrees(),
				0,
				0,
				0,
				0,
				0
			);

      		mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

			if(mt2 == null) {
				return null;
			}

      		if(Math.abs(gyro.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
      		{
        		//rejectUpdate = true;
				return null;
      		}
      		if(mt2.tagCount == 0) {
        		//rejectUpdate = true;
				return null;
      		}

			return mt2;
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
