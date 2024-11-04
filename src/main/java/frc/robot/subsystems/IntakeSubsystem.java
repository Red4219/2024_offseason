package frc.robot.subsystems;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    private boolean isSim = false;
    private SimDevice limeLightSim = null;
    private SimBoolean simTest = null;;

    public IntakeSubsystem() {
        limeLightSim = SimDevice.create("Limelight", 1);
		simTest = limeLightSim.createBoolean("found", edu.wpi.first.hal.SimDevice.Direction.kBidir, false);
    }

    public boolean hasItem() {
        return simTest.get();
    }

    @Override
	public void periodic() {
    }
}
