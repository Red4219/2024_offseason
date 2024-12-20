package frc.robot.Mechanisms;

import edu.wpi.first.wpilibj.PWM;


public class LED {
    private int channel = 0;
    private PWM pwm = null;
    private LEDStatus ledStatus = LEDStatus.ready;

    public static enum LEDStatus {
        ready,
        problem,
        targetAquired,
        targetSearching
    }

    public LED(int channel) {
        this.channel = channel;

        pwm = new PWM(channel);
        setStatus(LEDStatus.ready);
    }

    public void setStatus(LEDStatus ledStatus) {
        this.ledStatus = ledStatus;

        switch(ledStatus) {
            case ready:
                pwm.setPulseTimeMicroseconds(1855);
                break;
            case problem:
                pwm.setPulseTimeMicroseconds(1795);
                break;
            case targetAquired:
                pwm.setPulseTimeMicroseconds(1475);
                break;
            case targetSearching:
                pwm.setPulseTimeMicroseconds(1345);
                break;
            default:
                break;
        }
    }
}
