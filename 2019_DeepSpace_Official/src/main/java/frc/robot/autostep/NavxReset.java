package frc.robot.autostep;

import edu.wpi.first.wpilibj.Timer;
import com.kauailabs.navx.frc.AHRS;

import frc.robot.DriveTrain;

public class NavxReset extends AutoStep {

    public AHRS navx;
    public Timer resetTimer;
    public float resetTimeLengthSeconds;

    public NavxReset(DriveTrain driveTrain, AHRS navx, float resetTimeLengthSeconds) {
        super(driveTrain);
        this.navx = navx;
        this.resetTimeLengthSeconds = resetTimeLengthSeconds;
    }

    public void Begin() {
        navx.reset();
        resetTimer.reset();
        resetTimer.start();
    }

    public void Update() {
        if (resetTimer.get() > resetTimeLengthSeconds) {
            isDone = true;
        }
    }
}