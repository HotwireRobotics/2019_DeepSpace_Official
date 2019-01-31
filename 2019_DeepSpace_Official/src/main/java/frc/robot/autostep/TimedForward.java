package frc.robot.autostep;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.DriveTrain;

public class TimedForward extends AutoStep {

    public Timer driveTimer;
    public float driveTime;
    public float speed;
    

    public TimedForward(DriveTrain driveTrain, float driveTime, float speed) {
        super(driveTrain);
        
    }

    public void Begin() {
        driveTimer.reset();
        driveTimer.start();
        driveTrain.SetBothSpeed(speed);
    }

    public void Update() {
        if (driveTimer.get() > driveTime) {
            isDone = true;
        }
    }
}