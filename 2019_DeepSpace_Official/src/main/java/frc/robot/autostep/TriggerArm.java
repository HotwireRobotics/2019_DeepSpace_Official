package frc.robot.autostep;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.DriveTrain;
import frc.robot.*;

public class TriggerArm extends AutoStep {
    public Robot robot;
    public boolean down;

    public TriggerArm(Robot robot, boolean down) {
        super(null);
        this.robot = robot;
        this.down = down;
    }

    public void Begin() {

    }

    public void Update() {
        robot.autoSetArm = true;
        isDone = true;

        if (down) {
            robot.lowerBuffer = robot.groundTarget;
            robot.upperBuffer = robot.groundTarget - robot.armBuffer;
            robot.potTarget = robot.groundTarget;
            robot.armHold = false;
        } else {
            robot.lowerBuffer = robot.hatchTarget;
            robot.upperBuffer = robot.hatchTarget - robot.armBuffer;
            robot.potTarget = robot.hatchTarget;
            robot.armHold = false;
        }

    }
}