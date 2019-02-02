package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import jdk.internal.net.http.common.Demand;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Talon;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.hal.PDPJNI;
import frc.robot.autostep.*;

public class Robot extends TimedRobot {

	// Sensors
	public AHRS navx = new AHRS(SPI.Port.kMXP);
	// public Ultrasonic Ultrasonic = new Ultrasonic(0, 1);

	// Drivetrain
	public JoshMotorControllor badMotor = new JoshMotorControllor(5, 0.8f, false);
	public DriveTrain driveTrain = new DriveTrain(0, 1, 2, 3, navx);

	// neumatics
	public DoubleSolenoid hatch = new DoubleSolenoid(4, 5);
	public boolean buttonReleased;
	public boolean hatchReleased;

	// Joysticks
	public Joystick driver;
	public Joystick operator;
	public Joystick debug;

	// Arm
	public TalonSRX armLeft = new TalonSRX(8);
	public TalonSRX armRight = new TalonSRX(14);

	// Intake

	public TalonSRX intake = new TalonSRX(1);

	// PID Controllers
	public int pdpHandle;
	public GearRack gearRackFront = new GearRack("Front Gear Rack", 6, 0.01f, 0.0f, 0.0f, 0.0f, 1, pdpHandle, (byte) 6);
	public GearRack gearRackBackOne = new GearRack("Back Gear Rack One", 4, 0.01f, 0.0f, 0.0f, 0.0f, 1, pdpHandle,
			(byte) 5);
	public GearRack gearRackBackTwo = new GearRack("Back Gear Rack Two", 18, 0.01f, 0.0f, 0.0f, 0.0f, -1, pdpHandle,
			(byte) 4);

	// Ramp timer
	public float timerDelaySeconds = 0.1f;

	public float rampLengthSeconds = 5;
	public float rampCurrent = 0.0f;
	public float rampStep;

	public float rampTargetPoint = 0.0f;
	public Timer pidTimer = new Timer();
	public boolean timerTrue;

	public boolean hitTarget = false;
	public NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
	public NetworkTableEntry tx = table.getEntry("tx");
	public NetworkTableEntry ty = table.getEntry("ty");
	public NetworkTableEntry ta = table.getEntry("ta");

	public double x = tx.getDouble(0.0);
	public double y = ty.getDouble(0.0);
	public double area = ta.getDouble(0.0);

	public enum LimelightPlacement
	{
		Pickup, Place
	}

	enum RobotState {
		Autonomous, Teleop
	}

	RobotState currentRobotState = RobotState.Autonomous;

	// Autonomousi
	int currentAutoStep;
	AutoStep[] doubleHatchAuto;

	// Auto

	public void robotInit() {

		pidTimer.start();
		HatchRelease();

		// Construct auto
		doubleHatchAuto = new AutoStep[10];
		doubleHatchAuto[0] = new NavxReset(driveTrain, navx, 0.1f);
		doubleHatchAuto[1] = new LimelightTrack(driveTrain, this, LimelightPlacement.Place);
		doubleHatchAuto[2] = new NavxTurn(driveTrain, navx, 90, 0.4f);
		doubleHatchAuto[3] = new TimedForward(driveTrain, 3, 0.4f);
		doubleHatchAuto[4] = new NavxTurn(driveTrain, navx, 180, 0.4f);
		doubleHatchAuto[5] = new LimelightTrack(driveTrain, this, LimelightPlacement.Pickup);
		doubleHatchAuto[6] = new NavxTurn(driveTrain, navx, 0, 0.4f);
		doubleHatchAuto[7] = new TimedForward(driveTrain, 4, 0.4f);
		doubleHatchAuto[8] = new NavxTurn(driveTrain, navx, 270, 0.4f);
		doubleHatchAuto[9] = new LimelightTrack(driveTrain, this, LimelightPlacement.Place);

	}

	public void disabledPeriodic() {
		// Controllers
		driver = new Joystick(0);
		operator = new Joystick(1);
	}

	public void autonomousInit() {
		currentRobotState = RobotState.Autonomous;
		currentAutoStep = 0;
		// TODO put autonomous selection choice here
		doubleHatchAuto[0].Begin();
		RobotLoopInit();
	}

	public void autonomousPeriodic() {
		RobotLoop();
	}

	public void teleopInit() {
		currentRobotState = RobotState.Teleop;
		RobotLoopInit();
	}

	public void teleopPeriodic() {
		RobotLoop();
	}

	public void testInit() {
		// Controllers
		debug = new Joystick(3);
		driver = new Joystick(0);
		operator = new Joystick(1);
	}

	public void testPeriodic() {

		gearRackFront.Write();

		if (driver.getRawButton(2)) {
			gearRackFront.motor.set(ControlMode.PercentOutput, 0.5f);
			gearRackBackOne.motor.set(ControlMode.PercentOutput, 0.5f);
			gearRackBackTwo.motor.set(ControlMode.PercentOutput, 0.5f);
		} else {
			gearRackFront.motor.set(ControlMode.PercentOutput, 0.0f);
			gearRackBackOne.motor.set(ControlMode.PercentOutput, 0.0f);
			gearRackBackTwo.motor.set(ControlMode.PercentOutput, 0.0f);
		}

		if (driver.getRawButton(8)) {
			gearRackBackTwo.ResetEncoder();
			gearRackBackOne.ResetEncoder();
		}
	}

	// This is our custom robot initialization.
	public void RobotLoopInit() {
		// Controllers
		debug = new Joystick(3);
		driver = new Joystick(0);
		operator = new Joystick(1);

		// pdpHandle = PDPJNI.initializePDP(0);
		// rampCurrent = 0f;
	}

	// Custom robot loop, because we want autonomous and teleop to be exactly the
	// same.
	public void RobotLoop() {
		if (currentRobotState == RobotState.Autonomous) {

			if (doubleHatchAuto[currentAutoStep].isDone) {
				currentAutoStep++;
				if (currentAutoStep > doubleHatchAuto.length) {
					currentRobotState = RobotState.Teleop;
				}
				doubleHatchAuto[currentAutoStep].Begin();
			}

			// TODO add button combination to beak out of this and change robot state to
			// teleop

		} else if (currentRobotState == RobotState.Teleop) {
			// if (pidTimer.get() >= timerDelaySeconds) {
			// pidTimer.reset();
			// if (timerTrue) {
			// if (rampTargetPoint > rampCurrent) {
			// rampCurrent += rampStep;
			// if (rampCurrent > rampTargetPoint) {
			// rampCurrent = rampTargetPoint;
			// }
			// } else if (rampTargetPoint < rampCurrent) {
			// rampCurrent -= rampStep;
			// if (rampCurrent < rampTargetPoint) {
			// rampCurrent = rampTargetPoint;
			// }
			// }
			// }
			// }

			// gearRackFront.Write();
			// gearRackBackTwo.Write();
			// gearRackBackOne.Write();

			// Driver Controls

			if (operator.getRawButton(5)) {
				Limelight(LimelightPlacement.Place);
			} else if (operator.getRawButton(6)) {
				Limelight(LimelightPlacement.Pickup);

			} else {

				driver.setRumble(RumbleType.kLeftRumble, 0);
				hitTarget = false;
				ControllerDrive();

				// gearRackBackTwo.UpdateVelocity();
				// Front Gear Rack 3 1/2 times

				// SmartDashboard.putNumber("Ramp Target", rampTargetPoint);
				// SmartDashboard.putNumber("Ramp Value", rampCurrent);

				// float backMaxOutput = 0.3f;
				// gearRackBackOne.setOutputRange(0.0f, backMaxOutput);
				// gearRackBackTwo.setOutputRange(0.0f, backMaxOutput);

				// if (driver.getRawButton(1)) {

				// UpdateRampTarget(67864);
				// timerTrue = true;
				// rampRate

				// gearRackBackOne.EnablePID();
				// gearRackBackOne.setSetpoint(rampCurrent);
				// gearRackBackTwo.EnablePID();
				// gearRackBackTwo.setSetpoint(rampCurrent);

				// } else if (driver.getRawButton(4)) {
				// UpdateRampTarget(0);
				// timerTrue = true;
				// gearRackBackOne.EnablePID();
				// gearRackBackOne.setSetpoint(rampCurrent);
				// gearRackBackTwo.EnablePID();
				// gearRackBackTwo.setSetpoint(rampCurrent);

				// } else {
				// timerTrue = false;
				// gearRackBackTwo.DisablePID();
				// gearRackBackTwo.DisablePID();
				// gearRackBackOne.DisablePID();
				// }

				// if (driver.getRawButton(8)) {
				// gearRackBackTwo.ResetEncoder();
				// gearRackBackOne.ResetEncoder();
				// }

				/*
				 * if (driver.getRawButton(2)) {
				 * 
				 * //gearRackFront.EnablePID();
				 * 
				 * if (driver.getRawAxis(2) > 0.5) {
				 * 
				 * gearRackBackTwo.setSetpoint(speedTarget);
				 * 
				 * // gearRackFront.setOutputRange(0, 1.0); gearRackBackOne.setOutputRange(0,
				 * 1.0); gearRackBackTwo.setOutputRange(0, 1.0); } else {
				 * 
				 * gearRackBackTwo.motor.set(ControlMode.PercentOutput, 0.5f);
				 * gearRackFront.motor.set(ControlMode.PercentOutput, 0.5f);
				 * 
				 * //gearRackFront.setSetpoint(-speedTarget);
				 * 
				 * //gearRackFront.setOutputRange(-1, 0); gearRackBackOne.setOutputRange(-1, 0);
				 * gearRackBackTwo.setOutputRange(-1, 0); } } else {
				 * //gearRackFront.motor.set(ControlMode.PercentOutput, 0.0f);
				 * 
				 * //gearRackFront.DisablePID(); //gearRackBackOne.DisablePID();
				 * //gearRackBackTwo.DisablePID(); }
				 */

				// if (driver.getRawButton(4)) {
				// climbTarget = 0;
				// }

				// if (driver.getRawButton(8)) {
				// gearRackBackOne.ResetEncoder();
				// gearRackBackTwo.ResetEncoder();
				// gearRackFront.ResetEncoder();
				// }

				// Operator Controls
				// if (operator.getRawButton(4)) {
				// Climb(0.4f);
				// } else if (operator.getRawButton(1)) {
				// Climb(-0.4f);
				// }

				// Intake

				if (operator.getRawButton(1)) {
					Intake(0.9f);
				} else {
					if (operator.getRawButton(4)) {
						Outtake(0.9f);
					} else {
						Outtake(0.0f);
					}
				}

				// Hatch

				if (operator.getRawButton(3)) {
					if (buttonReleased == true) {
						buttonReleased = false;
						if (hatchReleased) {
							HatchHold();
						} else {
							HatchRelease();
						}
					}
				} else {
					buttonReleased = true;
				}

				// Arm

				if (operator.getRawButton(2)) {
					ArmMove(0.25f);
				} else {
					ArmMove(0.0f);
				}
			}
		}

		UpdateMotors();

	}

	public void UpdateMotors() {
		driveTrain.Update();
	}

	public float TranslateController(float input) {
		float deadzone = 0.15f;
		if (input > -deadzone && input < deadzone) {
			input = 0.0f;
		}
		float a = 0.7f;
		float output = (a * input * input * input) + (1 - a) * input;
		return output;
	}

	public void ControllerDrive() {
		float horJoystick = TranslateController((float) driver.getRawAxis(0));
		float verJoystick = TranslateController((float) driver.getRawAxis(5));

		driveTrain.SetRightSpeed(-verJoystick + -horJoystick);
		driveTrain.SetLeftSpeed(-verJoystick + horJoystick);
	}

	public void Intake(float speeed) {
		intake.set(ControlMode.PercentOutput, -speeed);
	}

	public void Outtake(float speeed) {
		intake.set(ControlMode.PercentOutput, speeed);
	}

	public void HatchRelease() {
		hatch.set(DoubleSolenoid.Value.kReverse);
		hatchReleased = true;
	}

	public void HatchHold() {
		hatch.set(DoubleSolenoid.Value.kForward);
		hatchReleased = false;
	}

	public void ArmMove(float speeed) {
		armLeft.set(ControlMode.PercentOutput, speeed);
		armRight.set(ControlMode.PercentOutput, -speeed);
	}

	public void DrivetrainBrakes(boolean brakes) {
		if (brakes = true) {
			// driveTrain.SetBreak();
		} else {
			// driveTrain.SetCoast();
		}
	}

	public void UpdateRampTarget(float newTarget) {
		if (newTarget != rampTargetPoint) {
			rampTargetPoint = newTarget;
			rampStep = Math.abs(rampCurrent - rampTargetPoint) / (rampLengthSeconds / timerDelaySeconds);
		}
	}

	public void Limelight(LimelightPlacement placement) {

		// TODO add back the backing up

		driver.setRumble(RumbleType.kLeftRumble, 1);
		
		float approachSpeed = 0.45f;
		float turnSpeed = 0.5f;
		float buffer = 4.0f;
		float approachtarget = 2.1f;
		float reverseSpeed = -0.5f;

		if (!hitTarget) {
			if (x >= buffer) {

				if (placement == LimelightPlacement.Place) {
					HatchHold();
				} else {
					HatchRelease();
				}

				driveTrain.SetLeftSpeed(turnSpeed);
				driveTrain.SetRightSpeed(-turnSpeed);
			} else if (x <= -buffer) {

				if (placement == LimelightPlacement.Place) {
					HatchHold();
				} else {
					HatchRelease();
				}

				driveTrain.SetLeftSpeed(-turnSpeed);
				driveTrain.SetRightSpeed(turnSpeed);
			} else {
				driveTrain.SetLeftSpeed(0f);
				driveTrain.SetRightSpeed(0f);
				if (area < approachtarget) {

					if (placement == LimelightPlacement.Place) {
						HatchHold();
					} else {
						HatchRelease();
					}

					driveTrain.SetLeftSpeed(approachSpeed);
					driveTrain.SetRightSpeed(approachSpeed);
				} else {
					hitTarget = true;
				}
			}
		} else {
			if (placement == LimelightPlacement.Place) {
				HatchRelease();
			} else {
				HatchHold();
			}
		}
	}
}