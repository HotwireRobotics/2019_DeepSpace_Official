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
import edu.wpi.first.wpilibj.AnalogInput;
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
	public Ultrasonic ultrasonic = new Ultrasonic(0, 1);
	public AnalogPotentiometer pot = new AnalogPotentiometer(0);
	// public DigitalInput limitSwich = new DigitalInput(9);

	// Drivetrain
	public JoshMotorControllor badMotor = new JoshMotorControllor(5, 0.8f, false);
	public DriveTrain driveTrain = new DriveTrain(0, 1, 2, 3, navx);

	// neumatics
	public DoubleSolenoid hatch = new DoubleSolenoid(4, 5);
	public DoubleSolenoid diskBrake = new DoubleSolenoid(2, 3);
	public boolean buttonReleased;
	public boolean hatchReleased;

	// Joysticks
	public Joystick driver;
	public Joystick operator;
	public Joystick debug;

	// Arm
	public TalonSRX armLeft = new TalonSRX(3);
	public TalonSRX armRight = new TalonSRX(4);
	public boolean armHold = true;
	public boolean povReleased = false;
	public double potTarget = 0;
	public double currentBuffer;
	public double groundTarget = 0.122;
	public double shipTarget = 0.142;
	public double rocketCargoTarget = 0.138;
	public double hatchTarget = 0.129;
	public float downForce = 0.0f;

	public double lowerBuffer = 0;
	public double upperBuffer = 0;

	// Intake

	public TalonSRX intake = new TalonSRX(1);

	// PID Controllers
	public int pdpHandle;
	public GearRack gearRackFrontOne = new GearRack("FGR1", 5, 0.01f, 0.0f, 0.0f, 0.0f, 1, pdpHandle,
			(byte) 6, 4, 0.2f);
	public GearRack gearRackFrontTwo = new GearRack("FGR2", 6, 0.01f, 0.0f, 0.0f, 0.0f, -1, pdpHandle,
			(byte) 4, 5, 0.2f);
	public GearRack gearRackBackOne = new GearRack("BGR1", 7, 0.01f, 0.0f, 0.0f, 0.0f, 1, pdpHandle,
			(byte) 5, 6, 0.2f);
	public GearRack gearRackBackTwo = new GearRack("BGR2", 8, 0.01f, 0.0f, 0.0f, 0.0f, -1, pdpHandle,
			(byte) 4, 7, 0.2f);

	// Ramp timer
	public float timerDelaySeconds = 0.1f;

	public float rampLengthSeconds = 5;
	public float rampCurrent = 0.0f;
	public float rampStep;

	public float rampTargetPoint = 0.0f;
	public Timer pidTimer = new Timer();
	public boolean timerTrue;

	public boolean hitTarget = false;

	public enum LimelightPlacement {
		Place, Pickup
	};

	// Auto
	public AutoStep[] autonomous;
	public int currentAutoStep;

	public void robotInit() {

		pidTimer.start();
	}

	public void disabledPeriodic() {
		// Controllers
		driver = new Joystick(0);
		operator = new Joystick(1);
	}

	public void autonomousInit() {
		currentAutoStep = 0;
		autonomous = new AutoStep[5];
		autonomous[0] = new NavxReset(driveTrain, navx);
		// autonomous[1] = new TimedForward(driveTrain, 1.0f, 0.75f);
		// autonomous[2] = new Wait(driveTrain, 0.5f);
		autonomous[1] = new NavxTurn(driveTrain, navx, 27.0f, 0.45f);
		autonomous[2] = new TimedForward(driveTrain, 1.0f, 0.5f);
		autonomous[3] = new NavxTurn(driveTrain, navx, 20.0f, 0.45f);
		autonomous[4] = new LimelightTrack(driveTrain, this, LimelightPlacement.Place, 1);
		autonomous[0].Begin();
	}

	public void autonomousPeriodic() {
		System.out.println(navx.getYaw());
		System.out.println("Current auto step " + currentAutoStep);
		if (currentAutoStep < autonomous.length) {

			autonomous[currentAutoStep].Update();

			if (autonomous[currentAutoStep].isDone) {
				currentAutoStep = currentAutoStep + 1;
				if (currentAutoStep < autonomous.length) {
					autonomous[currentAutoStep].Begin();
				}
			}
		} else {
			System.out.println("Autonomous Done");
			driveTrain.SetBothSpeed(0.0f);
		}

		UpdateMotors();

	}

	public void teleopInit() {

		ultrasonic.setAutomaticMode(true);

		potTarget = pot.get();

		// Controllers
		debug = new Joystick(3);
		driver = new Joystick(0);
		operator = new Joystick(1);
	}

	public void teleopPeriodic() {

		// Driver Controls

		// Limelight
		if (operator.getRawButton(5) || operator.getRawButton(6)) {

			if (operator.getRawButton(5)) {
				Limelight(LimelightPlacement.Place);
			} else {
				Limelight(LimelightPlacement.Pickup);
			}

		} else {

			driver.setRumble(RumbleType.kLeftRumble, 0);
			hitTarget = false;
			ControllerDrive();

			// Climb
			{

				// pid ramp timer
				{
					if (pidTimer.get() >= timerDelaySeconds) {
						pidTimer.reset();
						if (timerTrue) {
							if (rampTargetPoint > rampCurrent) {
								rampCurrent += rampStep;
								if (rampCurrent > rampTargetPoint) {
									rampCurrent = rampTargetPoint;
								}
							} else if (rampTargetPoint < rampCurrent) {
								rampCurrent -= rampStep;
								if (rampCurrent < rampTargetPoint) {
									rampCurrent = rampTargetPoint;
								}
							}
						}
					}
				}

				gearRackBackOne.Write();
				gearRackBackTwo.Write();
				gearRackFrontOne.Write();
				gearRackFrontTwo.Write();

				SmartDashboard.putNumber("Ramp Target", rampTargetPoint);
				SmartDashboard.putNumber("Ramp Value", rampCurrent);

				if (operator.getRawButton(7)) {
					gearRackBackOne.ResetEncoder();
					gearRackBackTwo.ResetEncoder();
					gearRackFrontOne.ResetEncoder();
					gearRackFrontTwo.ResetEncoder();

					gearRackBackOne.ResetPID();
					gearRackBackTwo.ResetPID();
					gearRackFrontOne.ResetPID();
					gearRackFrontTwo.ResetPID();
					rampCurrent = 0f;
				}

				if (driver.getRawButton(1)) {

					float topTarget = 67864;
					UpdateRampTarget(topTarget * 0.9f);
					timerTrue = true;

					gearRackBackOne.EnablePID();
					gearRackBackOne.setSetpoint(rampCurrent);
					gearRackBackTwo.EnablePID();
					gearRackBackTwo.setSetpoint(rampCurrent);
					gearRackFrontOne.EnablePID();
					gearRackFrontOne.setSetpoint(rampCurrent);
					gearRackFrontTwo.EnablePID();
					gearRackFrontTwo.setSetpoint(rampCurrent);

					float backMaxOutput = 0.3f;
					float frontMaxOutput = 0.7f;
					gearRackBackOne.setOutputRange(0.0f, backMaxOutput);
					gearRackBackTwo.setOutputRange(0.0f, backMaxOutput);
					gearRackFrontOne.setOutputRange(0.0f, frontMaxOutput);
					gearRackFrontTwo.setOutputRange(0.0f, frontMaxOutput);

				} else {

					ArmMove(0.0f);

					timerTrue = false;
					gearRackFrontOne.DisablePID();
					gearRackFrontTwo.DisablePID();
					gearRackBackTwo.DisablePID();
					gearRackBackOne.DisablePID();
				}
			}

			// Intake
			if (operator.getRawButton(1)) {
				Intake(0.9f);
			} else {
				if (operator.getRawButton(4)) {
					Outtake(0.4f);
				} else {
					Outtake(0.0f);
				}
			}

			double buffer = 0.001f;

			// Arm Targets
			if (povReleased == true && operator.getPOV() != -1) {
				povReleased = false;
				armHold = false;
				DiskBrakeDisable();

				if (operator.getPOV() == 0) {
					lowerBuffer = rocketCargoTarget - buffer;
					upperBuffer = rocketCargoTarget + buffer;
					potTarget = rocketCargoTarget;
					downForce = 0.1f;

				} else if (operator.getPOV() == 180) {
					lowerBuffer = groundTarget;
					upperBuffer = groundTarget + buffer;
					potTarget = groundTarget;
					downForce = 0.1f;

				} else if (operator.getPOV() == 90) {
					lowerBuffer = shipTarget - buffer;
					upperBuffer = shipTarget + buffer;
					potTarget = shipTarget;
					downForce = 0.0f;

				} else if (operator.getPOV() == 270) {
					lowerBuffer = hatchTarget;
					upperBuffer = hatchTarget + buffer;
					potTarget = hatchTarget;
					downForce = 0.075f;
				}
			}
			if (operator.getPOV() == -1) {
				povReleased = true;
			}

			if (!armHold) {
				if (pot.get() < lowerBuffer) {
					DiskBrakeDisable();
					// ArmMove(0.5f);

				} else if (pot.get() > upperBuffer) {
					DiskBrakeDisable();
					// ArmMove(downForce);

				} else {
					armHold = true;
				}
			}
			if (armHold) {
				DiskBrakeEnable();
				ArmMove(0);
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
		}

		UpdateMotors();
	}

	public void testInit() {
		// Controllers
		debug = new Joystick(3);
		driver = new Joystick(0);
		operator = new Joystick(1);

		DiskBrakeEnable();
	}

	public void testPeriodic() {
		System.out.println("navx" + navx.getYaw());
		System.out.println("pot" + pot.get());

		ArmMove(0.0f);
		ControllerDrive();
		UpdateMotors();

		if (operator.getRawButton(1)) {
			DiskBrakeDisable();
		} else {
			DiskBrakeEnable();
		}
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
		driveTrain.SetCoast();
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
			driveTrain.SetBreak();
		} else {
			driveTrain.SetCoast();
		}
	}

	public void UpdateRampTarget(float newTarget) {
		if (newTarget != rampTargetPoint) {
			rampTargetPoint = newTarget;
			rampCurrent = 0.0f;
			rampStep = Math.abs(rampCurrent - rampTargetPoint) / (rampLengthSeconds / timerDelaySeconds);

		}
	}

	public void DiskBrakeEnable() {
		diskBrake.set(DoubleSolenoid.Value.kForward);
	}

	public void DiskBrakeDisable() {
		diskBrake.set(DoubleSolenoid.Value.kReverse);
	}

	public boolean Limelight(LimelightPlacement placement) {

		NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
		NetworkTableEntry tx = table.getEntry("tx");
		NetworkTableEntry ty = table.getEntry("ty");
		NetworkTableEntry ta = table.getEntry("ta");

		double x = tx.getDouble(0.0);
		double y = ty.getDouble(0.0);
		double area = ta.getDouble(0.0);

		// driver.setRumble(RumbleType.kLeftRumble, 1);
		boolean isPlacing = (placement == LimelightPlacement.Place);

		driveTrain.SetBreak();

		// turning
		float turnSpeed = 0.35f;
		float turnBuffer = 3.0f;

		// approach
		float approachtarget = 2.9f;
		float approachCloseTA = 1.1f;
		float approachFarTA = 0.16f;
		float approachSpeedClose = 0.3f;
		float approachSpeedFar = 0.7f;

		// reverse
		float reverseSpeed = -0.5f;
		float stopArea = 1.1f;

		potTarget = hatchTarget;

		if (!hitTarget) {

			if (x >= turnBuffer) {

				if (isPlacing) {
					HatchHold();
				} else {
					HatchRelease();
				}

				driveTrain.SetLeftSpeed(turnSpeed);
				driveTrain.SetRightSpeed(-turnSpeed);
			} else if (x <= -turnBuffer) {

				if (isPlacing) {
					HatchHold();
				} else {
					HatchRelease();
				}

				driveTrain.SetLeftSpeed(-turnSpeed);
				driveTrain.SetRightSpeed(turnSpeed);
			} else {

				// On target so drive forward
				if (area < approachtarget) {

					if (isPlacing) {
						HatchHold();
					} else {
						HatchRelease();
					}

					// number between 0 and 1. 0 is lowest speed, 1 is quickest
					double normalizedApproachDist = (area - approachFarTA) / (approachCloseTA - approachFarTA);
					normalizedApproachDist = Math.max(0.0, normalizedApproachDist);
					normalizedApproachDist = Math.min(1.0, normalizedApproachDist);
					normalizedApproachDist = 1.0 - normalizedApproachDist;

					double approachSpeed = approachSpeedClose
							+ ((approachSpeedFar - approachSpeedClose) * normalizedApproachDist);

					System.out.println("APPROACH SPPED " + approachSpeed + " NORMALIZED " + normalizedApproachDist);
					driveTrain.SetBothSpeed((float) approachSpeed);
				} else {
					hitTarget = true;
				}
			}
		} else {
			if (isPlacing) {
				HatchRelease();
			} else {
				HatchHold();
			}

			driveTrain.SetBothSpeed(reverseSpeed);

			if (area < stopArea) {
				return true;
			}
		}

		return false;
	}
}