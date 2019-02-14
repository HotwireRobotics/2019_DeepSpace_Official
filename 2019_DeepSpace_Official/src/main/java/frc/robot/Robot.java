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
	public Ultrasonic ultrasonic = new Ultrasonic(4, 5);
	public AnalogPotentiometer pot = new AnalogPotentiometer(0);

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
	public boolean armHold = false;
	public boolean povReleased = false;
	public double potTarget = 0;
	public double currentBuffer;
	public double groundTarget = -170 ;
	public double shipTarget = 0.142;
	public double rocketCargoTarget = 0.138;
	public double hatchTarget = 1525;
	public float downForce = 0.0f;
	public float ultraheight = 6.5f;
	public double lowerBuffer = 0;
	public double upperBuffer = 0;
	public boolean checkPlatform = false;
	public float holdSpeed = 0.3f;
	public boolean foundPlatform = false;

	// Intake

	public TalonSRX intake = new TalonSRX(1);

	// PID Controllers
	public int pdpHandle;
	public GearRack gearRackFrontOne = new GearRack("FGR1", 5, 0.01f, 0.0f, 0.0f, 0.0f, -1, pdpHandle, (byte) 6, 4,
			0.2f, 3);
	public GearRack gearRackFrontTwo = new GearRack("FGR2", 6, 0.01f, 0.0f, 0.0f, 0.0f, 1, pdpHandle, (byte) 4, 5, 0.2f,
			0);
	public GearRack gearRackBackOne = new GearRack("BGR1", 7, 0.01f, 0.0f, 0.0f, 0.0f, 1, pdpHandle, (byte) 5, 6, 0.2f,
			2);
	public GearRack gearRackBackTwo = new GearRack("BGR2", 8, 0.01f, 0.0f, 0.0f, 0.0f, -1, pdpHandle, (byte) 4, 7, 0.2f,
			1);

	// Ramp timer
	public float timerDelaySeconds = 0.1f;

	public float rampLengthSeconds = 20;
	public float rampCurrent = 0.0f;
	public float rampStep;

	public float rampTargetPoint = 0.0f;
	public Timer pidTimer = new Timer();
	public boolean timerTrue;

	public boolean hitTarget = false;

	enum RobotState {
		Climbing, Autonomous, Teleop;
	}

	public RobotState currentState;

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
		autonomous = new AutoStep[14];
		autonomous[0] = new NavxReset(driveTrain, navx);
		// autonomous[1] = new TimedForward(driveTrain, 1.0f, 0.75f);
		// autonomous[2] = new Wait(driveTrain, 0.5f);
		autonomous[1] = new NavxTurn(driveTrain, navx, 27.0f, 0.45f);
		autonomous[2] = new TimedForward(driveTrain, 0.65f, 0.75f);
		autonomous[3] = new NavxTurn(driveTrain, navx, 20.0f, 0.45f);
		autonomous[4] = new LimelightTrack(driveTrain, this, LimelightPlacement.Place, 1);
		autonomous[5] = new NavxTurn(driveTrain, navx, -83.0f, 0.75f);
		autonomous[6] = new TimedForward(driveTrain, 0.9f, 1.0f);
		autonomous[7] = new Wait(driveTrain, 0.2f);
		autonomous[8] = new LimelightTrack(driveTrain, this, LimelightPlacement.Pickup, -1);
		autonomous[9] = new Wait(driveTrain, 0.2f);
		autonomous[10] = new TimedForward(driveTrain, 1.45f, -1.0f);
		autonomous[11] = new Wait(driveTrain, 0.1f);
		autonomous[12] = new NavxTurn(driveTrain, navx, 95.0f, -0.5f);
		autonomous[13] = new LimelightTrack(driveTrain, this, LimelightPlacement.Place, -1);
		// autonomous[7] = new NavxTurn(driveTrain, navx, -165.0f, 0.75f);
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

		currentState = RobotState.Teleop;

		checkPlatform = false;
		foundPlatform = false;
		ultrasonic.setAutomaticMode(true);

		potTarget = armRight.getSelectedSensorPosition();

		// Controllers
		debug = new Joystick(3);
		driver = new Joystick(0);
		operator = new Joystick(1);
	}

	public void teleopPeriodic() {
		RobotLoop();
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
		System.out.println("Encoder" + armRight.getSelectedSensorPosition());

		ArmMove(0.0f);

		ControllerDrive();
		UpdateMotors();

		if (operator.getRawButton(1)) {
			DiskBrakeDisable();
		} else {
			DiskBrakeEnable();
		}

		if (driver.getRawButton(2)) {
			ArmMove(0.5f);
		} else if (driver.getRawButton(3)) {
			ArmMove(-0.5f);
		} else {
			ArmMove(0.0f);
		}

	}

	public void RobotLoop() {
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
			// ControllerDrive();

			// Climb
			if (currentState == RobotState.Climbing) {

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
				System.out.println("Ultrasonic " + ultrasonic.getRangeInches());

				SmartDashboard.putNumber("Ramp Target", rampTargetPoint);
				SmartDashboard.putNumber("Ramp Value", rampCurrent);

				if (driver.getRawButton(7)) {
					gearRackBackOne.ResetEncoder();
					gearRackBackTwo.ResetEncoder();
					gearRackFrontOne.ResetEncoder();
					gearRackFrontTwo.ResetEncoder();

					gearRackBackOne.ResetPID();
					gearRackBackTwo.ResetPID();
					gearRackFrontOne.ResetPID();
					gearRackFrontTwo.ResetPID();
					rampCurrent = 0f;
					gearRackBackOne.foundPlatform = false;
					gearRackBackTwo.foundPlatform = false;
					gearRackFrontOne.foundPlatform = false;
					gearRackFrontTwo.foundPlatform = false;
				}

				if (driver.getRawButton(1)) {
					Intake(-0.4f);
					float topTarget = 67864;
					UpdateRampTarget(topTarget * 0.1f);
					timerTrue = true;
					float backMaxOutput = 0.3f;
					float frontMaxOutput = 1.0f;

					if (!foundPlatform) {
						System.out.println("Going up");
						gearRackBackOne.EnablePID();
						gearRackBackOne.setSetpoint(rampCurrent);
						gearRackBackTwo.EnablePID();
						gearRackBackTwo.setSetpoint(rampCurrent);
						gearRackFrontOne.EnablePID();
						gearRackFrontOne.setSetpoint(rampCurrent);
						gearRackFrontTwo.EnablePID();
						gearRackFrontTwo.setSetpoint(rampCurrent);
						gearRackBackOne.setOutputRange(0.0f, backMaxOutput);
						gearRackBackTwo.setOutputRange(0.0f, backMaxOutput);
						gearRackFrontOne.setOutputRange(0.0f, frontMaxOutput);
						gearRackFrontTwo.setOutputRange(0.0f, frontMaxOutput);
						ArmMove(-0.50f);
					}

					if (ultrasonic.getRangeInches() > 8) {
						checkPlatform = true;
						driveTrain.SetBothSpeed(0.4f);

					} else if (!checkPlatform) {
						driveTrain.SetLeftSpeed(0.0f);
						driveTrain.SetRightSpeed(0.0f);
					}

					if (checkPlatform == true) {
						System.out.println("Checking for platform");
						if (ultrasonic.getRangeInches() < ultraheight) {
							foundPlatform = true;
						}
					}

					if (foundPlatform) {
						gearRackFrontOne.foundPlatform = true;
						gearRackFrontTwo.foundPlatform = true;
						gearRackBackOne.DisablePID();
						gearRackBackTwo.DisablePID();
						gearRackBackOne.SetMotorSpeed(holdSpeed);
						gearRackBackTwo.SetMotorSpeed(holdSpeed);
						driveTrain.SetLeftSpeed(0.0f);
						driveTrain.SetRightSpeed(0.0f);

						System.out.println("Found Platform");
						ArmMove(0.0f);
						gearRackFrontOne.EnablePID();
						gearRackFrontTwo.EnablePID();
						gearRackFrontOne.setSetpoint(1000);
						gearRackFrontTwo.setSetpoint(1000);
						gearRackFrontOne.setOutputRange(-1.0, 1.0);
						gearRackFrontTwo.setOutputRange(-1.0, 1.0);

						// gearRackFrontOne.motor.set(ControlMode.PercentOutput, 1.0);
						// gearRackFrontTwo.motor.set(ControlMode.PercentOutput, -1.0);

					}

				} else {
					driveTrain.SetBothSpeed(0);
					ArmMove(0.0f);
					Intake(0.0f);
					timerTrue = false;
					gearRackFrontOne.DisablePID();
					gearRackFrontTwo.DisablePID();
					gearRackBackTwo.DisablePID();
					gearRackBackOne.DisablePID();
				}
			} else if (currentState == RobotState.Teleop) {
				System.out.println("Encoder" + armRight.getSelectedSensorPosition());

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

				// Arm Targets
				double buffer = 50f;
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
					if (armRight.getSelectedSensorPosition() < lowerBuffer) {
						DiskBrakeDisable();
						ArmMove(0.5f);

					} else if (armRight.getSelectedSensorPosition() > upperBuffer) {
						DiskBrakeDisable();
						ArmMove(downForce);

					} else {
						armHold = true;
					}
				}
				if (armHold) {
					DiskBrakeEnable();
					ArmMove(0);
				}

				// Hatch
				// TODO change this to buttonPressed
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
			} else if (currentState == RobotState.Autonomous) {
				// TODO put auto here
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
		float turnBufferPlace = 2.0f;
		float turnBufferPickup = 4.0f;
		double turningFarDist = 25;
		double turningSpeedMinimum = 0.25f;
		double maxTurnSpeed = 0.4f;

		// approach
		float approachTargetPlace = 2.95f;
		float approachTargetPickup = 2.5f;
		float approachCloseTA = 1.1f;
		float approachFarTA = 0.16f;
		float approachSpeedClose = 0.2f;
		float approachSpeedFar = 0.7f;

		// reverse
		float reverseSpeed = -0.5f;
		float stopArea = 1.1f;

		potTarget = hatchTarget;

		if (!hitTarget) {

			double normalized = Math.abs(maxTurnSpeed * (x / turningFarDist));
			float turnSpeed = (float) Math.max(normalized, turningSpeedMinimum);
			System.out.println(turnSpeed);

			float turnBuffer = 0.0f;
			if (isPlacing) {
				turnBuffer = turnBufferPlace;
			} else {
				turnBuffer = turnBufferPickup;
			}

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
				float approachTarget = 0.0f;
				if (isPlacing) {
					approachTarget = approachTargetPlace;
				} else {
					approachTarget = approachTargetPickup;
				}
				if (area < approachTarget) {

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
				System.out.println("Exiting");
				return true;
			}
		}

		return false;
	}
}