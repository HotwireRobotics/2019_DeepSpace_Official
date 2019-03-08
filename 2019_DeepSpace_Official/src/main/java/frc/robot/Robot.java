package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Talon;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;

import java.nio.Buffer;

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
import edu.wpi.first.wpilibj.Compressor;

public class Robot extends TimedRobot {

	// Sensors
	public AHRS navx = new AHRS(SPI.Port.kMXP);
	public Ultrasonic ultrasonic = new Ultrasonic(4, 5);
	public AnalogPotentiometer pot = new AnalogPotentiometer(0);
	public DigitalInput intakeLimit = new DigitalInput(9);
	public Compressor compressor = new Compressor();

	// Drivetrain
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

	// Arm Variables
	public boolean armHold = false;
	public boolean runArm = true;
	public boolean povReleased = false;
	public double potTarget = 0;
	public double currentBuffer;
	public double armBuffer = 0.006f;
	public Timer brakeTimer;

	// Arm targets
	public double groundTarget = 0.985;
	public double shipCargoTarget = 0.66;
	public double rocketCargoTargetBot = 0.78;
	public double rocketCargoTargetMid = 0.65f;
	public double hatchTarget = 0.944;

	// Climbing Variables
	public float downForce = 0.0f;
	public float ultraheight = 6.5f;
	public double lowerBuffer = 0;
	public double upperBuffer = 0;
	public boolean checkPlatform = false;
	public float holdSpeed = 0.0f;
	public boolean foundPlatform = false;

	// Intake
	public TalonSRX intakeTop = new TalonSRX(1);
	public TalonSRX intakeBottom = new TalonSRX(2);
	public boolean hitTarget = false;

	// PID Controllers
	public int pdpHandle;
	public GearRack gearRackFrontOne = new GearRack("FGR1", 5, 0.01f, 0.0f, 0.0f, 0.0f, -1, pdpHandle, (byte) 6, 0.2f,
			0.2f, 3);
	public GearRack gearRackFrontTwo = new GearRack("FGR2", 6, 0.01f, 0.0f, 0.0f, 0.0f, 1, pdpHandle, (byte) 4, 5, 0.2f,
			0);
	public GearRack gearRackBackOne = new GearRack("BGR1", 7, 0.01f, 0.0f, 0.0f, 0.0f, 1, pdpHandle, (byte) 5, 6, 0.2f,
			2);
	public GearRack gearRackBackTwo = new GearRack("BGR2", 8, 0.01f, 0.0f, 0.0f, 0.0f, -1, pdpHandle, (byte) 4, 7, 0.2f,
			1);

	enum RobotState {
		Autonomous, Teleop;
	}

	public RobotState currentState;

	public enum LimelightPlacement {
		Place, Pickup
	};

	// Auto
	public AutoStep[] autonomous;
	public int currentAutoStep;
	public boolean autoSetArm = false;

	public void robotInit() {
		ultrasonic.setAutomaticMode(true);

	}

	public void disabledInit() {
		// Controllers
		driver = new Joystick(0);
		operator = new Joystick(1);
		DiskBrakeEnable();
	}

	public void autonomousInit() {

		ArmMove(0);
		brakeTimer = new Timer();
		brakeTimer.start();

		currentState = RobotState.Autonomous;

		currentAutoStep = 0;
		autonomous = new AutoStep[22];

		autonomous[0] = new NavxReset(driveTrain, navx);
		autonomous[1] = new TriggerArm(this, false);
		autonomous[2] = new Wait(driveTrain, 0.2f);

		autonomous[3] = new TimedForward(driveTrain, 1.2f, 0.5f);
		autonomous[4] = new Wait(driveTrain, 0.15f);
		autonomous[5] = new LimelightTrack(driveTrain, this, LimelightPlacement.Place, 0);
		autonomous[6] = new Wait(driveTrain, 0.2f);
		autonomous[7] = new TriggerArm(this, true);

		autonomous[8] = new NavxTurn(driveTrain, navx, -86.0f, 0.75f);
		autonomous[9] = new TimedForward(driveTrain, 0.95f, 1.0f);

		autonomous[10] = new Wait(driveTrain, 0.2f);
		autonomous[11] = new TriggerArm(this, false);
		autonomous[12] = new LimelightTrack(driveTrain, this, LimelightPlacement.Pickup, -1);

		autonomous[13] = new Wait(driveTrain, 0.2f);
		autonomous[14] = new TimedForward(driveTrain, 0.5f, -1.0f);
		autonomous[15] = new Wait(driveTrain, 0.2f);
		autonomous[16] = new TimedTurn(driveTrain, 0.05f, 0.5f);
		autonomous[17] = new NavxTurn(driveTrain, navx, -165, 0.75f);
		autonomous[18] = new TimedForward(driveTrain, 1.25f, -0.85f);
		autonomous[19] = new Wait(driveTrain, 0.1f);
		autonomous[20] = new TimedTurn(driveTrain, 0.85f, -0.5f);
		autonomous[21] = new LimelightTrack(driveTrain, this, LimelightPlacement.Place, -1);

		autonomous[0].Begin();
	}

	public void autonomousPeriodic() {

		System.out.println("Navx: " + navx.getYaw());
		RobotLoop();
	}

	public void teleopInit() {
		brakeTimer = new Timer();
		brakeTimer.start();

		currentState = RobotState.Teleop;

		checkPlatform = false;
		foundPlatform = false;
		ultrasonic.setAutomaticMode(true);

		armHold = true;

		// Controllers
		debug = new Joystick(3);
		driver = new Joystick(0);
		operator = new Joystick(1);
	}

	public void teleopPeriodic() {
		RobotLoop();
	}

	public void testInit() {

		navx.reset();

		ultrasonic.setAutomaticMode(true);

		// Controllers
		debug = new Joystick(3);
		driver = new Joystick(0);
		operator = new Joystick(1);

		DiskBrakeEnable();
	}

	public void testPeriodic() {

		// System.out.println("Encoder Back Two : " + gearRackBackTwo.GetEncoderPosition());
		// System.out.println("Encoder Back One: " + gearRackBackOne.GetEncoderPosition());
		// System.out.println("Encoder Front One: " + gearRackFrontOne.GetEncoderPosition());
		// System.out.println("Encoder Front Two: " + gearRackFrontTwo.GetEncoderPosition());
		System.out.println("Ultrasonic: " + ultrasonic.getRangeInches());
		// System.out.println("Pot:  " + pot.get());

		// gearRackBackOne.Write();
		// gearRackFrontOne.Write();
		// gearRackBackTwo.Write();
		// gearRackFrontTwo.Write();

		NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
		NetworkTableEntry tx = table.getEntry("tx");
		NetworkTableEntry ty = table.getEntry("ty");
		NetworkTableEntry ta = table.getEntry("ta");

		double x = tx.getDouble(0.0);
		double y = ty.getDouble(0.0);
		double area = ta.getDouble(0.0);

		NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);

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

		if (operator.getRawButton(2)) {
			float backSpeed = 0.31f;
			float frontSpeed = 0.53f;
			gearRackFrontOne.SetMotorSpeed(-frontSpeed);
			gearRackFrontTwo.SetMotorSpeed(frontSpeed);
			gearRackBackOne.SetMotorSpeed(backSpeed);
			gearRackBackTwo.SetMotorSpeed(-backSpeed);

			Intake(0.0f);

		} else if (operator.getRawButton(3)) {
			gearRackFrontOne.SetMotorSpeed(1.0f);
			gearRackFrontTwo.SetMotorSpeed(-1.0f);
			gearRackBackOne.SetMotorSpeed(-1.0f);
			gearRackBackTwo.SetMotorSpeed(1.0f);
		} else {
			gearRackFrontOne.SetMotorSpeed(0.0f);
			gearRackFrontTwo.SetMotorSpeed(0.0f);
			gearRackBackOne.SetMotorSpeed(0.0f);
			gearRackBackTwo.SetMotorSpeed(0.0f);

			Intake(0.0f);
		}
	}

	public void RobotLoop() {

		gearRackBackOne.Write();
		gearRackBackTwo.Write();
		gearRackFrontOne.Write();
		gearRackFrontTwo.Write();
		SmartDashboard.putNumber("UltrasonicDown", ultrasonic.getRangeInches());

		if (currentState == RobotState.Teleop) {

			// Climbing

			driver.setRumble(RumbleType.kLeftRumble, 0);

			if (operator.getRawButtonPressed(7)) {

				gearRackBackOne.ResetEncoder();
				gearRackBackTwo.ResetEncoder();
				gearRackFrontOne.ResetEncoder();
				gearRackFrontTwo.ResetEncoder();

				gearRackBackOne.foundPlatform = false;
				gearRackBackTwo.foundPlatform = false;
				gearRackFrontOne.foundPlatform = false;
				gearRackFrontTwo.foundPlatform = false;

				foundPlatform = false;
			}

			if (operator.getRawButtonPressed(8)) {
				ArmMove(0.0f);
				runArm = false;
				DiskBrakeDisable();
			}

			if (operator.getRawButton(7) && operator.getRawButton(8)) {
				compressor.stop();
				armHold = false;
				runArm = false;
				DiskBrakeDisable();

				if (!foundPlatform) {
					if (ultrasonic.getRangeInches() > 16) {
						checkPlatform = true;
						ArmMove(0.0f);
						Outtake(0.8f);
						driveTrain.SetBothSpeed(0.7f);
					} else {
						Intake(0.0f);
						driveTrain.SetLeftSpeed(0.0f);
						driveTrain.SetRightSpeed(0.0f);
					}

					float backSpeed = 0.325f;
					float frontSpeed = 0.535f;
					float rollTarget = -7.5f;

					if (navx.getRoll() < rollTarget) {
						backSpeed = backSpeed + 0.1f;
						System.out.println("Adjusting Roll");
					}

					gearRackFrontOne.SetMotorSpeed(-frontSpeed);
					gearRackFrontTwo.SetMotorSpeed(frontSpeed);
					gearRackBackOne.SetMotorSpeed(backSpeed);
					gearRackBackTwo.SetMotorSpeed(-backSpeed);

					// ArmMove(0.5f);
					System.out.println("Going up");
				}

				if (checkPlatform) {
					System.out.println("Checking for platform");
					float platformUltraHeight = 5;
					if (ultrasonic.getRangeInches() < platformUltraHeight) {
						foundPlatform = true;
					}
				}

				if (foundPlatform) {

					System.out.println("Found Platform");

					if (operator.getRawAxis(3) > 0.1) {
						driveTrain.SetBothSpeed(0.0f);

						gearRackBackOne.SetMotorSpeed(-0.5);
						gearRackBackTwo.SetMotorSpeed(0.5);
					} else {

						int runningUpStopPosition = 100;
						if (gearRackFrontOne.GetEncoderPosition() > runningUpStopPosition) {
							gearRackFrontOne.SetMotorSpeed(1.0f);
						} else {
							gearRackFrontOne.SetMotorSpeed(0.0f);
						}
						if (gearRackFrontTwo.GetEncoderPosition() > runningUpStopPosition) {
							gearRackFrontTwo.SetMotorSpeed(-1.0f);
						} else {
							gearRackFrontTwo.SetMotorSpeed(0.0f);
						}

						if (gearRackFrontTwo.GetEncoderPosition() <= runningUpStopPosition
								&& gearRackFrontOne.GetEncoderPosition() <= runningUpStopPosition) {
							driveTrain.SetBothSpeed(0.3f);
						} else {
							driveTrain.SetBothSpeed(0.0f);
						}

						ArmMove(0.0f);
					}
				}

			} else {
				compressor.start();
				ControllerDrive();

				// Arm Targets
				if (povReleased == true && (operator.getPOV() != -1 || operator.getRawAxis(2) != 0)) {
					povReleased = false;
					armHold = false;
					runArm = true;
					DiskBrakeDisable();

					if (operator.getPOV() == 0) {
						lowerBuffer = rocketCargoTargetBot + armBuffer;
						upperBuffer = rocketCargoTargetBot - armBuffer;
						potTarget = rocketCargoTargetBot;

					} else if (operator.getPOV() == 180) {
						lowerBuffer = groundTarget;
						upperBuffer = groundTarget - armBuffer;
						potTarget = groundTarget;

					} else if (operator.getPOV() == 90) {
						lowerBuffer = shipCargoTarget + armBuffer;
						upperBuffer = shipCargoTarget - armBuffer;
						potTarget = shipCargoTarget;

					} else if (operator.getPOV() == 270) {
						lowerBuffer = hatchTarget;
						upperBuffer = hatchTarget - armBuffer;
						potTarget = hatchTarget;

					} else if (operator.getRawAxis(2) > 0.0f) {
						lowerBuffer = rocketCargoTargetMid + armBuffer;
						upperBuffer = rocketCargoTargetMid - armBuffer;
						potTarget = rocketCargoTargetMid;

					}
				}
				if (operator.getPOV() == -1) {
					povReleased = true;
				}

				RunArmControls();

				// Hatch
				if (operator.getRawButtonPressed(3)) {
					if (hatchReleased) {
						HatchHold();
					} else {
						HatchRelease();
					}
				}
				// Limelight
				if (operator.getRawButton(5) || operator.getRawButton(6)) {

					if (operator.getRawButtonPressed(5) || operator.getRawButtonPressed(6)) {
						hitTarget = false;
					}

					if (operator.getRawButton(5)) {
						Limelight(LimelightPlacement.Place);
					} else {
						Limelight(LimelightPlacement.Pickup);
					}

				} else {

					// Turn off limelight
					NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);

					// Intake
					if (operator.getRawButton(1)) {
						if (intakeLimit.get() == false) {
							Intake(0.9f);
						} else {
							Intake(0.0f);
						}
					} else if (operator.getRawButton(4)) {
						if (potTarget == rocketCargoTargetBot) {
							Outtake(0.8f);
						} else if (potTarget == rocketCargoTargetMid) {
							intakeTop.set(ControlMode.PercentOutput, -1f);
							intakeBottom.set(ControlMode.PercentOutput, -0.35f);
						} else {
							Outtake(0.4f);
						}
					} else {
						Outtake(0.0f);
					}

					gearRackBackOne.SetMotorSpeed(0);
					gearRackBackTwo.SetMotorSpeed(0);
					gearRackFrontOne.SetMotorSpeed(0);
					gearRackFrontTwo.SetMotorSpeed(0);
				}
			}

		} else if (currentState == RobotState.Autonomous) {

			if (operator.getRawButton(9) && operator.getRawButton(10)) {
				currentState = RobotState.Teleop;
			}

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
				// currentState = RobotState.Teleop;
			}

			RunArmControls();
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
		intakeTop.set(ControlMode.PercentOutput, speeed);
		intakeBottom.set(ControlMode.PercentOutput, speeed);
	}

	public void Outtake(float speeed) {
		intakeTop.set(ControlMode.PercentOutput, -speeed);
		intakeBottom.set(ControlMode.PercentOutput, -speeed);
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
		armLeft.set(ControlMode.PercentOutput, -speeed);
		armRight.set(ControlMode.PercentOutput, speeed);
	}

	public void DrivetrainBrakes(boolean brakes) {
		if (brakes = true) {
			driveTrain.SetBreak();
		} else {
			driveTrain.SetCoast();
		}
	}

	public void DiskBrakeEnable() {
		diskBrake.set(DoubleSolenoid.Value.kReverse);
	}

	public void DiskBrakeDisable() {
		diskBrake.set(DoubleSolenoid.Value.kForward);
	}

	public void RunArmControls() {
		if (runArm) {
			if (!armHold) {
				if (pot.get() > lowerBuffer) {
					DiskBrakeDisable();
					ArmMove(0.4f);

				} else if (pot.get() < upperBuffer) {
					DiskBrakeDisable();

					float downForce = 0.0f;
					if (pot.get() < 0.64f) {
						downForce = -0.15f;
					} else if (pot.get() < 0.777f) {
						downForce = 0.0f;
					} else {
						downForce = 0.075f;
					}
					ArmMove(downForce);

				} else {
					armHold = true;
					brakeTimer = new Timer();
					brakeTimer.start();
				}
			}

			if (armHold) {

				DiskBrakeEnable();
				if (brakeTimer.get() > 0.1) {
					ArmMove(0.0f);
				}
			}
		}
	}

	public boolean Limelight(LimelightPlacement placement) {

		NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
		NetworkTableEntry tx = table.getEntry("tx");
		NetworkTableEntry ty = table.getEntry("ty");
		NetworkTableEntry ta = table.getEntry("ta");
		NetworkTableEntry tv = table.getEntry("tv");

		double x = tx.getDouble(0.0);
		double y = ty.getDouble(0.0);
		double area = ta.getDouble(0.0);
		double value = tv.getDouble(0.0);

		// driver.setRumble(RumbleType.kLeftRumble, 1);
		boolean isPlacing = (placement == LimelightPlacement.Place);

		driveTrain.SetBreak();

		// turning
		float turnBufferPlace = 2.05f;
		float turnBufferPickup = 4.0f;
		double turningFarDist = 25;
		double turningSpeedMinimum = 0.32f;
		double maxTurnSpeed = 0.32f;

		// approach
		float approachTargetPlace = 9.85f;
		float approachTargetPickup = 6.85f;
		float approachCloseTA = 1.5f;
		float approachFarTA = 0.162f;
		float approachSpeedClose = 0.3f;
		float approachSpeedFar = 0.75f;

		// reverse
		float reverseSpeed = -0.5f;
		float stopArea = 6.6f;

		potTarget = hatchTarget;

		if (!hitTarget) {

			NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);

			if (value == 0) {
				driveTrain.SetBothSpeed(0.0f);

			} else {
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
						System.out.println("Approach Speed: " + approachSpeed);
					} else {
						hitTarget = true;
					}
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
				NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
				return true;
			}
		}

		return false;
	}
}