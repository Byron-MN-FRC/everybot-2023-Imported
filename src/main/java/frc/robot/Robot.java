// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Robot extends TimedRobot {

  /*
   * Drive motor controller instances.
   * 
   * Change the id's to match your robot.
   * Change kBrushed to kBrushless if you are using NEO's.
   * Use the appropriate other class if you are using different controllers.
   */
  WPI_VictorSPX frontLeftVictor = new WPI_VictorSPX(1); // 1
  WPI_VictorSPX frontRightVictor = new WPI_VictorSPX(3); // inv
  WPI_VictorSPX backLeftVictor = new WPI_VictorSPX(2); // 3
  WPI_VictorSPX backRightVictor = new WPI_VictorSPX(4); // inv

  /*
   * Mechanism motor controller instances.
   * 
   * Like the drive motors, set the CAN id's to match your robot or use different
   * motor controller classses (TalonFX, TalonSRX, Spark, VictorSP) to match your
   * robot.
   * 
   * The arm is a NEO on Everybud.
   * The intake is a NEO 550 on Everybud.
   */
  SparkMax arm = new SparkMax(6, MotorType.kBrushless);
  SparkMax intake = new SparkMax(5, MotorType.kBrushless);

  SparkMaxConfig armConfig = new SparkMaxConfig();
  SparkMaxConfig intakeConfig = new SparkMaxConfig();
  double currCurr = 0;

  /**
   * The starter code uses the most generic joystick class.
   */
  CommandXboxController m_controller = new CommandXboxController(0);

  /*
   * Magic numbers. Use these to adjust settings.
   */

  /**
   * How many amps the arm motor can use.
   */
  static final int ARM_CURRENT_LIMIT_A = 20;

  /**
   * Percent output to run the arm up/down at
   */
  static final double ARM_OUTPUT_POWER = 0.4;

  /**
   * How many amps the intake can use while picking up
   */
  static final int INTAKE_CURRENT_LIMIT_A = 25;

  /**
   * How many amps the intake can use while holding
   */
  static final int INTAKE_HOLD_CURRENT_LIMIT_A = 5;

  /**
   * Percent output for intaking
   */
  static final double INTAKE_OUTPUT_POWER = 1.0;

  /**
   * Percent output for holding
   */
  static final double INTAKE_HOLD_POWER = 0.07;

  /**
   * This method is run once when the robot is first started up.
   */
  @Override
  public void robotInit() {

    /*
     * You will need to change some of these from false to true.
     * 
     * In the setDriveMotors method, comment out all but 1 of the 4 calls
     * to the set() methods. Push the joystick forward. Reverse the motor
     * if it is going the wrong way. Repeat for the other 3 motors.
     */
    frontLeftVictor.setInverted(false);
    frontRightVictor.setInverted(true);
    backLeftVictor.setInverted(false);
    backRightVictor.setInverted(true);

    /*
     * Set the arm and intake to brake mode to help hold position.
     * If either one is reversed, change that here too. Arm out is defined
     * as positive, arm in is negative.
     */
    armConfig.inverted(false);
    armConfig.idleMode(IdleMode.kBrake);
    armConfig.smartCurrentLimit(ARM_CURRENT_LIMIT_A);

    intakeConfig.inverted(false);
    intakeConfig.idleMode(IdleMode.kBrake);
    intakeConfig.smartCurrentLimit(0);
    currCurr = 0;

    arm.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Calculate and set the power to apply to the left and right
   * drive motors.
   * 
   * @param forward Desired forward speed. Positive is forward.
   * @param turn    Desired turning speed. Positive is counter clockwise from
   *                above.
   */
  public void setDriveMotors(double forward, double turn) {
    SmartDashboard.putNumber("drive forward power (%)", forward);
    SmartDashboard.putNumber("drive turn power (%)", turn);

    /*
     * positive turn = counter clockwise, so the left side goes backwards
     */
    double left = forward - turn;
    double right = forward + turn;

    SmartDashboard.putNumber("drive left power (%)", left);
    SmartDashboard.putNumber("drive right power (%)", right);

    // see note above in robotInit about commenting these out one by one to set
    // directions.
    frontLeftVictor.set(left);
    frontRightVictor.set(right);
    backLeftVictor.set(left);
    backRightVictor.set(right);
  }

  /**
   * Set the arm output power. Positive is out, negative is in.
   * 
   * @param percent
   */
  public void setArmMotor(double percent) {
    arm.set(percent);
    SmartDashboard.putNumber("arm power (%)", percent);
    SmartDashboard.putNumber("arm motor current (amps)", arm.getOutputCurrent());
  }

  /**
   * Set the arm output power.
   * 
   * @param percent desired speed
   * @param amps current limit
   */
  public void setIntakeMotor(double percent, int amps) {
    intake.set(percent);

    if (amps != currCurr) {
      intakeConfig.smartCurrentLimit(amps);
      intake.configure(intakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    SmartDashboard.putNumber("intake power (%)", percent);
    SmartDashboard.putNumber("intake motor current (amps)", intake.getOutputCurrent());
  }

  /**
   * This method is called every 20 ms, no matter the mode. It runs after
   * the autonomous and teleop specific period methods.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Time (seconds)", Timer.getFPGATimestamp());
  }

  /**
   * Used to remember the last game piece picked up to apply some holding power.
   */
  static final int CONE = 1;
  static final int CUBE = 2;
  static final int NOTHING = 3;
  int lastGamePiece;

  @Override
  public void teleopInit() {
    frontLeftVictor.setNeutralMode(NeutralMode.Coast);
    frontRightVictor.setNeutralMode(NeutralMode.Coast);
    backLeftVictor.setNeutralMode(NeutralMode.Coast);
    backRightVictor.setNeutralMode(NeutralMode.Coast);

    lastGamePiece = NOTHING;
  }

  @Override
  public void teleopPeriodic() {
    double armPower;
    if (m_controller.leftTrigger().getAsBoolean()) {
      // lower the arm
      armPower = -ARM_OUTPUT_POWER;
    } else if (m_controller.leftBumper().getAsBoolean()) {
      // raise the arm
      armPower = ARM_OUTPUT_POWER;
    } else {
      // do nothing and let it sit where it is
      armPower = 0.0;
    }
    setArmMotor(armPower);
  
    double intakePower;
    int intakeAmps;
    if (m_controller.rightTrigger().getAsBoolean()) {
      // cube in or cone out
      intakePower = INTAKE_OUTPUT_POWER;
      intakeAmps = INTAKE_CURRENT_LIMIT_A;
      lastGamePiece = CUBE;
    } else if (m_controller.rightBumper().getAsBoolean()) {
      // cone in or cube out
      intakePower = -INTAKE_OUTPUT_POWER;
      intakeAmps = INTAKE_CURRENT_LIMIT_A;
      lastGamePiece = CONE;
    } else if (lastGamePiece == CUBE) {
      intakePower = INTAKE_HOLD_POWER;
      intakeAmps = INTAKE_HOLD_CURRENT_LIMIT_A;
    } else if (lastGamePiece == CONE) {
      intakePower = -INTAKE_HOLD_POWER;
      intakeAmps = INTAKE_HOLD_CURRENT_LIMIT_A;
    } else {
      intakePower = 0.0;
      intakeAmps = 0;
    }
    setIntakeMotor(intakePower, intakeAmps);

    /*
     * Negative signs here because the values from the analog sticks are backwards
     * from what we want. Forward returns a negative when we want it positive.
     */
    setDriveMotors(-m_controller.getLeftY(), -m_controller.getRightX());
  }
}