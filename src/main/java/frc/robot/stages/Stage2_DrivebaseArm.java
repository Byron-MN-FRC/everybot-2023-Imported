// ============================================================
// STAGE 2 of 3 — DRIVEBASE + ARM
// ============================================================
// Everything from Stage 1 is still here — the robot can still
// drive exactly the same way.
//
// NEW in this stage:
//   - A SparkMax motor controller for the arm (different brand
//     than the VictorSPX we used for driving!)
//   - A configuration object to set up the arm motor
//   - Constants to avoid "magic numbers" in our code
//   - setArmMotor() helper method
//   - Arm control in teleopPeriodic() using left trigger/bumper
//
// CONTROLLER MAP so far:
//   Left  Stick  (Y-axis) → Drive forward/backward
//   Right Stick  (X-axis) → Turn left/right
//   Left  Trigger         → Lower the arm
//   Left  Bumper          → Raise the arm
// ============================================================

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.stages;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

// NEW IMPORTS for the SparkMax arm motor controller.
// REV Robotics makes the SparkMax (and NEO motors).
// We need several classes to create, configure, and run it.
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

public class Stage2_DrivebaseArm extends TimedRobot {

  // DRIVE MOTOR FIELDS (same as Stage 1)
  WPI_VictorSPX frontLeftVictor  = new WPI_VictorSPX(1);
  WPI_VictorSPX frontRightVictor = new WPI_VictorSPX(3);
  WPI_VictorSPX backLeftVictor   = new WPI_VictorSPX(2);
  WPI_VictorSPX backRightVictor  = new WPI_VictorSPX(4);

  // ==========================================================
  // ARM MOTOR FIELDS — NEW!
  // ==========================================================
  // SparkMax is a different brand of motor controller made by
  // REV Robotics. It's used here because the arm motor (a NEO)
  // is also made by REV. Different motors often need different
  // controllers — just like different devices need different chargers.
  //
  // MotorType.kBrushless tells the SparkMax this is a NEO motor
  // (brushless). The number 6 is the CAN bus ID — change it to
  // match your robot's wiring.
  SparkMax arm = new SparkMax(6, MotorType.kBrushless);

  // SparkMaxConfig is a "settings object."
  // Instead of calling a bunch of separate setup methods,
  // we fill in this config object and send it to the SparkMax all at once.
  SparkMaxConfig armConfig = new SparkMaxConfig();

  // ==========================================================
  // CONTROLLER
  // ==========================================================
  CommandXboxController m_controller = new CommandXboxController(0);

  // ==========================================================
  // CONSTANTS — NEW!
  // ==========================================================
  // A CONSTANT is a variable whose value NEVER changes while
  // the program runs. We mark it with "static final":
  //
  //   static — belongs to the CLASS itself, not to any one object.
  //             Only one copy exists, shared by everything.
  //   final  — once set, it CANNOT be changed. The compiler
  //             will give you an error if you try.
  //
  // WHY USE CONSTANTS instead of typing the number directly?
  //   Bad:   arm.set(0.4);   ← What does 0.4 mean? Why 0.4?
  //   Good:  arm.set(ARM_OUTPUT_POWER);  ← Clear! And easy to change.
  //
  // If you want to make the arm faster, you change ONE line here
  // instead of hunting through all your code for every "0.4".
  // ==========================================================

  /** Maximum amps the arm motor is allowed to draw.
   *  Limiting current protects the motor from overheating. */
  static final int    ARM_CURRENT_LIMIT_A = 20;

  /** How fast to move the arm, as a fraction of full power.
   *  0.0 = stopped, 1.0 = full speed. 0.4 = 40% power. */
  static final double ARM_OUTPUT_POWER    = 0.4;


  // ==========================================================
  // METHOD: robotInit()
  // ==========================================================
  @Override
  public void robotInit() {

    // --- Drive motor setup (same as Stage 1) -----------------
    frontLeftVictor.setInverted(false);
    frontRightVictor.setInverted(true);
    backLeftVictor.setInverted(false);
    backRightVictor.setInverted(true);

    // --- Arm motor setup — NEW! ------------------------------
    // inverted(true): the arm motor is mounted so its "positive"
    // direction is physically backward. We flip it so positive
    // always means "arm goes OUT / up."
    armConfig.inverted(true);

    // idleMode(kBrake): when no power is sent to the arm motor,
    // it BRAKES — it actively resists movement.
    // This is important! Without brake mode, gravity would slowly
    // pull the arm down whenever the driver isn't holding a button.
    armConfig.idleMode(IdleMode.kBrake);

    // smartCurrentLimit: tell the SparkMax never to draw more than
    // ARM_CURRENT_LIMIT_A amps. Protects the motor from burning out.
    armConfig.smartCurrentLimit(ARM_CURRENT_LIMIT_A);

    // Now send the finished config object to the actual SparkMax.
    // kResetSafeParameters: wipe any old settings before applying ours.
    // kPersistParameters: save our settings to the controller's memory
    //   so they survive a power cycle.
    arm.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }


  // METHOD: setDriveMotors(forward, turn) — same as Stage 1
  public void setDriveMotors(double forward, double turn) {
    SmartDashboard.putNumber("drive forward power (%)", forward);
    SmartDashboard.putNumber("drive turn power (%)", turn);

    // Arcade drive math: left side slows on a left turn, right side speeds up
    double left  = forward - turn;
    double right = forward + turn;

    SmartDashboard.putNumber("drive left power (%)", left);
    SmartDashboard.putNumber("drive right power (%)", right);

    frontLeftVictor.set(left);
    frontRightVictor.set(right);
    backLeftVictor.set(left);
    backRightVictor.set(right);
  }


  // ==========================================================
  // METHOD: setArmMotor(percent) — NEW!
  // ==========================================================
  // Helper method that moves the arm at a given power level
  // and reports data to SmartDashboard.
  //
  // PARAMETER:
  //   percent — Power from -1.0 to +1.0.
  //             Positive = arm moves OUT (up/away from robot)
  //             Negative = arm moves IN (down/toward robot)
  //             Zero     = arm holds still (brake mode keeps it there)
  // ==========================================================
  public void setArmMotor(double percent) {
    arm.set(percent);  // Send power to the SparkMax

    // Report the arm's power and current draw to SmartDashboard.
    // getOutputCurrent() reads the actual amps the motor is using.
    // Watching this helps you spot if something is stalling/jamming.
    SmartDashboard.putNumber("arm power (%)", percent);
    SmartDashboard.putNumber("arm motor current (amps)", arm.getOutputCurrent());
  }


  // METHOD: robotPeriodic() — same as Stage 1
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Time (seconds)", Timer.getFPGATimestamp());
  }


  // ==========================================================
  // METHOD: teleopInit()
  // ==========================================================
  @Override
  public void teleopInit() {
    // Set drive motors to coast when teleop starts (same as Stage 1)
    frontLeftVictor.setNeutralMode(NeutralMode.Coast);
    frontRightVictor.setNeutralMode(NeutralMode.Coast);
    backLeftVictor.setNeutralMode(NeutralMode.Coast);
    backRightVictor.setNeutralMode(NeutralMode.Coast);
  }


  // ==========================================================
  // METHOD: teleopPeriodic()
  // ==========================================================
  @Override
  public void teleopPeriodic() {

    // --- ARM CONTROL — NEW! ----------------------------------
    // We use an if/else chain to decide what power to send
    // to the arm motor based on what buttons are pressed.
    //
    // An if/else chain works like this:
    //   - Check the first condition. If TRUE, do that block and SKIP the rest.
    //   - If FALSE, check the next condition. And so on.
    //   - The "else" at the end runs only if NOTHING above was true.
    //
    // We store the chosen power in a variable first, then call
    // setArmMotor() once at the end. This keeps things clean.

    double armPower;  // Declare a variable to hold the arm's target power

    if (m_controller.leftTrigger().getAsBoolean()) {
      // Left trigger is pressed → lower the arm
      // Negative power = arm goes IN (toward robot / downward)
      armPower = -ARM_OUTPUT_POWER;

    } else if (m_controller.leftBumper().getAsBoolean()) {
      // Left bumper is pressed → raise the arm
      // Positive power = arm goes OUT (away from robot / upward)
      armPower = ARM_OUTPUT_POWER;

    } else {
      // Neither button is pressed → stop the arm
      // Because armConfig has brake mode, the arm will hold its position.
      armPower = 0.0;
    }

    setArmMotor(armPower);  // Send the chosen power to the arm

    // --- DRIVE CONTROL (same as Stage 1) ---------------------
    // Negative signs flip the axis so pushing up = forward.
    setDriveMotors(
      -m_controller.getLeftY(),
      -m_controller.getRightX()
    );
  }

} // End of class Stage2_DrivebaseArm
