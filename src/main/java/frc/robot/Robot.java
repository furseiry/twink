// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.NoULib.lib.NoUGPIO;
import com.NoULib.lib.NoUMotor;
import com.NoULib.lib.NoUServo;
import com.NoULib.lib.NoUGPIO.GPIOMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  NoUGPIO g2 = new NoUGPIO(2, GPIOMode.WRITE_ONLY);

  NoUMotor flMotor = new NoUMotor(2);
  NoUMotor frMotor = new NoUMotor(1);
  NoUMotor blMotor = new NoUMotor(3);
  NoUMotor brMotor = new NoUMotor(4);

  NoUServo catapult = new NoUServo(1);

  Timer rsltimer = new Timer();

  CommandXboxController controller = new CommandXboxController(0);

  @Override
  public void robotInit() {
    catapult.setAngle(120);
    flMotor.setInverted(true);
    frMotor.setInverted(true);
    blMotor.setInverted(true);
    brMotor.setInverted(true);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    new InstantCommand(() -> catapult.setAngle(180)).andThen(new WaitCommand(0.5))
        .andThen(new InstantCommand(() -> catapult.setAngle(120))).andThen(new InstantCommand(() -> drive(0, -0.75, 0)))
        .andThen(new WaitCommand(3)).andThen(new InstantCommand(() -> drive(0, 0, 0))).schedule();
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    rsltimer.restart();

    DummySubsystem dummy = new DummySubsystem();
    controller.rightBumper().onTrue(new InstantCommand(() -> catapult.setAngle(60), dummy));
    controller.b().onTrue(new InstantCommand(() -> catapult.setAngle(180), dummy));
    controller.rightBumper().or(controller.b()).onFalse(new RunCommand(() -> {
      if (catapult.getAngle() < 120) {
        catapult.setAngle(catapult.getAngle() + 2);
      } else {
        catapult.setAngle(120);
      }
    }, dummy).until(() -> catapult.getAngle() == 120));
  }

  @Override
  public void teleopPeriodic() {
    // RSL
    if (rsltimer.advanceIfElapsed(1))
      g2.write(1 ^ g2.read());

    // read joysticks
    double xval = MathUtil.applyDeadband(controller.getLeftX(), 0.15);
    double yval = MathUtil.applyDeadband(controller.getLeftY(), 0.15);
    xval = Math.copySign(Math.pow(xval, 2), xval);
    yval = Math.copySign(Math.pow(yval, 2), yval);
    xval = xval * (1 - 0.35) + (Math.abs(xval) > 0.03 ? Math.copySign(0.35, xval) : 0.0);
    yval = yval * (1 - 0.35) + (Math.abs(yval) > 0.03 ? Math.copySign(0.35, yval) : 0.0);
    double rotation = controller.getRawAxis(2);

    drive(xval, yval, rotation);
  }

  void drive(double xval, double yval, double rotation) {
    // convert to polar coordinates
    double theta = Math.atan2(yval, xval);
    double magnitude = Math.hypot(xval, yval);

    // rotate vector to match the front of the robot
    theta = MathUtil.angleModulus(theta - Math.PI / 4.0);

    double xoutl = Math.cos(theta);
    double youtl = Math.sin(theta);
    double scalar = Math.abs(magnitude) / Math.max(Math.abs(xoutl), Math.abs(youtl));
    xoutl *= scalar;
    youtl *= scalar;
    double xoutr = -xoutl;
    double youtr = -youtl;

    rotation = Math.copySign(Math.max(Math.abs(rotation) - Math.max(Math.abs(xoutl), Math.abs(youtl)), 0), rotation);

    flMotor.set(xoutl + rotation);
    frMotor.set(youtl + rotation);
    blMotor.set(youtr + rotation);
    brMotor.set(xoutr + rotation);
  }

  @Override
  public void disabledInit() {
    rsltimer.stop();
    g2.write(1);
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
