package org.firstinspires.ftc.teamcode;


import static java.lang.Math.abs;
import static java.lang.Thread.sleep;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Queue;

public class MecanumDriveChassis
{
  
  private DcMotor leftFrontDrive = null;
  private DcMotor leftRearDrive = null;
  private DcMotor rightFrontDrive = null;
  private DcMotor rightRearDrive = null;
  
  private BNO055IMU imu;
  private static IMUTelemetry IMUTelemetry;
  private Orientation angles; // stores the current orientation of the bot from the IMU
  
  // a timer for the various automation activities.
  private ElapsedTime driveTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
  
  private static double leftFrontDriveSpeed;
  private static double leftRearDriveSpeed;
  private static double rightFrontDriveSpeed;
  private static double rightRearDriveSpeed;
  
  // Robot speed [-1, 1].  (speed in any direction that is not rotational)
  // does not have any angular component, just scaler velocity.
  // combined with the angular component for motion.  Even if angle is 0 (forward).
  private static double vD = 0;
  
  // Robot angle while moving [0, 2PI] or [0, +/-PI]. (angle to displace the center of the bot,
  // ASDF)
  // relative to the direction the bot is facing.
  private static double thetaD = 0;
  
  // Speed component for rotation about the Z axis. [-x, x]
  // controlled by the error signal from the heading PID
  private static double rotationalSpeed = 0;
  
  // heading about a unit circle in radians.
  private static double desiredHeading;  // rotates about the Z axis [0,2PI) rad.
  private static double currentHeading;  // rotates about the Z axis [0,2PI) rad.
  
  // Robot speed scaling factor (% of joystick input to use)
  // applied uniformly across all joystick inputs to the JoystickToMotion() method.
  private double speedScale = 0;
  
  // PID for the heading
  private final double propCoeff = 0.9;
  private final double integCoeff = 0.0;
  private final double diffCoeff = 0.00;
  private final double OutputLowLimit = -1;
  private final double OutputHighLimit = 1;
  private final double MaxIOutput = 1;
  private final double OutputRampRate = 0.1;
  private final double OutputFilter = 0;
  private final double SetpointRange = 2 * Math.PI;
  
  private final double headingThreshold = 0.05;  // threshold to accept joystick input to change angle.
  private final int headdingAverageNumberOfSamples = 5;
  
  // number of encoder counts equal to one inch of forward travel
  //  private final int countsPerDriveInch = 5000/117;
  
  // number of encoder counts equal to one inch of forward travel
  //  private  final int countsPerStrafeInch = 5000/51;
  
  private final double closeEnoughHeading = 0.05;  // Radians to call angle close enough to target.
  
  // how many seconds to run the motors for the current drive leg.
  private double driveTime = 0;
  
  private PID headingPID = null;
  private RollingAverage averageHeading = null;
  private Telemetry telemetry;
  
  MecanumDriveChassis(HardwareMap hardwareMap, Telemetry theTelemetry)
  {
    telemetry = theTelemetry;
    
    // Initialize the hardware variables. Note that the strings used here as parameters
    // to 'get' must correspond to the names assigned during the robot configuration
    // step (using the FTC Robot Controller app on the phone).
    leftFrontDrive = hardwareMap.get(DcMotor.class, "lFront");
    leftRearDrive = hardwareMap.get(DcMotor.class, "lRear");
    rightFrontDrive = hardwareMap.get(DcMotor.class, "rFront");
    rightRearDrive = hardwareMap.get(DcMotor.class, "rRear");
    
    // Get and initialize the IMU.
    imu = hardwareMap.get(BNO055IMU.class, "imu");
    
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    IMUTelemetry = new IMUTelemetry();
    
    // set the initial imu mode parameters.
    parameters.mode = BNO055IMU.SensorMode.IMU;
    parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
    parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    parameters.loggingEnabled = false;
    
    imu.initialize(parameters);
    
    leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    
    // Motors on one side reversed to drive forward
    // Reverse the motor that runs backwards when connected directly to the battery
    // A positive power number should drive the robot forward regardless of the motor's
    // position on the robot.
    leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
    leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
    rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
    rightRearDrive.setDirection(DcMotor.Direction.FORWARD);
    
    RunUsingEncoders();
    
    // set motion parameters.
    vD = 0;
    thetaD = 0;
    rotationalSpeed = 0;
    
    // Set all the motor speeds.
    rightFrontDriveSpeed = 0;
    leftFrontDriveSpeed = 0;
    rightRearDriveSpeed = 0;
    leftRearDriveSpeed = 0;
    
    rightFrontDrive.setPower(rightFrontDriveSpeed);
    leftFrontDrive.setPower(leftFrontDriveSpeed);
    rightRearDrive.setPower(rightRearDriveSpeed);
    leftRearDrive.setPower(leftRearDriveSpeed);
    
    // make sure the gyro is calibrated before continuing
    while (!imu.isGyroCalibrated())
    {
      try
      {
        sleep(50);
      } catch (InterruptedException e)
      {
        e.printStackTrace();
      }
    }
    //hello
    // create and initialize the PID for the heading
    headingPID = new PID(propCoeff, integCoeff, diffCoeff);
    
    // set initial desired heading to the current actual heading.
    desiredHeading = currentHeading;
    
    // smooths out the joystick input so it doesn't slam hi/lo
    averageHeading = new RollingAverage(headdingAverageNumberOfSamples);
    
    // initially setup the PID parameters
    headingPID.setOutputLimits(OutputLowLimit, OutputHighLimit);
    headingPID.setMaxIOutput(MaxIOutput);
    headingPID.setOutputRampRate(OutputRampRate);
    headingPID.setOutputFilter(OutputFilter);
    headingPID.setSetpointRange(SetpointRange);
    headingPID.setContinousInputRange(2 * Math.PI);
    headingPID.setContinous(true);  // lets PID know we are working with a continuous range [0-360)
  }
  
  // Left  Y = forward, backward movement
  // Left  X = side to side (strafe)
  // Right X = rotate in place
  void drive(float driveLeftY, float driveLeftX, float driveRightX, boolean goFast)
  {
    
    // calculate the vectors multiply input values by scaling factor for max speed.
    joystickToMotion(driveLeftY * speedScale, driveLeftX * speedScale,
      driveRightX * speedScale);
    
    if (goFast == false)
    {
      vD = (.0875 * vD);
      rotationalSpeed = (.275 * rotationalSpeed);
      telemetry.addData("goFast: ", "off");
    } else if (goFast == true)
    {
      vD = (.175 * vD);
      rotationalSpeed = (.325 * rotationalSpeed);
    }
    
    telemetry.addData("Theta(Degrees)", thetaD);
    // Math out what to send to the motors and send it.
    PowerToWheels();
  }
  
  
  /**
   * Process the joystick values into motion vector.
   *  Converts the left stick X, Y input into the translation angle and speed
   *  Converts the right stick X axis into rotation speed.
   *
   *  Overall this makes the joysticks like mouse & keyboard game controls with
   *  the left stick acting as the WASD keys and the right stick as the mouse.
   *
   *  vD = desired robot translation speed.
   *  thetaD = desired robot translation angle.
   *  vTheta = desired robot rotational speed.
   */
  private void joystickToMotion(double leftStickY, double leftStickX, double rightStickX)
  {
    // determines the translation speed by taking the hypotenuse of the vector created by
    // the X & Y components.
    vD = Math.min(Math.sqrt(Math.pow(leftStickX, 2) + Math.pow(-leftStickY, 2)), 1);
    
    // Converts the joystick inputs from cartesian to polar from 0 to +/- PI oriented
    // with 0 to the right of the robot. (standard polar plot)
    thetaD = Math.atan2(-leftStickY, leftStickX);
    // orient to the robot by rotating PI/2 to make the joystick zero at the forward of bot.
    // instead of the right side.
    //thetaD = thetaD - Math.PI / 2;
    // simply takes the right stick X value and invert to use as a rotational speed.
    // inverted since we want CW rotation on a positive value.
    // which is opposite of what PowerToWheels() wants in polar positive rotation (CCW).
    rotationalSpeed = rightStickX;
  }
  
  
  /**
   * Calculate the power settings and send to the wheels.  This also translates the force
   * for the Mecanum wheels to the rotated axis based on the degree of the wheel offset.
   * In our case 45 degrees or PI/4
   *
   * Assumes X is forward and Z is up then rotate XY PI/4 to align with wheel axises.
   * placing the positive X axis on the left front wheel and the positive Y axis on the
   * left rear wheel.
   *
   * Rotation is about a positive Z axis pointing UP.
   * Positive Y is to the left.
   *
   * Translation angle is in radians + is CCW - is CW with ZERO to the forward of the bot.
   * I.e. standard rotation about a positive Z axis pointing UP.
   * E.g:
   *    0     = forward
   *    PI/4  = 45 deg. forward and to the left
   *    PI/2  = to the left
   *    3PI/4 = 135 deg. backward and to the left
   *    PI    = backwards
   *
   *    -PI/4  (or) 7PI/4 = 45 deg. forward and to the right
   *    -PI/2  (or) 6PI/4 = to the right
   *    -3PI/4 (or) 5PI/4 = -135 deg. backward and to the left
   *    -PI    (or) PI    = backwards
   *
   * vTheta rotation is also standard rotation about a positive Z axis pointing UP.
   * thus a positive vTheta will turn the bot CCW about its Z axis.
   *
   **/
  private void PowerToWheels()
  {
    
    double currentLeftFrontSpeed = leftFrontDriveSpeed;
    double currentLeftRearSpeed = leftRearDriveSpeed;
    double currentRightFrontSpeed = rightFrontDriveSpeed;
    double currentRightRearSpeed = rightRearDriveSpeed;
    if (vD < .03)
    {
      leftFrontDriveSpeed = 0;
      rightFrontDriveSpeed = 0;
      leftRearDriveSpeed = 0;
      rightRearDriveSpeed = 0;
      telemetry.addData("Expected Direction: ", "Stopped");
    } else if (thetaD >= (Math.PI * -2) / 8 && thetaD < (Math.PI * 2 / 8))
    {
      leftFrontDriveSpeed = vD;
      rightFrontDriveSpeed = -vD;
      leftRearDriveSpeed = -vD;
      rightRearDriveSpeed = vD;
      telemetry.addData("Expected Direction: ", "Right");
    } else if (thetaD >= (Math.PI * 2 / 8) && thetaD < (Math.PI * 6 / 8))
    {
      leftFrontDriveSpeed = vD;
      rightFrontDriveSpeed = vD;
      leftRearDriveSpeed = vD;
      rightRearDriveSpeed = vD;
      telemetry.addData("Expected Direction: ", "Up");
    } else if (thetaD >= (Math.PI * 6 / 8) || thetaD < Math.PI * -6 / 8)
    {
      leftFrontDriveSpeed = -vD;
      rightFrontDriveSpeed = vD;
      leftRearDriveSpeed = vD;
      rightRearDriveSpeed = -vD;
      telemetry.addData("Expected Direction: ", "Left");
    } else if (thetaD >= (Math.PI * -6 / 8) && thetaD < (Math.PI * -2 / 8))
    {
      leftFrontDriveSpeed = -vD;
      rightFrontDriveSpeed = -vD;
      leftRearDriveSpeed = -vD;
      rightRearDriveSpeed = -vD;
      telemetry.addData("Expected Direction: ", "Down");
    } else
    {
      leftFrontDriveSpeed = 0;
      rightFrontDriveSpeed = 0;
      leftRearDriveSpeed = 0;
      rightRearDriveSpeed = 0;
    }
    
    
    leftFrontDriveSpeed += rotationalSpeed;
    rightFrontDriveSpeed += -rotationalSpeed;
    leftRearDriveSpeed += rotationalSpeed;
    rightRearDriveSpeed += -rotationalSpeed;
  
    // place all the power numbers in a list for collection manipulations
    // (easier to find min / max etc when in a list)
    List<Double> speeds = Arrays.asList(rightFrontDriveSpeed,
      leftFrontDriveSpeed, rightRearDriveSpeed, leftRearDriveSpeed);
    
    // scales the motor powers while maintaining power ratios.
    double minPower = Collections.min(speeds);
    double maxPower = Collections.max(speeds);
    double maxMag = Math.max(Math.abs(minPower), Math.abs(maxPower));
    if (maxMag > 1.0)
    {
      for (int i = 0; i < speeds.size(); i++)
      {
        speeds.set(i, speeds.get(i) / maxMag);
      }
    }
    
    if (currentRightRearSpeed > 0 && rightRearDriveSpeed < 0)
    {
      rightRearDrive.setPower(0);
    }
    
    if (currentRightRearSpeed < 0 && rightRearDriveSpeed > 0)
    {
      rightRearDrive.setPower(0);
    }
    
    if (currentLeftRearSpeed > 0 && leftRearDriveSpeed < 0)
    {
      leftRearDrive.setPower(0);
    }
    
    if (currentLeftRearSpeed < 0 && leftRearDriveSpeed > 0)
    {
      leftRearDrive.setPower(0);
    }
    
    if (currentLeftFrontSpeed > 0 && leftFrontDriveSpeed < 0)
    {
      leftFrontDrive.setPower(0);
    }
    
    if (currentLeftFrontSpeed < 0 && leftFrontDriveSpeed > 0)
    {
      leftFrontDrive.setPower(0);
    }
    
    if (currentRightFrontSpeed > 0 && rightFrontDriveSpeed < 0)
    {
      rightFrontDrive.setPower(0);
    }
    
    if (currentRightFrontSpeed < 0 && rightFrontDriveSpeed > 0)
    {
      rightFrontDrive.setPower(0);
    }
    
    
    // must be same order as placed in the list
    // send the speeds to the motors
    rightFrontDrive.setPower(speeds.get(0));
    leftFrontDrive.setPower(speeds.get(1));
    rightRearDrive.setPower(speeds.get(2));
    leftRearDrive.setPower(speeds.get(3));
  }
  
  
  void RunUsingEncoders()
  {
    leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
  }
}
