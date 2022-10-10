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
  private static double vTheta = 0;

  // heading about a unit circle in radians.
  private static double desiredHeading;  // rotates about the Z axis [0,2PI) rad.
  private static double currentHeading;  // rotates about the Z axis [0,2PI) rad.

  // keeps track of the drive plan state
  private Queue<Leg> plan;
  private Leg currentLeg;
  private boolean drivingAPlan = false;
  private enum DriveState {IDLE, DRIVING, TURNING}
  private DriveState driveState = DriveState.IDLE;

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
  private final double SetpointRange = 2*Math.PI;

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

  MecanumDriveChassis(HardwareMap hardwareMap)
  {
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
    parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
    parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    parameters.loggingEnabled      = false;

    imu.initialize(parameters);

    leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    // Motors on one side reversed to drive forward
    // Reverse the motor that runs backwards when connected directly to the battery
    // A positive power number should drive the robot forward regardless of the motor's
    // position on the robot.
    leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
    leftRearDrive.setDirection(DcMotor.Direction.FORWARD);
    rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
    rightRearDrive.setDirection(DcMotor.Direction.REVERSE);

    RunUsingEncoders();

    // set motion parameters.
    vD = 0;
    thetaD = 0;
    vTheta = 0;

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
      try {
        sleep(50);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }
  
    // create and initialize the PID for the heading
    headingPID = new PID(propCoeff, integCoeff, diffCoeff);

    // get the initial error and put valid data in the telemetry from the imu
    IMUAngleProcessing();

    // set initial desired heading to the current actual heading.
    desiredHeading = currentHeading;

    // smooths out the joystick input so it doesn't slam hi/lo
    averageHeading = new RollingAverage(headdingAverageNumberOfSamples);

    // initially setup the PID parameters
    headingPID.setOutputLimits( OutputLowLimit, OutputHighLimit);
    headingPID.setMaxIOutput(MaxIOutput);
    headingPID.setOutputRampRate(OutputRampRate);
    headingPID.setOutputFilter(OutputFilter);
    headingPID.setSetpointRange(SetpointRange);
    headingPID.setContinousInputRange(2*Math.PI);
    headingPID.setContinous(true);  // lets PID know we are working with a continuous range [0-360)
  }

  // Left  Y = forward, backward movement
  // Left  X = side to side (strafe)
  // Right X = rotate in place
  void drive(float driveLeftY, float driveLeftX, float driveRightX, Telemetry telemetry)
  {
    telemetry.addData("Heading (rad) ", " %.4f", IMUTelemetry.heading );
    telemetry.addData("Error (rad) ", " %.4f",IMUTelemetry.error );
    telemetry.update();

    // calculate the vectors multiply input values by scaling factor for max speed.
    joystickToMotion( driveLeftY * speedScale, driveLeftX * speedScale,
        driveRightX * speedScale  );

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
  private void joystickToMotion( double leftStickY, double leftStickX, double rightStickX ) {
    // determines the translation speed by taking the hypotenuse of the vector created by
    // the X & Y components.
    vD = Math.min(Math.sqrt(Math.pow(leftStickX, 2) + Math.pow(-leftStickY, 2)), 1);

    // Converts the joystick inputs from cartesian to polar from 0 to +/- PI oriented
    // with 0 to the right of the robot. (standard polar plot)
    thetaD = Math.atan2(-leftStickY, leftStickX);
    // orient to the robot by rotating PI/2 to make the joystick zero at the forward of bot.
    // instead of the right side.
    thetaD = thetaD - Math.PI / 2;
    // simply takes the right stick X value and invert to use as a rotational speed.
    // inverted since we want CW rotation on a positive value.
    // which is opposite of what PowerToWheels() wants in polar positive rotation (CCW).
//    vTheta = -rightStickX;

    // if there is new joystick input update the heading otherwise hold the current heading as
    // the setpoint.
    // headding is in radians so just using the +/- 1 from the joystick to add as a bias to the
    // current angle will put the desired head +/- 57 degrees from current.  This should be more
    // than enough to move the bot at max rotation speed.
    // The chasing of this setpoint is controled by the PID loop on the vTheta value.
  
  
    averageHeading.add(rightStickX);  // average in the current stick value

    // if the averaged stick input is greater then the headingThreshold go ahead and adjust the heading.
    // This keeps from updating the desiredHeading value if no joystick input is being made.
    // Otherwise, it will always drive the desiredHeading to 0 (neutral joystick position)
    if(Math.abs(averageHeading.getAverage()) > headingThreshold)
    {
      desiredHeading = currentHeading - averageHeading.getAverage();
      // keep heading a positive angle
      if (desiredHeading < 0)
      {
        desiredHeading += (2 * Math.PI);
      }
    }
    // get the imu angles in the format we need.
    IMUAngleProcessing();

    // PID controls the vTheta input to the wheel power equation.
    // vTheta = headingPID.getOutput(currentHeading, desiredHeading );
  }


  void autoDrive( Telemetry telemetry )
  {
//    telemetry.addLine().addData("Heading (rad) ", " %.4f", IMUTelemetry.heading )
//    .addData("Error (rad) ", " %.4f",IMUTelemetry.error );
//    telemetry.addLine().addData("Counts ", " %d", rightFrontDrive.getCurrentPosition() );
//    telemetry.update();

    // always crunch the IMP data and work the PID to keep heading.
    IMUAngleProcessing();

    // the bot is not currently driving an active leg so if there is another one in the plan,
    // start it, otherwise the plan is done.
    if( drivingAPlan )
    {
      switch (driveState)
      {
        case DRIVING:
          Driving();
          break;

        case TURNING:
          Turning();
          break;

        case IDLE:
        default:
          nextLeg();
          break;
      }
    }
    else // not driving a plan... probably because there are no more legs in plan.
    {
      // plan is done, no active plan.  Just stop the bot and keep it stopped.
      vD = 0;
      thetaD = 0;
    }

    // Math out what to send to the motors and send it.
    // keeps sending even at stop.  ( 0 power to wheels )
    PowerToWheels();
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
  private void PowerToWheels() {

    // Motors power = Y component of directional vector
    leftFrontDriveSpeed  = vD * Math.sin(-thetaD + Math.PI / 4) - vTheta;
    rightRearDriveSpeed  = vD * Math.sin(-thetaD + Math.PI / 4) + vTheta;

    // Motors power = X component of directional vector
    rightFrontDriveSpeed = vD * Math.cos(-thetaD + Math.PI / 4) + vTheta;
    leftRearDriveSpeed   = vD * Math.cos(-thetaD + Math.PI / 4) - vTheta;

    // place all the power numbers in a list for collection manipulations
    // (easier to find min / max etc when in a list)
    List<Double> speeds = Arrays.asList(rightFrontDriveSpeed,
        leftFrontDriveSpeed, rightRearDriveSpeed, leftRearDriveSpeed  );

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
    // must be same order as placed in the list
    // send the speeds to the motors
    rightFrontDrive.setPower(speeds.get(0));
    leftFrontDrive.setPower(speeds.get(1));
    rightRearDrive.setPower(speeds.get(2));
    leftRearDrive.setPower(speeds.get(3));
  }


  // grab the imu heading and crunch out the values used for navigation and telemetry.
  // This method produces the heading input component to the motors from the PID that holds the
  // desired angle.  The error from the PID is sent to the motors in the vTheta variable.
  private void IMUAngleProcessing()
  {
    // desired angle in degrees +/- 0 to 180 where CCW is + and CW is -
    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

    // convert  imu angle range to our [0, 2PI) range
    if (angles.firstAngle < 0)
    {
      currentHeading = angles.firstAngle + 2 * Math.PI;
    }
    else
    {
      currentHeading = angles.firstAngle;
    }

    IMUTelemetry.heading = currentHeading;
    IMUTelemetry.error = vTheta = headingPID.getOutput(currentHeading, desiredHeading);
  }


  /** @param speedVar    */
  public void setSpeedScale(double speedVar)
  {
    this.speedScale = speedVar;
  }


  // Set up a new drive plan (list of drive legs)
  // Must be called from the opmode to initiate a drive plan
  void startPlan(Queue<Leg> newPlan )
  {
    if( newPlan.size() !=0 && !drivingAPlan )
    {
      this.plan = newPlan;
      drivingAPlan = true;
      driveState = DriveState.IDLE;
    }
  }


  // execute a path (list of drive legs)
  boolean nextLeg()
  {

    if(plan.size() !=0 && drivingAPlan) // only run if there are legs in the plan and a plan is active.
    {
      currentLeg = plan.remove();  // legs are removed as they are driven

      switch (currentLeg.mode)
      {
        case FORWARD:
          driveForward(currentLeg.speed/100, currentLeg.distance );
          break;

        case BACKWARDS:
          driveBackwards(currentLeg.speed/100, currentLeg.distance );
          break;

        case LEFT:
          strafeLeft(currentLeg.speed/100, currentLeg.distance );
          break;

        case RIGHT:
          strafeRight(currentLeg.speed/100, currentLeg.distance );
          break;

        case TURN:
          turnToAbsoluteAngle(currentLeg.angle);
          break;
      }
    }
    // no more path to drive. (path is empty)
    else
    {
      drivingAPlan = false;
    }
    return drivingAPlan;
  }

  /** Must be called from main loop repeatedly and quickly to stop the motors when the
   * drive time is reached.
   * Timer is reset to zero at the start of each leg.
   **/
   void Driving()
   {
     if (driveTimer.time() > driveTime)
     {
       // stop the motors
       vD = 0;
       thetaD = 0;
       driveState = DriveState.IDLE;
     }
   }


  //Must be called from main loop repeatedly to know when the turn is completed.
  void Turning()
  {
    // check if close enough on either side of target value.
    if (abs(currentHeading - desiredHeading) < closeEnoughHeading)
    {
      driveState = DriveState.IDLE;
    }
  }


  void driveForward(double speed, double seconds)
  {
    driveTime = seconds;
    driveState = DriveState.DRIVING;
    vD = speed/.707;  // divide by 0.707 because thetaD is 0 and we want to range full speed.
    driveTimer.reset();
    thetaD = 0;     // no translation, currentHeading still valid (assumed)
  }


  void driveBackwards(double speed, double seconds)
  {
    driveTime = seconds;
    driveState = DriveState.DRIVING;
    vD = -speed/.707;     // divide by 0.707 because thetaD is 0 and we want to range full speed.
    driveTimer.reset();
    thetaD = 0;      // no translation, currentHeading still valid (assumed)
  }


  void strafeLeft(double speed, double seconds)
  {
    driveTime = seconds;
    driveState = DriveState.DRIVING;
    vD = -speed/.707;     // divide by 0.707 because thetaD is 0 and we want to range full speed.
    driveTimer.reset();
    thetaD = -Math.PI/2;   // translation angle
  }


  void strafeRight(double speed, double seconds)
  {
    driveTime = seconds;
    driveState = DriveState.DRIVING;
    vD = -speed/.707;     // divide by 0.707 because thetaD is 0 and we want to range full speed.
    driveTimer.reset();
    thetaD = Math.PI/2;   // translation angle
  }


  void turnToAbsoluteAngle(double newDesiredHeading)
  {
    driveState = DriveState.TURNING;
    this.desiredHeading = newDesiredHeading / 180 * Math.PI;  // convert to RADIANS
  }

  public boolean isDriving()
  {
    return this.drivingAPlan;
  }


  void RunUsingEncoders()
  {
    leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
  }
}
