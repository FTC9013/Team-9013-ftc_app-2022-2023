package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Leg;
import org.firstinspires.ftc.teamcode.ManipulatorPlatform;
import org.firstinspires.ftc.teamcode.MecanumDriveChassis;

import java.util.LinkedList;
import java.util.Queue;

@Autonomous(name = "AutonomousParkLazyRed", group = "Linear Opmode")

//@Disabled
public class AutonomousParkLazyRed extends LinearOpMode
{
  
  // Declare OpMode members.
  private MecanumDriveChassis driveChassis;
  private ManipulatorPlatform manipulatorPlatform;
  //private LEDs leds;
  
  private ElapsedTime manipulateTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
  
  // save for later
  // private static final float mmPerInch = 25.4f;
  // private static final float mmFTCFieldWidth = (12 * 6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
  // private static final float mmTargetHeight = (6) * mmPerInch; // the height of the center of the target image above the floor
  
  private boolean extendedFlag = false;
  
  private final int extenderRetracted  = 0;
  private final int extenderExtended  = 700;
  
  private final float searchTime = 5;  // sets the time (Seconds) to search for rings before declaring NONE.
  
  private final boolean forkExtend = true;
  private final boolean forkRetract = false;
  
  private final boolean shooterExtend = true;
  private final boolean shooterRetract = false;
  
  private final double spinnerSpeedFull = 0.70;
  private final double spinnerSpeedStop = 0;
  
  private final double shooterSpeedFull = 128;
  private final double shooterSpeedTolerance = 20;
  private final double shooterSpeedStop = 0;
  
  
  private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
  private static final String LABEL_FIRST_ELEMENT = "Quad";
  private static final String LABEL_SECOND_ELEMENT = "Single";
  
  private enum objectCount { UNKNOWN, SINGLE, QUAD, NONE }
  private objectCount ringCount = objectCount.UNKNOWN;
  
  /**
   * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
   * localization engine.
   */
  private VuforiaLocalizer vuforia;
  
  /**
   * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
   * Detection engine.
   */
  private TFObjectDetector tfod;
  
  WebcamName webcamName;
  
  // All of the following will need to take less than the time allotted for autonomous...
  @Override
  public void runOpMode() {
    driveChassis = new MecanumDriveChassis(hardwareMap);
    manipulatorPlatform = new ManipulatorPlatform(hardwareMap);
    webcamName = hardwareMap.get(WebcamName.class, "Webcam");
    
    //leds = new LEDs(hardwareMap);
    //leds.goOff();
    
    // build all the drive plans for drive by distance (time in seconds)
    //
    // Each leg of the trip is added to the queue in this code block.
    // As the opmode runs, the queue sent to the drive base for execution.
    //
    // mode:     {FORWARD, BACKWARDS, LEFT (strafe), RIGHT (strafe), TURN}
    // speed:    Ignored in TURN mode: the drive speed from 0-100%  (Slower speeds for longer
    //           times will be more precise)
    // angle:    Ignored in all but TURN mode:
    //           the desired angle of travel relative to the ZERO orientation in DEGREES
    //           ZERO is where the bot was facing when the IMU calibrated.
    //           desired angle in degrees 0 to 360 CCW
    // distance: Only used for FORWARD, BACKWARD, LEFT, RIGHT, modes:  the TIME in seconds to run
    //           the motors.
    
    // This is just a test routine to test all driving modes.
    Queue<Leg> TestAllFunctions = new LinkedList<>();
    TestAllFunctions.add(new Leg(Leg.Mode.LEFT, 50, 0, 1));
    TestAllFunctions.add(new Leg(Leg.Mode.RIGHT, 50, 0, 1));
    TestAllFunctions.add(new Leg(Leg.Mode.FORWARD, 50, 0, 1));
    TestAllFunctions.add(new Leg(Leg.Mode.TURN, 50, 90, 0));
    TestAllFunctions.add(new Leg(Leg.Mode.FORWARD, 50, 0, 1));
    TestAllFunctions.add(new Leg(Leg.Mode.TURN, 50, 180, 0));
    TestAllFunctions.add(new Leg(Leg.Mode.FORWARD, 50, 0, 1));
    TestAllFunctions.add(new Leg(Leg.Mode.TURN, 50, 270, 0));
    TestAllFunctions.add(new Leg(Leg.Mode.FORWARD, 50, 0, 1));
    TestAllFunctions.add(new Leg(Leg.Mode.TURN, 50, 0, 0));
    TestAllFunctions.add(new Leg(Leg.Mode.LEFT, 50, 0, 1));
    TestAllFunctions.add(new Leg(Leg.Mode.RIGHT, 50, 0, 1));
    
    
    // These are the working paths for the OpMode
    Queue<Leg> MoveToParkSpot = new LinkedList<>();
    MoveToParkSpot.add(new Leg(Leg.Mode.RIGHT, 70, 0, 1.4));
    
   
    
    
    // Wait for the game to start (driver presses PLAY)
    
    // Initialize the vision objects
    initVuforia();
    initTfod();
    
    /**
     * Activate TensorFlow Object Detection before we wait for the start command.
     * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
     **/
    if (tfod != null) {
      tfod.activate();
      
      // The TensorFlow software will scale the input images from the camera to a lower resolution.
      // This can result in lower detection accuracy at longer distances (> 55cm or 22").
      // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
      // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
      // should be set to the value of the images used to create the TensorFlow Object Detection model
      // (typically 1.78 or 16/9).
      
      // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
      tfod.setZoom(1.0, 1.78);
    }
    
    waitForStart();
    
    
    // for each piece of the drive & manipulate plan you will need to load a plan and then put
    // a while loop, like the following example, that repeatedly calls the autoDrive method
    // until the driving is done.
    // Put manipulator movements between the driving loops.
    // If you need more driving load another plan and make another loop.
    
    // potentially do manipulation here.  Make sure it is done before moving on.
    
    manipulatorPlatform.initGatherDropper();
    
    manipulateTimer.reset();
    while (opModeIsActive() && manipulateTimer.time()< 3.0)
    {
      driveChassis.autoDrive(telemetry);
    }
    
    manipulatorPlatform.invertGatherDropper();
    
    manipulatorPlatform.setArmPosition(manipulatorPlatform.armMid);
    
    driveChassis.startPlan(MoveToParkSpot);
    
    while (opModeIsActive() && driveChassis.isDriving())
    {
      driveChassis.autoDrive(telemetry);
    }
    
    manipulatorPlatform.setArmPosition(manipulatorPlatform.armGather+10);
    
    manipulateTimer.reset();
    while (opModeIsActive() && manipulateTimer.time()< 3.0)
    {
      driveChassis.autoDrive(telemetry);
    }
    
    
    
    // After driving do your manipulation.  You may need a timer based state machine but simple
    // actions can just be done inline.
    
    // Do some manipulation...
    
    // spin for a second to allow whatever to finish (example of wait timer).
//        manipulateTimer.reset();
//        while (opModeIsActive() && manipulateTimer.time()< 1.0);
//
//        // do second part of drive plan.
//        driveChassis.startPlan(PathPt2);
//        while (opModeIsActive() && driveChassis.isDriving())
//        {
//            // Process the drive chassis
//            driveChassis.autoDrive(telemetry);
//        }
    // After driving do your manipulation.  You may need a timer based state machine but simple
    // actions can just be done inline.
    
    // Do some manipulation...
    
    // spin for a second to allow whatever to finish (example of wait timer).
//        manipulateTimer.reset();
//        while (opModeIsActive() && manipulateTimer.time()< 1.0);
//
//        // do third part of drive plan.
//        driveChassis.startPlan(PathPt3);
//        while (opModeIsActive() && driveChassis.isDriving())
//        {
//            // Process the drive chassis
//            driveChassis.autoDrive(telemetry);
//        }
    
    // potentially do manipulation here.  Make sure it is done before moving on because the OpMode will end.
    
  }
  
  /*
   * Initialize the Vuforia localization engine.
   */
  private void initVuforia()
  {
    /*
     * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
     * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
     */
    int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
      "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    
    // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
    
    
    parameters.vuforiaLicenseKey = "ARz9Amr/////AAABmYnSycXqUkWZnTYbwDDfN5UDwEVORM6bykVMZjVch379D2K5QmoGTKd6jIw5efHY/XidkyYa93qUXRJCONKDuM1kuf5QtvcmeP/8mzMc9MCcqOIcfrURP1dhdtgXJuValhUhGcmem2+lKSIjWn92qkEv+6CRcwgI/BpFKlUAJ1cewCGb5K/2c+CRAdbMhbDtDFWhOkKuRBX9wb0GtR+X8SjH+O4qqLCJIipUF+34ITAYZifsXe+1jALmQqkck/hGgp5fsErEqXsPp7OxeDvwE3f5ecTOVYnBs1ZbjxmmmsS6PbUdAuHuahutptW2d99LbfpW1peOwWXGAKqzJ+v9k/7KzYWTKp33aqjeTC0KO9lO";
    parameters.cameraName = webcamName;
    parameters.useExtendedTracking = false;
    
    //  Instantiate the Vuforia engine
    vuforia = ClassFactory.getInstance().createVuforia(parameters);
  }
  
  /*
   * Initialize the Tensor Flow Object Detection engine.
   */
  private void initTfod()
  {
    int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
      "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    
    // init with monitor scree
    TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
    
    // init with no monitor screen
    // TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters();
    
    // set the minimumConfidence to a higher percentage to be more selective when identifying objects.
    tfodParameters.minResultConfidence = (float)0.75;
    
    tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
    tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
  }
}