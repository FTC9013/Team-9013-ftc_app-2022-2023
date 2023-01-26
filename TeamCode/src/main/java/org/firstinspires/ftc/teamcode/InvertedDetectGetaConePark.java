package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name = "Concept: InvertedDetectGetaConePark", group = "Concept")
public class InvertedDetectGetaConePark extends LinearOpMode
{
  private ElapsedTime runTime = new ElapsedTime();
  double power = 0.4;
  int distance = 1250;
  private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
  private static final String[] LABELS = {
    "1 Bolt",
    "2 Bulb",
    "3 Panel"
  };
  
  private double threshold = 0.85;
  
  private static final String VUFORIA_KEY = "ARz9Amr/////AAABmYnSycXqUkWZnTYbwDDfN5UDwEVORM6bykVMZjVch379D2K5QmoGTKd6jIw5efHY/XidkyYa93qUXRJCONKDuM1kuf5QtvcmeP/8mzMc9MCcqOIcfrURP1dhdtgXJuValhUhGcmem2+lKSIjWn92qkEv+6CRcwgI/BpFKlUAJ1cewCGb5K/2c+CRAdbMhbDtDFWhOkKuRBX9wb0GtR+X8SjH+O4qqLCJIipUF+34ITAYZifsXe+1jALmQqkck/hGgp5fsErEqXsPp7OxeDvwE3f5ecTOVYnBs1ZbjxmmmsS6PbUdAuHuahutptW2d99LbfpW1peOwWXGAKqzJ+v9k/7KzYWTKp33aqjeTC0KO9lO";
  
  
  private VuforiaLocalizer vuforia;
  private TFObjectDetector tfod;
  private ArmControl autoarm;
  private GripperControl autogripper;
  
  
  private void initTfod()
  {
    int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
      "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
    tfodParameters.minResultConfidence = 0.75f;
    tfodParameters.isModelTensorFlow2 = true;
    tfodParameters.inputSize = 300;
    tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
    // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
    // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
    tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
  }
  
  private void initVuforia()
  {
    /*
     * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
     */
    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
    
    parameters.vuforiaLicenseKey = VUFORIA_KEY;
    parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam");
    
    //  Instantiate the Vuforia engine
    vuforia = ClassFactory.getInstance().createVuforia(parameters);
  }
  
  Robot robot;
  
  public void stopMotor()
  {
    robot.leftFront.setPower(0);
    robot.leftRear.setPower(0);
    robot.rightFront.setPower(0);
    robot.rightRear.setPower(0);
  }
  
  public void moveRight(int distance)
  {
    robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    
    robot.leftFront.setTargetPosition(distance);
    robot.rightFront.setTargetPosition(-distance);
    robot.leftRear.setTargetPosition(-distance);
    robot.rightRear.setTargetPosition(distance);
    
    robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    
    robot.leftFront.setPower(power);
    robot.leftRear.setPower(power);
    robot.rightFront.setPower(power);
    robot.rightRear.setPower(power);
    
    while (opModeIsActive() && robot.leftFront.isBusy() && robot.rightFront.isBusy() && robot.leftRear.isBusy() && robot.rightRear.isBusy())
    {
      //Do nothing. Allows the motors to spin
    }
    stopMotor();
  }
  
  public void moveForward(int distance)
  {
    //double runDuration = 1.1;
    //runTime.reset();
    robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    
    robot.leftFront.setTargetPosition(distance);
    robot.rightFront.setTargetPosition(distance);
    robot.leftRear.setTargetPosition(distance);
    robot.rightRear.setTargetPosition(distance);
    
    robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    
    robot.leftFront.setPower(power);
    robot.leftRear.setPower(power);
    robot.rightFront.setPower(power);
    robot.rightRear.setPower(power);
    
    while (opModeIsActive() && robot.leftFront.isBusy() && robot.rightFront.isBusy() && robot.leftRear.isBusy() && robot.rightRear.isBusy())
    {
      //Do nothing. Allows the motors to spin
    }
    stopMotor();
  }
  
  private void moveBack(int distance)
  {
    robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    
    robot.leftFront.setTargetPosition(-distance);
    robot.rightFront.setTargetPosition(-distance);
    robot.leftRear.setTargetPosition(-distance);
    robot.rightRear.setTargetPosition(-distance);
    
    robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    
    robot.leftFront.setPower(power);
    robot.leftRear.setPower(power);
    robot.rightFront.setPower(power);
    robot.rightRear.setPower(power);
    
    while (opModeIsActive() && robot.leftFront.isBusy() && robot.rightFront.isBusy() && robot.leftRear.isBusy() && robot.rightRear.isBusy())
    {
      //Do nothing. Allows the motors to spin
    }
    stopMotor();
  }
  
  
  public void moveLeft(int distance)
  {
    robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    
    robot.leftFront.setTargetPosition(-distance);
    robot.rightFront.setTargetPosition(distance);
    robot.leftRear.setTargetPosition(distance);
    robot.rightRear.setTargetPosition(-distance);
    
    robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    
    robot.leftFront.setPower(power);
    robot.leftRear.setPower(power);
    robot.rightFront.setPower(power);
    robot.rightRear.setPower(power);
    
    while (opModeIsActive() && robot.leftFront.isBusy() && robot.rightFront.isBusy() && robot.leftRear.isBusy() && robot.rightRear.isBusy())
    {
      //Do nothing. Allows the motors to spin
    }
    stopMotor();
  }
  
  public void raise(double runDuration)
  {
    runTime.reset();
    autoarm.raise();
    while (opModeIsActive() && runTime.time() < runDuration)
    {
      //Do nothing. Allows the motors to spin
    }
    telemetry.addData("Raising", "True");
    autoarm.stop();
  }
  
  public void lower(double lowerDuration)
  {
    runTime.reset();
    autoarm.lower();
    
    while (opModeIsActive() && runTime.time() < lowerDuration)
    {
      //Do nothing. Allows the motors to spin
    }
    telemetry.addData("Lowering", "True");
    autoarm.stop();
  }
  
  private void drop(double gripperDuration)
  {
    runTime.reset();
    autogripper.pushOut();
    
    while (opModeIsActive() && runTime.time() < gripperDuration)
    {
      //Do nothing. Allows the motors to spin
    }
    autogripper.stop();
  }
  
  @Override
  public void runOpMode() throws InterruptedException
  {
    robot = new Robot(hardwareMap);
    initVuforia();
    autoarm = new ArmControl(hardwareMap, telemetry);
    autogripper = new GripperControl(hardwareMap, telemetry);
    initTfod();
    
    robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    robot.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    robot.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    
    if (tfod != null)
    {
      tfod.activate();
      
      // The TensorFlow software will scale the input images from the camera to a lower resolution.
      // This can result in lower detection accuracy at longer distances (> 55cm or 22").
      // If your target is at distance greater than 50 cm (20") you can increase the magnification value
      // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
      // should be set to the value of the images used to create the TensorFlow Object Detection model
      // (typically 16/9).
      tfod.setZoom(1.0, 16.0 / 9.0);
    }
    
    telemetry.addData(">", "Press Play to start op mode");
    telemetry.update();
    waitForStart();
    
    String label = null;
    moveForward(75);
    raise(1.25);
    if (opModeIsActive())
    {
      while (opModeIsActive() && label == null)
      {
        if (tfod != null)
        {
          // getUpdatedRecognitions() will return null if no new information is available since
          // the last time that call was made.
          List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
          if (updatedRecognitions != null)
          {
            telemetry.addData("# Objects Detected", updatedRecognitions.size());
            
            // step through the list of recognitions and display image position/size information for each one
            // Note: "Image number" refers to the randomized image orientation/number
            for (Recognition recognition : updatedRecognitions)
            {
              double col = (recognition.getLeft() + recognition.getRight()) / 2;
              double row = (recognition.getTop() + recognition.getBottom()) / 2;
              double width = Math.abs(recognition.getRight() - recognition.getLeft());
              double height = Math.abs(recognition.getTop() - recognition.getBottom());
              
              telemetry.addData("", " ");
              telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
              telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
              telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);
              
              if (recognition.getConfidence() * 100 > threshold)
              {
                label = recognition.getLabel();
                break;
              }
            }
            telemetry.update();
          } else // no objects are detected
          {
            //moveFow();
            telemetry.addData("No objects detected. Moving Forward.", 0);
            telemetry.update();
          }
        }
      }
      
      telemetry.addData("We have recognized the image!", label);
      telemetry.update();
      
      lower(0.75);
      moveForward(1175);
      moveLeft(450);
      raise(2);
      moveForward(300);
      sleep(2000);
      lower(1);
      drop(1);
      moveBack(325);
      
      if (label == LABELS[0])
      {
        telemetry.addData("Robot is supposed to move Left: ", 1000);
        moveLeft(650);
      } else if (label == LABELS[1])
      {
        telemetry.addData("Robot is supposed to move Forward", 1000);
        moveRight(650);
      } else if (label == LABELS[2])
      {
        telemetry.addData("Robot is supposed to move Right: ", 1000);
        moveRight(1700);
      }
    }
  }
}

