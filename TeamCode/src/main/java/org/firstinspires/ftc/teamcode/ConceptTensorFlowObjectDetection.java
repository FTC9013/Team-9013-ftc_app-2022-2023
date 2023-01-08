/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@Autonomous(name = "Concept: TensorFlow Object Detection", group = "Concept")
//@Disabled
public class ConceptTensorFlowObjectDetection extends LinearOpMode
{
  

//  private static final String[] LABELS = {
//    "1 Bolt",
//    "2 Bulb",
//    "3 Panel"
//  };

//  private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
//  private static final String TFOD_MODEL_ASSET = "model_unquant.tflite";
  private static final String TFOD_MODEL_ASSET = "ftc20222_matrix9013.tflite";
  private static final String LABEL_FIRST_ELEMENT = "Dog";
  private static final String LABEL_SECOND_ELEMENT = "Single";


//  private static final String LABEL_FIRST_ELEMENT = "Square";
//  private static final String LABEL_SECOND_ELEMENT = "Triangle";
//  private static final String LABEL_THIRD_ELEMENT = "Circle";
  
  private static final String VUFORIA_KEY =
          "ARz9Amr/////AAABmYnSycXqUkWZnTYbwDDfN5UDwEVORM6bykVMZjVch379D2K5QmoGTKd6jIw5efHY/XidkyYa93qUXRJCONKDuM1kuf5QtvcmeP/8mzMc9MCcqOIcfrURP1dhdtgXJuValhUhGcmem2+lKSIjWn92qkEv+6CRcwgI/BpFKlUAJ1cewCGb5K/2c+CRAdbMhbDtDFWhOkKuRBX9wb0GtR+X8SjH+O4qqLCJIipUF+34ITAYZifsXe+1jALmQqkck/hGgp5fsErEqXsPp7OxeDvwE3f5ecTOVYnBs1ZbjxmmmsS6PbUdAuHuahutptW2d99LbfpW1peOwWXGAKqzJ+v9k/7KzYWTKp33aqjeTC0KO9lO";
  
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
  
  @Override
  public void runOpMode()
  {
    // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
    // first.
    
    webcamName = hardwareMap.get(WebcamName.class, "Webcam");
    
    
    initVuforia();
    initTfod();
    
    /**
     * Activate TensorFlow Object Detection before we wait for the start command.
     * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
     **/
    if (tfod != null)
    {
      tfod.activate();
      
      // The TensorFlow software will scale the input images from the camera to a lower resolution.
      // This can result in lower detection accuracy at longer distances (> 55cm or 22").
      // If your target is at distance greater than 50 cm (20") you can increase the magnification value
      // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
      // should be set to the value of the images used to create the TensorFlow Object Detection model
      // (typically 16/9).
      tfod.setZoom(2.5, 16.0 / 9.0);
    }
    
    /** Wait for the game to begin */
    telemetry.addData(">", "Press Play to start op mode");
    telemetry.update();
    waitForStart();
    
    if (opModeIsActive())
    {
      while (opModeIsActive())
      {
        if (tfod != null)
        {
          // getUpdatedRecognitions() will return null if no new information is available since
          // the last time that call was made.
          List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
          if (updatedRecognitions != null)
          {
            telemetry.addData("# Objects Detected", updatedRecognitions.size());
            telemetry.addData("Vbrose", "Marker1");

            if (updatedRecognitions.size() == 0 )
            {
              // empty list.  no objects recognized.
              telemetry.addData("TFOD", "No items detected.");
//              telemetry.addData("Target Zone", "A");
            }
            else {
              // step through the list of recognitions and display image position/size information for each one
              // Note: "Image number" refers to the randomized image orientation/number

              int i =0;
              for (Recognition recognition : updatedRecognitions) {
//                double col = (recognition.getLeft() + recognition.getRight()) / 2;
//                double row = (recognition.getTop() + recognition.getBottom()) / 2;
//                double width = Math.abs(recognition.getRight() - recognition.getLeft());
//                double height = Math.abs(recognition.getTop() - recognition.getBottom());


//                telemetry.addData("", " ");
//                telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
//                telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
//                telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);
                  telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                  telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                          recognition.getLeft(), recognition.getTop());
                  telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                          recognition.getRight(), recognition.getBottom());
                  i++;

              }
            }
            telemetry.update();
          }
        }
      }
    }
  }
  
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
    
    //WebcamName
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
    tfodParameters.minResultConfidence = (float) 0.15;
    
    tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
    tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
  }

}

