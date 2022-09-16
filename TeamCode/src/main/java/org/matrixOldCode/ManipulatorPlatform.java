package org.matrixOldCode;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;

public class ManipulatorPlatform
{
  private DcMotorEx spinnerMotor = null;
  private DcMotorEx armMotor = null;
  private DcMotorEx gathererMotor = null;
  
  private Servo gatherDrop = null;
  
  public NormalizedColorSensor colorSensorL = null;
  public NormalizedColorSensor colorSensorR = null;
  
  static final double armP = 15;
  static final double armI = 0;
  static final double armD = 0;
  static final double armF = 0;

  static final double gatherP = 0;
  static final double gatherI = 0;
  static final double gatherD = 0;
  static final double gatherF = 0;

  static final double spinnerP = 40;
  static final double spinnerI = 0;
  static final double spinnerD = 0;
  static final double spinnerF = 0;

  public final double highSpeed = 1.0;
  public final double lowSpeed = 0.5;
  
  public final int armGather = 35;
  public final int armLow = 468;
  public final int armMid = 1396;
  public final int armMax = 2422;
  
  public final double spinnerSpeedFull = 1;
  public final double spinnerSpeedStop = 0;
  
  ManipulatorPlatform(HardwareMap hardwareMap)
  {
    // Initialize the hardware variables. Note that the strings used here as parameters
    // to 'get' must correspond to the names assigned during the robot configuration
    // step (using the FTC Robot Controller app on the phone).
    
    armMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "armMotor");
    armMotor.setDirection(DcMotor.Direction.REVERSE);
    PIDFCoefficients armPIDNew = new PIDFCoefficients(armP, armI, armD, armF);
    armMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, armPIDNew);
    armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    armMotor.setTargetPosition(0);
    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    armMotor.setPower(0.4);
    /*
    spinnerMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "spinnerMotor");
    spinnerMotor.setDirection(DcMotor.Direction.REVERSE);
    PIDFCoefficients spinnerPIDNew = new PIDFCoefficients( spinnerP, spinnerI, spinnerD, spinnerF );
    spinnerMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,spinnerPIDNew);
    spinnerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    spinnerMotor.setVelocity(0, DEGREES);
    */
    spinnerMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "spinnerMotor");
    spinnerMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    spinnerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    
    gathererMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "gatherMotor");
    gathererMotor.setDirection(DcMotor.Direction.REVERSE);
    //PIDFCoefficients gathererPIDNew = new PIDFCoefficients(gatherP, gatherI, gatherD, gatherF);
    //gathererMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,gathererPIDNew);
    gathererMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  
    gatherDrop = hardwareMap.get(Servo.class,"gatherDrop");
    
    colorSensorL = hardwareMap.get(NormalizedColorSensor.class, "leftSensor");
    colorSensorR = hardwareMap.get(NormalizedColorSensor.class, "rightSensor");
    
  }
  
  void setArmPosition(int count)
  {
    armMotor.setTargetPosition(count);
  }
  //void setGathererPosition(int count) { gathererMotor.setTargetPosition(count); }
  
//  void homeWobblePosition()
//  {
//    wobbleMotor.setPower(0.5);
//    wobbleMotor.setTargetPosition(5000);  // go far
//    while(!wobbleZeroSensor.isPressed())
//    {
//
//    }
//
//  }


//  void setShooterRPM(double RPMs)
//  {
//    shooterMotor.setPower(RPMs);
//  }
//
//  double getShooterRPM()
//  {
//   return shooterMotor.getVelocity(DEGREES) / 6; // degrees/second converted to RPM
//  }
  
  void setSpinnerRPM(double RPMs)
  {
    spinnerMotor.setVelocity((RPMs * 6), DEGREES); // needs to be passed as degrees/second
  }
  
  double getSpinnerRPM()
  {
    return spinnerMotor.getVelocity(DEGREES) / 6; // degrees/second converted to RPM
  }

  
  void setSpinnerPower(double power)
  {
    spinnerMotor.setPower(power);
  }
  
  double getSpinnerPower()
  {
    return spinnerMotor.getPower();
  }
  
  void setGatherRPM(double RPMs)
  {
    gathererMotor.setVelocity((RPMs * 6), DEGREES); // needs to be passed as degrees/second
  }
  
  void setGatherPower(double power)
  {
    gathererMotor.setPower(power); // needs to be passed as degrees/second
  }
  
  double getGatherRPM()
  {
    return gathererMotor.getVelocity(DEGREES) / 6; // degrees/second converted to RPM
  }
  
  void invertGatherDropper()
  {
    gatherDrop.setPosition(1 - gatherDrop.getPosition());
  }
  
  void initGatherDropper()
  {
    gatherDrop.setPosition(0);
  }
  
  /*  void resetExtender()
  {
    boolean calFlag = true;
    extenderPosition(-200);
    while( calFlag )
    {
      // wait till it hits the switch or stops moving
      if (extenderRetractedSensor.isPressed() || Math.abs(extenderMotor.getCurrentPosition()) > 150 )
      {
        calFlag = false;
      }
    }
    extenderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    extenderMotor.setTargetPosition(0);
    extenderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    extenderMotor.setPower(1);
  }*/

}
