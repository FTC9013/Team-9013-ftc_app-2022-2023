package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AutonomousMoveRight", group = "Concept")
public class AutonomousMoveRight extends LinearOpMode
{
  private ElapsedTime runTime = new ElapsedTime();
  Robot robot;
  
  double power = 0.55;
  public static double vD = 0;
  public static double thetaD = 0;
  int distance = 12;
  
  
  @Override
  public void runOpMode() throws InterruptedException
  {
    robot = new Robot(hardwareMap);
    
    telemetry.addData("Status", "Initialized");
    telemetry.update();
    
    robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    robot.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    robot.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    
    waitForStart();
    
    runTime.reset();
    double threshold = 1;
    moveRight(power);
    telemetry.addData("Robot is moving right: ", 1000);
    telemetry.update();
    
    while (opModeIsActive() && runTime.time() < threshold)
    {
      //Do nothing. Allows the motors to spin
    }
    stopMotor();
    telemetry.addData("Robot is done moving right: ", "");
    telemetry.update();
  }
  
  public void stopMotor()
  {
    robot.leftFront.setPower(0);
    robot.leftRear.setPower(0);
    robot.rightFront.setPower(0);
    robot.rightRear.setPower(0);
  }
  
  public void configureMotors()
  {
    robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
  }
  
  /*
  stopMotor();
    robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/
  
  
  public void moveRight(double power)
  {
    robot.leftFront.setPower(power);
    robot.leftRear.setPower(-power);
    robot.rightFront.setPower(-power);
    robot.rightRear.setPower(power);
  }
}


