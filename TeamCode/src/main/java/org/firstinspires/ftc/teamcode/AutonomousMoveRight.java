package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AutonomousMoveRight", group = "Concept")
public class AutonomousMoveRight extends LinearOpMode
{
  // parameters.vuforiaLicenseKey = "ARz9Amr/////AAABmYnSycXqUkWZnTYbwDDfN5UDwEVORM6bykVMZjVch379D2K5QmoGTKd6jIw5efHY/XidkyYa93qUXRJCONKDuM1kuf5QtvcmeP/8mzMc9MCcqOIcfrURP1dhdtgXJuValhUhGcmem2+lKSIjWn92qkEv+6CRcwgI/BpFKlUAJ1cewCGb5K/2c+CRAdbMhbDtDFWhOkKuRBX9wb0GtR+X8SjH+O4qqLCJIipUF+34ITAYZifsXe+1jALmQqkck/hGgp5fsErEqXsPp7OxeDvwE3f5ecTOVYnBs1ZbjxmmmsS6PbUdAuHuahutptW2d99LbfpW1peOwWXGAKqzJ+v9k/7KzYWTKp33aqjeTC0KO9lO";
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
    /*
    robot.leftFront.setTargetPosition(distance);
    robot.rightFront.setTargetPosition(-distance);
    robot.leftRear.setTargetPosition(-distance);
    robot.rightRear.setTargetPosition(distance);
    */
    robot.leftFront.setPower(power);
    robot.leftRear.setPower(-power);
    robot.rightFront.setPower(-power);
    robot.rightRear.setPower(power);
    /*
    while (robot.leftFront.isBusy() && robot.rightFront.isBusy() && robot.leftRear.isBusy() && robot.rightRear.isBusy())
    {*/
  }
}


