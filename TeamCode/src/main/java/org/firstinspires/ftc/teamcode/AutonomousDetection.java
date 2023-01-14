package org.firstinspires.ftc.teamcode;
/*
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;


@Autonomous(name = "AutonomousDetection", group = "Concept")
public abstract class AutonomousDetection extends LinearOpMode
{
  Robot robot = new Robot();
  private final Telemetry telemetry;
  
  AutonomousDetection(HardwareMap hardwareMap, Telemetry theTelemetry, Telemetry telemetry){
  
    this.telemetry = telemetry;
    double power = 0.5;
    int distance = 1;
    
    public void runOpMode() throws InterruptedException
    {
      telemetry.addData("Status", "Initialized");
      telemetry.update();
      
      
      waitForStart();
      while opModeisActive(){
      
      }
    }
    public void moveLeft(double power){
      robot.leftFront.setPower(power);
      robot.leftRear.setPower(power);
      robot.rightFront.setPower(-power);
      robot.rightRear.setPower(-power);
      
      robot.leftFront.setTargetPosition(100);
      robot.leftRear.setTargetPosition(100);
      robot.rightFront.setTargetPosition(-100);
      robot.rightRear.setTargetPosition(-100);
    }
    public void moveRight(double power){
      robot.leftFront.setPower(-power);
      robot.leftRear.setPower(-power);
      robot.rightFront.setPower(power);
      robot.rightRear.setPower(-power);
      
      robot.leftFront.setTargetPosition(-100);
      robot.leftRear.setTargetPosition(-100);
      robot.rightRear.setTargetPosition(100);
      robot.rightFront.setTargetPosition(100);
    }
    public void moveForward(double power){
      robot.leftFront.setPower(power);
      robot.rightFront.setPower(power);
      robot.leftRear.setPower(power);
      robot.rightRear.setPower(power);
      
      robot.leftFront.setTargetPosition(100);
      robot.rightFront.setTargetPosition(100);
      robot.rightRear.setTargetPosition(100);
      robot.leftRear.setTargetPosition(100);
    }
    public void detect(){
    
    }
  }
}*/


