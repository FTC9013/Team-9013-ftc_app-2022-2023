package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class GripperControl
{
  
  private final DcMotor gripperMotor;
  
  GripperControl(HardwareMap hardwareMap, Telemetry theTelemetry)
  {
  
    // Initialize the hardware variables
    gripperMotor = hardwareMap.get(DcMotor.class, "gripper");
  
  
    gripperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  
  
    // Motors on one side are reversed to drive forward
    // Reverse the motor that runs backwards when connected directly to the battery
    // A positive power number should drive the robot forward regardless of the motor's position on the robot
    gripperMotor.setDirection(DcMotor.Direction.REVERSE);
    
    gripperMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
  }
  
  public void pullIn()
  {
    gripperMotor.setPower(-0.2);
  }
  
  public void pushOut()
  {
    gripperMotor.setPower(0.05);
  }
  
  
  public void stop()
  {
    gripperMotor.setPower(0);
  }
}



  
