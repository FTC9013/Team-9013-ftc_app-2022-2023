package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmControl
{
  private final DcMotor armMotor;
  
  ArmControl(HardwareMap hardwareMap, Telemetry theTelemetry)
  {
  
    // Initialize the hardware variables
    armMotor = hardwareMap.get(DcMotor.class, "arm");
  
  
    armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  
  
    // Motors on one side reversed to drive forward
    // Reverse the motor that runs backwards when connected directly to the battery
    // A positive power number should drive the robot forward regardless of the motor's position on the robot.
    armMotor.setDirection(DcMotor.Direction.REVERSE);
    
    armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
  }
  
  public void raise()
  {
    armMotor.setPower(-0.5);
  }
  
  public void lower()
  {
    armMotor.setPower(0.5);
  }
}







