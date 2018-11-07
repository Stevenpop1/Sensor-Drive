/* Sensor Drive by Steven Popovitz
 * 
 * Autonomous driving class for FRC robots.
 * Requires: Three Ultrasonic sensors(front,right,and left) and a Analog IMU.
 * 
 * Creates a grid with the robot's starting position as the origin.
 * Accounts for obstacles in its path and adjusts the route accordingly.
 * Run each method in driver class until it returns true to get its full 
 * function.
 */

package com.analog.adis16448.frc;

import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.Timer;
import com.analog.adis16448.frc.ADIS16448_IMU;
public class SDrive {
	 private ADIS16448_IMU IMU;
	 private Ultrasonic UltraF; 
	 private Ultrasonic UltraR; 
	 private Ultrasonic UltraL;
	 private double disY; //Distance Y from origin
	 private double disX; //Distance X from origin
	 private double curX; //Robot X position on grid
	 private double curY; //Robot Y position on grid
	 private Talon rightm;
	 private Talon leftm;
	 private int step; //Step in around method
	 private char turn; //Direction to turn
	 private int mod; //If odd another obstacle
	 private Timer tim;
	 private double vel; //Velocity of robot
	 private boolean clear;
	 private int tu; //Turn step in move method
	 
	 
	 public SDrive(ADIS16448_IMU imu, Ultrasonic USfr,Ultrasonic USri,  Ultrasonic USlf, WPI_TalonSRX Tri, WPI_TalonSRX Tlf) //For teams using Talon SRXs
	 			  //IMU, Front UltraSonic, Right UltraSonic, Left UltraSonic, Right motor, Left motor
	 {
		 disY=0;
		 disX=0;
		 curX=0;
		 curY=0;
		 step=0;
		 turn=' ';
		 mod=0;
		 vel = 0;
		 clear = true;
		 tu = 0;
		 
		 //IMU
		 IMU = imu;
		 
		 //Ultrasonic
		 UltraF = USfr;
		 UltraF.setAutomaticMode(true);
		 UltraR = USri;
		 UltraR.setAutomaticMode(true);
		 UltraL = USlf;
		 UltraL.setAutomaticMode(true);
		 
		 //Motor
		 WPI_TalonSRX rightm = Tri;
		 WPI_TalonSRX leftm = Tlf;
	 }
	 public SDrive(ADIS16448_IMU imu, Ultrasonic USfr,Ultrasonic USri,  Ultrasonic USlf, Talon Tri, Talon Tlf) //For teams not using Talon SRXs
	  //IMU, Front UltraSonic, Right UltraSonic, Left UltraSonic, Right motor, Left motor
	 {
		 disY=0;
		 disX=0;
		 curX=0;
		 curY=0;
		 step=0;
		 turn=' ';
		 mod=0;
		 vel=0;
		 clear = true;
		 tu = 0;

		 //IMU
		 IMU = imu;

		 //Ultrasonic
		 UltraF = USfr;
		 UltraF.setAutomaticMode(true);
		 UltraR = USri;
		 UltraR.setAutomaticMode(true);
		 UltraL = USlf;
		 UltraL.setAutomaticMode(true);

		 //Motor
		 rightm = Tri;
		 leftm = Tlf;
	 }
	 
	 public boolean Turn(double Angle) { //Turns robot Angle amount in either right or left
		 double end = IMU.getAngle()+Angle;
		 if(Angle>0) { //Turn Right
			 if(IMU.getAngle()<end) {
				 leftm.set(.2);
				 if(IMU.getAngle() == end) {
					 leftm.set(0.0);
					 return true;
				 }
			 }
			 
		 }
		 else if(Angle<0) { //Turn Left
			 if(IMU.getAngle()>end) {
				 rightm.set(.2);
				 if(IMU.getAngle() == end) {
					 rightm.set(0.0);
					 return true;
				 }
			 }
		 }
		else if(Angle == 0) { //No turn
			 return true;
		 }
		 
		return false; //Not done
	 }
	 
	 public boolean Around() { //Moves around object
		tim.reset();
		tim.start();
		if(step == 0) {
		 if((disX-curX) < 0 && UltraL.getRangeMM()>850.0) { //Destination is left and left is clear
			 turn = 'L';
			 step++;
		 }
		 else if((disX-curX) > 0 && UltraR.getRangeMM()>850.0) { //Destination is right and right is clear
			 turn = 'R';
			 step++;
		 }
		 else{ 	//Direction of destination is blocked
			 if(UltraR.getRangeMM()>850.0) { //Right is clear
				 turn = 'R';
				 step++;
			 }
			 else if(UltraL.getRangeMM()>850.0) { //Left is clear
				 turn = 'L';
				 step++;
			 }
		 }
	 }
		else if(step == 1) {     //Turning phase
			if(turn == 'R') { //Right
				if(Turn(90)) {
					step++;
				}
			}
			else if(turn == 'L') { //Left
				if(Turn(-90)) {
					step++;
				}
			}
		}
		else if(step == 2) {	//Moving phase
			if((mod&2) != 0) { //Changes parameters for when the robot hits another obstacle
				if(turn == 'R') {
					turn = 'L';
				}
				else
					turn = 'R';
			}
			if(turn == 'R') { //Originally turned right
				if(UltraF.getRangeMM()< 100) {	//If robot hits another object
					mod++;
					step--;
					rightm.set(0.0);
					leftm.set(0.0);
					vel = 0;
				}
				else if(UltraL.getRangeMM()<280.0) {	//Clear to turn
					rightm.set(0.0);
					leftm.set(0.0);
					vel = 0;
					turn = 'L';
					step++;
				}
				else {		//Moves robot and adjusts its position stored in the code
					rightm.set(.5);
					leftm.set(.5);
					if((mod%2)==0) {
						curX += (vel*tim.get())+(.5*IMU.getAccelX()*Math.pow(tim.get(), 2));
						vel += IMU.getAccelX()*tim.get();
					}
					else {	//For when the robot hits the second obstacle to update its position 
						curY -= (vel*tim.get())+(.5*IMU.getAccelX()*Math.pow(tim.get(), 2));
						vel += IMU.getAccelX()*tim.get();
					}
					
				}
			}
			else if(turn == 'L') {
				if(UltraF.getRangeMM()< 100) {
					mod++;
					step--;
					rightm.set(0.0);
					leftm.set(0.0);
					vel = 0;
				}
				else if(UltraR.getRangeMM()<280.0) {
					rightm.set(0.0);
					leftm.set(0.0);
					vel = 0;
					turn = 'R';
					step++;
				}
				else {
					rightm.set(.5);
					leftm.set(.5);
					if((mod%2)==0) {
						curX -= (vel*tim.get())+(.5*IMU.getAccelX()*Math.pow(tim.get(), 2));
						vel += IMU.getAccelX()*tim.get();
					}
					else {
						curY -= (vel*tim.get())+(.5*IMU.getAccelX()*Math.pow(tim.get(), 2));
						vel += IMU.getAccelX()*tim.get();
					}
				}
			}
		}
		else if(step == 3) {  //Second turning phase
			if(turn == 'R') {
				if(Turn(90)) {
					if((mod%2)==0) { //Done
						step = 0;
						return true;
					}
					else { //Runs move phase again for the robot that hit another object
						step = 2;
						mod++;
						turn = 'L';
					}
				}
			}
			else if(turn == 'L') {
				if(Turn(-90)) {
					if((mod%2)==0) {
						step = 0;
						return true;
						}
						else {
							step = 2;
							mod++;
							turn = 'R';
						}
				}
			}
		}
		 return false; //Not done
	 
	 }
	 
	 public void reset_origin() { //Resets origin to the robot's current position
		 curX = 0;
		 curY = 0;
	 }
	 
	 public boolean move(double X, double Y) { //Moves the robot to the designated location
		 disX = X;
		 disY = Y;
		 tim.reset();
		 tim.start();
		 if(disX == curX && disY == curY) { //Reached destination 
			 rightm.set(0.0);
			 leftm.set(0.0);
			 vel = 0;
			 return true;
		 }
		 
		 else if(UltraF.getRangeMM()< 100||!clear) { //Obstacle
			 clear = false;
			 if(Around()) {
				 clear = true;
			 }
		 }
		 else {
			 if(curY<disY) { //If ahead
				 rightm.set(0.5);
				 leftm.set(0.5);
				 curY += (vel*tim.get())+(.5*IMU.getAccelY()*Math.pow(tim.get(), 2));
				 vel += IMU.getAccelY()*tim.get();
			 }
			 else if((curY>disY)) { //If behind
				 rightm.set(-0.5);
				 leftm.set(-0.5);
				 curY += (vel*tim.get())+(.5*IMU.getAccelY()*Math.pow(tim.get(), 2));
				 vel += IMU.getAccelY()*tim.get();
			 }
			 else {
				 if(disX>curX) { //If to the right
					 if(tu == 0) { //Turn phase
						 rightm.set(0.0);
						 leftm.set(0.0);
						 vel = 0;
						 if(Turn(90.0)) {
							 tu=1;
						 }
					 }
					 if(tu == 1) { //Move phase
						 curX += (vel*tim.get())+(.5*IMU.getAccelX()*Math.pow(tim.get(), 2)); //Updates robot location
						 vel += IMU.getAccelX()*tim.get(); //Updates velocity
						 rightm.set(0.5);
						 leftm.set(0.5);
					 }
				 }
				 else if(disX<curX) { //If to the left
					 if(tu == 0) {
						 rightm.set(0.0);
						 leftm.set(0.0);
						 vel = 0;
						 if(Turn(-90.0)) {
							 tu=1;
						 }
					 }
					 if(tu == 1) {
						 curX -= (vel*tim.get())+(.5*IMU.getAccelX()*Math.pow(tim.get(), 2));
						 vel += IMU.getAccelX()*tim.get();
						 rightm.set(0.5);
						 leftm.set(0.5);
					 }
				 }
			 }
		 }
		 
		 
		 
		 return false; //Not done
	 }
}
