#include <mbed.h>
#include <dynamixel.h>
#include <mpu9250.h>
//#include <ros.h>
//#include "geometry_msgs/Twist.h"
//#include "sensor_msgs/Imu.h"

// sensor_msgs::Imu imu_msg;
// class Dynamixel
// {
// Dynamixel();
// };
struct Position {
    float x;
    float y;
    float theta;
};
//namespace ThreeWheelOmni{
class Parkself
{
private:
    //uint8_t ids[3] = {0x01, 0x02, 0x03};
    //uint8_t id_lift = 0x04;
    bool dirs[3];
    uint16_t speeds[3];

public:
    Dynamixel *dx;
    MPU9250 *imu;
    Serial *pc;
    Parkself(void) {
        dx = new Dynamixel(PA_9, PA_10, 1000000, PB_0);
        imu  = new MPU9250(PB_9, PB_8);
        pc = new Serial(USBTX, USBRX);
    }
    void liftUp ();
    void liftDown ();
    //void moveVel (const geometry_msgs::Twist &twist_msg);

};
/*void Parkself::moveVel (const geometry_msgs::Twist &twist_msg){
    float V[3];
    float Vx = twist_msg.linear.x;
    float Vy = twist_msg.linear.y;
    float W = twist_msg.angular.z;
    V[0] = Vy + W * R;
    V[1] = Vx * 0.866025404 - Vy * 0.5 + W * R;
    V[2] = -Vx * 0.866025404 - Vy * 0.5 + W * R;
    for (uint8_t i = 0; i < 3; i++)
    {
        dirs[i] = V[i] < 0;
        speeds[i] = abs(V[i]) / r / 0.0116;
        if (speeds[i] > 1023)
            speeds[i] = 1023;
    }
    dx.setWheel3Speed(ids, dirs, speeds);
}*/
//}
/*
ros::NodeHandle nh;

uint8_t ids[3] = {0x01, 0x02, 0x03};
bool dirs[3];
uint16_t speeds[3];

float R = 0.1;
float r = 0.03;

void cmd_velCB(const geometry_msgs::Twist &twist_msg)
{
    float V[3];
    float Vx = twist_msg.linear.x;
    float Vy = twist_msg.linear.y;
    float W = twist_msg.angular.z;
    V[0] = Vy + W * R;
    V[1] = Vx * 0.866025404 - Vy * 0.5 + W * R;
    V[2] = -Vx * 0.866025404 - Vy * 0.5 + W * R;
    for (uint8_t i = 0; i < 3; i++)
    {
        dirs[i] = V[i] < 0;
        speeds[i] = abs(V[i]) / r / 0.0116;
        if (speeds[i] > 1023)
            speeds[i] = 1023;
    }
    dx.setWheel3Speed(ids, dirs, speeds);
}

ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", &cmd_velCB);
*/

//float gyroBias[3] = {0};
//float accelBias[3] = {0};
float selfTest[6] = {0};
int main()
{
    //ThreeWheelOmni::Parkself Robot;
    Parkself Robot;

    //ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", &Robot.moveVel);
    Robot.dx->setMode(0xFE, WHEEL);
    Robot.pc->baud(9600);
    Robot.pc->printf("CPU SystemCoreClock is %d Hz\r\n", SystemCoreClock);
    Robot.pc->printf("I AM 0x%X\n\r", Robot.imu->whoami());
    //Robot.imu.reset();
    Robot.imu->test(&selfTest[0]);
    Robot.pc->printf("x-axis self test: acceleration trim within : %f % of factory value\n\r", selfTest[0]);
    Robot.pc->printf("y-axis self test: acceleration trim within : %f % of factory value\n\r", selfTest[1]);
    Robot.pc->printf("z-axis self test: acceleration trim within : %f % of factory value\n\r", selfTest[2]);
    Robot.pc->printf("x-axis self test: gyration trim within : %f % of factory value\n\r", selfTest[3]);
    Robot.pc->printf("y-axis self test: gyration trim within : %f % of factory value\n\r", selfTest[4]);
    Robot.pc->printf("z-axis self test: gyration trim within : %f % of factory value\n\r", selfTest[5]);
    // imu.calibrate(gyroBias,accelBias);

    // wait(1.0);
    // dx.setLed(2, 1);
    // wait(1.0);
    // nh.initNode();
    // nh.subscribe(cmd_vel);

    while (1) 
    {
        Robot.imu->updateData();
        //Robot.pc->printf("A :: %f %f %f\n",Robot.imu->gyroData[0],Robot.imu->gyroData[1],Robot.imu->gyroData[2]);
        //Robot.pc->printf("B :: %f %f %f\n",Robot.imu->accelData[0],Robot.imu->accelData[1],Robot.imu->accelData[2]);
        //Robot.pc->printf("C :: %f %f %f\n",Robot.imu->magData[0],Robot.imu->magData[1],Robot.imu->magData[2]);
        //Robot.pc->printf("C :: %f %f %f\n",Robot.imu->magCount[0],Robot.imu->magCount[1],Robot.imu->magCount[2]);
        Robot.pc->printf("Quarternion %f %f %f %f\n\r", Robot.imu->q[0], Robot.imu->q[1], Robot.imu->q[2], Robot.imu->q[3]);
        //Robot.pc->printf("Yaw, Pitch, Roll: %f %f %f\n\r", Robot.imu->yaw, Robot.imu->pitch, Robot.imu->roll);
        //Robot.pc->printf("-----------");
        // nh.spinOnce();
        // wait_ms(1);
        // dx.setLed(2, 0);
        // wait(1.0);
        // dx.setLed(2, 1);
        // wait(1.0);
        // drive(0, 0.5, 0);
        // wait(2.0);
        // drive(0.5, 0, 0);
        // wait(2.0);
        // drive(0, -0.5, 0);
        // wait(2.0);
        // drive(-0.5, 0, 0);
        // wait(2.0);
     }
}