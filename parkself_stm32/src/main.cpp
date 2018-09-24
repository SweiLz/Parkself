#include <mbed.h>
#include <dynamixel.h>
#include <mpu9250.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Empty.h>
geometry_msgs::Twist cmd_msg;
sensor_msgs::Imu imu_msg;

Dynamixel dx(PA_9, PA_10, 1000000, PB_0);
MPU9250 imu(PB_9, PB_8);
bool lift = false;

void servoCB(const std_msgs::Empty &empty_msg)
{
    lift = !lift;
    if (lift)
        dx.setPosition(0x04, 700, 350);
    else
        dx.setPosition(0x04, 1900, 350);
}
void moveBase(const geometry_msgs::Twist &twist_msg)
{
    uint8_t ids[3] = {0x01, 0x02, 0x03};
    float V[3];
    bool dirs[3];
    uint16_t speeds[3];
    float Vx = twist_msg.linear.x;
    float Vy = twist_msg.linear.y;
    float W = twist_msg.angular.z;
    V[0] = Vy + W * 0.115;
    V[1] = -Vx * 0.866025404 - Vy * 0.5 + W * 0.115;
    V[2] = Vx * 0.866025404 - Vy * 0.5 + W * 0.115;
    for (uint8_t i = 0; i < 3; i++)
    {
        dirs[i] = V[i] < 0;
        speeds[i] = (uint16_t)(abs(V[i]) * 5712.9);
        if (speeds[i] > 1023)
            speeds[i] = 1023;
        if (speeds[i] < 1)
            speeds[i] = 0;
    }
    dx.setWheel3Speed(ids, dirs, speeds);
}
void drive(float vx, float vy, float w)
{
    cmd_msg.linear.x = vx;
    cmd_msg.linear.y = vy;
    cmd_msg.angular.z = w;
    moveBase(cmd_msg);
}

ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", &moveBase);
ros::Subscriber<std_msgs::Empty> servo("servo", &servoCB);
ros::Publisher imu_pub("imu", &imu_msg);

/*
class Parkself
{
  private:
    bool lift;
    uint8_t ids[3];

  public:
    Dynamixel *dx;
    MPU9250 *imu;
    ros::Subscriber<geometry_msgs::Twist> *cmd_vel;
    ros::Subscriber<std_msgs::Empty> *servo;
    Parkself(void)
    {
        cmd_vel = new ros::Subscriber<geometry_msgs::Twist>("cmd_vel", &moveBase);
        servo = new ros::Subscriber<std_msgs::Empty>("servo", &servoCB);
        dx = new Dynamixel(PA_9, PA_10, 1000000, PB_0);
        imu = new MPU9250(PB_9, PB_8);
        ids[0] = 0x01;
        ids[1] = 0x02;
        ids[2] = 0x03;
        lift = false;
    }
    void servoCB(const std_msgs::Empty &empty_msg)
    {
        lift = !lift;
        if (lift)
            dx->setPosition(0x04, 700, 350);
        else
            dx->setPosition(0x04, 1900, 350);
    }
    void moveBase(const geometry_msgs::Twist &twist_msg)
    {
        float V[3];
        bool dirs[3];
        uint16_t speeds[3];
        float Vx = twist_msg.linear.x;
        float Vy = twist_msg.linear.y;
        float W = twist_msg.angular.z;
        V[0] = Vy + W * 0.115;
        V[1] = -Vx * 0.866025404 - Vy * 0.5 + W * 0.115;
        V[2] = Vx * 0.866025404 - Vy * 0.5 + W * 0.115;
        for (uint8_t i = 0; i < 3; i++)
        {
            dirs[i] = V[i] < 0;
            speeds[i] = (uint16_t)(abs(V[i]) * 5712.9);
            if (speeds[i] > 1023)
                speeds[i] = 1023;
            if (speeds[i] < 1)
                speeds[i] = 0;
        }
        dx->setWheel3Speed(ids, dirs, speeds);
    }
    void drive(float vx, float vy, float w)
    {
        cmd_msg.linear.x = vx;
        cmd_msg.linear.y = vy;
        cmd_msg.angular.z = w;
        moveBase(cmd_msg);
    }
};*/

ros::NodeHandle nh;

float selfTest[6] = {0};
int main()
{
    // Parkself robot;
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.subscribe(cmd_vel);
    nh.subscribe(servo);
    nh.advertise(imu_pub);
    imu_msg.header.frame_id = "base_stabilized";
    wait(1.0);

    // dx.setMode(0x01, WHEEL);
    // dx.setMode(0x02, WHEEL);
    // dx.setMode(0x03, WHEEL);
    // dx.setID(0xFE, 0x04);
    // dx.setMode(0x04, SERVO, 0, 2048);
    // dx.setBaudrate(0xFE, 0);
    dx.setPosition(0x04, 1900, 350);

    Timer tim;
    tim.start();
    while (1)
    {

        imu.updateData();
        if (tim.read_ms() > 500)
        {
            imu_msg.header.stamp = nh.now();
            imu_msg.angular_velocity.x = imu.gyroData[0];
            imu_msg.angular_velocity.y = imu.gyroData[1];
            imu_msg.angular_velocity.z = imu.gyroData[2];
            imu_msg.linear_acceleration.x = imu.accelData[0];
            imu_msg.linear_acceleration.y = imu.accelData[1];
            imu_msg.linear_acceleration.z = imu.accelData[2];
            imu_msg.orientation.x = imu.q[0];
            imu_msg.orientation.y = imu.q[1];
            imu_msg.orientation.z = imu.q[2];
            imu_msg.orientation.w = imu.q[3];
            imu_pub.publish(&imu_msg);
            tim.reset();
        }
        nh.spinOnce();

        // dx.setLed(1, 0);
        // dx.setLed(2, 0);
        // dx.setLed(3, 0);
        // wait(0.5);
        // dx.setLed(1, 1);
        // wait(0.5);
        // dx.setLed(2, 1);
        // wait(0.5);
        // dx.setLed(3, 1);
        // wait(0.5);

        // dx.setWheelSpeed(1, 0, 1000);
        // dx.setWheelSpeed(2, 0, 1000);
        // wait_ms(2000);

        // dx.setPosition(0x04, 0, 0);
        // wait(2.0);
        // dx.setPosition(0x04, 0, 0);
        // wait(2.0);
        // imu.readData();
        /*        pc.printf("ax = %f", 1000 * imu.accelData[0]);
        pc.printf(" ay = %f", 1000 * imu.accelData[1]);
        pc.printf(" az = %f  mg\n\r", 1000 * imu.accelData[2]);

        pc.printf("gx = %f", imu.gyroData[0]);
        pc.printf(" gy = %f", imu.gyroData[1]);
        pc.printf(" gz = %f  deg/s\n\r", imu.gyroData[2]);*/
        // pc.printf("gx, gy, gz: %f %f %f\n\r", imu.gyroData[0], imu.gyroData[1], imu.gyroData[2]);
        // wait_ms(100);
        // wait_ms(1);

        // drive(0, 0.3, 0);
        // wait(2);
        // drive(0.3, 0, 0);
        // wait(2);
        // drive(0, -0.3, 0);
        // wait(2);
        // drive(-0.3, 0, 0);
        // wait(2);
    }
}