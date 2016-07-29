/// CONTROL     //
// (SPORADIC)  //
/////////////////

#define n_accx  0.0080653 // *10
#define n_accy  0.0076799 // *10
#define n_accz  0.043222 // *100
#define n_posx  0.00000019239
#define n_posy  0.0000040953
#define n_posz  0.0000042075
#define n_velx  0.000041904
#define n_vely  0.00072175
#define n_velz  0.00087454
#define n_bias  0.001

#include <sys/time.h>

//c++
#include <cmath>
#include <fstream>
#include <sstream>
#include <iostream>

#include "module0.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

using namespace std;

struct timeval starttime2, endtime2;

double dt=0.035;
int gps_flag=0;
rotors_control::EigenOdometry odometry;
// define thrust message
mav_msgs::RollPitchYawrateThrust thrust_msg;
ros::Publisher chatter_pub;
ros::Publisher vel_pub;


float A_mat[9][9]={{1, dt, -(dt*dt)/2, 0, 0, 0, 0, 0, 0},
                   {0, 1, -dt, 0, 0, 0, 0, 0, 0},
                   {0, 0, 1, 0, 0, 0, 0, 0, 0},
                   {0, 0, 0, 1, dt, -(dt*dt)/2, 0, 0, 0},
                   {0, 0, 0, 0, 1, -dt, 0, 0, 0},
                   {0, 0, 0, 0, 0, 1, 0, 0, 0},
                   {0, 0, 0, 0, 0, 0, 1, dt, -(dt*dt)/2},
                   {0, 0, 0, 0, 0, 0, 0, 1, -dt},
                   {0, 0, 0, 0, 0, 0, 0, 0, 1}};

float Q_mat[9][9]={{n_accx*(dt*dt*dt*dt)/4, n_accx*(dt*dt*dt)/2, n_accx*(dt*dt)/2, 0, 0, 0, 0, 0, 0},
                   {n_accx*(dt*dt*dt)/2, n_accx*(dt*dt), n_accx*dt, 0, 0, 0, 0, 0, 0},
                   {n_accx*(dt*dt)/2, n_accx*dt, n_accx, 0, 0, 0, 0, 0, 0},
                   {0, 0, 0, n_accy*(dt*dt*dt*dt)/4, n_accy*(dt*dt*dt)/2,n_accy*(dt*dt)/2, 0, 0, 0},
                   {0, 0, 0, n_accy*(dt*dt*dt)/2, n_accy*(dt*dt), n_accy*dt, 0, 0, 0},
                   {0, 0, 0, n_accy*(dt*dt)/2, n_accy*dt, n_accy, 0, 0, 0},
                   {0, 0, 0, 0, 0, 0, n_accz*(dt*dt*dt*dt)/4, n_accz*(dt*dt*dt)/2, n_accz*(dt*dt)/2},
                   {0, 0, 0, 0, 0, 0, n_accz*(dt*dt*dt)/2, n_accz*(dt*dt), n_accz*dt},
                   {0, 0, 0, 0, 0, 0, n_accz*(dt*dt)/2, n_accz*dt, n_accz}};


float B_mat[9][3]={{(dt*dt)/2, 0, 0},
                   {dt, 0, 0},
                   {1, 0, 0},
                   {0, (dt*dt)/2, 0},
                   {0, dt, 0},
                   {0, 1, 0},
                   {0, 0, (dt*dt)/2},
                   {0, 0, dt},
                   {0, 0, 1}};

float R_mat[9][9]={{n_posx,0,0,0,0,0,0,0,0},
                   {0,n_velx,0,0,0,0,0,0,0},
                   {0,0,n_bias,0,0,0,0,0,0},
                   {0,0,0,n_posy,0,0,0,0,0},
                   {0,0,0,0,n_vely,0,0,0,0},
                   {0,0,0,0,0,n_bias,0,0,0},
                   {0,0,0,0,0,0,n_posz,0,0},
                   {0,0,0,0,0,0,0,n_velz,0},
                   {0,0,0,0,0,0,0,0,n_bias}};

cv::Mat Aa(9,9,CV_32F,A_mat);
cv::Mat Aq(9,9,CV_32F,Q_mat);
cv::Mat Ab(9,3,CV_32F,B_mat);
cv::Mat Ah = Mat::eye(9,9,CV_32F);
cv::Mat Ar(9,9,CV_32F,R_mat);
cv::Mat Ap = Mat::zeros(9,9,CV_32F);
cv::Mat Au(3,1,CV_32F);
cv::Mat Ax(9,1,CV_32F);
cv::Mat Ak = Mat::zeros(9,9,CV_32F);
cv::Mat Ay(9,1,CV_32F);

///GLOBAL VARIABLES (defined in module2.cpp)
extern int16_t CTRL_pitch;
extern int16_t CTRL_roll;
extern int16_t CTRL_yaw;
extern u_int16_t CTRL_thrust;
extern int32_t angle_pitch;
extern int32_t angle_roll;
extern int32_t angle_yaw;
extern int initialize;
extern int ended;
extern int16_t acc_x;
extern int16_t acc_y;
extern int16_t acc_z;
//extern int height;
extern int landing;
//////////////////////

void
Module0::loadParams() {
    kpx = 300.0;
    kpy = 150.0;
    kpz = 300.0;
    kpyaw = 1500.0;
    kdx = 1000.0;
    kdy = 400.0;
    kdz = 400.0;
    kdyaw=0;
    kix = 0;
    kiy = 0;
    kiz = 0;
    kiyaw = 0;
    kpv = 50.0;
    kiv = 5.0;
    vtarg= 0.2;
    kpz2 = 100.0;
    kdz2 = 300.0;
    kiz2= 7.0;
    //xg=-2.5;
    //yg=0;
    //zg=0;
    //yawg=0;
    cumulx=0,
    cumuly=0,
    cumulz=0,
    cumulyaw=0;
    cumulvz=0;
    thrustOffset = 2475; // final offset = 2475;
    pitchOffset = 124;
    rollOffset = 63;
    yawOffset = 0;

    ifstream fs("../config/params_in.txt");
    if(fs.is_open()){
        //file format:
        //1st line: thrustOffset pitchOffset rollOffset yawOffset
        //2nd line: kpx kpy kpz kpyaw
        //3rd line: kdx kdy kdz kdyaw
        //4th line: kix kiy kiz kiyaw
        //5th line: xg, yg, zg, yawg
        fs >> thrustOffset;
        fs >> pitchOffset;
        fs >> rollOffset;
        fs >> yawOffset;
        fs >> kpx;
        fs >> kpy;
        fs >> kpz;
        fs >> kpyaw;
        fs >> kdx;
        fs >> kdy;
        fs >> kdz;
        fs >> kdyaw;
        fs >> kix;
        fs >> kiy;
        fs >> kiz;
        fs >> kiyaw;
        fs >> kpv;
        fs >> kiv;
        fs >> vtarg;
        fs >> kpz2;
        fs >> kdz2;
        fs >> kiz2;
        //fs >> xg;
        //fs >> yg;
        //fs >> zg;
        //fs >> yawg;
        cout << endl << "params loaded" << endl;
    } else {
        cerr << endl << "File not open!" << endl;
    }
}

void
Module0::printParams() {


    cout << "gains:" << endl;
    cout << "  kpx: " << kpx << endl;
    cout << "  kpy: " << kpy << endl;
    cout << "  kpz: " << kpz << endl;
    cout << "  kpyaw: " << kpyaw << endl;
    cout << "  kdx: " << kdx << endl;
    cout << "  kdy: " << kdy << endl;
    cout << "  kdz: " << kdz << endl;
    cout << "  kdyaw: " << kdyaw << endl;
    cout << "  kix: " << kix << endl;
    cout << "  kiy: " << kiy << endl;
    cout << "  kiz: " << kiz << endl;
    cout << "  kiyaw: " << kiyaw << endl;
    cout << "  kpv: " << kpv << endl;
    cout << "  kiv: " << kiv << endl;
    cout << "  vtarg: " << vtarg << endl;
    cout << "  kpz2: " << kpz2 << endl;
    cout << "  kdz2: " << kdz2 << endl;
    cout << "  kiz2: " << kiz2 << endl;
    //cout << "target:" << endl;
    //cout << "  x: " << xg << endl;
    //cout << "  y: " << yg << endl;
    //cout << "  z: " << zg << endl;
    //cout << "  yaw: " << yawg << endl;
    printf("end\n");


}

void
Module0::initializeDataSaving() {
    ofstream fs2("dataasctec.txt", std::ofstream::out | std::ofstream::trunc);

    if(fs2.is_open()){
        fs2 << "Commands, Positions(kalman), Velocities, Angles, Accelerations" << "\n";
        fs2 << "time \t";
        fs2 << "ctrlpitch\t";
        fs2 << "ctrlroll\t";
        fs2 << "ctrlthrust\t";
        fs2 << "ctrlyaw\t";
        //fs2 << "ctrlpitch2\t";
        //fs2 << "ctrlroll2\t";
        //fs2 << "ctrlthrust2\t";
        //fs2 << "ctrlyaw2\t";
        fs2 << "x\t";
        fs2 << "y\t";
        fs2 << "z\t";
        fs2 << "dx\t";
        fs2 << "dy\t";
        fs2 << "dz\t";
        //fs2 << "x2\t";
        //fs2 << "y2\t";
        //fs2 << "z2\t";
        //fs2 << "dx2\t";
        //fs2 << "dy2\t";
        //fs2 << "dz2\t";
        fs2 << "pitch\t";
        fs2 << "roll\t";
        fs2 << "yaw\t";
        //fs2 << "ax\t";
        //fs2 << "ay\t";
        //fs2 << "az\t";
        //fs2 << "xtarg\t";
        //fs2 << "ytarg\t";
        //fs2 << "ztarg\t";
        fs2 << "\n";


        cout << "File with controls initialized" << endl;
    } else {
        cerr << "File not open!" << endl;
    }

}

Module0::Module0 (ETDispatch* dis)
        :ETExpert(dis, 0, 0)
{
}

Module0::~Module0()
{
}

void
Module0::Init()
{
    message_rec = 0;
    AddRequest(1); // user interface
    AddRequest(2); // IMU
    AddRequest(3); // positions
    AddRequest(4); // waypoints
    SetActivationCondition(GetAllMsgTypeMask());
    loadParams();
    initializeDataSaving();
    thr2=100;
    gettimeofday(&starttime2, NULL);
    mtime=0;
    accum_time=0;
    dax=0, day=0, daz=0;
    printstep = 70;
    printTimeCounter = 0;
    yawg=0;
    cumulvz=0;
    mass = 1.8;
}

void odometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {

    ROS_INFO_ONCE("[pos_ctl_nd] First odometry message received");


    rotors_control::eigenOdometryFromMsg(odometry_msg, &odometry);

    thrust_msg.header.stamp = odometry_msg->header.stamp;

}

void velocityCommandCallback(const geometry_msgs::Pose vel_msg) {
    xg = vel_msg.position.x;
    yg = vel_msg.position.y;
    zg = vel_msg.position.z;
    yawg = mav_msgs::yawFromQuaternion(vel_msg.orientation);

}

int main(int argc, char **argv) {
    double theta;
    double phi;
    double accx, accy, accz;
    double ax, ay, az;
    double x_t_2, x_t_1;
    double y_t_2, y_t_1;
    double z_t_2, z_t_1;
    double yaw_t_2, yaw_t_1;
    const int spinRate = 50;
    double step = 1 / spinRate;

    int16_t CTRL_pitch2;
    int16_t CTRL_roll2;
    int16_t CTRL_thrust2;

    ros::Subscriber odometry_sub = n.subscribe("/firefly/ground_truth/odometry", 1, &odometryCallback);
    ros::Subscriber vel_sp_sub = n.subscribe("/firefly/position_control/vel_sp", 1, &velocityCommandCallback);
    ros::Subscriber pos_sp_sub = n.subscribe("/firefly/position_control/pos_sp", 1, &positionCommandCallback);

    ros::Rate rate(spinRate);

    while (ros::ok()) {

        xr = odometry.position[0];
        yr = odometry.position[1];
        zr = odometry.position[2];
        qx = odometry.orientation[0];
        qy = odometry.orientation[1];
        qz = odometry.orientation[2];
        qw = odometry.orientation[3];

        yawr = std::atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)) * M_PI / 180;

        x_t_2 = x_t_1;
        x_t_1 = xr;

        y_t_2 = y_t_1;
        y_t_1 = yr;

        z_t_2 = z_t_1;
        z_t_1 = zr;

        yaw_t_2 = yaw_t_1;
        yaw_t_1 = yawr;

        dxr = (x_t_2 - 4 * x_t_1 + 3 * x) / 2 * step;
        dyr = (y_t_2 - 4 * y_t_1 + 3 * y) / 2 * step;
        dzr = (z_t_2 - 4 * z_t_1 + 3 * z) / 2 * step;
        dyawr = (yaw_t_2 - 4 * yaw_t_1 + 3 * yaw) / 2 * step;


        printTimeCounter++;

        ////////test
        double xx, yy, zz;
        //double exx, eyy, ezz;
        double dexx, deyy, dezz;
        xx = xr;
        yy = yr;
        zz = zr;
        //exx = xg - xr;
        //eyy = yg - yr;
        //ezz = zg - zr;
        dexx = -dxr;
        deyy = -dyr;
        dezz = -dzr;
        /////////////
        theta = 0.001 * (double) angle_pitch * M_PI / 180.0;
        phi = 0.001 * (double) angle_roll * M_PI / 180.0;

        ax = acc_x * 9.81 / 10000;
        ay = acc_y * 9.81 / 10000;
        az = acc_z * 9.81 / 10000;

        static double *R = new double[9];

        R[0] = cos(theta) * cos(yawr);
        R[1] = -cos(phi) * sin(yawr) + sin(phi) * sin(theta) * cos(yawr);
        R[2] = sin(phi) * sin(yawr) + cos(phi) * sin(theta) * cos(yawr);
        //second row
        R[3] = cos(theta) * sin(yawr);
        R[4] = cos(phi) * cos(yawr) + sin(phi) * sin(theta) * sin(yawr);
        R[5] = -sin(phi) * cos(yawr) + cos(phi) * sin(theta) * sin(yawr);
        //third row
        R[6] = -sin(theta);
        R[7] = sin(phi) * cos(theta);
        R[8] = cos(phi) * cos(theta);

        accx = R[0] * ax + R[1] * ay + R[2] * az;
        accy = R[3] * ax + R[4] * ay + R[5] * az;
        accz = R[6] * ax + R[7] * ay + R[8] * az;

        Au.at<float>(0, 0) = accx;
        Au.at<float>(1, 0) = accy;
        Au.at<float>(2, 0) = accz;

        gettimeofday(&endtime2, NULL);
        secc = endtime2.tv_sec - starttime2.tv_sec;
        msec = endtime2.tv_usec - starttime2.tv_usec;
        mtime = ((secc) + (msec / 1000000.0)); //time in sec
        dt = mtime;
        accum_time = accum_time + mtime;
        gettimeofday(&starttime2, NULL);

        ///Kalman prediction

        Ax = Aa * Ax + Ab * Au;
        Ap = Aa * Ap * (Aa.t()) + Aq;     // prediction ok

        Ay.at<float>(0, 0) = xr;
        Ay.at<float>(1, 0) = dxr;
        Ay.at<float>(2, 0) = 0;
        Ay.at<float>(3, 0) = yr;
        Ay.at<float>(4, 0) = dyr;
        Ay.at<float>(5, 0) = 0;
        Ay.at<float>(6, 0) = zr;
        Ay.at<float>(7, 0) = dzr;
        Ay.at<float>(8, 0) = 0;

        //Kalman Update

        Ak = Ap * (Ah.t()) * ((Ah * Ap * (Ah.t()) + Ar).inv());
        Ax = Ax + Ak * (Ay - Ax);
        Ap = (Ah - Ak * Ah) * Ap;

        ///End Kalman

        double xk, yk, zk;

        xk = Ax.at<float>(0, 0);
        dex = -Ax.at<float>(1, 0);
        yk = Ax.at<float>(3, 0);
        dey = -Ax.at<float>(4, 0);
        zk = Ax.at<float>(6, 0);
        dez = -Ax.at<float>(7, 0);


        ex = xg - xr;
        ey = yg - yr;   /// if Kalman use xk, altrimenti xr
        ez = zg - zr;
        eyaw = yawg - yawr;
        dex = -dxr;
        dey = -dyr;        // if Kalman comment
        dez = -dzr;
        deyaw = -dyawr;

        if ((initialize == 0) | (ended == 1)) {
            cumulx = 0;
            cumuly = 0;
            cumulz = 0;
            cumulyaw = 0;
            dax = 0;
            day = 0;
            daz = 0;
            dback = 0;
            duy = 0;
            cumulvz = 0;

        }

        if ((initialize == 1) & (ended == 0)) {

            cumulx = cumulx + ex * mtime;
            cumuly = cumuly + ey * mtime;
            cumulz = cumulz + ez * mtime;
            cumulyaw = cumulyaw + eyaw * mtime;

            //// for test
            //double daxx=-(kpx*exx+kdx*dexx);
            //double dayy=(kpy*eyy+kdy*deyy);
            //double dazz=-(kpz*ezz+kdz*dezz);
            //////////////
            dback = dax;
            dax = -(kpx * ex + kdx * dex + kix * cumulx);


            if ((dax - dback) > thr2) {
                dax = dback + thr2;
            }
            else if ((dax - dback) < -thr2) {
                dax = dback - thr2;
            }

            dback = day;
            day = -(kpy * ey + kdy * dey + kiy * cumuly);

            if ((day - dback) > thr2) {
                day = dback + thr2;
            }
            else if ((day - dback) < -thr2) {
                day = dback - thr2;
            }

            dback = daz;
            daz = -(kpz * ez + kdz * dez + kiz * cumulz);

            if (landing == 1) {
                daz = -(kpz2 * ez + kdz2 * dez + kiz2 * cumulz);
            }

            if ((daz - dback) > thr2) {
                daz = dback + thr2;
            }
            else if ((daz - dback) < -thr2) {
                daz = dback - thr2;
            }



            //if (landing==1)
            // {
            //     evz=vtarg-dzr;
            //     cumulvz=cumulvz+evz*mtime;
            //     daz=-(kpv*evz+kiv*cumulvz);
            // }


            dback = duy;
            duy = (kpyaw * eyaw + kdyaw * deyaw + kiyaw * cumulyaw);
            if ((duy - dback) > thr2) {
                duy = dback + thr2;
            }
            else if ((duy - dback) < -thr2) {
                duy = dback - thr2;
            }

            double ut2;

            if ((landing == 1)) {
                ut2 = ut2 - 0.5;
                //   if (z>1.20)
                //   {ut2=ut2-0.75;}
            }

            else {
                ut2 = 149.11 * mass *
                      sqrt(dax * dax / (450 * 450) + day * day / (450 * 450) +
                           (daz / (450) + 9.81) * (daz / (450) + 9.81));
                ut2 = ut2 * (1 - ((0.105 / (4 * (1.46 - zr))) * (0.105 / (4 * (1.46 - zr)))));
            }

            if (ut2 < 1700) { ut2 = 1700; }


            double ur2 = rollOffset +
                         4586 *
                         (1.692 * asin(((dax / (450) * sin(yawr)) - (day / (450) * cos(yawr))) / (ut2 / 149.11)));
            double up2 = pitchOffset +
                         4586 *
                         (atan((((dax / (450)) * cos(yawr)) + ((day / (450)) * sin(yawr))) / (daz / (450) + 9.81)));

            CTRL_pitch2 = (int16_t) up2;
            CTRL_roll2 = (int16_t) ur2;
            CTRL_thrust2 = (int16_t) ut2;

            //uy = duy;
            static double *Rt = new double[9];
            //first rowa
            Rt[0] = cos(theta) * cos(yawr);
            Rt[1] = cos(theta) * sin(yawr);
            Rt[2] = -sin(theta);
            //second row
            Rt[3] = -cos(phi) * sin(yawr) + sin(phi) * sin(theta) * cos(yawr);
            Rt[4] = cos(phi) * cos(yawr) + sin(phi) * sin(theta) * sin(yawr);
            Rt[5] = sin(phi) * cos(theta);
            //third row
            Rt[6] = sin(phi) * sin(yawr) + cos(phi) * sin(theta) * cos(yawr);
            Rt[7] = -sin(phi) * cos(yawr) + cos(phi) * sin(theta) * sin(yawr);
            Rt[8] = cos(phi) * cos(theta);

            up = pitchOffset + (Rt[0] * dax + Rt[1] * day + Rt[2] * daz);
            ur = rollOffset - (Rt[3] * dax + Rt[4] * day + Rt[5] * daz);
            ut = thrustOffset + (Rt[6] * dax + Rt[7] * day + Rt[8] * daz);

            //up = pitchOffset + (cos(yawr)*dax+sin(yawr)*day);
            //ur= rollOffset + (-sin(yawr)*dax+cos(yawr)*day);
            //ut = thrustOffset + daz;
            //////test
            //double up2 = pitchOffset + (Rt[0]*daxx+Rt[1]*dayy+Rt[2]*dazz);
            //double ur2= rollOffset - (Rt[3]*daxx+Rt[4]*dayy+Rt[5]*dazz);
            //double ut2= thrustOffset +(Rt[6]*daxx+Rt[7]*dayy+Rt[8]*dazz);
            //////////
            uy = duy;

            int16_t CTRL_back;
            int16_t thr3 = 50;
            CTRL_back = CTRL_pitch;
            CTRL_pitch = (int16_t) up2;

            if ((CTRL_pitch - CTRL_back) > thr3) {
                CTRL_pitch = CTRL_back + thr3;
            }
            else if ((CTRL_pitch - CTRL_back) < -thr3) {
                CTRL_pitch = CTRL_back - thr3;
            }

            CTRL_back = CTRL_roll;
            CTRL_roll = (int16_t) ur2;

            if ((CTRL_roll - CTRL_back) > thr3) {
                CTRL_roll = CTRL_back + thr3;
            }
            else if ((CTRL_roll - CTRL_back) < -thr3) {
                CTRL_roll = CTRL_back - thr3;
            }


            CTRL_back = CTRL_thrust;
            CTRL_thrust = (int16_t) ut2;

            if ((CTRL_thrust - CTRL_back) > thr3) {
                CTRL_thrust = CTRL_back + thr3;
            }
            else if ((CTRL_thrust - CTRL_back) < -thr3) {
                CTRL_thrust = CTRL_back - thr3;
            }

            CTRL_back = CTRL_yaw;
            CTRL_yaw = (int16_t) uy;

            if ((CTRL_yaw - CTRL_back) > thr3) {
                CTRL_yaw = CTRL_back + thr3;
            }
            else if ((CTRL_yaw - CTRL_back) < -thr3) {
                CTRL_yaw = CTRL_back - thr3;
            }


            if (printTimeCounter >= printstep) {



                //printf ("gps: long %d lat %d height %d \n", longitude, latitude, height);
                //printf ("gps_cartesian: long %f lat %f height %f \n", long_cart, lat_cart, height_cart);
                //printf ("gps: fus long %d lat %d height %d \n", fus_longitude, fus_latitude, fus_height);
                //printf ("gps: heading: %d, yaw: %f, accuracy: %d, height accuracy: %d, sat: %d, status: %d  \n", GPS_heading, psi, position_accuracy, height_accuracy, GPS_num, GPS_status);
                //printf ("test: %f \n", yawr);

                printf("robot position x, y, z, yaw: %f %f %f %f \n", xr, yr, zr, yawr * 180 / M_PI);
                //printf("robot acceleration: accx %f accy %f accz %f \n", ax, ay, az);
                //printf("robot position vs. target x, y, z: %f %f %f \n", xr-xg, yr-yg, zr-zg);
                //printf("robot position vs. target in robot system, y, z: %f %f %f \n", (Rt[0]*(xr-xg)+Rt[1]*(yr-yg)+Rt[2]*(zr-zg)), -(Rt[3]*(xr-xg)+Rt[4]*(yr-yg)+Rt[5]*(zr-zg)), (Rt[6]*(xr-xg)+Rt[7]*(yr-yg)+Rt[8]*(zr-zg)));
                printf("command pitch, roll, thrust, yaw: %d %d %d %d \n", CTRL_pitch, CTRL_roll, CTRL_thrust,
                       CTRL_yaw);
                //printf("test: %f, %f, %f \n", kpz*ez, kdz*dez, kiz*cumulz);
                //printf("command pitch, roll, thrust, yaw: %d %d %d %d \n", CTRL_pitch2, CTRL_roll2, CTRL_thrust2, CTRL_yaw);

                //printf("robot velocity dx, dy, dz, dpsi: %f %f %f %f \n", dxr, dyr, dzr, dyawr*180/M_PI);
                //printf("extimated height : %d \n", height);
                //printf("command pitch, roll, thrust, yaw: %d %d %d %d \n", CTRL_pitch, CTRL_roll, CTRL_thrust, CTRL_yaw);
                //printf("kx, ky, kz, kyaw: %f %f %f %f \n", -kpx*ex, kpy*ey, -kpz*ez, kpyaw*eyaw);
                //printf("kdx, kdy, kdz, kdyaw: %f, %f %f %f \n", -kdx*dex, kdy*dey, -kdz*dez, kdyaw*deyaw);
                //printf("robot errors x, y, z, yaw: %f %f %f %f \n", ex, ey, ez, eyaw*180/M_PI);
                printTimeCounter = 0;
            }

            //CTRL_pitch=0;
            //CTRL_roll=0;
            //CTRL_thrust=100;
            //CTRL_yaw=0;
            //aciUpdateCmdPacket(cmdCTRL);
        }

        ofstream fs2("dataasctec.txt", std::ofstream::out | std::ofstream::app);


        if (fs2.is_open()) {
            fs2 << accum_time << "\t";
            fs2 << CTRL_pitch << "\t" << CTRL_roll << "\t" << CTRL_thrust << "\t" << CTRL_yaw << "\t";
            //fs2 << CTRL_pitch2 << "\t" << CTRL_roll2 << "\t" << CTRL_thrust2 << "\t" << CTRL_yaw << "\t";
            //fs2 << xk << "\t" << yk << "\t" << zk << "\t" << Ax.at<float> (1,0) << "\t" << Ax.at<float>(4,0) << "\t" << Ax.at<float>(7,0) << "\t";
            fs2 << xx << "\t" << yy << "\t" << zz << "\t" << -dexx << "\t" << -deyy << "\t" << -dezz << "\t";
            //fs2 << xr-xg << "\t" << yr-yg << "\t" << zr-zg << "\t" << (Rt[0]*(xr-xg)+Rt[1]*(yr-yg)+Rt[2]*(zr-zg)) << "\t" << -(Rt[3]*(xr-xg)+Rt[4]*(yr-yg)+Rt[5]*(zr-zg)) << "\t" << (Rt[6]*(xr-xg)+Rt[7]*(yr-yg)+Rt[8]*(zr-zg)) << "\t";
            fs2 << theta << "\t" << phi << "\t" << psi << "\t";   ///// NB qui poi rimettici yawr per i marker
            //fs2 << ax << "\t" << ay << "\t" << az << "\t";
            //fs2 << xg << "\t" << yg << "\t" << zg << "\t";
            fs2 << "\n";
        }
        else {
            cerr << "File not open!" << endl;
        }
    }

    thrust_msg.roll = CTRL_roll;
    thrust_msg.pitch = CTRL_pitch;
    thrust_msg.yaw_rate = CTRL_yaw;
    thrust_msg.thrust = CTRL_thrust;
    chatter_pub.publish(thrust_msg);
}



void
Module0::Close()
{
    printParams();
    printf("\nSporadic N %d  ",ExpertId);
    printf(" executed %d times ",message_rec);
    fflush(stdout);
}


