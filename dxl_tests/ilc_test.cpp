#include <iostream>
#include <dynamixel_sdk.h>
#include <chrono>
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <cmath>
#include <thread>
#include <fstream>

// Default setting
#define DXL_ID              5                   // Dynamixel ID: 1
#define BAUDRATE            1000000
#define DEVICENAME          "/dev/ttyUSB0"      // Check which port is being used on your controller

// Protocol version
#define PROTOCOL_VERSION    1.0                 // See which protocol version is used in the Dynamixel

#define GOAL_POS_ADDR       30
#define PRESENT_POS_ADDR    36
#define KP_ADDR				28

#define FREQUENCY           30.0
#define DURATION            10.
#define N_POINTS            long(FREQUENCY*DURATION)

#define CURVE_FREQUENCY     .8
#define CURVE_RANGE         (30*M_PI/180)
#define CURVE_AVG           (180*M_PI/180)//4.1612
#define FINAL_DEVIATION     (-15.*M_PI/180)

#define VEL_MAX             (115*2*M_PI/60)
#define DEFAULT_VEL         .5

#define KP                  12
#define LAMBDA              2
#define UBAT                12.

//#define RAD_PER_MOTOR       (.088*M_PI/180.)
//#define RADS_PER_MOTOR      (0.114*2*M_PI/60.)

//AX-12
#define RAD_PER_MOTOR       (.29*M_PI/180.)
#define RADS_PER_MOTOR       (.111*2*M_PI/60.)



#define MY_ABS(x) ((x>=0)?(x):(-x))

inline uint16_t rad2motor(double rad){
    return uint16_t(rad/RAD_PER_MOTOR);
}

inline double motor2rad(uint16_t motor){
    return motor*RAD_PER_MOTOR;
}

inline uint16_t rads2motor(double rads){
    return uint16_t(MY_ABS(rads)/RADS_PER_MOTOR);
}

inline double motor2rads(uint16_t motor){
    return (motor-1024)*RADS_PER_MOTOR;
}


//Trajetoria especificada pelo usuario para um motor: q
double  qV(double t)
{
    double F = 1.5,q = 0;//s
    //q   =  sin(F*pow(t,(6./5.))) + pow(cos((9.*F*pow(t,(13./10.)))/10.),3.) - pow((t/20. - 1./10.),0.1) + 1.;
    q   =  sin(F*pow(t,(6./5.))) + pow(cos((9.*F*pow(t,(13./10.)))/10.),3.) + M_PI;
    return q;
}

//Derivada da Trajetoria especificada pelo usuario para um motor: qDot
double qDotV(double t)
{
    double F = 0.8,qDot = 0;//s
    qDot = (6.*F*pow(t,(1./5.))*cos(F*pow(t,(6./5.))))/5.  - (351.*F*pow(t,(3./10.))*pow(cos((9.*F*pow(t,(13./10.)))/10.),2.)*sin((9.*F*pow(t,(13./10.)))/10.))/100.;
    //qDot = (6.*F*pow(t,(1./5.))*cos(F*pow(t,(6./5.))))/5. - (3*pow((t/20. - 1./10.),2))/20. - (351.*F*pow(t,(3./10.))*pow(cos((9.*F*pow(t,(13./10.)))/10.),2.)*sin((9.*F*pow(t,(13./10.)))/10.))/100.;
    return qDot;
}

//Derivada segunda da Trajetoria especificada pelo usuario para um motor: qDot2
double qDot2V(double t)
{
    double F = 1.5,qDot2 = 0;//s
    qDot2 = (6.*F*cos(F*pow(t,(6./5.))))/(25.*pow(t,(4./5.))) - (41067.*F*F*pow(t,(3./5.))*pow(cos((9.*F*pow(t,(13./10.)))/10.),3.))/10000. - (3.*t)/4000. - (36.*F*F*pow(t,(2./5.))*sin(F*pow(t,(6./5.))))/25. + (41067.*F*F*pow(t,(3./5.))*cos((9.*F*pow(t,(13./10.)))/10.)*pow(sin((9.*F*pow(t,(13./10.)))/10.),2.))/5000. - (1053.*F*pow(cos((9.*F*pow(t,(13./10.)))/10.),2.)*sin((9.*F*pow(t,(13./10.)))/10.))/(1000.*pow(t,(7./10.))) + 3./2000.;
    return qDot2;
}

int main(){

    //    mlockall(MCL_FUTURE);

    struct sched_param param;
    param.__sched_priority = 51;

    std::cout << pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) << std::endl;

    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

    // Initialize PacketHandler instance
    // Set the protocol version
    // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    // Open port
    if (portHandler->openPort())
    {
        printf("Succeeded to open the port!\n");
    }
    else
    {
        printf("Failed to open the port!\n");
//        printf("Press ENTER to terminate...\n");
//        getchar();
        //        return 0;
    }

    // Set port baudrate
    if (portHandler->setBaudRate(BAUDRATE))
    {
        printf("Succeeded to change the baudrate!\n");
    }
    else
    {
        printf("Failed to change the baudrate!\n");
//        printf("Press ENTER to terminate...\n");
//        getchar();
        //        return 0;
    }

    auto period = std::chrono::microseconds(long((1/FREQUENCY)*1e6));

    std::cout << period.count() << std::endl;

    double qref[N_POINTS];
    double q[N_POINTS];
    double qdot[N_POINTS];
    double qd[N_POINTS];
    double qddot[N_POINTS];
    double U[N_POINTS];
    double error[N_POINTS];

    double time[N_POINTS];

    uint16_t qd_dxl[N_POINTS][2];
    uint16_t q_dxl[N_POINTS][2];

    for(unsigned int i = 0; i < N_POINTS; i++){
        time[i] = (i/FREQUENCY);
        qref[i] = sin(CURVE_FREQUENCY*2*M_PI*time[i])*CURVE_RANGE+ CURVE_AVG + (FINAL_DEVIATION/DURATION)*time[i];
//        qref[i] = qV(time[i]);
        U[i] = 0;
        error[i] = 0;
    }


    std::chrono::steady_clock::time_point now;

    std::ofstream file;

    int it = 0;

    packetHandler->write1ByteTxOnly(portHandler,DXL_ID,KP_ADDR,KP);

    uint16_t defaultGoalPos[] = {rad2motor(CURVE_AVG),rads2motor(DEFAULT_VEL)};

    while(true){

        packetHandler->writeTxOnly(portHandler,DXL_ID,GOAL_POS_ADDR,4,reinterpret_cast<uint8_t*>(defaultGoalPos));

        for(unsigned int i = 0; i < N_POINTS; i++){
            qd[i] = 8/(KP*UBAT)*U[i]+qref[i];
            U[i] = U[i]+LAMBDA*error[i];
            if(i>0){
                qddot[i] = (qd[i]-qd[i-1])/(1./FREQUENCY)+1;
                qddot[i] = (qddot[i]>VEL_MAX)?VEL_MAX:qddot[i];
            }
            qd_dxl[i][0] = rad2motor(qd[i]);
            qd_dxl[i][1] = rads2motor(qddot[i]);
        }

        qddot[0] = qddot[1];
        qd_dxl[0][1] = qd_dxl[1][1];

        std::cout << "Pressione ENTER para comecar a iteracao " << it << " (ou Ctrl+C para encerrar)";
        getchar();

        auto nextTime = std::chrono::steady_clock::now();

        for(unsigned int i = 0; i < N_POINTS; i++){

            packetHandler->readTxRx(portHandler,DXL_ID,PRESENT_POS_ADDR,4,reinterpret_cast<uint8_t*>(q_dxl[i]));
            q[i] = motor2rad(q_dxl[i][0]);
            qdot[i] = motor2rads(q_dxl[i][1]);

            packetHandler->writeTxOnly(portHandler,DXL_ID,GOAL_POS_ADDR,4,reinterpret_cast<uint8_t*>(qd_dxl[i]));

//            std::cout << time[i] << " " << qref[i] << " " << qd[i] << " " << qddot[i] << " " << q[i] << " " << U[i] << " " << error[i] << std::endl;

            nextTime += period;
            std::this_thread::sleep_until(nextTime);
        }

        double sumError = 0;

        for(unsigned int i = 0; i < N_POINTS; i++){
            error[i] = qref[i]-q[(i+1<N_POINTS)?i+1:i];
            sumError += error[i]*error[i];
        }


        sumError = sqrt(sumError/N_POINTS);

        std::string fileName = "K"+std::to_string(it++)+".txt";

        std::cout << "error = " << sumError << std::endl;
        std::cout << "Salvo em " << fileName << std::endl;

        std::ofstream outputFile (fileName);
        if (outputFile.is_open())
        {
            for(unsigned int i = 0; i < N_POINTS-1; i++)
                outputFile << time[i] << " " << qref[i] << " " << qd[i] << " " << qddot[i] << " " << q[i] << " " << qdot[i] << " " << U[i] << " " << error[i] << std::endl;


            outputFile.close();
        }
    }

    //        munlockall();

//    return 0;
}
