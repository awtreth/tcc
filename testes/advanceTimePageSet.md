*DESCRIÇÃO

O presente teste monta um PageSet com 2 Pages com poses e timestamps diferentes. Avança-se o tempo do PageSet e avalia
o progresso do tempo, avaliando as poses correntes.


*OUTPUT

Model(Model1) Motion(Motion1)
ts(0) tn(2) motor0(110.000000,111.000000) motor1(112.000000,113.000000) 
ts(2) tn(2) motor0(120.000000,121.000000) motor1(122.000000,123.000000) 
ts(4) tn(2) motor0(132.000000,133.000000) motor1(132.000000,133.000000) 
Model(Model2) Motion(Motion2)
ts(0) tn(3) motor2(210.000000,211.000000) motor3(212.000000,213.000000) 
ts(3) tn(5) motor2(222.000000,223.000000) motor3(222.000000,223.000000) 

currentTime = 0 hasPose 1
motor0(110.000000,111.000000) motor1(112.000000,113.000000) motor2(210.000000,211.000000) motor3(212.000000,213.000000) 
currentTime = 1 hasPose 0
currentTime = 2 hasPose 1
motor0(120.000000,121.000000) motor1(122.000000,123.000000) 
currentTime = 3 hasPose 1
motor2(222.000000,223.000000) motor3(222.000000,223.000000) 
currentTime = 4 hasPose 1
motor0(132.000000,133.000000) motor1(132.000000,133.000000) 
currentTime = 5 hasPose 0
currentTime = 6 hasPose 1
motor0(110.000000,111.000000) motor1(112.000000,113.000000) 
currentTime = 7 hasPose 0
currentTime = 8 hasPose 1
motor0(120.000000,121.000000) motor1(122.000000,123.000000) motor2(210.000000,211.000000) motor3(212.000000,213.000000) 
currentTime = 9 hasPose 0
currentTime = 10 hasPose 1
motor0(132.000000,133.000000) motor1(132.000000,133.000000) 
currentTime = 11 hasPose 1
motor2(222.000000,223.000000) motor3(222.000000,223.000000) 
currentTime = 12 hasPose 1
motor0(110.000000,111.000000) motor1(112.000000,113.000000) 
currentTime = 13 hasPose 0
currentTime = 14 hasPose 1
motor0(120.000000,121.000000) motor1(122.000000,123.000000) 
currentTime = 15 hasPose 0
currentTime = 16 hasPose 1
motor0(132.000000,133.000000) motor1(132.000000,133.000000) motor2(210.000000,211.000000) motor3(212.000000,213.000000) 
currentTime = 17 hasPose 0


*CODE

    Pose p11, p12, p13, p21, p22;
    Page page1, page2;

    p11.addPosVel(PosVel(110,111,"motor0"));
    p11.addPosVel(PosVel(112,113,"motor1"));
    p11.setTimeToNext(2);

    p12.addPosVel(PosVel(120,121,"motor0"));
    p12.addPosVel(PosVel(122,123,"motor1"));
    p12.setTimeToNext(2);


    p13.addPosVel(PosVel(132,133,"motor0"));
    p13.addPosVel(PosVel(132,133,"motor1"));
    p13.setTimeToNext(2);


    p21.addPosVel(PosVel(210,211,"motor2"));
    p21.addPosVel(PosVel(212,213,"motor3"));
    p21.setTimeToNext(3);

    p22.addPosVel(PosVel(222,223,"motor2"));
    p22.addPosVel(PosVel(222,223,"motor3"));
    p22.setTimeToNext(5);

    page1.addPose(p11);
    page1.addPose(p12);
    page1.addPose(p13);

    page1.setModelName("Model1");
    page1.setMotionName("Motion1");
    page1.setTimesByTimeToNext();

    page2.addPose(p21);
    page2.addPose(p22);

    page2.setModelName("Model2");
    page2.setMotionName("Motion2");
    page2.setTimesByTimeToNext();

    PageSet pset;

    pset.setPage(page1);
    pset.setPage(page2);

    std::cout << pset.toString() << std::endl;


    int i = 0;

    std::cout << "currentTime = " << i << " hasPose ";

    if(pset.resetTime()){
        std::cout << true << std::endl;
        std::cout << pset.getCurrentPose().toString() << std::endl;
    }else
        std::cout << false << std::endl;

    for(i = 1; i < 18; i++){
        std::cout << "currentTime = " << i << " hasPose ";

        if(pset.advanceTime(1)){
            std::cout << true << std::endl;
            std::cout << pset.getCurrentPose().toString() << std::endl;
        }else
            std::cout << false << std::endl;

    }

