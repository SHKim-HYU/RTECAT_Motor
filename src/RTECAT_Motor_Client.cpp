#include "RTECAT_Motor_Client.h"

ROBOT_INFO info;

RT_TASK motor_task;
RT_TASK print_task;
RT_TASK xddp_writer;

using namespace std;

int initAxes()
{
	for (int i = 0; i < JOINTNUM; i++)
	{	
		Axis[i].setGearRatio(gearRatio[i]);
		Axis[i].setGearEfficiency(EFFICIENCY);
		Axis[i].setPulsePerRevolution(ecat_master.SDO_ENCODER_RESOLUTION(i));
		Axis[i].setTauRateCur(((double)ecat_master.SDO_RATE_CURRENT(i))/1000.0);
		Axis[i].setTauK(((double)ecat_master.SDO_TORQUE_CONSTANT(i))/1000000.0);
		Axis[i].setZeroPos(zeroPos[i]);

		Axis[i].setDirQ(ecat_master.SDO_MOTOR_DIRECTION(i));
		Axis[i].setDirTau(ecat_master.SDO_MOTOR_DIRECTION(i));

		Axis[i].setConversionConstants();

		Axis[i].setTrajPeriod(period);
		
		Axis[i].setTarVelInRPM(0);
		Axis[i].setTarTorInCnt(0);

        info.des.e = JVec::Zero();
        info.des.eint = JVec::Zero();
	}
	
	return 1;
}

void readData()
{
    // ecat_master.Motor_STATE(info.q_inc, info.dq_inc, info.tau_per, info.statusword, info.modeofop);
    ecat_master.RxUpdate();
    for(int i=0; i<JOINTNUM;i++)
    {

        Axis[i].setCurrentPosInCnt(ecat_iservo[i].position_);
        Axis[i].setCurrentVelInRPM(ecat_iservo[i].velocity_);
        Axis[i].setCurrentTorInCnt(ecat_iservo[i].torque_);
        
        Axis[i].setCurrentTime(gt);

        info.act.q(i) = Axis[i].getCurrPosInRad();
        info.act.q_dot(i) = Axis[i].getCurrVelInRad();
        info.act.tau(i) = Axis[i].getCurrTorInNm();

        // For Inital target
        if(!system_ready)
        {
            Axis[i].setTarPosInRad(info.act.q(i));
            Axis[i].setDesPosInRad(info.act.q(i));
            if (i == JOINTNUM-1) system_ready = 1;
        }

    }
}

/****************************************************************************/
void trajectory_generation(){
	/////////////Trajectory for Joint Space//////////////
    if(!Axis[0].trajInitialized())
    {
	    switch(motion)
	    {
	    case 1:
	    	info.q_target(0)=1.5709; //info.q_target(1)=-1.5709;
	    	traj_time = 0.5;
	    	motion++;
	        break;
	    case 2:
	    	info.q_target(0)=0.0; //info.q_target(1)=0.0;
	    	traj_time = 0.5;
	    	motion++;
	    	// motion=1;
	        break;
	    case 3:
	    	info.q_target(0)=-1.5709; //info.q_target(1)=1.5709;
	    	traj_time = 0.5;
	    	motion++;
	        break;
	    case 4:
	    	info.q_target(0)=0.0; //info.q_target(1)=0.0;
	    	traj_time = 0.5;
	    	motion=1;
	    	break;
	    default:
	    	info.q_target(0)=info.act.q(0);
            // info.q_target(1)=info.act.q(1);

	    	motion=1;
	    	break;
	    }
	}

	for(int i=0;i<JOINTNUM;i++)
	{
		if(!Axis[i].trajInitialized())
		{
			Axis[i].setTrajInitialQuintic();
			Axis[i].setTarPosInRad(info.q_target(i));
			Axis[i].setTarVelInRad(0);
			Axis[i].setTrajTargetQuintic(traj_time);
		}

		Axis[i].TrajQuintic();

		info.des.q(i)=Axis[i].getDesPosInRad();
		info.des.q_dot(i)=Axis[i].getDesVelInRad();
		info.des.q_ddot(i)=Axis[i].getDesAccInRad();
	}
}

void compute()
{

}

void control()
{

    double Kp = 15;
    double Kd = 5;


    for (int i = 0; i<JOINTNUM; i++)
    {
        if (ecat_iservo[i].mode_of_operation_ == ecat_iservo[i].MODE_CYCLIC_SYNC_TORQUE)
        {
            info.des.e(i) = info.des.q(i)-info.act.q(i);
            info.des.eint(i) = info.des.eint(i) + info.des.e(i)*period;
            info.des.tau(i) = Kp*info.des.e(i)+Kd*(info.des.q_dot(i)-info.act.q_dot(i));
        }
        else if (ecat_iservo[i].mode_of_operation_ == ecat_iservo[i].MODE_CYCLIC_SYNC_VELOCITY)
        {
            info.des.e(i) = info.des.q(i)-info.act.q(i);
            info.des.tau(i) = info.des.q_dot(i) + 0.1*(info.des.q_dot(i)-info.act.q_dot(i)) + 1*info.des.e(i);
        }
    }
}

void writeData()
{
    for(int i=1;i<=JOINTNUM;i++){
        if (ecat_iservo[i-1].mode_of_operation_ == ecat_iservo[i-1].MODE_CYCLIC_SYNC_TORQUE)
        {
            Axis[i-1].setDesTorInNm(info.des.tau(i-1));
                
            INT16 temp = Axis[i-1].getDesTorInPer();
            // rt_printf("temp: %d\n", temp);
            if (temp > 1000)
            {
                temp = 1000;            
            }
            else if (temp<-1000)
            {
                temp = -1000;
            }
            // rt_printf("temp: %d\n\n", temp);
            // ecat_master.RxPDO1_SEND(i, (short)temp);
            ecat_iservo[i-1].writeTorque(temp);
        }
        else if (ecat_iservo[i-1].mode_of_operation_ == ecat_iservo[i-1].MODE_CYCLIC_SYNC_VELOCITY)
        {
            // rt_printf("velocity: %d\n",Axis[i-1].getDesVelInRPM());
            ecat_iservo[i-1].writeVelocity(Axis[i-1].getDesVelInRPM(info.des.tau(i-1)));
        }
        // ecat_master.TxUpdate();
	}
    ecat_master.SyncEcatMaster(rt_timer_read());
}

void motor_run(void *arg)
{
    RTIME beginCycle, endCycle;
   
    memset(&info, 0, sizeof(ROBOT_INFO));

    // ecat_master.activate_all(DEVICE2, config, JOINTNUM);

    for(int j=0; j<JOINTNUM; ++j)
	{
		ecat_master.addSlaveiServo(0, j, &ecat_iservo[j]);
		ecat_iservo[j].mode_of_operation_ = ecat_iservo[j].MODE_CYCLIC_SYNC_VELOCITY;
	}

    initAxes();

    // Print Motor parameters
    for(int i=0; i<JOINTNUM; i++)
    {
        rt_printf("rate current: %lf\n", Axis[i].getTauRateCur());
        rt_printf("torque constant: %lf\n", Axis[i].getTauK());
        rt_printf("encoder resol: %d\n", Axis[i].getPulsePerRevolution());
        rt_printf("motor direction: %d\n", Axis[i].getDirQ());

    }

    ecat_master.activateWithDC(0, cycle_ns);

    rt_task_set_periodic(NULL, TM_NOW, cycle_ns);
    while (1) {
        beginCycle = rt_timer_read();
        // Read Joints Data
        readData();
       
        // Trajectory Generation
        trajectory_generation();
        
        // Compute KDL
        // compute();	

        
        // Controller
        control();
                
        // Write Joint Data
        writeData();

        endCycle = rt_timer_read();
		periodCycle = (unsigned long) endCycle - beginCycle;

        gt+= period;
        if (periodCycle > cycle_ns) overruns++;
        rt_task_wait_period(NULL); //wait for next cycle
    }
}

void runQtApplication(int argc, char* argv[]) {
  QApplication a(argc, argv);
  // style our application with custom dark style
  QApplication::setStyle(new DarkStyle);

  // create frameless window (and set windowState or title)
  FramelessWindow framelessWindow;

  // create our mainwindow instance
  MainWindow *mainWindow = new MainWindow;
  // add the mainwindow to our custom frameless window
  framelessWindow.resize(1600,600);
  framelessWindow.setContent(mainWindow);
  framelessWindow.show();
  a.exec();
}

void print_run(void *arg)
{
	RTIME now, previous=0;
	int i;
	unsigned long itime=0, step;
	long stick=0;
	int count=0;
		
	/* Arguments: &task (NULL=self),
	 *            start time,
	 *            period (here: 100ms = 0.1s)
	 */
	rt_task_set_periodic(NULL, TM_NOW, cycle_ns*100);
	
	while (1)
	{
		rt_task_wait_period(NULL); //wait for next cycle
		if (++count==10)
		{
			++stick;
			count=0;
		}
		
		if (system_ready)
		{
			now = rt_timer_read();
			step=(unsigned long)(now - previous) / 1000000;
			itime+=step;
			previous=now;

			rt_printf("Time=%0.3lfs, cycle_dt=%lius,  overrun=%d\n", gt, periodCycle/1000, overruns);
			
			for(int j=0; j<JOINTNUM; ++j){
				rt_printf("ID: %d", j);
			// 	//rt_printf("\t CtrlWord: 0x%04X, ",		ControlWord[j]);
			// 	//rt_printf("\t StatWord: 0x%04X, \n",	StatusWord[j]);
			//     //rt_printf("\t DeviceState: %d, ",		DeviceState[j]);
			// 	//rt_printf("\t ModeOfOp: %d,	\n",		ModeOfOperationDisplay[j]);
                // rt_printf("\t[cnt] pos: %d, vel: %d, [per] tor: %d\n",info.q_inc[j], info.dq_inc[j], info.tau_per[j]);
                // rt_printf("\t[rad] pos: %lf, vel: %lf, [Nm] tor: %lf\n",info.act.q[j], info.act.q_dot[j], info.act.tau[j]);
				rt_printf("\t ActPos: %lf, ActVel: %lf \n",info.act.q(j), info.act.q_dot(j));
				rt_printf("\t DesPos: %lf, DesVel :%lf, DesAcc :%lf\n",info.des.q[j],info.des.q_dot[j],info.des.q_ddot[j]);
				rt_printf("\t TarTor: %lf, ActTor: %lf, ExtTor: %lf \n", info.des.tau(j), info.act.tau(j), info.act.tau_ext(j));
			}

			rt_printf("\n");

		}
		else
		{
			if (count==0){
				rt_printf("%i", stick);
				for(i=0; i<stick; ++i)
					rt_printf(".");
				rt_printf("\n");
			}
		}
	}
}


void signal_handler(int signum)
{
    rt_task_delete(&motor_task);
    rt_task_delete(&print_task);
    rt_task_delete(&xddp_writer);

    

    printf("\n\n");
	if(signum==SIGINT)
		printf("╔════════════════[SIGNAL INPUT SIGINT]═══════════════╗\n");
	else if(signum==SIGTERM)
		printf("╔═══════════════[SIGNAL INPUT SIGTERM]═══════════════╗\n");	
	else if(signum==SIGTSTP)
		printf("╔═══════════════[SIGNAL INPUT SIGTSTP]══════════════╗\n");
    printf("║                Servo drives Stopped!               ║\n");
	printf("╚════════════════════════════════════════════════════╝\n");	

    ecat_master.deactivate();

    exit(1);
}

int main(int argc, char *argv[])
{
    // Perform auto-init of rt_print buffers if the task doesn't do so
    rt_print_init(0, NULL);

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
	signal(SIGTSTP, signal_handler);

    /* Avoids memory swapping for this program */
    mlockall(MCL_CURRENT|MCL_FUTURE);

    cpu_set_t cpuset_qt, cpuset_rt1, cpuset_rt2;
    CPU_ZERO(&cpuset_qt);
    CPU_ZERO(&cpuset_rt1);  
    CPU_ZERO(&cpuset_rt2);  

    CPU_SET(6, &cpuset_qt);  
    CPU_SET(7, &cpuset_rt1);  
    CPU_SET(5, &cpuset_rt2);  

    // // For CST (cyclic synchronous torque) control
	// if (ecat_master.init(OP_MODE_CYCLIC_SYNC_TORQUE, cycle_ns) == -1)
	// {
	// 	printf("System Initialization Failed\n");
	//     return 0;
	// }
	// for (int i = 0; i < JOINTNUM; ++i)
	// 	ModeOfOperation[i] = OP_MODE_CYCLIC_SYNC_TORQUE;
    
    // std::thread qtThread(runQtApplication, argc, argv);
    // pthread_t pthread = qtThread.native_handle();
    // int rc = pthread_setaffinity_np(pthread, sizeof(cpu_set_t), &cpuset_qt);

    rt_task_create(&motor_task, "motor_task", 0, 99, 0);
    rt_task_set_affinity(&motor_task, &cpuset_rt2);
    rt_task_start(&motor_task, &motor_run, NULL);

    rt_task_create(&print_task, "print_task", 0, 70, 0);
    rt_task_set_affinity(&print_task, &cpuset_rt2);
    rt_task_start(&print_task, &print_run, NULL);

    // Must pause here
    pause();
    // qtThread.join();

    // Finalize
    signal_handler(0);

    return 0;
}

