#ifndef __USER_H
#define __USER_H

// ���Ӷ���USE_����_BOARD
#define USE_LEFT_BACK_BOARD
//RF ����
// ����ת������
#define yawBaseOff_RF 18146  //18126  //17540
#define hipBaseOff_RF 2346  //3706
#define kneeBaseOff_RF 15590  //900
// ����ת�����Ŷ��壬����
#define yawBaseSym_RF 1  //-1
#define hipBaseSym_RF 1
#define kneeBaseSym_RF -1
// ���ת����Ŷ��壬����
#define yawSpeedSym_RF 1  //-1
#define hipSpeedSym_RF -1
#define kneeSpeedSym_RF -1

//LF ����
#define yawBaseOff_LF 16200  //15724  //4398
#define hipBaseOff_LF 23034   //10994
#define kneeBaseOff_LF 24577 //25180  //33560  //23645 

#define yawBaseSym_LF 1  //-1
#define hipBaseSym_LF -1
#define kneeBaseSym_LF 1 

#define yawSpeedSym_LF -1
#define hipSpeedSym_LF 1
#define kneeSpeedSym_LF 1 
//RB ����
#define yawBaseOff_RB 15824  //17055  //17483   //17055
#define hipBaseOff_RB 19744  //30364  //30894  //8069
#define kneeBaseOff_RB 28050   //27860  //12335 

#define yawBaseSym_RB -1
#define hipBaseSym_RB 1
#define kneeBaseSym_RB -1

#define yawSpeedSym_RB -1
#define hipSpeedSym_RB 1
#define kneeSpeedSym_RB 1 
//LB ����
#define yawBaseOff_LB 19176  //19203  //18620 //+800
#define hipBaseOff_LB 5800 //5316  //5996  //5691	//+100
#define kneeBaseOff_LB 3916  //3740  //9030 //8770  //+465

#define yawBaseSym_LB -1
#define hipBaseSym_LB -1
#define kneeBaseSym_LB 1 

#define yawSpeedSym_LB 1
#define hipSpeedSym_LB -1
#define kneeSpeedSym_LB -1 

#define Joint_Num 3
	
#define Knee_Minangle_RF 2206+80 //-17278+500   
#define Knee_Maxangle_RF 2917-80 //-6457-500  
#define Hip_Maxangle_RF 885-80 //9396-500
#define Hip_Minangle_RF 95+80 //-2628+500           
#define Yaw_Maxangle_RF 2256-80 //3000-500
#define Yaw_Minangle_RF 1813+80 //-4164+500    

#define Knee_Minangle_LF 1706+80 //-17801+500   
#define Knee_Maxangle_LF 2441-80  //-6736-500   
#define Hip_Maxangle_LF 2787-80 //7982-500
#define Hip_Minangle_LF 2041+80 //-2628+500    
#define Yaw_Maxangle_LF 2072-80 //4164-500
#define Yaw_Minangle_LF 1592+80 //-2731+500  

#define Knee_Minangle_RB 2036+80 //5933+500   
#define Knee_Maxangle_RB 2808-80 //17819-500 
#define Hip_Maxangle_RB 2424-80 //2750-500   //2576-500
#define Hip_Minangle_RB 1658+80 //-9047+500   //-8768+500          
#define Yaw_Maxangle_RB 2077-80 //3183-500   //4912-500	//3534
#define Yaw_Minangle_RB 1600+80 //-4164+500  // -1980+500   //-3436

#define Knee_Minangle_LB 793+80 //5400+500  //5916+500   
#define Knee_Maxangle_LB 1556-80 //17278-500  //16859-500  
#define Hip_Maxangle_LB 1187-80 //2715-500  //2567-500
#define Hip_Minangle_LB 455+80 //-8139+500  //-8602+500     
#define Yaw_Maxangle_LB 2405-80 //4146-500  //3148-500
#define Yaw_Minangle_LB 1916+80 //-3532+500  //-4042+500    


#define speedMax 5000
#define speedMin -5000

#define PC_ID					0X00
#define POWER_BOARD_ID			0X01
#define LEFT_FRONT_BOARD_ID     0X02
#define LEFT_BACK_BOARD_ID      0X03
#define RIGHT_FRONT_BOARD_ID    0X04
#define RIGHT_BACK_BOARD_ID		0X05
#define HEADER_BOARD_ID      	0X06
#define TAIL_BOARD_ID			0X07

// �����л�����
#ifdef USE_RIGHT_BACK_BOARD
	#define FILTER_ID	RIGHT_BACK_BOARD_ID
	
	#define yawBaseOff yawBaseOff_RB
	#define hipBaseOff hipBaseOff_RB
	#define kneeBaseOff kneeBaseOff_RB
	
	#define yawBaseSym yawBaseSym_RB
	#define hipBaseSym hipBaseSym_RB
	#define kneeBaseSym kneeBaseSym_RB
	
	#define yawSpeedSym yawSpeedSym_RB
	#define hipSpeedSym hipSpeedSym_RB
	#define kneeSpeedSym kneeSpeedSym_RB	
	
	#define Knee_Minangle Knee_Minangle_RB
	#define Knee_Maxangle Knee_Maxangle_RB
	#define Hip_Maxangle Hip_Maxangle_RB
	#define Hip_Minangle Hip_Minangle_RB
	#define Yaw_Maxangle Yaw_Maxangle_RB
	#define Yaw_Minangle Yaw_Minangle_RB

#endif

#ifdef USE_LEFT_BACK_BOARD
	#define FILTER_ID	LEFT_BACK_BOARD_ID
	
	#define yawBaseOff yawBaseOff_LB
	#define hipBaseOff hipBaseOff_LB
	#define kneeBaseOff kneeBaseOff_LB
	
	#define yawBaseSym yawBaseSym_LB
	#define hipBaseSym hipBaseSym_LB
	#define kneeBaseSym kneeBaseSym_LB
	
	#define yawSpeedSym yawSpeedSym_LB
	#define hipSpeedSym hipSpeedSym_LB
	#define kneeSpeedSym kneeSpeedSym_LB
	
	#define Knee_Minangle Knee_Minangle_LB
	#define Knee_Maxangle Knee_Maxangle_LB
	#define Hip_Maxangle Hip_Maxangle_LB
	#define Hip_Minangle Hip_Minangle_LB
	#define Yaw_Maxangle Yaw_Maxangle_LB
	#define Yaw_Minangle Yaw_Minangle_LB	
	
#endif

#ifdef USE_LEFT_FRONT_BOARD
	#define FILTER_ID	LEFT_FRONT_BOARD_ID
	
	#define yawBaseOff yawBaseOff_LF
	#define hipBaseOff hipBaseOff_LF
	#define kneeBaseOff kneeBaseOff_LF
	
	#define yawBaseSym yawBaseSym_LF
	#define hipBaseSym hipBaseSym_LF
	#define kneeBaseSym kneeBaseSym_LF
	
	#define yawSpeedSym yawSpeedSym_LF
	#define hipSpeedSym hipSpeedSym_LF
	#define kneeSpeedSym kneeSpeedSym_LF
	
	#define Knee_Minangle Knee_Minangle_LF
	#define Knee_Maxangle Knee_Maxangle_LF
	#define Hip_Maxangle Hip_Maxangle_LF
	#define Hip_Minangle Hip_Minangle_LF
	#define Yaw_Maxangle Yaw_Maxangle_LF
	#define Yaw_Minangle Yaw_Minangle_LF	
	
#endif

#ifdef USE_RIGHT_FRONT_BOARD
	#define FILTER_ID	RIGHT_FRONT_BOARD_ID
	
	#define yawBaseOff yawBaseOff_RF
	#define hipBaseOff hipBaseOff_RF
	#define kneeBaseOff kneeBaseOff_RF
	
	#define yawBaseSym yawBaseSym_RF
	#define hipBaseSym hipBaseSym_RF
	#define kneeBaseSym kneeBaseSym_RF
	
	#define yawSpeedSym yawSpeedSym_RF
	#define hipSpeedSym hipSpeedSym_RF
	#define kneeSpeedSym kneeSpeedSym_RF
	
	#define Knee_Minangle Knee_Minangle_RF
	#define Knee_Maxangle Knee_Maxangle_RF
	#define Hip_Maxangle Hip_Maxangle_RF
	#define Hip_Minangle Hip_Minangle_RF
	#define Yaw_Maxangle Yaw_Maxangle_RF
	#define Yaw_Minangle Yaw_Minangle_RF
	
#endif

#ifdef TEST_ANGLE_
double fun_0(int scale, int offset, short c) {
  double pos = 0;
  pos = 3600 * (double)c / 4096 * 10;
  pos = scale * (pos - offset);
  pos = pos * 314.15926 / 180;
  pos = pos / 10000;

  return pos;
}

int main() {
  std::cout << "FL:\n"
      << "YAW:\t"
      << fun_0(yawBaseSym_LF, yawBaseOff_LF, Yaw_Minangle_LF) << " "
      << fun_0(yawBaseSym_LF, yawBaseOff_LF, Yaw_Maxangle_LF) << "\n"
      << "HIP:\t"
      << fun_0(hipBaseSym_LF, hipBaseOff_LF, Hip_Minangle_LF) << " "
      << fun_0(hipBaseSym_LF, hipBaseOff_LF, Hip_Maxangle_LF) << "\n"
      << "KNE:\t"
      << fun_0(kneeBaseSym_LF, kneeBaseOff_LF, Knee_Minangle_LF) << " "
      << fun_0(kneeBaseSym_LF, kneeBaseOff_LF, Knee_Maxangle_LF) << "\n"<< std::endl;

  std::cout << "HL:\n"
      << "YAW:\t"
      << fun_0(yawBaseSym_LB, yawBaseOff_LB, Yaw_Minangle_LB) << " "
      << fun_0(yawBaseSym_LB, yawBaseOff_LB, Yaw_Maxangle_LB) << "\n"
      << "HIP:\t"
      << fun_0(hipBaseSym_LB, hipBaseOff_LB, Hip_Minangle_LB) << " "
      << fun_0(hipBaseSym_LB, hipBaseOff_LB, Hip_Maxangle_LB) << "\n"
      << "KNE:\t"
      << fun_0(kneeBaseSym_LB, kneeBaseOff_LB, Knee_Minangle_LB) << " "
      << fun_0(kneeBaseSym_LB, kneeBaseOff_LB, Knee_Maxangle_LB) << "\n"<< std::endl;

  std::cout << "FR:\n"
      << "YAW:\t"
      << fun_0(yawBaseSym_RF, yawBaseOff_RF, Yaw_Minangle_RF) << " "
      << fun_0(yawBaseSym_RF, yawBaseOff_RF, Yaw_Maxangle_RF) << "\n"
      << "HIP:\t"
      << fun_0(hipBaseSym_RF, hipBaseOff_RF, Hip_Minangle_RF) << " "
      << fun_0(hipBaseSym_RF, hipBaseOff_RF, Hip_Maxangle_RF) << "\n"
      << "KNE:\t"
      << fun_0(kneeBaseSym_RF, kneeBaseOff_RF, Knee_Minangle_RF) << " "
      << fun_0(kneeBaseSym_RF, kneeBaseOff_RF, Knee_Maxangle_RF) << "\n"<< std::endl;

  std::cout << "HR:\n"
      << "YAW:\t"
      << fun_0(yawBaseSym_RB, yawBaseOff_RB, Yaw_Minangle_RB) << " "
      << fun_0(yawBaseSym_RB, yawBaseOff_RB, Yaw_Maxangle_RB) << "\n"
      << "HIP:\t"
      << fun_0(hipBaseSym_RB, hipBaseOff_RB, Hip_Minangle_RB) << " "
      << fun_0(hipBaseSym_RB, hipBaseOff_RB, Hip_Maxangle_RB) << "\n"
      << "KNE:\t"
      << fun_0(kneeBaseSym_RB, kneeBaseOff_RB, Knee_Minangle_RB) << " "
      << fun_0(kneeBaseSym_RB, kneeBaseOff_RB, Knee_Maxangle_RB) << "\n"<< std::endl;

  return 0;
}
#endif

#endif
