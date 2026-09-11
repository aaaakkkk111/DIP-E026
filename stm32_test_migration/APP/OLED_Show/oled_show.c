#include "oled_show.h"


void show_mode_oled(void)
{
	static uint8_t mode_old = 0;
	
	if(mode == mode_old) return; 
	
	//不相等赋值  Unequal assignment
	mode_old = (uint8_t)mode;
		
	switch (mode_old)
	{
		case Normal: OLED_Draw_Line("1.Standard Mode", 1, true, true);  				 			break;
		case U_Follow: OLED_Draw_Line("2.UT Follow", 1, true, true);   		 			break;
		case U_Avoid: OLED_Draw_Line("3.UT Avoid", 1, true, true);   		 			break;
		case Weight_M: OLED_Draw_Line("4.Load Movement", 1, true, true);   		 			break;
		case PS2_Control: OLED_Draw_Line("5.Handle Control", 1, true, true);  			 			break;
		case Line_Track: OLED_Draw_Line("6.IR Track", 1, true, true);  			break;
		case Diff_Line_track: OLED_Draw_Line("7.Adv IR Track", 1, true, true);  break;
		case K210_QR: OLED_Draw_Line("8.K210 QR Rec", 1, true, true);  			 			break;
		case K210_Line: OLED_Draw_Line("9.K210 Track", 1, true, true);  	 		  break;
		case K210_Follow: OLED_Draw_Line("10.K210 Follow", 1, true, true);     break;
		case K210_SelfLearn: OLED_Draw_Line("11.K210 Self Learn", 1, true, true);    break;
		case K210_mnist: OLED_Draw_Line("12.K210 Num Rec", 1, true, true);     	break;
		case LiDar_avoid: OLED_Draw_Line("13.LiDAR Avoid", 1, true, true);  	  break;
		case LiDar_Follow: OLED_Draw_Line("14.LiDAR Follow", 1, true, true);   break;
		case LiDar_aralm: OLED_Draw_Line("15.LiDAR Guard", 1, true, true);  		break;
		case LiDar_Patrol: OLED_Draw_Line("16.LiDAR Patrol", 1, true, true); 	break;
		case LiDar_Line: OLED_Draw_Line("17.LiDAR StrLine1", 1, true, true);  			break;
		case LiDar_wall_Line: OLED_Draw_Line("18.LiDAR StrLine2", 1, true, true);  			break;
		case CCD_Mode: OLED_Draw_Line("19.CCD Track", 1, true, true);  				 			break;
		case ElE_Mode: OLED_Draw_Line("20.EM Track", 1, true, true);  				 			break;
		case TEST_Mode: OLED_Draw_Line("21.TEST Neural", 1, true, true); break;
	}
	
}

