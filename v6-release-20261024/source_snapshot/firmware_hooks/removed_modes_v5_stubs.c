/* removed_modes_v5_stubs.c -- empty stand-ins for the modes taken out of v5 (2026-10-21).
 *
 * The board only has ~64 KB of reliable flash and v5 (mode 27) ran out of room.
 * The user keeps ONLY mode 27 ("除了我的这个模式，其他的比如26以及之前的都不需要"), with
 * Bluetooth.  The sources of line tracking (IR / CCD / electromagnetic), K210, lidar and
 * the old load modes 22-26 are out of the build; Mode_select() always selects mode 27,
 * so none of these can run.  The stock code that still names them links here.
 * Signatures copied from the stock headers.
 */
#include "AllHeader.h"

/* lidar */
u8      g_lidar_go_flag = 0;
uint8_t lidar_new_pack = 0;
void Deal_lidardata(void) {}
void Car_Alarm(void) {}
void Car_Avoid(void) {}
void Car_Follow(void) {}
void Car_Patrol(void) {}
void Get_DIS_Group(void) {}
void LiDar_Straight(void) {}
void LiDar_Straight_wall(void) {}
void Start_Lidar(void) {}
void Stop_Lidar(void) {}
void USART3_init(u32 baudrate) { (void)baudrate; }

/* K210 */
void APP_K210X_Y_Follow_PID(void) {}
void Change_state_QR(void) {}
void Change_state_minst(void) {}
void Change_state_self(void) {}
void Deal_K210_AI(uint8_t recv_msg) { (void)recv_msg; }
void Deal_K210_QR(uint8_t recv_msg) { (void)recv_msg; }
void Deal_K210_minst(uint8_t recv_msg) { (void)recv_msg; }
void Deal_K210_self(uint8_t recv_msg) { (void)recv_msg; }
void Set_K210track_speed(void) {}
int  Turn_K210_PD(float gyro) { (void)gyro; return 0; }

/* IR / CCD / electromagnetic line tracking */
uint8_t* CCD_Get_ADC_128X32(void) { return 0; }
void OLED_Show_CCD_Image(uint8_t* p_img) { (void)p_img; }
void Set_CCDtrack_speed(void) {}
void Set_IRtrack_speed(void) {}
void Set_eletrack_speed(void) {}
int  Turn_CCD_PD(float gyro) { (void)gyro; return 0; }
int  Turn_ELE_PD(float gyro) { (void)gyro; return 0; }
int  Turn_IRTrack_PD(float gyro) { (void)gyro; return 0; }
void ccd_Init(void) {}
void deal_data_ccd(void) {}
void ele_Init(void) {}
void EleDataDeal(void) {}
void getEleData(void) {}
void irtracking_init(void) {}

/* old load modes 22-25 (load_ctrl.c) and 26 (load_adapt.c) */
float LD_EMPTY_ABOVE = 0.0f;
float LD_LOADED_BELOW = 0.0f;
int   ld_force = 0;
int   ld_heavy = 0;
float ld_rms = 0.0f;
void  LD_Reset(void) {}
void  LD_SetProfile(int p) { (void)p; }
void  LD_Tick(void) {}
float la_k = 0.0f;
float la_lean = 0.0f;
int   la_level = 0;
float la_osc = 0.0f;
void  LA_Reset(void) {}
void  LA_Tick(int ml, int mr, int el, int er) { (void)ml; (void)mr; (void)el; (void)er; }
