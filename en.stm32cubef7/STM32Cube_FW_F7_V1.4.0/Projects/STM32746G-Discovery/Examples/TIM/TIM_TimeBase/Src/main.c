/**
  ******************************************************************************
  * @file    TIM/TIM_TimeBase/Src/main.c
  * @author  MCD Application Team
  * @version V1.0.3
  * @date    22-April-2016 
  * @brief   This sample code shows how to use STM32F7xx TIM HAL API to generate
  *          a time base of one second with the corresponding Interrupt request.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#define ARM_MATH_CM7
#include "main.h"
#include "math.h"
#include "core_cm7.h"
#include "arm_math.h"
#include "arm_const_structs.h"
//#include "stm32f7xx_hal_spi.c"
//#include "math_helper.h"
/** @addtogroup STM32F7xx_HAL_Examples
  * @{
  */

/** @addtogroup TIM_TimeBase
  * @{
  */

enum {
	TRANSFER_WAIT,
	TRANSFER_COMPLETE,
	TRANSFER_ERROR
};
//extern void HAL_SPI_IRQHandler(SPI_HandleTypeDef *hspi);

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* TIM handle declaration */
TIM_HandleTypeDef    TimHandle;
SPI_HandleTypeDef SpiHandle;
/* Prescaler declaration */
uint32_t uwPrescalerValue = 0;

/* transfer state */
__IO uint32_t wTransferState = TRANSFER_WAIT;
static GPIO_InitTypeDef  GPIO_InitStruct;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void Error_Handler(void);
static void CPU_CACHE_Enable(void);
static void MPU_Config(void);
static uint16_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/



/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define TRANSMITTER_BOARD
//#define __FPU_PRESENT 1
/* Private functions ---------------------------------------------------------*/
/* UART handler declaration */
UART_HandleTypeDef UartHandle;
__IO ITStatus UartReady = RESET;
__IO uint32_t UserButtonStatus = 0;  /* set to 1 after User Button interrupt  */

/* Buffer used for transmission */
uint8_t aTxBuffer[] = " ****UART_TwoBoards communication based on DMA****  ****UART_TwoBoards communication based on DMA****  ****UART_TwoBoards communication based on DMA***\n";
uint8_t aTxBuffer2[] = " Hello World \n";
uint8_t aTxBuffer3[256];
/* Buffer used for reception */
uint8_t aRxBuffer[RXBUFFERSIZE];
uint32_t system_time =0;
float32_t dummy[1];
float32_t test[1];

float32_t F_Test=20; // 1Hz
float32_t Ts=.001; // Sample Time
float32_t Test_Signal;
float32_t clock_counter=0;


float32_t F_Array[8] = {200,210,220,230,240,250,260,270};
float32_t V_Array[8] = {1,1,1,1,1,1,1,1};
float32_t Demod_Sine_Signal[1];
float32_t Demod_Cosine_Signal[1];
float32_t Demod_Sine_Signal_Filtered;
float32_t Square_Wave_Test;
float32_t Demod_Cosine_Signal_Filtered;
float32_t MA_Filter_Array[1000];
int32_t MA_Filter_Length=100;
int32_t MA_Filter_Counter=0;

float32_t FIR_Input[1];
float32_t FIR_Output[1];

#define NUM_TAPS              1000
#define BLOCK_SIZE            1
static float32_t firStateF32[BLOCK_SIZE + NUM_TAPS - 1];

const float32_t firCoeffs32[NUM_TAPS] = {
  1.698076744e-06,5.104382126e-06,8.524179975e-06,1.195741515e-05,1.540403537e-05,
  1.886398604e-05,2.233721352e-05,2.582366142e-05,2.932327334e-05,3.283599654e-05,
   3.63617728e-05,3.990054392e-05,4.345225534e-05,4.701685248e-05,5.059426985e-05,
  5.418445289e-05,5.778734703e-05,6.140288315e-05,6.503101758e-05,6.867167394e-05,
  7.232480129e-05,7.599033415e-05,7.966821431e-05,8.335837629e-05,8.706076915e-05,
  9.077532013e-05,9.450197103e-05,9.824064909e-05,0.0001019913107,0.0001057538757,
   0.000109528286,0.0001133144688,0.0001171123731,0.0001209219263,0.0001247430628,
  0.0001285757171,0.0001324198238,0.0001362753101,0.0001401421177, 0.000144020174,
  0.0001479094062,0.0001518097561,0.0001557211508,0.0001596435177,0.0001635767985,
   0.000167520906,0.0001714757818,0.0001754413533,0.0001794175478,0.0001834043069,
  0.0001874015434,0.0001914091845,0.0001954271575,0.0001994554041,0.0002034938516,
  0.0002075423981,0.0002116010146,0.0002156695846,0.0002197480499,0.0002238363522,
  0.0002279343898,0.0002320421045,0.0002361594088,0.0002402862447,0.0002444225247,
  0.0002485681616,0.0002527231118,0.0002568872587,0.0002610605152,0.0002652428811,
   0.000269434182,0.0002736344177,0.0002778434427,0.0002820612572,0.0002862876863,
  0.0002905227302,0.0002947662724,0.0002990182256,0.0003032785316,0.0003075471031,
  0.0003118238528,0.0003161086934,0.0003204015666, 0.000324702356,0.0003290110035,
  0.0003333274217,0.0003376515233, 0.000341983221,0.0003463224566,0.0003506691137,
   0.000355023134,0.0003593844303,0.0003637528862,0.0003681284434,0.0003725110146,
  0.0003769005125,0.0003812968498,0.0003856999683,0.0003901097225,0.0003945260542,
  0.0003989489051,0.0004033781588,0.0004078137281,0.0004122555256,0.0004167034931,
  0.0004211574851,0.0004256174725,0.0004300833098,0.0004345549678,0.0004390323302,
  0.0004435152805,0.0004480037605, 0.000452497683,0.0004569969606,0.0004615014768,
  0.0004660111736,0.0004705259344,0.0004750456719,0.0004795702989,0.0004840997281,
   0.000488633872,0.0004931726144,0.0004977158969,0.0005022636033,0.0005068156752,
  0.0005113719963,0.0005159324501,0.0005204969784,0.0005250654649,0.0005296378513,
  0.0005342140212,0.0005387938581, 0.000543377304,0.0005479642423,0.0005525545566,
   0.000557148247,0.0005617451388,0.0005663451157,0.0005709481775,0.0005755541497,
  0.0005801629159, 0.000584774476,0.0005893887137,0.0005940054543,0.0005986246397,
  0.0006032462115,0.0006078700535,0.0006124960491,0.0006171241403,0.0006217542104,
  0.0006263861433,0.0006310198223,0.0006356552476,0.0006402922445,0.0006449306966,
  0.0006495706039,0.0006542117917,0.0006588541437,0.0006634976016,0.0006681421073,
  0.0006727874861,0.0006774336798, 0.000682080572,0.0006867280463,0.0006913761026,
  0.0006960245082,0.0007006732631,0.0007053222507,0.0007099713548,0.0007146204589,
  0.0007192695048, 0.000723918376,0.0007285669562, 0.000733215129,0.0007378628943,
  0.0007425100193,0.0007471565041,0.0007518022321, 0.000756447087,0.0007610909524,
  0.0007657337701,0.0007703753654,0.0007750157383,0.0007796547143, 0.000784292235,
  0.0007889281842,0.0007935624453,0.0007981949602,0.0008028256125,0.0008074542857,
  0.0008120808634,0.0008167052874,0.0008213274996, 0.000825947267,0.0008305645897,
  0.0008351793513,0.0008397914353,0.0008444007253,0.0008490071632,0.0008536106325,
   0.000858211075,0.0008628082578,0.0008674022392,0.0008719928446,0.0008765799575,
  0.0008811635198,0.0008857433568,0.0008903194685,0.0008948917384,0.0008994599921,
  0.0009040241712,0.0009085841593,0.0009131398983,0.0009176912718,0.0009222381632,
  0.0009267805144,0.0009313181508,0.0009358510724,0.0009403790464,0.0009449020727,
  0.0009494200349,0.0009539328748,0.0009584403597,0.0009629425476,0.0009674392059,
  0.0009719303343,0.0009764157585,0.0009808954783,0.0009853693191,0.0009898371063,
  0.0009942989564,  0.00099875452, 0.001003203914, 0.001007646904, 0.001012083376,
   0.001016513328, 0.001020936645, 0.001025353209, 0.001029762905, 0.001034165616,
   0.001038561342, 0.001042949851, 0.001047331141, 0.001051705098, 0.001056071604,
   0.001060430659, 0.001064781914, 0.001069125487, 0.001073461259, 0.001077789115,
   0.001082108938, 0.001086420612, 0.001090724021, 0.001095019164, 0.001099305809,
   0.001103584073, 0.001107853721, 0.001112114522, 0.001116366708, 0.001120609813,
   0.001124844071, 0.001129069249, 0.001133285114, 0.001137491781, 0.001141689136,
   0.001145876944, 0.001150055206, 0.001154223806, 0.001158382627, 0.001162531669,
   0.001166670816, 0.001170799835, 0.001174918725, 0.001179027488, 0.001183125889,
   0.001187213813, 0.001191291376, 0.001195358229, 0.001199414488,  0.00120345992,
   0.001207494526, 0.001211518073, 0.001215530676, 0.001219532103, 0.001223522355,
   0.001227501198, 0.001231468632, 0.001235424541, 0.001239368925, 0.001243301667,
   0.001247222535, 0.001251131529, 0.001255028648,  0.00125891366, 0.001262786682,
   0.001266647363, 0.001270495704, 0.001274331706,  0.00127815525, 0.001281966106,
   0.001285764389, 0.001289549982, 0.001293322654, 0.001297082403,  0.00130082923,
   0.001304562902, 0.001308283419, 0.001311990665, 0.001315684523, 0.001319364994,
    0.00132303196, 0.001326685306, 0.001330324914, 0.001333950786, 0.001337562921,
   0.001341160969, 0.001344745047,  0.00134831504, 0.001351870829, 0.001355412416,
   0.001358939568, 0.001362452284, 0.001365950564, 0.001369434176, 0.001372903236,
   0.001376357395, 0.001379796769, 0.001383221359, 0.001386630815, 0.001390025252,
   0.001393404556,  0.00139676861, 0.001400117297, 0.001403450733,  0.00140676857,
    0.00141007104, 0.001413357793, 0.001416628831, 0.001419884153, 0.001423123642,
   0.001426347182, 0.001429554773,   0.0014327463, 0.001435921644, 0.001439080806,
   0.001442223671, 0.001445350237, 0.001448460273, 0.001451553893, 0.001454630867,
   0.001457691309, 0.001460734871, 0.001463761786, 0.001466771821, 0.001469764858,
     0.0014727409, 0.001475699944,  0.00147864176, 0.001481566462, 0.001484473818,
   0.001487363945, 0.001490236493, 0.001493091695, 0.001495929318, 0.001498749363,
   0.001501551596, 0.001504336251, 0.001507102977, 0.001509852009,  0.00151258288,
   0.001515295939, 0.001517990837, 0.001520667691, 0.001523326267, 0.001525966683,
   0.001528588706, 0.001531192334, 0.001533777569, 0.001536344411, 0.001538892626,
   0.001541422214,  0.00154393306, 0.001546425279, 0.001548898639,  0.00155135314,
   0.001553788781, 0.001556205563, 0.001558603137, 0.001560981735,  0.00156334124,
   0.001565681421,  0.00156800251, 0.001570304157, 0.001572586596, 0.001574849593,
   0.001577093033, 0.001579317031, 0.001581521472, 0.001583706355, 0.001585871563,
   0.001588016981, 0.001590142609, 0.001592248562, 0.001594334492, 0.001596400631,
   0.001598446746, 0.001600472839, 0.001602478907, 0.001604464836, 0.001606430626,
   0.001608376275, 0.001610301551, 0.001612206688, 0.001614091336, 0.001615955727,
   0.001617799629, 0.001619623043, 0.001621426083, 0.001623208402, 0.001624970231,
   0.001626711339, 0.001628431841, 0.001630131621, 0.001631810679, 0.001633468899,
    0.00163510628, 0.001636722707, 0.001638318296, 0.001639892929, 0.001641446608,
   0.001642979216, 0.001644490752, 0.001645981218, 0.001647450612, 0.001648898702,
   0.001650325721, 0.001651731436, 0.001653115847, 0.001654479071, 0.001655820874,
   0.001657141373, 0.001658440451, 0.001659718109, 0.001660974231, 0.001662208932,
   0.001663422212, 0.001664613839,  0.00166578393, 0.001666932367, 0.001668059267,
   0.001669164398, 0.001670247992, 0.001671309699,  0.00167234987, 0.001673368155,
    0.00167436467, 0.001675339416, 0.001676292391, 0.001677223481, 0.001678132685,
   0.001679020002, 0.001679885318, 0.001680728863,  0.00168155029,  0.00168234983,
   0.001683127368, 0.001683882903, 0.001684616436, 0.001685327967, 0.001686017378,
   0.001686684671, 0.001687329961, 0.001687953016, 0.001688554068, 0.001689132885,
   0.001689689583, 0.001690224162, 0.001690736506, 0.001691226731, 0.001691694721,
   0.001692140475, 0.001692563994, 0.001692965277, 0.001693344326, 0.001693701139,
   0.001694035716, 0.001694348059, 0.001694638049, 0.001694905804, 0.001695151208,
   0.001695374376, 0.001695575193, 0.001695753774, 0.001695910003, 0.001696043881,
   0.001696155523, 0.001696244814, 0.001696311752, 0.001696356339, 0.001696378691,
   0.001696378691, 0.001696356339, 0.001696311752, 0.001696244814, 0.001696155523,
   0.001696043881, 0.001695910003, 0.001695753774, 0.001695575193, 0.001695374376,
   0.001695151208, 0.001694905804, 0.001694638049, 0.001694348059, 0.001694035716,
   0.001693701139, 0.001693344326, 0.001692965277, 0.001692563994, 0.001692140475,
   0.001691694721, 0.001691226731, 0.001690736506, 0.001690224162, 0.001689689583,
   0.001689132885, 0.001688554068, 0.001687953016, 0.001687329961, 0.001686684671,
   0.001686017378, 0.001685327967, 0.001684616436, 0.001683882903, 0.001683127368,
    0.00168234983,  0.00168155029, 0.001680728863, 0.001679885318, 0.001679020002,
   0.001678132685, 0.001677223481, 0.001676292391, 0.001675339416,  0.00167436467,
   0.001673368155,  0.00167234987, 0.001671309699, 0.001670247992, 0.001669164398,
   0.001668059267, 0.001666932367,  0.00166578393, 0.001664613839, 0.001663422212,
   0.001662208932, 0.001660974231, 0.001659718109, 0.001658440451, 0.001657141373,
   0.001655820874, 0.001654479071, 0.001653115847, 0.001651731436, 0.001650325721,
   0.001648898702, 0.001647450612, 0.001645981218, 0.001644490752, 0.001642979216,
   0.001641446608, 0.001639892929, 0.001638318296, 0.001636722707,  0.00163510628,
   0.001633468899, 0.001631810679, 0.001630131621, 0.001628431841, 0.001626711339,
   0.001624970231, 0.001623208402, 0.001621426083, 0.001619623043, 0.001617799629,
   0.001615955727, 0.001614091336, 0.001612206688, 0.001610301551, 0.001608376275,
   0.001606430626, 0.001604464836, 0.001602478907, 0.001600472839, 0.001598446746,
   0.001596400631, 0.001594334492, 0.001592248562, 0.001590142609, 0.001588016981,
   0.001585871563, 0.001583706355, 0.001581521472, 0.001579317031, 0.001577093033,
   0.001574849593, 0.001572586596, 0.001570304157,  0.00156800251, 0.001565681421,
    0.00156334124, 0.001560981735, 0.001558603137, 0.001556205563, 0.001553788781,
    0.00155135314, 0.001548898639, 0.001546425279,  0.00154393306, 0.001541422214,
   0.001538892626, 0.001536344411, 0.001533777569, 0.001531192334, 0.001528588706,
   0.001525966683, 0.001523326267, 0.001520667691, 0.001517990837, 0.001515295939,
    0.00151258288, 0.001509852009, 0.001507102977, 0.001504336251, 0.001501551596,
   0.001498749363, 0.001495929318, 0.001493091695, 0.001490236493, 0.001487363945,
   0.001484473818, 0.001481566462,  0.00147864176, 0.001475699944,   0.0014727409,
   0.001469764858, 0.001466771821, 0.001463761786, 0.001460734871, 0.001457691309,
   0.001454630867, 0.001451553893, 0.001448460273, 0.001445350237, 0.001442223671,
   0.001439080806, 0.001435921644,   0.0014327463, 0.001429554773, 0.001426347182,
   0.001423123642, 0.001419884153, 0.001416628831, 0.001413357793,  0.00141007104,
    0.00140676857, 0.001403450733, 0.001400117297,  0.00139676861, 0.001393404556,
   0.001390025252, 0.001386630815, 0.001383221359, 0.001379796769, 0.001376357395,
   0.001372903236, 0.001369434176, 0.001365950564, 0.001362452284, 0.001358939568,
   0.001355412416, 0.001351870829,  0.00134831504, 0.001344745047, 0.001341160969,
   0.001337562921, 0.001333950786, 0.001330324914, 0.001326685306,  0.00132303196,
   0.001319364994, 0.001315684523, 0.001311990665, 0.001308283419, 0.001304562902,
    0.00130082923, 0.001297082403, 0.001293322654, 0.001289549982, 0.001285764389,
   0.001281966106,  0.00127815525, 0.001274331706, 0.001270495704, 0.001266647363,
   0.001262786682,  0.00125891366, 0.001255028648, 0.001251131529, 0.001247222535,
   0.001243301667, 0.001239368925, 0.001235424541, 0.001231468632, 0.001227501198,
   0.001223522355, 0.001219532103, 0.001215530676, 0.001211518073, 0.001207494526,
    0.00120345992, 0.001199414488, 0.001195358229, 0.001191291376, 0.001187213813,
   0.001183125889, 0.001179027488, 0.001174918725, 0.001170799835, 0.001166670816,
   0.001162531669, 0.001158382627, 0.001154223806, 0.001150055206, 0.001145876944,
   0.001141689136, 0.001137491781, 0.001133285114, 0.001129069249, 0.001124844071,
   0.001120609813, 0.001116366708, 0.001112114522, 0.001107853721, 0.001103584073,
   0.001099305809, 0.001095019164, 0.001090724021, 0.001086420612, 0.001082108938,
   0.001077789115, 0.001073461259, 0.001069125487, 0.001064781914, 0.001060430659,
   0.001056071604, 0.001051705098, 0.001047331141, 0.001042949851, 0.001038561342,
   0.001034165616, 0.001029762905, 0.001025353209, 0.001020936645, 0.001016513328,
   0.001012083376, 0.001007646904, 0.001003203914,  0.00099875452,0.0009942989564,
  0.0009898371063,0.0009853693191,0.0009808954783,0.0009764157585,0.0009719303343,
  0.0009674392059,0.0009629425476,0.0009584403597,0.0009539328748,0.0009494200349,
  0.0009449020727,0.0009403790464,0.0009358510724,0.0009313181508,0.0009267805144,
  0.0009222381632,0.0009176912718,0.0009131398983,0.0009085841593,0.0009040241712,
  0.0008994599921,0.0008948917384,0.0008903194685,0.0008857433568,0.0008811635198,
  0.0008765799575,0.0008719928446,0.0008674022392,0.0008628082578, 0.000858211075,
  0.0008536106325,0.0008490071632,0.0008444007253,0.0008397914353,0.0008351793513,
  0.0008305645897, 0.000825947267,0.0008213274996,0.0008167052874,0.0008120808634,
  0.0008074542857,0.0008028256125,0.0007981949602,0.0007935624453,0.0007889281842,
   0.000784292235,0.0007796547143,0.0007750157383,0.0007703753654,0.0007657337701,
  0.0007610909524, 0.000756447087,0.0007518022321,0.0007471565041,0.0007425100193,
  0.0007378628943, 0.000733215129,0.0007285669562, 0.000723918376,0.0007192695048,
  0.0007146204589,0.0007099713548,0.0007053222507,0.0007006732631,0.0006960245082,
  0.0006913761026,0.0006867280463, 0.000682080572,0.0006774336798,0.0006727874861,
  0.0006681421073,0.0006634976016,0.0006588541437,0.0006542117917,0.0006495706039,
  0.0006449306966,0.0006402922445,0.0006356552476,0.0006310198223,0.0006263861433,
  0.0006217542104,0.0006171241403,0.0006124960491,0.0006078700535,0.0006032462115,
  0.0005986246397,0.0005940054543,0.0005893887137, 0.000584774476,0.0005801629159,
  0.0005755541497,0.0005709481775,0.0005663451157,0.0005617451388, 0.000557148247,
  0.0005525545566,0.0005479642423, 0.000543377304,0.0005387938581,0.0005342140212,
  0.0005296378513,0.0005250654649,0.0005204969784,0.0005159324501,0.0005113719963,
  0.0005068156752,0.0005022636033,0.0004977158969,0.0004931726144, 0.000488633872,
  0.0004840997281,0.0004795702989,0.0004750456719,0.0004705259344,0.0004660111736,
  0.0004615014768,0.0004569969606, 0.000452497683,0.0004480037605,0.0004435152805,
  0.0004390323302,0.0004345549678,0.0004300833098,0.0004256174725,0.0004211574851,
  0.0004167034931,0.0004122555256,0.0004078137281,0.0004033781588,0.0003989489051,
  0.0003945260542,0.0003901097225,0.0003856999683,0.0003812968498,0.0003769005125,
  0.0003725110146,0.0003681284434,0.0003637528862,0.0003593844303, 0.000355023134,
  0.0003506691137,0.0003463224566, 0.000341983221,0.0003376515233,0.0003333274217,
  0.0003290110035, 0.000324702356,0.0003204015666,0.0003161086934,0.0003118238528,
  0.0003075471031,0.0003032785316,0.0002990182256,0.0002947662724,0.0002905227302,
  0.0002862876863,0.0002820612572,0.0002778434427,0.0002736344177, 0.000269434182,
  0.0002652428811,0.0002610605152,0.0002568872587,0.0002527231118,0.0002485681616,
  0.0002444225247,0.0002402862447,0.0002361594088,0.0002320421045,0.0002279343898,
  0.0002238363522,0.0002197480499,0.0002156695846,0.0002116010146,0.0002075423981,
  0.0002034938516,0.0001994554041,0.0001954271575,0.0001914091845,0.0001874015434,
  0.0001834043069,0.0001794175478,0.0001754413533,0.0001714757818, 0.000167520906,
  0.0001635767985,0.0001596435177,0.0001557211508,0.0001518097561,0.0001479094062,
   0.000144020174,0.0001401421177,0.0001362753101,0.0001324198238,0.0001285757171,
  0.0001247430628,0.0001209219263,0.0001171123731,0.0001133144688, 0.000109528286,
  0.0001057538757,0.0001019913107,9.824064909e-05,9.450197103e-05,9.077532013e-05,
  8.706076915e-05,8.335837629e-05,7.966821431e-05,7.599033415e-05,7.232480129e-05,
  6.867167394e-05,6.503101758e-05,6.140288315e-05,5.778734703e-05,5.418445289e-05,
  5.059426985e-05,4.701685248e-05,4.345225534e-05,3.990054392e-05, 3.63617728e-05,
  3.283599654e-05,2.932327334e-05,2.582366142e-05,2.233721352e-05,1.886398604e-05,
  1.540403537e-05,1.195741515e-05,8.524179975e-06,5.104382126e-06,1.698076744e-06
};

uint32_t blockSize = BLOCK_SIZE;
//uint32_t numBlocks = TEST_LENGTH_SAMPLES/BLOCK_SIZE;
	arm_fir_instance_f32 S;
  arm_status status;
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */







int main(void)
{
	MPU_Config();
arm_fir_init_f32(&S, NUM_TAPS, (float32_t *)&firCoeffs32[0], &firStateF32[0], blockSize);

  /* Enable the CPU Cache */
  CPU_CACHE_Enable();
  
  /* STM32F7xx HAL library initialization:
       - Configure the Flash ART accelerator on ITCM interface
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  HAL_Init();

  /* Configure the system clock to 216 MHz */
  SystemClock_Config();

  /* Configure LED1 */
  BSP_LED_Init(LED1);
	
	__GPIOB_CLK_ENABLE();
	
	GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pin = GPIO_PIN_15;
	
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*##-1- Configure the TIM peripheral #######################################*/
  /* -----------------------------------------------------------------------
    In this example TIM3 input clock (TIM3CLK)  is set to APB1 clock (PCLK1) x2,
    since APB1 prescaler is equal to 4.
      TIM3CLK = PCLK1*2
      PCLK1 = HCLK/4
      => TIM3CLK = HCLK/2 = SystemCoreClock/2
    To get TIM3 counter clock at 10 KHz, the Prescaler is computed as follows:
    Prescaler = (TIM3CLK / TIM3 counter clock) - 1
    Prescaler = ((SystemCoreClock/2) /10 KHz) - 1

    Note:
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f7xx.c file.
     Each time the core clock (HCLK) changes, user had to update SystemCoreClock
     variable value. Otherwise, any configuration based on this variable will be incorrect.
     This variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetSysClockFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency
  ----------------------------------------------------------------------- */

  /* Compute the prescaler value to have TIMx counter clock equal to 10000 Hz */
  uwPrescalerValue = (uint32_t)((SystemCoreClock / 2) / 10000) - 1;

  /* Set TIMx instance */
  TimHandle.Instance = TIMx;

  /* Initialize TIMx peripheral as follows:
       + Period = 10000 - 1
       + Prescaler = ((SystemCoreClock / 2)/10000) - 1
       + ClockDivision = 0
       + Counter direction = Up
  */
  TimHandle.Init.Period            = 1000 - 1;
  TimHandle.Init.Prescaler         = uwPrescalerValue;
  TimHandle.Init.ClockDivision     = 0;
  TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  TimHandle.Init.RepetitionCounter = 0;

  if (HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /*##-2- Start the TIM Base generation in interrupt mode ####################*/
  /* Start Channel1 */
  if (HAL_TIM_Base_Start_IT(&TimHandle) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }

	  /*##-1- Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART configured as follows:
      - Word Length = 8 Bits
      - Stop Bit = One Stop bit
      - Parity = None
      - BaudRate = 9600 baud
      - Hardware flow control disabled (RTS and CTS signals) */
  UartHandle.Instance        = USARTx;

  UartHandle.Init.BaudRate   = 112500;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits   = UART_STOPBITS_1;
  UartHandle.Init.Parity     = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode       = UART_MODE_TX_RX;
  if(HAL_UART_DeInit(&UartHandle) != HAL_OK)
  {
    Error_Handler();
  }  
  if(HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    Error_Handler();
  }
  
	

	
	
	
	
	
  while (1)
  {
		
//		     HAL_Delay(500);
//	int	len=strlen(aTxBuffer2);
//	  	HAL_UART_Transmit_DMA(&UartHandle, (uint8_t*)aTxBuffer2, len);
		
		
//		system_time=HAL_GetTick();
//		sprintf(aTxBuffer3,"Time = %d \n",system_time);
		//HAL_Delay(500);
//		HAL_UART_Transmit_DMA(&UartHandle, (uint8_t*)aTxBuffer3, strlen(aTxBuffer3));
		float x=0;
		for(int i=0; i<1000000; i++)
		{
			
			//x=i*i;
		//	arm_sin_cos_f32(i,test,dummy);
			x=arm_cos_f32(i);
		 // x=cosf(i);
		}
	//	system_time=HAL_GetTick();
	//	sprintf(aTxBuffer3,"Time = %d \n",system_time);
		//HAL_Delay(500);
	//	HAL_UART_Transmit_DMA(&UartHandle, (uint8_t*)aTxBuffer3, strlen(aTxBuffer3));
		
		
		
		
		
  }
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  BSP_LED_Toggle(LED1);
	Test_Signal=V_Array[0]*arm_sin_f32(2*PI*F_Array[0]*clock_counter*Ts)+V_Array[1]*arm_sin_f32(2*PI*F_Array[1]*clock_counter*Ts)+V_Array[2]*arm_sin_f32(2*PI*F_Array[2]*clock_counter*Ts)+V_Array[3]*arm_sin_f32(2*PI*F_Array[3]*clock_counter*Ts)+V_Array[4]*arm_sin_f32(2*PI*F_Array[4]*clock_counter*Ts)+V_Array[5]*arm_sin_f32(2*PI*F_Array[5]*clock_counter*Ts)+V_Array[6]*arm_sin_f32(2*PI*F_Array[6]*clock_counter*Ts)+V_Array[7]*arm_sin_f32(2*PI*F_Array[7]*clock_counter*Ts);
	Demod_Sine_Signal[0]=Test_Signal*arm_sin_f32(2*PI*F_Array[1]*clock_counter*Ts);
	Demod_Cosine_Signal[0]=Test_Signal*arm_sin_f32(2*PI*F_Array[1]*clock_counter*Ts);
	//MA_Filter_Array[MA_Filter_Counter]
	Square_Wave_Test=(arm_sin_f32(2*PI*F_Array[1]*clock_counter*Ts)>0)-(arm_sin_f32(2*PI*F_Array[1]*clock_counter*Ts)<0);
	FIR_Input[0]=Demod_Sine_Signal[0];
	 arm_fir_f32(&S, &FIR_Input[0], &FIR_Output[0], blockSize);
	//sprintf(aTxBuffer3,"%.12f \t %.12f \t %.12f \t %d \n",Test_Signal,FIR_Output[0],Square_Wave_Test, GPIOB->IDR & 0x8000 );
	sprintf(aTxBuffer3,"%d \n", GPIOB->IDR & 0x8000 );

	//HAL_Delay(500);
  HAL_UART_Transmit_DMA(&UartHandle, (uint8_t*)aTxBuffer3, strlen(aTxBuffer3));
	clock_counter++;
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 216000000
  *            HCLK(Hz)                       = 216000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 25
  *            PLL_N                          = 432
  *            PLL_P                          = 2
  *            PLL_Q                          = 9
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 7
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef ret = HAL_OK;

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;  
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  
  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
  
  /* Activate the OverDrive to reach the 216 MHz Frequency */  
  ret = HAL_PWREx_EnableOverDrive();
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2; 
  
  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }  
}

/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle. 
  * @note   This example shows a simple way to report end of DMA Tx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: trasfer complete*/
  UartReady = SET;

  
}

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and 
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: trasfer complete*/
  UartReady = SET;

  
}

/**
  * @brief  UART error callbacks
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
    Error_Handler();
}


/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == KEY_BUTTON_PIN)
  {  
    UserButtonStatus = 1;
  }
}

/**
  * @brief  Compares two buffers.
  * @param  pBuffer1, pBuffer2: buffers to be compared.
  * @param  BufferLength: buffer's length
  * @retval 0  : pBuffer1 identical to pBuffer2
  *         >0 : pBuffer1 differs from pBuffer2
  */
static uint16_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
  while (BufferLength--)
  {
    if ((*pBuffer1) != *pBuffer2)
    {
      return BufferLength;
    }
    pBuffer1++;
    pBuffer2++;
  }

  return 0;
}

static void Error_Handler(void)
{
  /* Turn LED1 on */
  BSP_LED_On(LED1);
  while (1)
  {
  }
}
#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}

#endif

/**
  * @brief  CPU L1-Cache enable.
  * @param  None
  * @retval None
  */
static void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
static void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct;
  
  /* Disable the MPU */
  HAL_MPU_Disable();

  /* Configure the MPU attributes as WT for SRAM */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0x20010000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_256KB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Enable the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
  * @brief  TxRx Transfer completed callback.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report end of DMA TxRx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  /* Configure LED1 which is shared with SPI2_SCK signal */
  BSP_LED_Init(LED1);
  /* Turn LED1 on: Transfer in transmission/reception process is complete */
  BSP_LED_On(LED1);
  wTransferState = TRANSFER_COMPLETE;
}

/**
  * @brief  SPI error callbacks.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  wTransferState = TRANSFER_ERROR;
}
