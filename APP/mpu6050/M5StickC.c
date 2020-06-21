
#include "stdlib.h"
#include "math.h"
#include "system.h"
#include "SysTick.h"
#include "hw_config.h"
#include "M5StickC.h"
#include "iompu6050.h" 

float Kp = 2;
float Ki = 0.1;
float Kd = 0.4;


float power = 0.0;

float thetaP, thetaI, thetaD;


int16_t accX = 0;
int16_t accY = 0;
int16_t accZ = 0;

int16_t gyroX = 0;
int16_t gyroY = 0;
int16_t gyroZ = 0;

//加速度センサ オフセット
int accXoffset = 0;
int accYoffset = 0;
int accZoffset = 0;

//ジャイロセンサ オフセット
int gyroXoffset = 0;
int gyroYoffset = 0;
int gyroZoffset = 0;



//センサ バラつき取得用変数
int sample_num = 100;
int meas_interval = 10;
float theta_deg = 0.0;
float theta_dot = 0.0;
float theta_dot2;

float theta_mean;
float theta_variance;
float theta_dot_mean;
float theta_dot_variance;


//=========================================================
//カルマン・フィルタ用変数
//=========================================================

//カルマン・フィルタ処理 
const float theta_update_freq = 100; //Hz
const float theta_update_interval = 1.0/(theta_update_freq); //10msec
//状態ベクトルx
//[[theta(degree)], [offset of theta_dot(degree/sec)]]
//状態ベクトルの予測値
float theta_data_predict[2][1];
float theta_data[2][1];
//共分散行列
float P_theta_predict[2][2];
float P_theta[2][2];
//状態方程式の"A"
float A_theta[2][2] = {{1, -theta_update_interval}, {0, 1}};
//状態方程式の"B"
float B_theta[2][1] = {{theta_update_interval}, {0}};
//出力方程式の"C"
float C_theta[1][2] = {{1, 0}};


//=========================================================
// 行列演算関数
//=========================================================

//行列の和
void mat_add(float *m1, float *m2, float *sol, int row, int column) {
    for(int i=0; i<row; i++) {
        for(int j=0; j<column; j++) {
            sol[i*column + j] = m1[i*column + j] + m2[i*column + j];    
        }    
    }
    return;
}

//行列の差
void mat_sub(float *m1, float *m2, float *sol, int row, int column){
    for(int i=0; i<row; i++) {
        for(int j=0; j<column; j++) {
            sol[i*column + j] = m1[i*column + j] - m2[i*column + j];    
        }    
    }
    return;
}

//行列の積
void mat_mul(float *m1, float *m2, float *sol, int row1, int column1, int row2, int column2){
    for(int i=0; i<row1; i++){
        for(int j=0; j<column2; j++){
            sol[i*column2 + j] = 0;
            for(int k=0; k<column1; k++) {
                sol[i*column2 + j] += m1[i*column1 + k]*m2[k*column2 + j];    
            }
        }    
    }
    return;
}

//転置行列算出
void mat_tran(float *m1, float *sol, int row_original, int column_original) {
    for(int i=0; i<row_original; i++) {
        for(int j=0; j<column_original; j++) {
            sol[j*row_original + i] = m1[i*column_original + j];    
        }    
    }
    return;
}

//行列の定数倍算出
void mat_mul_const(float *m1,float c, float *sol, int row, int column){
    for(int i=0; i<row; i++){
        for(int j=0; j<column; j++){
            sol[i*column + j] = c * m1[i*column + j];    
        }    
    }
    return;
}

//逆行列算出
void mat_inv(float *m, float *sol, int column, int row){
    //allocate memory for a temporary matrix
    float* temp = (float *)malloc( column*2*row*sizeof(float) );
    
    //make the augmented matrix
    for(int i=0; i<column; i++) {
        //copy original matrix
        for(int j=0; j<row; j++) {
            temp[i*(2*row) + j] = m[i*row + j];  
        }
        
        //make identity matrix
        for(int j=row; j<row*2; j++) {
            if(j-row == i) {
                temp[i*(2*row) + j] = 1;
            }    
            else {
                temp[i*(2*row) + j] = 0;    
            }
        }
    }

    //Sweep (down)
    for(int i=0; i<column; i++) {
        //pivot selection
        float pivot = temp[i*(2*row) + i];
        int pivot_index = i;
        float pivot_temp;
        for(int j=i; j<column;j++) {
            if( temp[j*(2*row)+i] > pivot ) {
                pivot = temp[j*(2*row) + i];
                pivot_index = j;
            }    
        }  
        if(pivot_index != i) {
            for(int j=0; j<2*row; j++) {
                pivot_temp = temp[ pivot_index * (2*row) + j ];
                temp[pivot_index * (2*row) + j] = temp[i*(2*row) + j];
                temp[i*(2*row) + j] = pivot_temp;    
            }    
        }
        
        //division
        for(int j=0; j<2*row; j++) {
            temp[i*(2*row) + j] /= pivot;    
        }
        
        //sweep
        for(int j=i+1; j<column; j++) {
            float temp2 = temp[j*(2*row) + i];
            
            //sweep each row
            for(int k=0; k<row*2; k++) {
                temp[j*(2*row) + k] -= temp2 * temp[ i*(2*row) + k ];    
            }    
        }
    }
        
    //Sweep (up)
    for(int i=0; i<column-1; i++) {
        for(int j=i+1; j<column; j++) {
            float pivot = temp[ (column-1-j)*(2*row) + (row-1-i)];   
            for(int k=0; k<2*row; k++) {
                temp[(column-1-j)*(2*row)+k] -= pivot * temp[(column-1-i)*(2*row)+k];    
            }
        }    
    }     
    
    //copy result
    for(int i=0; i<column; i++) {
        for(int j=0; j<row; j++) {
            sol[i*row + j] = temp[i*(2*row) + (j+row)];    
        }    
    }
    free(temp);
    return;
}


//センサオフセット算出
void offset_cal(){
  delay_ms(1000);
  accXoffset = 0;
  accYoffset = 0;
  accZoffset = 0;
  gyroXoffset = 0;
  gyroYoffset = 0;
  gyroZoffset = 0;

  for(int i=0; i<10; i++) {
    MPU6050_Get_Accelerometer(&accY,&accX,&accZ);
    MPU6050_Get_Gyroscope(&gyroY,&gyroX,&gyroZ);
    delay_ms(meas_interval);
    accXoffset += accX;
    accYoffset += accY;
    accZoffset += accZ;
    gyroXoffset += gyroX;
    gyroYoffset += gyroY;
    gyroZoffset += gyroZ;
  }

  accXoffset /= 10;
  accYoffset /= 10;
	accZoffset = accZoffset / 10 - 16384;
  gyroXoffset /= 10;
  gyroYoffset /= 10;
  gyroZoffset /= 10;
}


//加速度センサから傾きデータ取得 [deg]
float get_acc_data() {
  MPU6050_Get_Accelerometer(&accY,&accX,&accZ);
  //得られたセンサ値はオフセット引いて使用
  //傾斜角導出 単位はdeg
  theta_deg  = atan( (float)(accY - accYoffset) / (float)(-1 * accZ - accZoffset) ) * 57.29578f;
  return theta_deg;
}


//加速度センサによる傾きのばらつき測定
void acc_init(){
  float theta_array[sample_num];
  for(int i=0; i<sample_num; i++) {
    theta_array[i] = get_acc_data();
    delay_ms(meas_interval);
  }

  //平均値
  theta_mean = 0;
  for(int i=0; i<sample_num; i++) {
    theta_mean += theta_array[i];
  }
  theta_mean /= sample_num;
    
  //分散
  float temp;
  theta_variance = 0;
  for(int i=0; i<sample_num; i++) {
    temp = theta_array[i] - theta_mean;
    theta_variance += temp*temp;
  }
  theta_variance /= sample_num;
}


//x軸 角速度取得
float get_gyro_data() {
  MPU6050_Get_Gyroscope(&gyroY,&gyroX,&gyroZ);
  //得られたセンサ値はオフセット引いて使用
  theta_dot = ((float) (gyroX - gyroXoffset)) / 16.384;
  return theta_dot;
}

//ジャイロセンサばらつき測定
void gyro_init() {
  float theta_dot_array[sample_num];
  for(int i=0;i<sample_num;i++) {
    theta_dot_array[i] =  get_gyro_data();
    delay_ms(meas_interval);
  }
    
  //平均値
  theta_dot_mean = 0;
  for(int i=0;i<sample_num;i++) {
    theta_dot_mean += theta_dot_array[i];    
  }
  theta_dot_mean /= sample_num;
 
  //分散
  float temp;
  theta_dot_variance = 0;
  for(int i=0; i<sample_num; i++) {
    temp = theta_dot_array[i] - theta_dot_mean;
    theta_dot_variance += temp*temp;    
  }
  theta_dot_variance /= sample_num;
}


//=========================================================
//カルマン・フィルタアルゴリズム処理
//=========================================================
void update_theta()
{     
    //加速度センサによる角度測定
    float y = get_acc_data(); //degree
    
    //入力データ：角速度
    float theta_dot_gyro = get_gyro_data(); //degree/sec
      
    //カルマン・ゲイン算出: G = P'C^T(W+CP'C^T)^-1
    float P_CT[2][1] = {0};
    float tran_C_theta[2][1] = {0};
    mat_tran(C_theta[0], tran_C_theta[0], 1, 2);//C^T
    mat_mul(P_theta_predict[0], tran_C_theta[0], P_CT[0], 2, 2, 2, 1);//P'C^T
    float G_temp1[1][1] = {0};
    mat_mul(C_theta[0], P_CT[0], G_temp1[0], 1,2, 2,1);//CP'C^T
    float G_temp2 = 1.0f / (G_temp1[0][0] + theta_variance);//(W+CP'C^T)^-1
    float G1[2][1] = {0};
    mat_mul_const(P_CT[0], G_temp2, G1[0], 2, 1);//P'C^T(W+CP'C^T)^-1
    
    //傾斜角推定値算出: theta = theta'+G(y-Ctheta')
    float C_theta_theta[1][1] = {0};
    mat_mul(C_theta[0], theta_data_predict[0], C_theta_theta[0], 1, 2, 2, 1);//Ctheta'
    float delta_y = y - C_theta_theta[0][0];//y-Ctheta'
    float delta_theta[2][1] = {0};
    mat_mul_const(G1[0], delta_y, delta_theta[0], 2, 1);
    mat_add(theta_data_predict[0], delta_theta[0], theta_data[0], 2, 1);
           
    //共分散行列算出: P=(I-GC)P'
    float GC[2][2] = {0};
    float I2[2][2] = {{1,0},{0,1}};
    mat_mul(G1[0], C_theta[0], GC[0], 2, 1, 1, 2);//GC
    float I2_GC[2][2] = {0};
    mat_sub(I2[0], GC[0], I2_GC[0], 2, 2);//I-GC
    mat_mul(I2_GC[0], P_theta_predict[0], P_theta[0], 2, 2, 2, 2);//(I-GC)P'
      
    //次時刻の傾斜角の予測値算出: theta'=Atheta+Bu
    float A_theta_theta[2][1] = {0};
    float B_theta_dot[2][1] = {0};
    mat_mul(A_theta[0], theta_data[0], A_theta_theta[0], 2, 2, 2, 1);//Atheta
    mat_mul_const(B_theta[0], theta_dot_gyro, B_theta_dot[0], 2, 1);//Bu
    mat_add(A_theta_theta[0], B_theta_dot[0], theta_data_predict[0], 2, 1);//Atheta+Bu 
    
    //次時刻の共分散行列算出: P'=APA^T + BUB^T
    float AP[2][2] = {0};   
    float APAT[2][2] = {0};
    float tran_A_theta[2][2] = {0};
    mat_tran(A_theta[0], tran_A_theta[0], 2, 2);//A^T 
    mat_mul(A_theta[0], P_theta[0], AP[0], 2, 2, 2, 2);//AP
    mat_mul(AP[0], tran_A_theta[0], APAT[0], 2, 2, 2, 2);//APA^T
    float BBT[2][2];
    float tran_B_theta[1][2] = {0};
    mat_tran(B_theta[0], tran_B_theta[0], 2, 1);//B^T
    mat_mul(B_theta[0], tran_B_theta[0], BBT[0], 2, 1, 1, 2);//BB^T
    float BUBT[2][2] = {0};
    mat_mul_const(BBT[0], theta_dot_variance, BUBT[0], 2, 2);//BUB^T
    mat_add(APAT[0], BUBT[0], P_theta_predict[0], 2, 2);//APA^T+BUB^T

    //角速度
    theta_dot2 = theta_dot_gyro - theta_data[1][0];
		
		//usb_printf("Offset:%d,%d,%d,%d,%d,%d\r\n",accXoffset,accYoffset,accZoffset,gyroXoffset,gyroYoffset,gyroZoffset);
		
		//PID部分
		thetaP = theta_data[0][0] / 90.0;
		thetaI += thetaP;
		thetaD = theta_dot2 / 250.0;

//		power = thetaP * Kp + thetaI * Ki + thetaD * Kd;
//		power = fmaxf(-1.0, fminf(1.0, power));

//		int Vdata = ((float)(9 - 3) * fabs(power)) + 3;
//		usb_printf("P=%7.3f,I=%7.3f,D=%7.3f,power=%7.3f,Vdata=%7.3f\r\n",thetaP,thetaI,thetaD,power,Vdata); 
}


//カルマンフィルタの初期設定 初期姿勢は0°(直立)を想定
void ini_theta(){
  //センサオフセット算出
  offset_cal();
	 
  //センサばらつき取得 100回測って分散導出
  acc_init();
  gyro_init();

  //カルマンフィルタの初期設定 初期姿勢は0°(直立)を想定
  theta_data_predict[0][0] = 0;
  theta_data_predict[1][0] = theta_dot_mean;
  
  P_theta_predict[0][0] = 1;
  P_theta_predict[0][1] = 0;
  P_theta_predict[1][0] = 0;
  P_theta_predict[1][1] = theta_dot_variance;

  thetaI = 0.0;
}
