// 1/11 本番使ったやつ

#include "mbed.h"
#include "platform/mbed_thread.h"

#define BLINKING_RATE_MS_basis                                               5
#define BLINKING_RATE_MS_spe1                                                1300
#define BLINKING_RATE_MS_spe2                                                390
#define BLINKING_RATE_MS_spe3                                                1500
#define VCC (3.3)
#define b_last                                                               0.07// -> 2.86v

#define delta                                                                1

#define PPR                                                                  20
#define G                                                                    1/38.2
#define r                                                                    27.5

float integ_ave = 0.0;

class Counter_R {
public:
    Counter_R(PinName pin) : _interrupt(A6) {        
        _interrupt.rise(callback(this, &Counter_R::increment)); 
    }

    void increment() {
        _count_R++;
    }

    float read() {
        _count_pre_R = _count_R;
        _count_R = 0;
        return _count_pre_R;
    }

private:
    InterruptIn _interrupt;
    volatile float _count_pre_R;
    volatile float _count_R;
};

class Counter_L {
public:
    Counter_L(PinName pin) : _interrupt(A4) {        
        _interrupt.rise(callback(this, &Counter_L::increment)); 
    }

    void increment() {
        _count_L++;
    }

    float read() {
        _count_pre_L = _count_L;
        _count_L = 0;
        return _count_pre_L;
    }

private:
    InterruptIn _interrupt;
    volatile float _count_pre_L;
    volatile float _count_L;
};

Counter_R counter_R(D13);
Counter_L counter_L(D13);

DigitalOut Digt1_R(D5);
PwmOut mypwm_R(D9);

DigitalOut Digt1_L(D2);
PwmOut mypwm_L(D1);

AnalogIn L_Ain1(A1);
AnalogIn R_Ain2(A2);
AnalogIn S_Ain3(A0);

float max_min_control(float cont_val){

    if(cont_val>1.0){
        cont_val = 1.0;
    }else if(cont_val<0.0){
        cont_val = 0.1;
    }

    return cont_val;
}

float P_R = 0.7;//変更禁止
float P_L = 0.5;

float D_R = 0.005;
float D_L = 0.005;

float diff_R_pre = 0.0;
float diff_L_pre = 0.0;

float PID_LIGHT_R(float Senser_Value_R, float Target_Value_R){
    float diff_R = Senser_Value_R - Target_Value_R ;
    float integ_R = (diff_R - diff_R_pre) / 0.005;
    diff_R_pre = diff_R;

    return diff_R * P_R  +  integ_R * D_R;
}

float PID_LIGHT_L(float Senser_Value_L, float Target_Value_L){
    float diff_L = Senser_Value_L - Target_Value_L ;
    float integ_L = (diff_L - diff_L_pre) / 0.005;
    diff_L_pre = diff_L;

    return diff_L * P_L  + integ_L * D_L;
}

float P_R_PD = 0.7;
float D_R_PD = 2.7;
float diff_R_pre_PD = 0.0;

float PD(float volt_R, float volt_L ,float volt_S){
    float diff = volt_R - volt_L-0.05;
    float integ = (diff - diff_R_pre_PD);

    diff_R_pre_PD = diff;
    float x_PD = 3.3 - volt_S;
//    float duty = diff * P_R_PD + -x_PD*( (volt_L-volt_R)*10 )*0.18;
    float duty = diff * P_R_PD + integ * D_R_PD;

    return duty;

}

float v_R(){

    float omega_R = (2 * 3.14 * counter_R.read() * G) / (PPR * delta);
    float v_R     = r * omega_R;

    return v_R;
}

float v_L(){

    float omega_L = (2 * 3.14 * counter_L.read() * G) / (PPR * delta);
    float v_L     = r * omega_L;

    return v_L;
}

float Integration(float integ_ave){
    integ_ave += (v_R()*delta + v_L()*delta)/2;
    return integ_ave;
}

float norm_L_integ;
float perc_L_integ;
float volt_L_integ;

float norm_R_integ;
float perc_R_integ;
float volt_R_integ;

float integ(int special_time,float integ_ave_integ){

    if( (360 < integ_ave_integ) && (special_time==1) ){
        unsigned short int Time_first_curve = 0;
        float PID_Value_R;
        float PID_Value_L;
        float Power_R;
        float Power_L;
        unsigned short int count = 0;

        while( Time_first_curve < 500 ){
            norm_L_integ = L_Ain1.read();
            volt_L_integ= norm_L_integ * VCC;

            norm_R_integ = R_Ain2.read();
            volt_R_integ = norm_R_integ * VCC;

            if(volt_R_integ < volt_L_integ){
                mypwm_R.write(0.5-b_last);
                mypwm_L.write(0.2);
            }
            else if(volt_R_integ > volt_L_integ){
                mypwm_R.write(0.0);
                mypwm_L.write(0.5-b_last);
            }
            else {
                mypwm_R.write(0.44);
                mypwm_L.write(0.42);
            }
 
            if(count%100==0){
                integ_ave = Integration(integ_ave);
            }

            Time_first_curve++;
            count++;
            thread_sleep_for(5);
        }
        special_time++;
    }

    else if( (900<integ_ave_integ) && (special_time==2) ){
        unsigned short int Time = 0;
        float PID_Value_R;
        float PID_Value_L;
        float Power_R;
        float Power_L;
        unsigned short int count = 0;  
        
        while(Time < 200){
            norm_L_integ = L_Ain1.read();
            volt_L_integ= norm_L_integ * VCC;

            norm_R_integ = R_Ain2.read();
            volt_R_integ = norm_R_integ * VCC;

            PID_Value_R = PID_LIGHT_R(volt_R_integ , 2.6);
            PID_Value_L = PID_LIGHT_L(volt_L_integ , 2.6);

            Power_R = max_min_control(0.3-b_last + PID_Value_R - PID_Value_L);
            Power_L = max_min_control(0.3-b_last + PID_Value_L - PID_Value_R);
        
            mypwm_L.write(Power_R);
            mypwm_R.write(Power_L);
 
            if(count%100==0){
                integ_ave = Integration(integ_ave);
            }

            Time++;
            count++;
            thread_sleep_for(5);

        }
        
        
        //float muimi_count = counter_R.read();
            //  muimi_count = counter_L.read(); 
            int  muimi_count = 0;

        //Digt1_R = 1;                            
        while(muimi_count<170){
            norm_L_integ = L_Ain1.read();
            perc_L_integ = norm_L_integ * 100;
            mypwm_R.write(0);
            mypwm_L.write(0.5-b_last);
            muimi_count += 1;
            thread_sleep_for(10);

        } 
        while(perc_L_integ<90){
            norm_L_integ = L_Ain1.read();
            perc_L_integ = norm_L_integ * 100;
            mypwm_R.write(0.3);
            mypwm_L.write(0.0);
            thread_sleep_for(BLINKING_RATE_MS_basis);
        
            //thread_sleep_for(10);
        }

        perc_L_integ = 0.0;

        /*mypwm_R.write(0.2);
        mypwm_L.write(0);
        thread_sleep_for(30);*/

        special_time++;
        //Digt1_R = 0;

        muimi_count = counter_R.read();
        muimi_count = counter_L.read(); 


        count = 0;

        while( 1450 >= integ_ave ){
            norm_L_integ = L_Ain1.read();
            perc_L_integ= norm_L_integ * 100;

            norm_R_integ = R_Ain2.read();
            perc_R_integ = norm_R_integ * 100;
            
            if(perc_R_integ < perc_L_integ){
                mypwm_R.write(0.5-b_last);
                mypwm_L.write(0.0);
            }
            else if(perc_R_integ > perc_L_integ){
                mypwm_R.write(0.0);
                mypwm_L.write(0.5-b_last);
            }
            else {
                mypwm_R.write(0.44);
                mypwm_L.write(0.42);
            }

            if(count%100 == 0){
                integ_ave = Integration(integ_ave);
            }

            count++;
            thread_sleep_for(5);  

        }

        unsigned short int spe_count1 = 0;

        while(spe_count1<120){

            norm_L_integ = L_Ain1.read();
            volt_L_integ = norm_L_integ * VCC;

            norm_R_integ = R_Ain2.read();
            volt_R_integ = norm_R_integ * VCC;

            PID_Value_R = PID_LIGHT_R(volt_R_integ , 2.6 );
            PID_Value_L = PID_LIGHT_L(volt_L_integ , 2.6 );

            Power_R = max_min_control(0.35-b_last + PID_Value_R - PID_Value_L);
            Power_L = max_min_control(0.35-b_last + PID_Value_L - PID_Value_R);
        
            mypwm_L.write(Power_R);
            mypwm_R.write(Power_L);
            spe_count1++;
            thread_sleep_for(BLINKING_RATE_MS_basis);
        }
 
        mypwm_R.write(0.2-b_last); 
        mypwm_L.write(0.5-b_last);
        thread_sleep_for(BLINKING_RATE_MS_spe2);

        perc_L_integ = 0.0;

        while(perc_L_integ<90){
            norm_L_integ = L_Ain1.read();
            perc_L_integ = norm_L_integ * 100;
            mypwm_R.write(0.3);
            mypwm_L.write(0.2-b_last);
            thread_sleep_for(BLINKING_RATE_MS_basis);
        }

        while(1){

            norm_R_integ = R_Ain2.read();
            perc_R_integ = norm_R_integ * 100;
            
            if(perc_R_integ>90){
                Digt1_R = 1;
                mypwm_R.write(0.6-b_last);
                mypwm_L.write(0.6-b_last);
            }else if(perc_R_integ <= 90){
                Digt1_R = 0;
                mypwm_R.write(0.4-b_last);
                mypwm_L.write(0.15-b_last);
            }

            thread_sleep_for(10);
        }
    }
    return special_time;
}

int main()
{
    mypwm_R.period(0.0001f);
    Digt1_R = 0;

    mypwm_L.period(0.0001f);
    Digt1_L = 0;

    float norm_S;
    float perc_S;
    float volt_S;

    float norm_L;
    float perc_L;
    float volt_L;

    float norm_R;
    float perc_R;
    float volt_R;

    unsigned int count = 0;
    unsigned short int special_time = 1;

    float Power_R;
    float Power_L;

    float PD_Amount;
    float Defalt_Power = 0.5;

    while(1){

        norm_L = L_Ain1.read();
        perc_L = norm_L * 100;
        volt_L = norm_L * VCC;

        norm_R = R_Ain2.read();
        perc_R = norm_R * 100;
        volt_R = norm_R * VCC;

        norm_S = S_Ain3.read();
        perc_S = norm_S * 100;
        volt_S = norm_S * VCC;

        PD_Amount = PD(volt_R, volt_L, volt_S);

        Power_R = max_min_control( Defalt_Power - ((PD_Amount*volt_R) / (volt_R+volt_L)) );
        Power_L = max_min_control( Defalt_Power + ((PD_Amount*volt_L) / (volt_L+volt_R)) );

        mypwm_L.write(Power_L);
        mypwm_R.write(Power_R);

        if(count%100==0){
            integ_ave = Integration(integ_ave);
            special_time = integ(special_time, integ_ave);
        }

        count++;
        thread_sleep_for(BLINKING_RATE_MS_basis);
    }
}