#include "ardrone/ardrone.h"

#define KEY_DOWN(key) (GetAsyncKeyState(key) & 0x8000)
#define KEY_PUSH(key) (GetAsyncKeyState(key) & 0x0001)

// --------------------------------------------------------------------------
// main(引数の数、引数リスト)
// メイン関数です
// 戻り値 正常終了:0 エラー:-1
// --------------------------------------------------------------------------
int main(int argc, char **argv)
{
    // AR.Droneクラス
    ARDrone ardrone;

    // 初期化
    if (!ardrone.open()) {
        printf("ARDroneの初期化に失敗しました\n");
        return -1;
    }

    // メインループ
    while (!GetAsyncKeyState(VK_ESCAPE)) {
        // AR.Droneの更新
        if (!ardrone.update()) break;

        // 画像の取得
        IplImage *image = ardrone.getImage();

        //// ナビゲーションデータの取得
        //double roll  = ardrone.getRoll();
        //double pitch = ardrone.getPitch();
        //double yaw   = ardrone.getYaw();
        //printf("ardrone.roll  = %3.2f [deg]\n", roll  * RAD_TO_DEG);
        //printf("ardrone.pitch = %3.2f [deg]\n", pitch * RAD_TO_DEG);
        //printf("ardrone.yaw   = %3.2f [deg]\n", yaw   * RAD_TO_DEG);

        // 高度
        double altitude = ardrone.getAltitude();
        printf("ardrone.altitude = %3.2f [m]\n", altitude);

        // 速度
        double vx, vy, vz;
        double velocity = ardrone.getVelocity(&vx, &vy, &vz);
        printf("ardrone.vx = %3.2f [m/s]\n", vx);
        printf("ardrone.vy = %3.2f [m/s]\n", vy);
        printf("ardrone.vz = %3.2f [m/s]\n", vz);

        // バッテリ残量
        int battery = ardrone.getBatteryPercentage();
        printf("ardrone.battery = %d [％] (残り約%d分)\n", battery, 12*battery/100);

        // 離陸・着陸
        if (KEY_PUSH(VK_SPACE)) {
            if (ardrone.onGround()) ardrone.takeoff();
            else                    ardrone.landing();
        }

        // 緊急停止
        if (KEY_PUSH(VK_RETURN)) ardrone.emergency();

        // AR.Droneが飛行状態
        if (!ardrone.onGround()) {
            // 速度指令
            double x = 0.0, y = 0.0, z = 0.0, r = 0.0;
            if (KEY_DOWN(VK_UP))    x =  0.5;
            if (KEY_DOWN(VK_DOWN))  x = -0.5;
            if (KEY_DOWN(VK_LEFT))  r =  0.5;
            if (KEY_DOWN(VK_RIGHT)) r = -0.5;
            if (KEY_DOWN('Q'))      z =  0.5;
            if (KEY_DOWN('A'))      z = -0.5;
            //ardrone.move3D(x, y, z, r);

			//ゲームパット
			JOYINFOEX JoyInfoEx;

			JoyInfoEx.dwSize = sizeof(JOYINFOEX);
			JoyInfoEx.dwFlags = JOY_RETURNALL; // 全ての情報を取得

			if(joyGetPosEx(0, &JoyInfoEx) == JOYERR_NOERROR){ // 成功

				int y_pad = -((int)JoyInfoEx.dwXpos - 0x7FFF)/32512.0*100.0;
				int x_pad = -((int)JoyInfoEx.dwYpos - 0x7FFF)/32512.0*100.0;
				int r_pad = -((int)JoyInfoEx.dwZpos - 0x7FFF)/32512.0*100.0;
				int z_pad = ((int)JoyInfoEx.dwRpos - 0x7FFF)/32512.0*100.0;

				printf("X = %d  ",x_pad);
				printf("Y = %d  ",y_pad);
				printf("Z = %d  ",z_pad);
				printf("R = %d\n",r_pad);

				x = 0.5 * x_pad / 100;
				y = 0.5 * y_pad / 100;
				z = 0.5 * z_pad / 100;
				r = 0.5 * r_pad / 100;

				if(JoyInfoEx.dwButtons & JOY_BUTTON1) printf("ボタン1");
				if(JoyInfoEx.dwButtons & JOY_BUTTON2) if (ardrone.onGround()) ardrone.takeoff();
				if(JoyInfoEx.dwButtons & JOY_BUTTON3) ardrone.landing();
				if(JoyInfoEx.dwButtons & JOY_BUTTON4) printf("ボタン4");
			}
		ardrone.move3D(x, y, z, r);
		}


        // カメラ切り替え
        static int mode = 0;
        if (KEY_PUSH('C')) {
            // AR.Drone 2.0
            if (ardrone.getVersion() == ARDRONE_VERSION_2) {
                if (mode == 0) mode = 1;
                else           mode = 0;
            }
            // AR.Drone 1.0
            else {
                if (mode == 0) mode = 2;
                else           mode = 0;
            }
            ardrone.setCamera(mode);
        }

        // 表示
        cvShowImage("camera", image);
        cvWaitKey(30);
    }

    // さようなら
    ardrone.close();

    return 0;
}

