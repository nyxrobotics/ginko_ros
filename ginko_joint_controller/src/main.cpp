
#include "main.h"

//main here
int main(int argc, char **argv) {
	// Init ROS node
	ros::init(argc, argv, "ginko_controller");
	GinkoController ginko_controller; //宣言のタイミングでginko_controllerのコンストラクタが呼ばれる。
	//loop_rateはポートごとに個別に呼ばないと周期がバグる
	//各ポートのタイミングは思ったより揃う。(100us以内くらい)
	ros::Rate loop_rate0(LOOP_FREQUENCY);
	ros::Rate loop_rate1(LOOP_FREQUENCY);
	ros::Rate loop_rate2(LOOP_FREQUENCY);
	ros::Rate loop_rate3(LOOP_FREQUENCY);

    //結局このやり方以外ではCPU使用率が60%を超えるほど異常に消費する問題を抜けられない
    //おそらく、whileの中でスレッドを分けるを、スレッド分割・元に戻す、の作業を毎回やってしまうらしい。
	//とにかく、スレッドが分かれている処理と、そうでない処理を混在させることができなかった。single、barrier系も一切正常動作しない。
	//おそらくbarrier処理が走った時にCPU負荷が大きく上昇する。
	//kernel 4.15と4.13で試したが両方ダメ。
	//clock_gettimeのCPU負荷が以上に高いこととおそらく関係がある。同期を取るときに使っているのではないだろうか・・・
	//sharedオプションの中に入れないとコピーを行おうとし、周期がバグる。
	#pragma omp parallel num_threads(4) shared(loop_rate0,loop_rate1,loop_rate2,loop_rate3,ginko_controller)
	{
		if(omp_get_thread_num() == 0){
			while (ros::ok()){
				ginko_controller.control_loop_com(0);
				//ROS_INFO("thread:%d / %d / %d" , omp_get_thread_num(), omp_get_num_threads(),sysconf(_SC_NPROCESSORS_ONLN));
				loop_rate0.sleep();
				ginko_controller.control_loop_main();
				//ROS_INFO("thread:end----");
			}
		} else if(omp_get_thread_num() == 1){
			while (ros::ok()){
				ginko_controller.control_loop_com(1);
				//ROS_INFO("thread:%d / %d / %d" , omp_get_thread_num(), omp_get_num_threads(),sysconf(_SC_NPROCESSORS_ONLN));
				ros::spinOnce();
				loop_rate1.sleep();
			}
		} else if(omp_get_thread_num() == 2){//ここだけサーボが一個多いので他で実行すれば十分な関数を入れないように注意
			while (ros::ok()){
				ginko_controller.control_loop_com(2);
				//ROS_INFO("thread:%d / %d / %d" , omp_get_thread_num(), omp_get_num_threads(),sysconf(_SC_NPROCESSORS_ONLN));
				loop_rate2.sleep();
			}
		} else if(omp_get_thread_num() == 3){
			while (ros::ok()){
				ginko_controller.control_loop_com(3);
				//ROS_INFO("thread:%d / %d / %d" , omp_get_thread_num(), omp_get_num_threads(),sysconf(_SC_NPROCESSORS_ONLN));
				loop_rate3.sleep();
			}
		}
	}


	return 0;
}
