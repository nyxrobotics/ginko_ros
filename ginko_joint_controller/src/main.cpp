
#include "main.h"

//main here
int main(int argc, char **argv) {
	// Init ROS node
	ros::init(argc, argv, "ginko_controller");
	GinkoController ginko_controller; //宣言のタイミングでginko_controllerの初期化が呼ばれる。
	ros::Rate loop_rate(LOOP_FREQUENCY);

    //結局このやり方以外ではCPU使用率が60%を超えるほど異常に消費する問題を抜けられない
    //おそらく、whileの中でスレッドを分けるを、スレッド分割・元に戻す、の作業を毎回やってしまうらしい。
	//とにかく、スレッドが分かれている処理と、そうでない処理を混在させることができなかった。single、barrier系も一切正常動作しない。
	//おそらくbarrier処理が走った時にCPU負荷が大きく上昇する。
	//kernel 4.15と4.13で試したが両方ダメ。
	//clock_gettimeのCPU負荷が以上に高いこととおそらく関係がある。同期を取るときに使っているのではないだろうか・・・
	#pragma omp parallel num_threads(4)
	{
		if(omp_get_thread_num() == 0){
			while (ros::ok()){
				ROS_INFO("thread:%d / %d / %d" , omp_get_thread_num(), omp_get_num_threads(),sysconf(_SC_NPROCESSORS_ONLN));
				ros::spinOnce();
				loop_rate.sleep();
				ROS_INFO("thread:end----");
			}
		} else if(omp_get_thread_num() == 1){
			while (ros::ok()){
				ROS_INFO("thread:%d / %d / %d" , omp_get_thread_num(), omp_get_num_threads(),sysconf(_SC_NPROCESSORS_ONLN));
				loop_rate.sleep();
			}
		} else if(omp_get_thread_num() == 2){
			while (ros::ok()){
				ROS_INFO("thread:%d / %d / %d" , omp_get_thread_num(), omp_get_num_threads(),sysconf(_SC_NPROCESSORS_ONLN));
				loop_rate.sleep();
			}
		} else if(omp_get_thread_num() == 3){
			while (ros::ok()){
				ROS_INFO("thread:%d / %d / %d" , omp_get_thread_num(), omp_get_num_threads(),sysconf(_SC_NPROCESSORS_ONLN));
				loop_rate.sleep();
			}
		}
	}

/*
	#pragma omp parallel num_threads(4)
	#pragma omp sections nowait
	{
		#pragma omp section
		{
			while (ros::ok()){
				#pragma omp critical
				ROS_INFO("thread:%d / %d / %d" , omp_get_thread_num(), omp_get_num_threads(),sysconf(_SC_NPROCESSORS_ONLN));
				ros::spinOnce();
				loop_rate.sleep();
				#pragma omp critical
				ROS_INFO("thread:end----");
			}
		}
		#pragma omp section
		{
			while (ros::ok()){
				#pragma omp critical
				ROS_INFO("thread:%d / %d / %d" , omp_get_thread_num(), omp_get_num_threads(),sysconf(_SC_NPROCESSORS_ONLN));
				loop_rate.sleep();
			}
		}
		#pragma omp section
		{
			while (ros::ok()){
				#pragma omp critical
				ROS_INFO("thread:%d / %d / %d" , omp_get_thread_num(), omp_get_num_threads(),sysconf(_SC_NPROCESSORS_ONLN));
				loop_rate.sleep();
			}
		}
		#pragma omp section
		{
			while (ros::ok()){
				#pragma omp critical
				ROS_INFO("thread:%d / %d / %d" , omp_get_thread_num(), omp_get_num_threads(),sysconf(_SC_NPROCESSORS_ONLN));
				loop_rate.sleep();
			}
		}
	}
	*/


/*
	while (ros::ok()) {
		#pragma omp parallel num_threads(4)
		#pragma omp for nowait
		for(int comnum=0;comnum<4;comnum++){
		//		while (ros::ok()) {
					//ginko_controller.control_loop();
						#pragma omp critical
						ROS_INFO("thread:%d / %d / %d" , omp_get_thread_num(), omp_get_num_threads(),sysconf(_SC_NPROCESSORS_ONLN));
					//#pragma omp barrier

		//		}
		}
		#pragma omp single
		{
			ros::spinOnce();
			loop_rate.sleep();
			ROS_INFO("thread:end----");
		}

	}
*/

/*
	#pragma omp parallel num_threads(4)
	{
		while (ros::ok()){
//			#pragma omp parallel num_threads(4)
//			#pragma omp sections nowait
//						#pragma omp parallel num_threads(4)
			#pragma omp sections nowait
			{
				#pragma omp section
				{
					ROS_INFO("thread:%d / %d / %d" , omp_get_thread_num(), omp_get_num_threads(),sysconf(_SC_NPROCESSORS_ONLN));
//					ros::spinOnce();
//					loop_rate.sleep();
//					ROS_INFO("thread:end----");
				}
				#pragma omp section
				{
					ROS_INFO("thread:%d / %d / %d" , omp_get_thread_num(), omp_get_num_threads(),sysconf(_SC_NPROCESSORS_ONLN));
				}
				#pragma omp section
				{
					ROS_INFO("thread:%d / %d / %d" , omp_get_thread_num(), omp_get_num_threads(),sysconf(_SC_NPROCESSORS_ONLN));
				}
				#pragma omp section
				{
					ROS_INFO("thread:%d / %d / %d" , omp_get_thread_num(), omp_get_num_threads(),sysconf(_SC_NPROCESSORS_ONLN));
				}
			}
			#pragma omp barrier
			#pragma omp single
			{
				ros::spinOnce();
				loop_rate.sleep();
				ROS_INFO("thread:end----");
			}
//			#pragma omp barrier
		}
	}
*/


	/*
	#pragma omp parallel num_threads(4) nowait
	{
		while (ros::ok()){
//			#pragma omp barrier
			if(omp_get_thread_num() == 0){
				ROS_INFO("thread:%d / %d / %d" , omp_get_thread_num(), omp_get_num_threads(),sysconf(_SC_NPROCESSORS_ONLN));
				ros::spinOnce();
				loop_rate.sleep();
				ROS_INFO("thread:end----");
			} else if(omp_get_thread_num() == 1){
				ROS_INFO("thread:%d / %d / %d" , omp_get_thread_num(), omp_get_num_threads(),sysconf(_SC_NPROCESSORS_ONLN));
//				ros::spinOnce();
//				loop_rate.sleep();
			} else if(omp_get_thread_num() == 2){
				ROS_INFO("thread:%d / %d / %d" , omp_get_thread_num(), omp_get_num_threads(),sysconf(_SC_NPROCESSORS_ONLN));
//				ros::spinOnce();
//				loop_rate.sleep();
			} else if(omp_get_thread_num() == 3){
				ROS_INFO("thread:%d / %d / %d" , omp_get_thread_num(), omp_get_num_threads(),sysconf(_SC_NPROCESSORS_ONLN));
//				ros::spinOnce();
//				loop_rate.sleep();
			}
//			#pragma omp single
//				loop_rate.sleep();
		#pragma omp barrier
//		#pragma omp single
//#pragma omp master
//			{
//				ros::spinOnce();
//				loop_rate.sleep();
//				ROS_INFO("thread:end----");
//			}

		}
	}
*/
	return 0;
}
