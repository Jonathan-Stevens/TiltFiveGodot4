#pragma once

#include <TiltFiveNative.h>
#include <chrono>
#include <vector>
#include <mutex>
#include <thread>

using namespace std::chrono_literals;

namespace T5Integration {

using GameboardList = std::vector<T5_GameboardPose>;

class GameboardService {
public:
	bool start(T5_Glasses handle);
	void stop();
	bool is_running();

	void get_gameboard_data(GameboardList& list);

	T5_Result get_last_error();

private:
	void monitor_gameboards(std::stop_token s_token);

	T5_Glasses _glasses_handle;
	GameboardList _gameboard_list;

	std::jthread _thread;
	std::mutex _list_access;
	std::atomic_bool _running;

	std::chrono::milliseconds _poll_rate_for_retry = 20ms;
	std::chrono::milliseconds _poll_rate_for_monitoring = 10ms;

	T5_Result _last_gameboard_error;

	static const uint16_t MAX_SUPPORTED_GAMEBOARDS = 1;
};

}