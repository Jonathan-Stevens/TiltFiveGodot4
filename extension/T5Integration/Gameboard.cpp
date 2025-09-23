#include <Gameboard.h>

namespace T5Integration {

bool GameboardService::start(T5_Glasses handle) {
	_glasses_handle = handle;
	_gameboard_list.reserve(MAX_SUPPORTED_GAMEBOARDS);
	_last_gameboard_error = T5_SUCCESS;
	_running = true;
	_thread = std::jthread([this](std::stop_token s_token) { monitor_gameboards(s_token); });
	return _running;
}

void GameboardService::stop() {
	_thread.get_stop_source().request_stop();
	if (_thread.joinable())
		_thread.join();
}

bool GameboardService::is_running() {
	return _running;
}

void GameboardService::get_gameboard_data(GameboardList& list) {
	std::lock_guard lock(_list_access);
	list.clear();
	for (auto pose : _gameboard_list) {
		list.push_back(pose);
	}
}

T5_Result GameboardService::get_last_error() {
	auto tmp = _last_gameboard_error;
	_last_gameboard_error = T5_SUCCESS;
	return tmp;
}

void GameboardService::monitor_gameboards(std::stop_token s_token) {
	while (!s_token.stop_requested()) {
		T5_GameboardPose obtained_gameboard_poses[GameboardService::MAX_SUPPORTED_GAMEBOARDS];
		uint16_t gameboard_pose_count = MAX_SUPPORTED_GAMEBOARDS;
		T5_Result result = t5GetGameboardPoses(_glasses_handle, &obtained_gameboard_poses[0], &gameboard_pose_count);
		if (result == T5_ERROR_TRY_AGAIN) {
			// When we see this error, we are not tracking the primary gameboard, and consequently can't track other gameboards.
			// This can commonly happen if the user looks away.
			std::lock_guard lock(_list_access);
			_gameboard_list.clear();
			continue;
		}
		else if (result != T5_SUCCESS) {
			_last_gameboard_error = result;
			std::this_thread::sleep_for(_poll_rate_for_retry);
		}
		// Successfully obtained gameboard poses; update the _gameboard_list
		else {

			{
				std::lock_guard lock(_list_access);
				_gameboard_list.clear();
				for (int i = 0; i < gameboard_pose_count; ++i) {
					_gameboard_list.push_back(obtained_gameboard_poses[i]);
				}
			}
			std::this_thread::sleep_for(_poll_rate_for_monitoring);
		}
	}
}

}