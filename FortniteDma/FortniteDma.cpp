#include <iostream>
#include "Memory/Memory.h"
#include "UnrealContainers.hpp"
#include "Settings.hpp"
#include "Utils.hpp"
#include <thread>

#include "Weapons.hpp"

bool aimbot_in_action;
void aimbot() {
	double damp_coeff = 1; //increase this value to increase starting aggressiveness

	std::vector<MeshInfoContainer> nearest_mesh_info;
	nearest_mesh_info.reserve(10);

	double previous_pitch_error = 0;
	double previous_yaw_error = 0;
	double pitch_integral = 0;
	double yaw_integral = 0;
	//double kp = 0.5; //increase this value to increase aggressiveness
	double last_elapsed = 0;
	while (true) {
		double ki = 0.0001; //increase this value to increase aggressiveness
		double kd = 15.2; //increase this value to increase aggressiveness
		if (aimbot_in_action && settings::aimbot::enable) {
			if (cache::local_camera.fov < 70) {
				kd = 9.3;
			}
			try {
				// aimbot code
				// Measure performance
				GEN_CHRONO_VAR_WITH_NUM_SUFFIX(start, 0)
				cache::closest_mesh_info = MeshInfoContainer();

				nearest_mesh_info.clear();
				// Get the nearest 3 mesh info
				{
					// Lock mutex
					std::lock_guard lock(cache::mem_mutex);
					// Get the nearest 3 mesh info if 3 are available, otherwise only one
					for (int i = 0; i < 3 && i < cache::mesh_info_cache.size(); i++) {
						nearest_mesh_info.push_back(cache::mesh_info_cache[i]);
					}
				}

				if (nearest_mesh_info.empty()) {
					EXEC_ON_ARG(std::cout << "No mesh info available" << std::endl, false)
					continue;
				}

				//if (mem.GetKeyboard()->IsKeyDown(VK_LBUTTON)) {
				//	if (damp_coeff > 0.5) { //reduce this to increase damping power
				//		damp_coeff -= 0.001;
				//		// reduce this value to reduce damping effect
				//	}
				//}
				//else {
				//	damp_coeff = 1;
				//}


				// Check the distances of cached nearest mesh info
				cache::closest_distance = FLT_MAX;
				for (auto& i : nearest_mesh_info)
				{
					// Get real head bone
					FTransform head_bone = mem.Read<FTransform>(i.head_bone);
					// Get real component to world
					FTransform component_to_world = mem.Read<FTransform>(i.component_to_world);
					Vector2 screen_center = { (double)settings::screen_center_x, (double)settings::screen_center_y };
					Vector3 world_space = get_world_space_coords(head_bone, component_to_world);
					double world_space_distance = world_space.distance(cache::local_camera.location);
					Vector2 a_screen = project_world_to_screen(world_space);
					double distance = 0.999 * (a_screen - screen_center).x * (a_screen - screen_center).x + 0.999 * (a_screen - screen_center).y * (a_screen - screen_center).y + 0.001 * world_space_distance * world_space_distance;
					EXEC_ON_DEBUG(std::cout << "Distance: " << world_space_distance << std::endl);
					bool is_dying = mem.Read<BYTE>(mem.ReadChain(i.player_state, { (UINT)offsets::playerstate_to_ppawn }) + offsets::pawn_to_isdying) != 0;
					bool is_dbno = mem.Read<BYTE>(mem.ReadChain(i.player_state, { (UINT)offsets::playerstate_to_ppawn }) + offsets::pawn_to_isdbno) >> 4 & 1;

					if (distance < cache::closest_distance && !is_dying && !is_dbno) {
						cache::closest_distance = distance;
						i.cached_head_bone = head_bone;
						i.cached_component_to_world = component_to_world;
						cache::closest_mesh_info = i;
					}
				}

				if (cache::closest_mesh_info.player_state == 0) {
					continue;
				}

				// Get the head bone and component to world of the closest mesh
				FTransform head_bone = cache::closest_mesh_info.cached_head_bone;
				FTransform component_to_world = cache::closest_mesh_info.cached_component_to_world;
				// Get the world space coordinates of the head bone
				Vector3 head_world = get_world_space_coords(head_bone, component_to_world);

				Vector3 velocity = mem.Read<Vector3>(cache::closest_mesh_info.root_component + offsets::component_to_velocity);
				//std::cout << "Velocity: " << velocity.x << " " << velocity.y << " " << velocity.z << std::endl;
				float dist = head_world.distance(cache::local_camera.location);
				Vector3 head_world_adjusted = predict_location(head_world, velocity, projectile_stats::projectile_speed, projectile_stats::gravity_scale, dist);

				// Project the world space coordinates to screen
				Vector2 head_screen = project_world_to_screen(head_world_adjusted);

				// Print the head screen coordinates
				EXEC_ON_DEBUG(std::cout << "Head Screen X: " << head_screen.x << " Y: " << head_screen.y << std::endl);

				// Get the center of the screen
				Vector2 screen_center = { (double)settings::screen_center_x, (double)settings::screen_center_y };

				// Calculate the difference between the head screen and the screen center
				Vector2 diff = head_screen - screen_center;

				// Adjust player controller rotation until diff is below a certain threshold

				// Calculate vector length squared
				double vector_length_squared = diff.x * diff.x + diff.y * diff.y;

				if (vector_length_squared > 2 && abs(diff.x) < settings::screen_center_x + 300 && abs(diff.y) < settings::screen_center_y + 300) {
					GEN_CHRONO_VAR_WITH_NUM_SUFFIX(start, 1)
					pitch_integral += -diff.y / settings::screen_center_x;
					yaw_integral += diff.x / settings::screen_center_x;

					double derivative_pitch = -diff.y / settings::screen_center_x - previous_pitch_error;
					double derivative_yaw = diff.x / settings::screen_center_x - previous_yaw_error;

					// Get the player controller rotation
					uintptr_t rotation_input = cache::player_controller + offsets::playercontroller_to_rotationinput;
					uintptr_t rotation_input2 = cache::player_controller + offsets::playercontroller_to_rotationinput2;
					//double smoothness_y = 20.0;
					//double smoothness_x = 20.0;

					// Print the difference
					EXEC_ON_DEBUG(std::cout << "Diff X: " << diff.x << " Y: " << diff.y << std::endl);
					/*if (abs(diff.y) < 10) {
						smoothness_y = 30.0;
					}
					if (abs(diff.y) < 5) {
						smoothness_y = 20.0;
					}
					if (abs(diff.x) < 10) {
						smoothness_x = 30.0;
					}
					if (abs(diff.x) < 5) {
						smoothness_x = 20.0;
					}*/

					// Set pitch rotation
					if (abs(diff.y) > 2) {
						if (cache::local_camera.fov < 39) {
							ki = 0;
							kd = 9.0f;
							mem.Write<double>(rotation_input, std::clamp(-diff.y / settings::screen_center_x * 7.0 + pitch_integral * ki + kd * derivative_pitch, -0.2, 0.2));
							EXEC_ON_DEBUG(std::cout << "Pitch: " << -diff.y / settings::screen_center_x * 7.0 + pitch_integral * ki + kd * derivative_pitch << std::endl);
						}
						else {
							mem.Write<double>(rotation_input, -diff.y / settings::screen_center_x * 13.1 + pitch_integral * ki + kd * derivative_pitch);
							EXEC_ON_DEBUG(std::cout << "Pitch: " << -diff.y / settings::screen_center_x * 13.1 + pitch_integral * ki + kd * derivative_pitch << std::endl);
						}
					}
					else {
						pitch_integral = 0;
					}

					// Set yaw rotation
					if (abs(diff.x) > 2) {
						if (cache::local_camera.fov < 39) {
							ki = 0;
							kd = 9.0f;
							mem.Write<double>(rotation_input + 0x8, std::clamp(diff.x / settings::screen_center_x * 7.0 + yaw_integral * ki + kd * derivative_yaw, -0.2, 0.2));
							EXEC_ON_DEBUG(std::cout << "Yaw: " << diff.x / settings::screen_center_x * 7.0 + yaw_integral * ki + kd * derivative_yaw << std::endl);
						}
						else {
							mem.Write<double>(rotation_input + 0x8, diff.x / settings::screen_center_x * 13.1 + yaw_integral * ki + kd * derivative_yaw);
							EXEC_ON_DEBUG(std::cout << "Yaw: " << diff.x / settings::screen_center_x * 13.1 + yaw_integral * ki + kd * derivative_yaw << std::endl);
						}
					}
					else {
						yaw_integral = 0;
					}
					previous_pitch_error = -diff.y / settings::screen_center_x;
					previous_yaw_error = diff.x / settings::screen_center_x;
					GEN_CHRONO_VAR_WITH_NUM_SUFFIX(end, 1)
					GET_CHRONO_ELAPSED(elapsed1, start1, end1)
					if (elapsed1.count() - last_elapsed > 0.01) {
						EXEC_ON_ARG(std::cout << "bad performance on MEM WRITE " << elapsed1.count() << std::endl, true)
						last_elapsed = elapsed1.count();
					}
				}

				// Print performance
				GEN_CHRONO_VAR_WITH_NUM_SUFFIX(end, 0)
				GET_CHRONO_ELAPSED(elapsed, start0, end0)
				if (elapsed.count() > 0.1) {
					EXEC_ON_ARG(std::cout << "bad performance " << elapsed.count() << std::endl, true)
				}
				EXEC_ON_DEBUG(std::cout << "Aimbot performance: " << elapsed.count() << "s" << std::endl);
				//Sleep(2);
			}
			catch (std::exception& e) {
				std::cout << e.what() << std::endl;
			}
		}
		else {
			previous_pitch_error = 0;
			previous_yaw_error = 0;
			pitch_integral = 0;
			yaw_integral = 0;
		}
		//Sleep(1);
	}
}


void aimbot_activation_thread() {
	while (true) {
		if (mem.GetKeyboard()->IsKeyDown(VK_RBUTTON)) {
			aimbot_in_action = true;
		}
		else {
			aimbot_in_action = false;
		}
		Sleep(100);
	}
}

void aimbot_toggle_check_thread() {
	while (true) {
		if (mem.GetKeyboard()->IsKeyDown(VK_NUMPAD0)) {
			if (settings::aimbot::enable) {
				std::cout << "Aimbot disabled" << std::endl;
			}
			else {
				std::cout << "Aimbot enabled" << std::endl;
			}
			settings::aimbot::enable = !settings::aimbot::enable;
		}
		Sleep(200);
	}
}

void print_weapon_name_thread()
{
	while (true)
	{
		auto weapon_name = get_weapon_name();
		if (weapon_name == nullptr || wcslen(weapon_name.get()) == 0)
		{
			// std::wcout << "Failed to get weapon name" << std::endl;
			Sleep(1500);
			continue;
		}
		EXEC_ON_ARG(std::wcout << "Weapon name: " << weapon_name << std::endl, false)
		// Get weapon projectile stats from weapon name
		auto weapon_stats = weapon_projectile_map.find(weapon_name.get());
		if (weapon_stats != weapon_projectile_map.end())
		{
			std::wcout << "Weapon projectile stats found" << std::endl;
			projectile_stats::projectile_speed = weapon_stats->second.speed;
			projectile_stats::gravity_scale = weapon_stats->second.gravity_scale;
			// std::cout << "Projectile speed: " << projectile_stats::projectile_speed << " Gravity scale: " << projectile_stats::gravity_scale << std::endl;
		}
		else
		{
			// std::wcout << "Weapon projectile stats not found" << std::endl;
			projectile_stats::projectile_speed = 80000.0f;
			projectile_stats::gravity_scale = 0.0f;

		}
		Sleep(1500);
	}
}


int main()
{

	/*if (!mem.Init("testproj.exe", true, true))
	{
		std::cout << "Failed to initialize memory class\n";
		return 1;
	}

	if (!mem.FixCr3())
		std::cout << "Failed to fix CR3" << std::endl;
	else
		std::cout << "CR3 fixed" << std::endl;

	uintptr_t base_address1 = mem.GetBaseDaddy("testproj.exe");

	if (!base_address1)
	{
		std::cout << "Failed to get base address\n";
		return 1;
	}

	uintptr_t uworld = mem.ReadChain(base_address1 + 0x0EABDEE8, { 0x00 });
	uintptr_t local_player_controller = mem.ReadChain(uworld, { 0x1B8, 0x38, 0x00, 0x30 });
	uintptr_t yaw_input = local_player_controller + 0x560;
	while (true) {
		mem.Write<double>(yaw_input, 1.0);
		Sleep(1000 / 250);
	}*/


	if (!mem.Init("FortniteClient-Win64-Shipping.exe", true, true))
	{
		std::cout << "Failed to initialize memory class\n";
		return 1;
	}

	Sleep(2000);

	uintptr_t base_address = mem.GetBaseDaddy("FortniteClient-Win64-Shipping.exe");
	if (!base_address)
	{
		std::cout << "Failed to get base address\n";
		return 1;
	}

	if (!mem.GetKeyboard()->InitKeyboard())
	{
		std::cout << "Failed to initialize keyboard hotkeys through kernel." << std::endl;
		return 1;
	}

	cache::uworld = mem.ReadChain(base_address + offsets::uworld, { 0x00 });

	uintptr_t local_player = mem.ReadChain(base_address + offsets::uworld, { 0x00, 0x1D8, 0x38, 0x00 });


	if (!local_player)
	{
		std::cout << "Failed to get local player\n";
		return 1;
	}
	cache::player_controller = mem.ReadChain(local_player, { 0x30 });
	// Print the player controller address
	std::cout << "Player controller address: " << std::hex << cache::player_controller << std::endl;
	// Restore decimal output
	std::cout << std::dec;

	cache::local_pawn = mem.ReadChain(cache::player_controller, { (UINT)offsets::playercontroller_to_pawn });
	// Print the player pawn address
	std::cout << "Player pawn address: " << std::hex << cache::local_pawn << std::endl;
	// Restore decimal output
	std::cout << std::dec;

	cache::my_team_id = mem.Read<BYTE>(mem.ReadChain(cache::player_controller, { (UINT)offsets::playercontroller_to_playerstate }) + offsets::playerstate_to_teamid);

	// Start a new thread to get all player meshes every 20 seconds
	std::thread get_all_player_meshes_thread(get_all_player_meshes, 500);
	//get_all_player_meshes_thread.detach();

	// Start aimbot toggle check thread
	std::thread aimbot_toggle_check_thread_run(aimbot_toggle_check_thread);
	//aimbot_toggle_check_thread_run.detach();

	// Start aimbot activation thread
	std::thread aimbot_activation_thread_run(aimbot_activation_thread);
	//aimbot_activation_thread_run.detach();

	// Start weapon name print thread
	std::thread print_weapon_name_thread_run(print_weapon_name_thread);

	// Create aimbot thread
	std::thread aimbot_thread(aimbot);
	aimbot_thread.join();

	//uintptr_t player_controller = mem.ReadChain(local_player, { 0x30 });
	//if (!player_controller)
	//{
	//	std::cout << "Failed to get player controller\n";
	//	return 1;
	//}

	//uintptr_t player_pawn = mem.ReadChain(player_controller, { 0x338 });
	//// Print the player pawn address
	//std::cout << "Player pawn address: " << std::hex << player_pawn << std::endl;
	//// Restore decimal output
	//std::cout << std::dec;
	//if (!player_pawn)
	//{
	//	std::cout << "Failed to get player pawn\n";
	//	return 1;
	//}

	//uintptr_t player_mesh_component = mem.ReadChain(player_pawn, { 0x318 });
	//if (!player_mesh_component)
	//{
	//	std::cout << "Failed to get player mesh component\n";
	//	return 1;
	//}

	//std::cout << "Player mesh component address: " << std::hex << player_mesh_component << std::endl;
	//std::cout << std::dec;

	///*uintptr_t player_mesh = mem.ReadChain(player_root, { 0x280 });
	//if (!player_mesh)
	//{
	//	std::cout << "Failed to get player mesh\n";
	//	return 1;
	//}*/


	//// Get player bone array cache
	//uintptr_t player_bone_array_cache = mem.ReadChain(player_mesh_component, { 0x5f8 });
	//// Print the bone array cache address
	//std::cout << "Bone array cache address: " << std::hex << player_bone_array_cache << std::endl;
	//// Restore decimal output
	//std::cout << std::dec;

	//uintptr_t player_bone_array_final = mem.Read<uintptr_t>(player_mesh_component + 0x5b0);
	//// Print the bone array address
	//std::cout << "Bone array address: " << std::hex << player_bone_array_final << std::endl;
	//// Restore decimal output
	//std::cout << std::dec;

	//while (true)
	//{

	//	/*
	//	* This iterates all the bones of the player and prints their positions
	//	for (unsigned long long i = 0; i < 100; i++)
	//	{
	//		uintptr_t bone = player_bone_array + i * 0x60;
	//		if (!bone)
	//		{
	//			std::cout << "Failed to get bone\n";
	//			return 1;
	//		}

	//		float bone_pos[3] = { };
	//		mem.Read(bone + 0x20, bone_pos, sizeof(bone_pos));

	//		std::cout << "Bone " << i << " X: " << bone_pos[0] << " Y: " << bone_pos[1] << " Z: " << bone_pos[2] << std::endl;

	//	}*/

	//	// Get the bone of the head and print its position
	//	uintptr_t head_bone = player_bone_array_final + 0x60 * 110;

	//	FTransform head_transform = mem.Read<FTransform>(head_bone);
	//	std::cout << "Head X: " << head_transform.translation.x << " Y: " << head_transform.translation.y << " Z: " << head_transform.translation.z << std::endl;

	//	// Convert to world space
	//	FTransform component_to_world = mem.Read<FTransform>(player_mesh_component + 0x1C0);
	//	D3DMATRIX bone_matrix = matrix_multiplication(head_transform.to_matrix_with_scale(), component_to_world.to_matrix_with_scale());
	//	
	//	Vector3 head_world = { bone_matrix._41, bone_matrix._42, bone_matrix._43 };

	//	std::cout << "Head World X: " << head_world.x << " Y: " << head_world.y << " Z: " << head_world.z << std::endl;
	//	// Project world to screen
	//	Vector2 head_screen = project_world_to_screen(head_world);
	//	//Print camera location
	//	//std::cout << "Camera Rotation X: " << cache::local_camera.location.x << " Y: " << cache::local_camera.location.y << " Z: " << cache::local_camera.location.z << "fov: " << cache::local_camera.fov << std::endl;
	//	std::cout << "Head Screen X: " << head_screen.x << " Y: " << head_screen.y << std::endl;

	//	Sleep(1000 / 60);

	//}


	return 0;
}
