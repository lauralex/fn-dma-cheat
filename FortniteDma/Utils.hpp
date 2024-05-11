#pragma once
#include <d3d9.h>
#include <vector>
#include "Settings.hpp"
#include <mutex>
#include <unordered_map>
#pragma comment(lib, "d3d9.lib")
#define M_PI 3.14159265358979323846264338327950288419716939937510
#define DEBUG false
#define EXEC_ON_DEBUG(x) if (DEBUG) { x; }
#define EXEC_ON_ARG(x, y) if (y) { x; }
#define GEN_CHRONO_VAR_WITH_NUM_SUFFIX(x, y) auto x##y = std::chrono::high_resolution_clock::now();
#define GET_CHRONO_ELAPSED(x, y, z) std::chrono::duration<double> x = y - z;


class Vector2
{
public:
	Vector2() : x(0.f), y(0.f) {}
	Vector2(const double _x, const double _y) : x(_x), y(_y) {}

	Vector2 operator-(Vector2 v) const { return { x - v.x, y - v.y }; }
	Vector2 operator+(Vector2 v) const { return { x + v.x, y + v.y }; }
	double x, y;
};

class Vector3
{
public:
	Vector3() : x(0.f), y(0.f), z(0.f) {}
	Vector3(const double _x, const double _y, const double _z) : x(_x), y(_y), z(_z) {}
	double x, y, z;
	double dot(Vector3 v) const { return x * v.x + y * v.y + z * v.z; }
	double distance(Vector3 v) const { return double(sqrtf(powf(v.x - x, 2.0) + powf(v.y - y, 2.0) + powf(v.z - z, 2.0))); }
	Vector3 operator-(Vector3 v) const { return { x - v.x, y - v.y, z - v.z }; }
};

struct FQuat { double x, y, z, w; };
struct FTransform
{
	FQuat rot;
	Vector3 translation;
	UINT8 pad[0x8];
	Vector3 scale;
	UINT8 pad1[0x8];

	[[nodiscard]] D3DMATRIX to_matrix_with_scale() const
	{
		D3DMATRIX m{};
		m._41 = translation.x;
		m._42 = translation.y;
		m._43 = translation.z;
		double x2 = rot.x + rot.x;
		double y2 = rot.y + rot.y;
		double z2 = rot.z + rot.z;
		double xx2 = rot.x * x2;
		double yy2 = rot.y * y2;
		double zz2 = rot.z * z2;
		m._11 = (1.0f - (yy2 + zz2)) * scale.x;
		m._22 = (1.0f - (xx2 + zz2)) * scale.y;
		m._33 = (1.0f - (xx2 + yy2)) * scale.z;
		double yz2 = rot.y * z2;
		double wx2 = rot.w * x2;
		m._32 = (yz2 - wx2) * scale.z;
		m._23 = (yz2 + wx2) * scale.y;
		double xy2 = rot.x * y2;
		double wz2 = rot.w * z2;
		m._21 = (xy2 - wz2) * scale.y;
		m._12 = (xy2 + wz2) * scale.x;
		double xz2 = rot.x * z2;
		double wy2 = rot.w * y2;
		m._31 = (xz2 + wy2) * scale.z;
		m._13 = (xz2 - wy2) * scale.x;
		m._14 = 0.0f;
		m._24 = 0.0f;
		m._34 = 0.0f;
		m._44 = 1.0f;
		return m;
	}
};

inline D3DMATRIX matrix_multiplication(const D3DMATRIX& pm1, const D3DMATRIX& pm2)
{
	D3DMATRIX pout{};
	pout._11 = pm1._11 * pm2._11 + pm1._12 * pm2._21 + pm1._13 * pm2._31 + pm1._14 * pm2._41;
	pout._12 = pm1._11 * pm2._12 + pm1._12 * pm2._22 + pm1._13 * pm2._32 + pm1._14 * pm2._42;
	pout._13 = pm1._11 * pm2._13 + pm1._12 * pm2._23 + pm1._13 * pm2._33 + pm1._14 * pm2._43;
	pout._14 = pm1._11 * pm2._14 + pm1._12 * pm2._24 + pm1._13 * pm2._34 + pm1._14 * pm2._44;
	pout._21 = pm1._21 * pm2._11 + pm1._22 * pm2._21 + pm1._23 * pm2._31 + pm1._24 * pm2._41;
	pout._22 = pm1._21 * pm2._12 + pm1._22 * pm2._22 + pm1._23 * pm2._32 + pm1._24 * pm2._42;
	pout._23 = pm1._21 * pm2._13 + pm1._22 * pm2._23 + pm1._23 * pm2._33 + pm1._24 * pm2._43;
	pout._24 = pm1._21 * pm2._14 + pm1._22 * pm2._24 + pm1._23 * pm2._34 + pm1._24 * pm2._44;
	pout._31 = pm1._31 * pm2._11 + pm1._32 * pm2._21 + pm1._33 * pm2._31 + pm1._34 * pm2._41;
	pout._32 = pm1._31 * pm2._12 + pm1._32 * pm2._22 + pm1._33 * pm2._32 + pm1._34 * pm2._42;
	pout._33 = pm1._31 * pm2._13 + pm1._32 * pm2._23 + pm1._33 * pm2._33 + pm1._34 * pm2._43;
	pout._34 = pm1._31 * pm2._14 + pm1._32 * pm2._24 + pm1._33 * pm2._34 + pm1._34 * pm2._44;
	pout._41 = pm1._41 * pm2._11 + pm1._42 * pm2._21 + pm1._43 * pm2._31 + pm1._44 * pm2._41;
	pout._42 = pm1._41 * pm2._12 + pm1._42 * pm2._22 + pm1._43 * pm2._32 + pm1._44 * pm2._42;
	pout._43 = pm1._41 * pm2._13 + pm1._42 * pm2._23 + pm1._43 * pm2._33 + pm1._44 * pm2._43;
	pout._44 = pm1._41 * pm2._14 + pm1._42 * pm2._24 + pm1._43 * pm2._34 + pm1._44 * pm2._44;
	return pout;
}

inline D3DMATRIX to_matrix(const Vector3& rot, const Vector3& origin = Vector3(0, 0, 0))
{
	double radpitch = (rot.x * M_PI / 180);
	double radyaw = (rot.y * M_PI / 180);
	double radroll = (rot.z * M_PI / 180);
	double sp = sinf(radpitch);
	double cp = cosf(radpitch);
	double sy = sinf(radyaw);
	double cy = cosf(radyaw);
	double sr = sinf(radroll);
	double cr = cosf(radroll);
	D3DMATRIX matrix{};
	matrix.m[0][0] = cp * cy;
	matrix.m[0][1] = cp * sy;
	matrix.m[0][2] = sp;
	matrix.m[0][3] = 0.f;
	matrix.m[1][0] = sr * sp * cy - cr * sy;
	matrix.m[1][1] = sr * sp * sy + cr * cy;
	matrix.m[1][2] = -sr * cp;
	matrix.m[1][3] = 0.f;
	matrix.m[2][0] = -(cr * sp * cy + sr * sy);
	matrix.m[2][1] = cy * sr - cr * sp * sy;
	matrix.m[2][2] = cr * cp;
	matrix.m[2][3] = 0.f;
	matrix.m[3][0] = origin.x;
	matrix.m[3][1] = origin.y;
	matrix.m[3][2] = origin.z;
	matrix.m[3][3] = 1.f;
	return matrix;
}

struct Camera
{
	Vector3 location;
	Vector3 rotation;
	float fov;
};

struct FRotator
{
	double pitch;
	double yaw;
	double roll;
};

struct FNRot
{
	double a;
	char pad_0008[24];
	double b;
	char pad_0028[424];
	double c;
};

struct MeshInfoContainer
{
	uintptr_t player_state{0};
	uintptr_t head_bone;
	FTransform cached_head_bone;
	uintptr_t component_to_world;
	FTransform cached_component_to_world;
	uintptr_t root_component;
	uintptr_t player_pawn;
};

inline bool is_valid(const uintptr_t address)
{
	if (address <= 0x400000 || address == 0xCCCCCCCCCCCCCCCC || address == 0 || address >
		0x7FFFFFFFFFFFFFFF)
	{
		return false;
	}

	return true;
}


namespace offsets
{
	inline int uworld = 0x126cf528;
	inline int uworld_to_pgamestate = 0x160;
	inline int gamestate_to_tpplayerstate_array = 0x2a8;
	inline int playerstate_to_ppawn = 0x308;
	inline int playerstate_to_isabot = 0x29a;
	inline int playerstate_to_teamid = 0x1211;
	inline int pawn_to_pmeshcomponent = 0x318;
	inline int pawn_to_controller = 0x2c8;
	inline int meshcomponent_to_bonearray = 0x5b0;
	inline int meshcomponent_to_componenttoworld = 0x1c0;
	inline int playercontroller_to_rotationinput = 0x520;
	inline int playercontroller_to_rotationinput2 = 0x810;
	inline int playercontroller_to_playerstate = 0x298;
	inline int playercontroller_to_pawn = 0x338;
	inline int pawn_to_isdying = 0x758;
	inline int pawn_to_isdbno = 0x982;
	inline int pawn_to_isattacking = 0x6987;
	inline int pawn_to_currentweapon = 0xa68;
	inline int pawn_to_rootcomponent = 0x198;
	inline int weapon_to_weapondata = 0x500;
	inline int weapondata_to_weaponname = 0x40;
	inline int weaponname_to_buf = 0x28;
	inline int component_to_velocity = 0x168;
}

namespace cache
{
	inline uintptr_t uworld;
	inline uintptr_t game_instance;
	inline uintptr_t local_players;
	inline uintptr_t player_controller;
	inline uintptr_t local_pawn;
	inline uintptr_t root_component;
	inline uintptr_t player_state;
	inline BYTE my_team_id;
	inline uintptr_t game_state;
	inline uintptr_t player_array;
	inline int player_count;
	inline double closest_distance;
	inline uintptr_t closest_mesh;
	inline MeshInfoContainer closest_mesh_info;
	inline Camera local_camera;
	inline std::vector<MeshInfoContainer> mesh_info;
	// Hashmap that maps player state to mesh info
	inline std::unordered_map<uintptr_t, MeshInfoContainer> mesh_info_map;
	// mesh_info cache to be used by aimbot thread
	inline std::vector<MeshInfoContainer> mesh_info_cache;
	inline std::vector<MeshInfoContainer> mesh_info_cache_copy;
	// Mutex for DMA memory read between threads
	inline std::mutex mem_mutex;
	inline VMMDLL_SCATTER_HANDLE camera_view_scatter_handle;
}

inline Camera get_view_point()
{
	Camera view_point{};
	uintptr_t location_pointer;
	uintptr_t rotation_pointer;
	mem.AddScatterReadRequest(cache::camera_view_scatter_handle, cache::uworld + 0x110, &location_pointer, sizeof(uintptr_t));
	mem.AddScatterReadRequest(cache::camera_view_scatter_handle, cache::uworld + 0x120, &rotation_pointer, sizeof(uintptr_t));
	mem.ExecuteReadScatter(cache::camera_view_scatter_handle);

	FNRot fnrot{};
	mem.AddScatterReadRequest(cache::camera_view_scatter_handle, rotation_pointer, &fnrot.a, sizeof(double));
	mem.AddScatterReadRequest(cache::camera_view_scatter_handle, rotation_pointer + 0x20, &fnrot.b, sizeof(double));
	mem.AddScatterReadRequest(cache::camera_view_scatter_handle, rotation_pointer + 0x1D0, &fnrot.c, sizeof(double));
	mem.ExecuteReadScatter(cache::camera_view_scatter_handle);

	float fov;
	mem.AddScatterReadRequest(cache::camera_view_scatter_handle, location_pointer, &view_point.location, sizeof(Vector3));
	mem.AddScatterReadRequest(cache::camera_view_scatter_handle, cache::player_controller + 0x394, &fov, sizeof(float));
	mem.ExecuteReadScatter(cache::camera_view_scatter_handle);
	view_point.rotation.x = asin(fnrot.c) * (180.0 / M_PI);
	view_point.rotation.y = ((atan2(fnrot.a * -1, fnrot.b) * (180.0 / M_PI)) * -1) * -1;
	view_point.fov = fov * 90.f;
	//EXEC_ON_DEBUG(std::cout << "Location: " << view_point.location.x << " " << view_point.location.y << " " << view_point.location.z << " " << view_point.fov << std::endl);

	return view_point;
}

inline Vector2 project_world_to_screen(const Vector3& world_location)
{
	cache::local_camera = get_view_point();
	D3DMATRIX temp_matrix = to_matrix(cache::local_camera.rotation);
	Vector3 vaxisx = Vector3(temp_matrix.m[0][0], temp_matrix.m[0][1], temp_matrix.m[0][2]);
	Vector3 vaxisy = Vector3(temp_matrix.m[1][0], temp_matrix.m[1][1], temp_matrix.m[1][2]);
	Vector3 vaxisz = Vector3(temp_matrix.m[2][0], temp_matrix.m[2][1], temp_matrix.m[2][2]);
	Vector3 vdelta = world_location - cache::local_camera.location;
	Vector3 vtransformed = Vector3(vdelta.dot(vaxisy), vdelta.dot(vaxisz), vdelta.dot(vaxisx));
	if (vtransformed.z < 1) vtransformed.z = 1;
	return { settings::screen_center_x + vtransformed.x * ((settings::screen_center_x / tanf(cache::local_camera.fov * M_PI / 360))) / vtransformed.z, settings::screen_center_y - vtransformed.y * ((settings::screen_center_x / tanf(cache::local_camera.fov * M_PI / 360))) / vtransformed.z };
}

inline Vector3 get_updated_viewpoint_location()
{
	uintptr_t location_pointer = mem.Read<uintptr_t>(cache::uworld + 0x110);
	return mem.Read<Vector3>(location_pointer);
}

inline Vector3 get_world_space_coords(const FTransform& head_bone, const FTransform& component_to_world)
{
	D3DMATRIX bone_matrix = matrix_multiplication(head_bone.to_matrix_with_scale(), component_to_world.to_matrix_with_scale());
	return { bone_matrix._41, bone_matrix._42, bone_matrix._43 };
}

struct CompareDistance
{
	bool operator()(const MeshInfoContainer& a, const MeshInfoContainer& b) const
	{
		// Distance from center screen
		Vector2 screen_center = { (double)settings::screen_center_x, (double)settings::screen_center_y };
		Vector3 world_space_a = get_world_space_coords(a.cached_head_bone, a.cached_component_to_world);
		Vector3 world_space_b = get_world_space_coords(b.cached_head_bone, b.cached_component_to_world);
		Vector2 a_screen = project_world_to_screen(world_space_a);
		Vector2 b_screen = project_world_to_screen(world_space_b);
		double world_space_distance_a = world_space_a.distance(cache::local_camera.location);
		double world_space_distance_b = world_space_b.distance(cache::local_camera.location);
		double a_distance = 0.699 * (a_screen - screen_center).x * (a_screen - screen_center).x + 0.699 * (a_screen - screen_center).y * (a_screen - screen_center).y + 0.301 * world_space_distance_a * world_space_distance_a;
		double b_distance = 0.699 * (b_screen - screen_center).x * (b_screen - screen_center).x + 0.699 * (b_screen - screen_center).y * (b_screen - screen_center).y + 0.301 * world_space_distance_b * world_space_distance_b;
		return a_distance < b_distance;
	}
};

inline uintptr_t get_all_player_meshes(int interval) {
	uintptr_t pgame_state = mem.Read<uintptr_t>(cache::uworld + offsets::uworld_to_pgamestate);
	uintptr_t player_state_tarray = pgame_state + offsets::gamestate_to_tpplayerstate_array;
	cache::player_array = player_state_tarray;

	// Set mesh_info capacity to 200
	cache::mesh_info.reserve(200);
	// Set mesh_info_cache capacity to 200
	cache::mesh_info_cache.reserve(200);
	// Set mesh_info_cache_copy capacity to 200
	cache::mesh_info_cache_copy.reserve(200);

	uintptr_t player_state_arr[200];
	auto scatter_handle = mem.CreateScatterHandle();

	while (true) {
		{
			try {

				// Measure performance
				auto start = std::chrono::high_resolution_clock::now();

				if (!is_valid(cache::player_array)) {
					uintptr_t pgame_state = mem.Read<uintptr_t>(cache::uworld + offsets::uworld_to_pgamestate);
					uintptr_t player_state_tarray = pgame_state + offsets::gamestate_to_tpplayerstate_array;
					cache::player_array = player_state_tarray;
				}

				int player_count = mem.Read<int>(cache::player_array + 0x8);
				cache::player_count = player_count;

				uintptr_t player_array_cursor = mem.Read<uintptr_t>(cache::player_array);
				if (!is_valid(player_array_cursor)) {
					continue;
				}

				cache::mesh_info.clear();

				mem.Read(player_array_cursor, player_state_arr, player_count * 0x8);

				// Update all mesh info for all player states in player_state_arr found in mesh_info, otherwise read mesh info from memory and add it to mesh_info
				for (int i = 0; i < player_count; i++) {
					// Lock mutex
					//std::lock_guard<std::mutex> lock(cache::mem_mutex);
					// 
					// Check if player state in player_state_arr is found in mesh_info_map
					/*if (cache::mesh_info_map.find(player_state_arr[i]) != cache::mesh_info_map.end()) {
						auto cached_mesh_info = cache::mesh_info_map[player_state_arr[i]];
						cached_mesh_info.cached_head_bone = mem.Read<FTransform>(cached_mesh_info.head_bone);
						cached_mesh_info.cached_component_to_world = mem.Read<FTransform>(cached_mesh_info.component_to_world);
						cache::mesh_info.push_back(cached_mesh_info);
						continue;
					}*/
					// Update mesh info for player state
					uintptr_t player_state = player_state_arr[i];
					if (!is_valid(player_state)) {
						continue;
					}
					BYTE player_team_id;
					uintptr_t player_pawn;
					BYTE is_bot;
					mem.AddScatterReadRequest(scatter_handle, player_state + offsets::playerstate_to_teamid, &player_team_id, sizeof(BYTE));
					mem.AddScatterReadRequest(scatter_handle, player_state + offsets::playerstate_to_ppawn, &player_pawn, sizeof(uintptr_t));
					mem.AddScatterReadRequest(scatter_handle, player_state + offsets::playerstate_to_isabot, &is_bot, sizeof(BYTE));
					mem.ExecuteReadScatter(scatter_handle);
					is_bot = is_bot >> 3 & 1;
					if (player_team_id == cache::my_team_id) {
						continue;
					}
					if (!is_valid(player_pawn)) {
						continue;
					}

					bool is_attacking_bot = false;
					if (is_bot) {
						is_attacking_bot = mem.Read<BYTE>(mem.ReadChain(player_state, { (UINT)offsets::playerstate_to_ppawn }) + offsets::pawn_to_isattacking) == 3;
					}
					if (is_bot && !is_attacking_bot) {
						continue;
					}

					uintptr_t player_controller;
					uintptr_t player_mesh_component;
					uintptr_t root_component;
					mem.AddScatterReadRequest(scatter_handle, player_pawn + offsets::pawn_to_controller, &player_controller, sizeof(uintptr_t));
					mem.AddScatterReadRequest(scatter_handle, player_pawn + offsets::pawn_to_pmeshcomponent, &player_mesh_component, sizeof(uintptr_t));
					mem.AddScatterReadRequest(scatter_handle, player_pawn + offsets::pawn_to_rootcomponent, &root_component, sizeof(uintptr_t));
					mem.ExecuteReadScatter(scatter_handle);
					if (!is_valid(player_mesh_component)) {
						continue;
					}
					// skip local player
					if (player_controller == cache::player_controller) {
						continue;
					}

					uintptr_t player_bone_array;
					uintptr_t player_bone_array_cache;
					uintptr_t component_to_world = player_mesh_component + offsets::meshcomponent_to_componenttoworld;

					mem.AddScatterReadRequest(scatter_handle, player_mesh_component + offsets::meshcomponent_to_bonearray, &player_bone_array, sizeof(uintptr_t));
					mem.AddScatterReadRequest(scatter_handle, player_mesh_component + offsets::meshcomponent_to_bonearray + 0x10, &player_bone_array_cache, sizeof(uintptr_t));
					mem.ExecuteReadScatter(scatter_handle);

					if (!is_valid(player_bone_array)) {
						player_bone_array = player_bone_array_cache;
						if (!is_valid(player_bone_array)) {
							continue;
						}
					}

					if (!is_valid(component_to_world)) {
						continue;
					}

					uintptr_t head_bone = player_bone_array + 0x60 * 110;

					FTransform head_transform;
					FTransform component_to_world_transform;
					mem.AddScatterReadRequest(scatter_handle, head_bone, &head_transform, sizeof(FTransform));
					mem.AddScatterReadRequest(scatter_handle, component_to_world, &component_to_world_transform, sizeof(FTransform));
					mem.ExecuteReadScatter(scatter_handle);

					cache::mesh_info.push_back({ player_state, head_bone, head_transform, component_to_world, component_to_world_transform, root_component, player_pawn });
					//cache::mesh_info_map[player_state] = { player_state, head_bone, head_transform, component_to_world, component_to_world_transform };
				}


				//// Update camera location
				//cache::local_camera.location = get_updated_viewpoint_location();
				// Sort by distance
				//cache::mesh_info_cache_copy = cache::mesh_info;

				std::ranges::sort(cache::mesh_info, CompareDistance());
				{
					// Lock mutex
					std::lock_guard lock(cache::mem_mutex);
					// Recreate mesh_info_cache
					cache::mesh_info_cache.clear();
					int numElementsToCopy = min(static_cast<int>(cache::mesh_info.size()), 5);
					EXEC_ON_ARG(std::cout << "Num elements to copy: " << numElementsToCopy << '\n', false);
					std::copy_n(cache::mesh_info.begin(), numElementsToCopy, std::back_inserter(cache::mesh_info_cache));
				}


				// Print performance
				auto end = std::chrono::high_resolution_clock::now();
				std::chrono::duration<double> elapsed = end - start;
				EXEC_ON_DEBUG(std::cout << "Time to read all player meshes: " << elapsed.count() << "s" << '\n');
			}
			catch (std::exception& e) {
				EXEC_ON_DEBUG(std::cout << "Error reading player meshes: " << e.what() << '\n');
			}
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(interval));
		// Sleep(interval);
	}
}

inline Vector3 predict_location(Vector3 target, const Vector3& target_velocity, float projectile_speed,
                                float projectile_gravity_scale, float distance)
{
	if (projectile_speed == 0)
	{
		return target;
	}
	float horizontalTime = distance / projectile_speed;
	float verticalTime = distance / projectile_speed;

	//std::cout << "Horizontal time: " << horizontalTime << std::endl;
	//std::cout << "Vertical time: " << verticalTime << std::endl;

	target.x += target_velocity.x * horizontalTime;
	target.y += target_velocity.y * horizontalTime;
	target.z += target_velocity.z * verticalTime +
		abs(-980 * projectile_gravity_scale) * 0.5f * (verticalTime * verticalTime);

	return target;
}

inline std::unique_ptr<wchar_t[]> get_weapon_name() {
	uintptr_t current_weapon = mem.Read<uintptr_t>(cache::local_pawn + offsets::pawn_to_currentweapon);
	if (!is_valid(current_weapon)) {
		return nullptr;
	}

	uintptr_t weapon_data = mem.Read<uintptr_t>(current_weapon + offsets::weapon_to_weapondata);
	if (!is_valid(weapon_data)) {
		return nullptr;
	}

	uintptr_t weapon_text_data = mem.Read<uintptr_t>(weapon_data + offsets::weapondata_to_weaponname);
	if (!is_valid(weapon_text_data)) {
		return nullptr;
	}

	uintptr_t weapon_name_buffer = mem.Read<uintptr_t>(weapon_text_data + offsets::weaponname_to_buf);
	if (!is_valid(weapon_name_buffer)) {
		return nullptr;
	}
	UINT weapon_name_length = mem.Read<UINT>(weapon_text_data + offsets::weaponname_to_buf + 0x8);
	if (weapon_name_length == 0 || weapon_name_length > 50) {
		return nullptr;
	}

	std::unique_ptr<wchar_t[]> weapon_name = std::make_unique<wchar_t[]>(weapon_name_length + 1);
	mem.Read(weapon_name_buffer, weapon_name.get(), weapon_name_length * sizeof(wchar_t));
	weapon_name[weapon_name_length] = L'\0';
	return weapon_name;
}