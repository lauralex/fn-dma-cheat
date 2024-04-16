#pragma once
#include <d3d9.h>
#include <vector>
#include "Settings.hpp"
#include <mutex>
#include <unordered_map>
#pragma comment(lib, "d3d9.lib")
#define M_PI 3.14159265358979323846264338327950288419716939937510
#define DEBUG true
#define EXEC_ON_DEBUG(x) if (DEBUG) { x; }

class Vector2
{
public:
	Vector2() : x(0.f), y(0.f) {}
	Vector2(double _x, double _y) : x(_x), y(_y) {}
	~Vector2() {}

	Vector2 operator-(Vector2 v) { return Vector2(x - v.x, y - v.y); }
	Vector2 operator+(Vector2 v) { return Vector2(x + v.x, y + v.y); }
	double x, y;
};

class Vector3
{
public:
	Vector3() : x(0.f), y(0.f), z(0.f) {}
	Vector3(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}
	~Vector3() {}
	double x, y, z;
	inline double dot(Vector3 v) { return x * v.x + y * v.y + z * v.z; }
	inline double distance(Vector3 v) { return double(sqrtf(powf(v.x - x, 2.0) + powf(v.y - y, 2.0) + powf(v.z - z, 2.0))); }
	Vector3 operator-(Vector3 v) { return Vector3(x - v.x, y - v.y, z - v.z); }
};

struct FQuat { double x, y, z, w; };
struct FTransform
{
	FQuat rot;
	Vector3 translation;
	UINT8 pad[0x8];
	Vector3 scale;
	UINT8 pad1[0x8];
	D3DMATRIX to_matrix_with_scale()
	{
		D3DMATRIX m{};
		m._41 = translation.x;
		m._42 = translation.y;
		m._43 = translation.z;
		float x2 = rot.x + rot.x;
		float y2 = rot.y + rot.y;
		float z2 = rot.z + rot.z;
		float xx2 = rot.x * x2;
		float yy2 = rot.y * y2;
		float zz2 = rot.z * z2;
		m._11 = (1.0f - (yy2 + zz2)) * scale.x;
		m._22 = (1.0f - (xx2 + zz2)) * scale.y;
		m._33 = (1.0f - (xx2 + yy2)) * scale.z;
		float yz2 = rot.y * z2;
		float wx2 = rot.w * x2;
		m._32 = (yz2 - wx2) * scale.z;
		m._23 = (yz2 + wx2) * scale.y;
		float xy2 = rot.x * y2;
		float wz2 = rot.w * z2;
		m._21 = (xy2 - wz2) * scale.y;
		m._12 = (xy2 + wz2) * scale.x;
		float xz2 = rot.x * z2;
		float wy2 = rot.w * y2;
		m._31 = (xz2 + wy2) * scale.z;
		m._13 = (xz2 - wy2) * scale.x;
		m._14 = 0.0f;
		m._24 = 0.0f;
		m._34 = 0.0f;
		m._44 = 1.0f;
		return m;
	}
};

D3DMATRIX matrix_multiplication(D3DMATRIX pm1, D3DMATRIX pm2)
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

D3DMATRIX to_matrix(Vector3 rot, Vector3 origin = Vector3(0, 0, 0))
{
	float radpitch = (rot.x * M_PI / 180);
	float radyaw = (rot.y * M_PI / 180);
	float radroll = (rot.z * M_PI / 180);
	float sp = sinf(radpitch);
	float cp = cosf(radpitch);
	float sy = sinf(radyaw);
	float cy = cosf(radyaw);
	float sr = sinf(radroll);
	float cr = cosf(radroll);
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
	uintptr_t player_state;
	uintptr_t head_bone;
	FTransform cached_head_bone;
	uintptr_t component_to_world;
	FTransform cached_component_to_world;
};


namespace offsets
{
	inline int uworld = 0x1216b7b8;
	inline int uworld_to_pgamestate = 0x160;
	inline int gamestate_to_tpplayerstate_array = 0x2a8;
	inline int playerstate_to_ppawn = 0x308;
	inline int pawn_to_pmeshcomponent = 0x318;
	inline int pawn_to_controller = 0x2c8;
	inline int meshcomponent_to_bonearray = 0x5b0;
	inline int meshcomponent_to_componenttoworld = 0x1c0;
	inline int playercontroller_to_rotationinput = 0x520;
	inline int pawn_to_isdying = 0x758;

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
	inline int my_team_id;
	inline uintptr_t game_state;
	inline uintptr_t player_array;
	inline int player_count;
	inline float closest_distance;
	inline uintptr_t closest_mesh;
	inline MeshInfoContainer closest_mesh_info;
	inline Camera local_camera;
	inline std::vector<MeshInfoContainer> mesh_info;
	// Hashmap that maps player state to mesh info
	inline std::unordered_map<uintptr_t, MeshInfoContainer> mesh_info_map;
	// mesh_info cache to be used by aimbot thread
	inline std::vector<MeshInfoContainer> mesh_info_cache;
	// Mutex for DMA memory read between threads
	inline std::mutex mem_mutex;
}

Camera get_view_point()
{
	Camera view_point{};
	uintptr_t location_pointer = mem.Read<uintptr_t>(cache::uworld + 0x110);
	uintptr_t rotation_pointer = mem.Read<uintptr_t>(cache::uworld + 0x120);
	FNRot fnrot{};
	fnrot.a = mem.Read<double>(rotation_pointer);
	fnrot.b = mem.Read<double>(rotation_pointer + 0x20);
	fnrot.c = mem.Read<double>(rotation_pointer + 0x1D0);
	view_point.location = mem.Read<Vector3>(location_pointer);
	view_point.rotation.x = asin(fnrot.c) * (180.0 / M_PI);
	view_point.rotation.y = ((atan2(fnrot.a * -1, fnrot.b) * (180.0 / M_PI)) * -1) * -1;
	view_point.fov = mem.Read<float>(cache::player_controller + 0x394) * 90.f;
	return view_point;
}

Vector2 project_world_to_screen(Vector3 world_location)
{
	cache::local_camera = get_view_point();
	D3DMATRIX temp_matrix = to_matrix(cache::local_camera.rotation);
	Vector3 vaxisx = Vector3(temp_matrix.m[0][0], temp_matrix.m[0][1], temp_matrix.m[0][2]);
	Vector3 vaxisy = Vector3(temp_matrix.m[1][0], temp_matrix.m[1][1], temp_matrix.m[1][2]);
	Vector3 vaxisz = Vector3(temp_matrix.m[2][0], temp_matrix.m[2][1], temp_matrix.m[2][2]);
	Vector3 vdelta = world_location - cache::local_camera.location;
	Vector3 vtransformed = Vector3(vdelta.dot(vaxisy), vdelta.dot(vaxisz), vdelta.dot(vaxisx));
	if (vtransformed.z < 1) vtransformed.z = 1;
	return Vector2(settings::screen_center_x + vtransformed.x * ((settings::screen_center_x / tanf(cache::local_camera.fov * M_PI / 360))) / vtransformed.z, settings::screen_center_y - vtransformed.y * ((settings::screen_center_x / tanf(cache::local_camera.fov * M_PI / 360))) / vtransformed.z);
}

Vector3 get_updated_viewpoint_location()
{
	uintptr_t location_pointer = mem.Read<uintptr_t>(cache::uworld + 0x110);
	return mem.Read<Vector3>(location_pointer);
}

Vector3 get_world_space_coords(FTransform head_bone, FTransform component_to_world)
{
	D3DMATRIX bone_matrix = matrix_multiplication(head_bone.to_matrix_with_scale(), component_to_world.to_matrix_with_scale());
	return Vector3(bone_matrix._41, bone_matrix._42, bone_matrix._43);
}

struct CompareDistance
{
	bool operator()(const MeshInfoContainer& a, const MeshInfoContainer& b)
	{
		// Distance from center screen
		Vector2 screen_center = { (double)settings::screen_center_x, (double)settings::screen_center_y };
		Vector3 world_space_a = get_world_space_coords(a.cached_head_bone, a.cached_component_to_world);
		Vector3 world_space_b = get_world_space_coords(b.cached_head_bone, b.cached_component_to_world);
		Vector2 a_screen = project_world_to_screen(world_space_a);
		Vector2 b_screen = project_world_to_screen(world_space_b);
		float world_space_distance_a = world_space_a.distance(cache::local_camera.location);
		float world_space_distance_b = world_space_b.distance(cache::local_camera.location);
		double a_distance = (a_screen - screen_center).x * (a_screen - screen_center).x + (a_screen - screen_center).y * (a_screen - screen_center).y + world_space_distance_a * world_space_distance_a;
		double b_distance = (b_screen - screen_center).x * (b_screen - screen_center).x + (b_screen - screen_center).y * (b_screen - screen_center).y + world_space_distance_b * world_space_distance_b;
		return a_distance < b_distance;
	}
};

uintptr_t get_all_player_meshes(int interval) {
	uintptr_t pgame_state = mem.Read<uintptr_t>(cache::uworld + offsets::uworld_to_pgamestate);
	uintptr_t player_state_tarray = pgame_state + offsets::gamestate_to_tpplayerstate_array;
	cache::player_array = player_state_tarray;

	while (true) {
		{
			
			// Measure performance
			auto start = std::chrono::high_resolution_clock::now();

			int player_count = mem.Read<int>(cache::player_array + 0x8);
			cache::player_count = player_count;

			uintptr_t player_array_cursor = mem.Read<uintptr_t>(cache::player_array);

			cache::mesh_info.clear();
			uintptr_t player_state_arr[200];
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
				uintptr_t player_pawn = mem.Read<uintptr_t>(player_state + offsets::playerstate_to_ppawn);
				uintptr_t player_controller = mem.Read<uintptr_t>(player_pawn + offsets::pawn_to_controller);
				if (player_controller == cache::player_controller) {
					// Skip because it's our player
					continue;
				}
				uintptr_t player_mesh_component = mem.Read<uintptr_t>(player_pawn + offsets::pawn_to_pmeshcomponent);

				uintptr_t player_bone_array = mem.Read<uintptr_t>(player_mesh_component + offsets::meshcomponent_to_bonearray);
				uintptr_t head_bone = player_bone_array + 0x60 * 110;
				FTransform head_transform = mem.Read<FTransform>(head_bone);
				uintptr_t component_to_world = player_mesh_component + offsets::meshcomponent_to_componenttoworld;
				FTransform component_to_world_transform = mem.Read<FTransform>(component_to_world);
				cache::mesh_info.push_back({ player_state, head_bone, head_transform, component_to_world, component_to_world_transform });
				//cache::mesh_info_map[player_state] = { player_state, head_bone, head_transform, component_to_world, component_to_world_transform };
			}
			

			//// Update camera location
			//cache::local_camera.location = get_updated_viewpoint_location();
			// Sort by distance
			std::vector<MeshInfoContainer> mesh_info_copy = cache::mesh_info;
			
			std::sort(mesh_info_copy.begin(), mesh_info_copy.end(), CompareDistance());

			{
				// Lock mutex
				std::lock_guard<std::mutex> lock(cache::mem_mutex);
				cache::mesh_info_cache = std::move(mesh_info_copy);
			}
			
			

			// Print performance
			auto end = std::chrono::high_resolution_clock::now();
			std::chrono::duration<double> elapsed = end - start;
			EXEC_ON_DEBUG(std::cout << "Time to read all player meshes: " << elapsed.count() << "s" << std::endl);
		}

		Sleep(interval);
	}
}

