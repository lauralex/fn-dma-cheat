#pragma once
#include <map>
#include <string>

// Fortnite Weapon Data
namespace projectile_stats
{
	inline float projectile_speed = 0;
	inline float gravity_scale = 0;
}

struct weapon_projectile_data
{
	float speed;
	float gravity_scale;
};

// Map weapon names to projectile data
static std::map<std::wstring, weapon_projectile_data> weapon_projectile_map = {
	{L"Default", {0.0f, 0.0f}},
	{L"Rifle", {0.0f, 0.0f}},
	{L"Tactical Assault Rifle", {80000.0f, 3.5f}},
	{L"Striker AR", {80000.0f, 3.5f}},
	{L"Nemesis AR", {80000.0f, 3.5f}},
	{L"Huntress DMR", {96000.0f, 2.5f}},
	{L"Harbinger SMG", {70000.0f, 3.0f}},
	{L"Hades' Harbinger SMG", {70000.0f, 3.0f}},
	{L"Thunder Burst SMG", {70000.0f, 3.0f}},
	{L"Warforged Assault Rifle", {80000.0f, 3.5f}},
	{L"Ares' Warforged Assault Rifle", {80000.0f, 3.5f}},
	{L"Reaper Sniper Rifle", {50000.0f, 3.5f}},
	{L"Drum Gun", {75000.0f, 3.0f}},
	{L"Midas Drum Gun", {75000.0f, 3.0f}},
};
