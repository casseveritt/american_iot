#pragma once

#include <string>

// Initialize the LED system and shader management
void hween_init();

// Change to a random shader
void hween_change_to_random_shader();

// Change to a specific shader by name (returns false if shader not found)
bool hween_change_shader(const std::string& shader_name);

// Get the current shader name
std::string hween_get_current_shader();

// Run the main LED update loop (blocks)
void hween_run();
