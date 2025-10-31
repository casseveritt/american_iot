#pragma once
#include <cstdint>
#include <optional>
#include <span>

// Forward declarations
struct PixInfo;

enum Status { Waiting, Processing, Done };

struct TreeShader {
  virtual ~TreeShader() = default;
  virtual void init(float t) {}
  virtual void shade(std::span<PixInfo> pixInfo, uint8_t *buffer, float t) = 0;
};

struct Work {
  TreeShader *shader;
  std::span<PixInfo> pixInfo;
  uint8_t *buffer;
  float t_s;
  Status status = Waiting;
};

// Function declarations
int work_size();
void push_work(const Work &work);
std::optional<Work> pop_work();
std::optional<Work *> fetch_work();
void mark_work_done(Work *w);

void add_worker(int id);

void shutdown();