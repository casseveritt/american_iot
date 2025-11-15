#pragma once
#include <cstdint>
#include <optional>
#include <span>

// Forward declarations
struct PixInfo;

enum Status { Waiting, Processing, Done };

struct TreeShader {
  virtual ~TreeShader() = default;
  virtual void init(std::span<PixInfo> pixInfo, uint64_t t_ns) {}
  virtual void shade(std::span<PixInfo> pixInfo, uint8_t *buffer,
                     uint64_t t_ns) = 0;
};

struct Work {
  TreeShader *shader;
  std::span<PixInfo> pixInfo;
  uint8_t *buffer;
  uint64_t t_ns;
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