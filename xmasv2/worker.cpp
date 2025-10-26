#include "worker.h"

#include <stdio.h>
#include <unistd.h>

#include <deque>
#include <mutex>
#include <thread>

namespace {
bool terminate = false;

std::deque<Work> workQueue;

std::mutex workMutex;
}  // namespace

int work_size() { return int(workQueue.size()); }

void push_work(const Work &work) {
  std::scoped_lock lock(workMutex);
  workQueue.push_back(work);
}

std::optional<Work> pop_work() {
  std::scoped_lock lock(workMutex);
  if (workQueue.size() == 0) {
    return {};
  }
  auto work = workQueue.front();
  // printf("pop_work() status = %d\n", int(work.status));
  if (work.status != Done) {
    return {};
  }
  workQueue.pop_front();
  return work;
}

std::optional<Work *> fetch_work() {
  std::scoped_lock lock(workMutex);
  if (workQueue.size() == 0) {
    return {};
  }
  for (auto &w : workQueue) {
    if (w.status == Waiting) {
      w.status = Processing;
      return &w;
    }
  }
  return {};
}

void mark_work_done(Work *w) {
  std::scoped_lock lock(workMutex);
  w->status = Done;
}

void worker(int id) {
  printf("Starting worker %d\n", id);
  while (!terminate) {
    std::optional<Work *> work = fetch_work();
    if (work) {
      auto &w = *work.value();
      w.shader->shade(w.pixInfo, w.buffer, w.t_s);
      w.status = Done;
      mark_work_done(&w);
    } else {
      usleep(100);
    }
  }
}

void add_worker(int id) {
  std::thread t(worker, id);
  t.detach();
}

void shutdown() {
  std::scoped_lock lock(workMutex);
  terminate = true;
}
