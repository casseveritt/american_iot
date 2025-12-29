#pragma once

#include <atomic>
#include <thread>

class HttpServer {
 public:
  explicit HttpServer(int port = 8080);
  ~HttpServer();

  // Start the server on a background thread
  bool start();

  // Stop the server and join the thread
  void stop();

  // Check if server is running
  bool isRunning() const { return running_; }

 private:
  void serverLoop();
  void handleClient(int client_fd);

  int port_;
  std::atomic<bool> running_;
  std::thread server_thread_;
};
