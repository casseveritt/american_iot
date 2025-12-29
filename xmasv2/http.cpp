#include "http.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <iostream>
#include <sstream>
#include <thread>

#include "hween.h"

HttpServer::HttpServer(int port) : port_(port), running_(false) {}

HttpServer::~HttpServer() { stop(); }

bool HttpServer::start() {
  if (running_) {
    return false;
  }

  running_ = true;
  server_thread_ = std::thread(&HttpServer::serverLoop, this);
  return true;
}

void HttpServer::stop() {
  if (running_) {
    running_ = false;
    if (server_thread_.joinable()) {
      server_thread_.join();
    }
  }
}

void HttpServer::serverLoop() {
  int server_fd = socket(AF_INET, SOCK_STREAM, 0);
  if (server_fd < 0) {
    std::cerr << "Failed to create socket\n";
    return;
  }

  // Allow socket reuse
  int opt = 1;
  if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
    std::cerr << "Failed to set socket options\n";
    close(server_fd);
    return;
  }

  struct sockaddr_in address;
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = INADDR_ANY;
  address.sin_port = htons(port_);

  if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
    std::cerr << "Failed to bind to port " << port_ << "\n";
    close(server_fd);
    return;
  }

  if (listen(server_fd, 10) < 0) {
    std::cerr << "Failed to listen on socket\n";
    close(server_fd);
    return;
  }

  std::cout << "HTTP server listening on port " << port_ << "\n";

  while (running_) {
    struct sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);

    // Set timeout for accept
    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    setsockopt(server_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    int client_fd =
        accept(server_fd, (struct sockaddr*)&client_addr, &client_len);
    if (client_fd < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        continue;
      }
      continue;
    }

    handleClient(client_fd);
    close(client_fd);
  }

  close(server_fd);
}

void HttpServer::handleClient(int client_fd) {
  char buffer[4096];
  ssize_t bytes_read = read(client_fd, buffer, sizeof(buffer) - 1);

  if (bytes_read <= 0) {
    return;
  }

  buffer[bytes_read] = '\0';

  // Parse first line to get method and path
  std::string request(buffer);
  std::istringstream iss(request);
  std::string method, path, version;
  iss >> method >> path >> version;

  // Handle POST request for shader change
  if (method == "POST" && path == "/change_shader") {
    // Call hween to change to random shader
    hween_change_to_random_shader();

    // Redirect back to main page
    std::ostringstream response;
    response << "HTTP/1.1 303 See Other\r\n";
    response << "Location: /\r\n";
    response << "Connection: close\r\n";
    response << "\r\n";
    std::string response_str = response.str();
    write(client_fd, response_str.c_str(), response_str.length());
    return;
  }

  // Generate HTML form
  std::ostringstream body;
  body << "<!DOCTYPE html>\n";
  body << "<html><head><title>LED Shader Control</title>";
  body << "<style>";
  body << "body { font-family: Arial, sans-serif; max-width: 800px; margin: "
          "50px auto; padding: 20px; text-align: center; }";
  body << "h1 { color: #333; }";
  body << ".current { background: #e8f5e9; padding: 20px; border-radius: 5px; "
          "margin: 30px 0; font-size: 24px; }";
  body << "button { padding: 20px 40px; font-size: 20px; cursor: pointer; "
          "border: 2px solid #1976d2; background: #2196f3; color: white; "
          "border-radius: 5px; transition: all 0.3s; }";
  body << "button:hover { background: #1976d2; transform: scale(1.05); }";
  body << "</style></head><body>";
  body << "<h1>LED Shader Control</h1>";
  body << "<div class='current'>Current shader: <strong>"
       << hween_get_current_shader() << "</strong></div>";
  body << "<form method='POST' action='/change_shader'>";
  body << "<button type='submit'>Change to Random Shader</button>";
  body << "</form>";
  body << "</body></html>";

  std::string response_body = body.str();

  std::ostringstream response;
  response << "HTTP/1.1 200 OK\r\n";
  response << "Content-Type: text/html\r\n";
  response << "Content-Length: " << response_body.length() << "\r\n";
  response << "Connection: close\r\n";
  response << "\r\n";
  response << response_body;

  std::string response_str = response.str();
  write(client_fd, response_str.c_str(), response_str.length());
}
