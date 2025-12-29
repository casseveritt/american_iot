#include "http.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
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
  std::cout << "[HTTP] Starting server loop...\n";
  int server_fd = socket(AF_INET, SOCK_STREAM, 0);
  if (server_fd < 0) {
    std::cerr << "[HTTP] Failed to create socket: " << strerror(errno) << "\n";
    return;
  }
  std::cout << "[HTTP] Socket created (fd=" << server_fd << ")\n";

  // Allow socket reuse
  int opt = 1;
  if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
    std::cerr << "[HTTP] Failed to set socket options: " << strerror(errno)
              << "\n";
    close(server_fd);
    return;
  }
  std::cout << "[HTTP] Socket options set\n";

  struct sockaddr_in address;
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = INADDR_ANY;
  address.sin_port = htons(port_);

  std::cout << "[HTTP] Attempting to bind to 0.0.0.0:" << port_ << "\n";
  if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
    std::cerr << "[HTTP] Failed to bind to port " << port_ << ": "
              << strerror(errno) << "\n";
    close(server_fd);
    return;
  }
  std::cout << "[HTTP] Bind successful\n";

  if (listen(server_fd, 10) < 0) {
    std::cerr << "[HTTP] Failed to listen on socket: " << strerror(errno)
              << "\n";
    close(server_fd);
    return;
  }

  std::cout << "[HTTP] HTTP server listening on port " << port_ << "\n";
  std::cout << "[HTTP] Server ready to accept connections\n";

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
      std::cerr << "[HTTP] Accept error: " << strerror(errno) << "\n";
      continue;
    }

    char client_ip[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, INET_ADDRSTRLEN);
    std::cout << "[HTTP] Accepted connection from " << client_ip << ":"
              << ntohs(client_addr.sin_port) << "\n";

    handleClient(client_fd);
    close(client_fd);
    std::cout << "[HTTP] Connection closed\n";
  }

  close(server_fd);
}

void HttpServer::handleClient(int client_fd) {
  char buffer[4096];
  ssize_t bytes_read = read(client_fd, buffer, sizeof(buffer) - 1);

  std::cout << "[HTTP] Read " << bytes_read << " bytes from client\n";

  if (bytes_read <= 0) {
    std::cerr << "[HTTP] Read error or empty request\n";
    return;
  }

  buffer[bytes_read] = '\0';

  // Parse first line to get method and path
  std::string request(buffer);
  std::istringstream iss(request);
  std::string method, path, version;
  iss >> method >> path >> version;

  std::cout << "[HTTP] Request: " << method << " " << path << " " << version
            << "\n";

  // Handle POST request for shader change
  if (method == "POST" && path == "/change_shader") {
    std::cout << "[HTTP] Processing shader change request\n";
    // Call hween to change to random shader
    hween_change_to_random_shader();
    std::cout << "[HTTP] Shader changed to: " << hween_get_current_shader()
              << "\n";

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

  // Handle POST request for brightness change
  if (method == "POST" && path == "/set_brightness") {
    std::cout << "[HTTP] Processing brightness change request\n";

    // Find the body after double newline
    std::string body_str;
    size_t body_pos = request.find("\r\n\r\n");
    if (body_pos != std::string::npos) {
      body_str = request.substr(body_pos + 4);
    }

    // Parse brightness value
    size_t brightness_pos = body_str.find("brightness=");
    if (brightness_pos != std::string::npos) {
      std::string brightness_str = body_str.substr(brightness_pos + 11);
      size_t end_pos = brightness_str.find("&");
      if (end_pos != std::string::npos) {
        brightness_str = brightness_str.substr(0, end_pos);
      }

      try {
        int brightness = std::stoi(brightness_str);
        brightness = std::clamp(brightness, 0, 100);
        hween_set_brightness(brightness);
        std::cout << "[HTTP] Brightness set to: " << brightness << "\n";
      } catch (...) {
        std::cerr << "[HTTP] Failed to parse brightness value\n";
      }
    }

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
  body << ".control { background: #f5f5f5; padding: 20px; border-radius: 5px; "
          "margin: 20px 0; }";
  body << ".slider-container { margin: 20px 0; }";
  body << ".slider { width: 80%; height: 15px; border-radius: 5px; "
          "background: #d3d3d3; outline: none; }";
  body << ".slider::-webkit-slider-thumb { -webkit-appearance: none; "
          "appearance: none; width: 25px; height: 25px; border-radius: 50%; "
          "background: #2196f3; cursor: pointer; }";
  body << ".slider::-moz-range-thumb { width: 25px; height: 25px; "
          "border-radius: 50%; background: #2196f3; cursor: pointer; "
          "border: none; }";
  body << "button { padding: 20px 40px; font-size: 20px; cursor: pointer; "
          "border: 2px solid #1976d2; background: #2196f3; color: white; "
          "border-radius: 5px; transition: all 0.3s; margin: 10px; }";
  body << "button:hover { background: #1976d2; transform: scale(1.05); }";
  body << "</style></head><body>";
  body << "<h1>LED Shader Control</h1>";
  body << "<div class='current'>Current shader: <strong>"
       << hween_get_current_shader() << "</strong></div>";
  body << "<form method='POST' action='/change_shader'>";
  body << "<button type='submit'>Change to Random Shader</button>";
  body << "</form>";

  body << "<div class='control'>";
  body << "<h2>Brightness Control</h2>";
  body << "<form method='POST' action='/set_brightness'>";
  body << "<div class='slider-container'>";
  body << "<label for='brightness'>Brightness: <span id='brightness-value'>"
       << hween_get_brightness() << "</span>%</label><br>";
  body << "<input type='range' id='brightness' name='brightness' "
          "class='slider' min='0' max='100' value='"
       << hween_get_brightness()
       << "' "
          "oninput='document.getElementById(\"brightness-value\").innerText="
          "this.value'>";
  body << "</div>";
  body << "<button type='submit'>Set Brightness</button>";
  body << "</form>";
  body << "</div>";

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
