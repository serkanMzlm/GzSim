#ifndef UDP_HPP
#define UDP_HPP

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <poll.h>
#include <fcntl.h>
#include <errno.h>

#include <iostream>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <chrono>
#include <string>
#include <memory>
#include <stdexcept>

class Udp {
public:
    Udp(int local_port = 14553, 
        const std::string& remote_ip = "127.0.0.1", 
        int remote_port = 14554);
    ~Udp();

    Udp(const Udp&) = delete;
    Udp& operator=(const Udp&) = delete;

    bool initializeSocket();
    void disconnect();
    
    int readOneByte(uint8_t& byte);
    int readData(uint8_t* buffer, size_t size);
    
    void writeOneByte(uint8_t byte);
    void writeData(const uint8_t* buffer, size_t size);
    
    void openPort();
    bool isOpenPort() const { return _is_open_port; }
    
    void setTimeout(uint16_t timeout_ms) { _time_out = timeout_ms; }
    uint16_t getTimeout() const { return _time_out; }

private:
    int _sock_fd{-1};
    struct sockaddr_in _local_addr{};
    struct sockaddr_in _remote_addr{};
    struct pollfd _pollfds[1]{};

    const int _local_port;
    const std::string _remote_ip;
    const int _remote_port;

    uint16_t _time_out{5000};
    bool _is_open_port{false};

    mutable std::mutex _mtx;
    std::chrono::steady_clock::time_point _timestamp;

    void setupSocketOptions();
    void bindSocket();
    void setupPoll();
};

#endif 