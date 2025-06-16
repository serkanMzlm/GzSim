#include "udp.hpp"
#include <thread>
#include <system_error>

Udp::Udp(int local_port, const std::string& remote_ip, int remote_port)
    : _local_port(local_port),
      _remote_ip(remote_ip),
      _remote_port(remote_port) {
    _timestamp = std::chrono::steady_clock::now();
}

Udp::~Udp() {
    disconnect();
}

void Udp::openPort() {
    std::lock_guard<std::mutex> lock(_mtx);
    _timestamp = std::chrono::steady_clock::now();
    
    while (!initializeSocket()) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - _timestamp);

        if (elapsed.count() > _time_out) {
            _is_open_port = false;
            throw std::runtime_error("Failed to open UDP port within timeout");
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    _is_open_port = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

bool Udp::initializeSocket() {
    _sock_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (_sock_fd < 0) {
        std::cerr << "Failed to create socket: " << strerror(errno) << std::endl;
        return false;
    }

    try {
        setupSocketOptions();
        bindSocket();
        setupPoll();
    } catch (const std::exception& e) {
        std::cerr << "Initialization failed: " << e.what() << std::endl;
        close(_sock_fd);
        _sock_fd = -1;
        return false;
    }

    return true;
}

void Udp::setupSocketOptions() {
    if (fcntl(_sock_fd, F_SETFL, O_NONBLOCK) < 0) {
        throw std::system_error(errno, std::system_category(), "fcntl(O_NONBLOCK) failed");
    }

    int enable = 1;
    if (setsockopt(_sock_fd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable)) < 0) {
        throw std::system_error(errno, std::system_category(), "setsockopt(SO_REUSEADDR) failed");
    }
}

void Udp::bindSocket() {
    memset(&_local_addr, 0, sizeof(_local_addr));
    _local_addr.sin_family = AF_INET;
    _local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    _local_addr.sin_port = htons(_local_port);

    if (bind(_sock_fd, reinterpret_cast<struct sockaddr*>(&_local_addr), sizeof(_local_addr)) < 0) {
        throw std::system_error(errno, std::system_category(), "bind failed");
    }

    memset(&_remote_addr, 0, sizeof(_remote_addr));
    _remote_addr.sin_family = AF_INET;
    _remote_addr.sin_addr.s_addr = inet_addr(_remote_ip.c_str());
    _remote_addr.sin_port = htons(_remote_port);
}

void Udp::setupPoll() {
    _pollfds[0].fd = _sock_fd;
    _pollfds[0].events = POLLIN;
}

void Udp::disconnect() {
    std::lock_guard<std::mutex> lock(_mtx);
    if (_sock_fd >= 0) {
        close(_sock_fd);
        _sock_fd = -1;
    }
    _is_open_port = false;
}

int Udp::readData(uint8_t* buffer, size_t size) {
    std::lock_guard<std::mutex> lock(_mtx);
    if (!_is_open_port) return -1;

    int retval = poll(_pollfds, 1, 100);
    if (retval < 0) {
        throw std::system_error(errno, std::system_category(), "poll failed");
    }
    if (retval == 0) return 0; 

    if (_pollfds[0].revents & POLLIN) {
        socklen_t addr_len = sizeof(_remote_addr);
        int read_bytes = recvfrom(_sock_fd, buffer, size, 0,
                                reinterpret_cast<struct sockaddr*>(&_remote_addr), &addr_len);

        if (read_bytes < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;
            throw std::system_error(errno, std::system_category(), "recvfrom failed");
        }

        return read_bytes;
    }

    if (_pollfds[0].revents & (POLLERR | POLLHUP | POLLNVAL)) {
        throw std::runtime_error("Socket error detected");
    }

    return 0;
}

int Udp::readOneByte(uint8_t& byte) {
    return readData(&byte, 1);
}

void Udp::writeData(const uint8_t* buffer, size_t size) {
    std::lock_guard<std::mutex> lock(_mtx);
    if (!_is_open_port) throw std::runtime_error("Socket not open");

    int written = sendto(_sock_fd, buffer, size, 0,
                        reinterpret_cast<struct sockaddr*>(&_remote_addr), sizeof(_remote_addr));

    if (written < 0) {
        throw std::system_error(errno, std::system_category(), "sendto failed");
    }
}

void Udp::writeOneByte(uint8_t byte) {
    writeData(&byte, 1);
}