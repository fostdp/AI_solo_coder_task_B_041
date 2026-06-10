#include "udp_server.h"
#include <iostream>
#include <cstring>

namespace turbine_monitor {

UDPServer::UDPServer(const std::string& host, uint16_t port, size_t threadPoolSize)
    : host_(host), port_(port), socket_(INVALID_SOCKET), running_(false),
      receiveBuffer_(UDP_BUFFER_SIZE) {
    processThreads_.reserve(threadPoolSize);
}

UDPServer::~UDPServer() {
    stop();
}

void UDPServer::cleanup() {
#ifdef _WIN32
    if (socket_ != INVALID_SOCKET) {
        closesocket(socket_);
    }
    WSACleanup();
#else
    if (socket_ != INVALID_SOCKET) {
        close(socket_);
    }
#endif
    socket_ = INVALID_SOCKET;
}

bool UDPServer::start() {
#ifdef _WIN32
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        std::cerr << "WSAStartup failed" << std::endl;
        return false;
    }
#endif

    socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (socket_ == INVALID_SOCKET) {
        std::cerr << "Failed to create socket" << std::endl;
        cleanup();
        return false;
    }

    int opt = 1;
#ifdef _WIN32
    if (setsockopt(socket_, SOL_SOCKET, SO_RCVBUF, (const char*)&opt, sizeof(opt)) == SOCKET_ERROR) {
#else
    if (setsockopt(socket_, SOL_SOCKET, SO_RCVBUF, &opt, sizeof(opt)) == SOCKET_ERROR) {
#endif
        std::cerr << "Failed to set socket options" << std::endl;
    }

    sockaddr_in serverAddr{};
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port_);
    if (host_ == "0.0.0.0" || host_.empty()) {
        serverAddr.sin_addr.s_addr = INADDR_ANY;
    } else {
        inet_pton(AF_INET, host_.c_str(), &serverAddr.sin_addr);
    }

    if (bind(socket_, (sockaddr*)&serverAddr, sizeof(serverAddr)) == SOCKET_ERROR) {
        std::cerr << "Failed to bind socket to " << host_ << ":" << port_ << std::endl;
        cleanup();
        return false;
    }

    running_ = true;
    receiveThread_ = std::thread(&UDPServer::receiveLoop, this);

    for (size_t i = 0; i < processThreads_.capacity(); ++i) {
        processThreads_.emplace_back(&UDPServer::processLoop, this);
    }

    std::cout << "UDP Server started on " << host_ << ":" << port_ << std::endl;
    return true;
}

void UDPServer::stop() {
    running_ = false;

    queueCond_.notify_all();

    if (receiveThread_.joinable()) {
        receiveThread_.join();
    }

    for (auto& t : processThreads_) {
        if (t.joinable()) {
            t.join();
        }
    }

    cleanup();
}

void UDPServer::setDataCallback(DataCallback callback) {
    std::lock_guard<std::mutex> lock(callbackMutex_);
    dataCallback_ = std::move(callback);
}

void UDPServer::receiveLoop() {
    sockaddr_in clientAddr{};
#ifdef _WIN32
    int addrLen = sizeof(clientAddr);
#else
    socklen_t addrLen = sizeof(clientAddr);
#endif

    while (running_) {
        int bytesReceived = recvfrom(
            socket_,
            receiveBuffer_.data(),
            receiveBuffer_.size(),
            0,
            (sockaddr*)&clientAddr,
            &addrLen
        );

        if (bytesReceived == SOCKET_ERROR || bytesReceived < static_cast<int>(sizeof(UDPDataPacket))) {
            if (running_) {
                std::cerr << "Receive error or invalid packet size" << std::endl;
            }
            continue;
        }

        const UDPDataPacket* packet = reinterpret_cast<const UDPDataPacket*>(receiveBuffer_.data());
        RawSensorData data;
        if (parsePacket(packet, data)) {
            std::lock_guard<std::mutex> lock(queueMutex_);
            dataQueue_.push(std::move(data));
            queueCond_.notify_one();
        }
    }
}

void UDPServer::processLoop() {
    while (running_) {
        std::unique_lock<std::mutex> lock(queueMutex_);
        queueCond_.wait(lock, [this]() { return !dataQueue_.empty() || !running_; });

        if (!running_ && dataQueue_.empty()) {
            break;
        }

        RawSensorData data = std::move(dataQueue_.front());
        dataQueue_.pop();
        lock.unlock();

        std::lock_guard<std::mutex> cbLock(callbackMutex_);
        if (dataCallback_) {
            dataCallback_(data);
        }
    }
}

bool UDPServer::parsePacket(const UDPDataPacket* packet, RawSensorData& data) {
    if (!packet || packet->sample_count > 128) {
        return false;
    }

    data.timestamp       = packet->timestamp;
    data.turbine_id      = packet->turbine_id;
    data.sensor_type     = static_cast<SensorType>(packet->sensor_type);
    data.sensor_id       = packet->sensor_id;
    data.sensor_position = static_cast<SensorPosition>(packet->sensor_position);
    data.blade_id        = packet->blade_id;
    data.amplitude       = packet->amplitude;
    data.sample_rate     = packet->sample_rate;
    data.batch_id        = packet->batch_id;

    data.raw_data.assign(packet->data, packet->data + packet->sample_count);
    return true;
}

}
