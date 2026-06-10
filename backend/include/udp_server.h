#pragma once

#include <string>
#include <thread>
#include <atomic>
#include <vector>
#include <functional>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <queue>
#include "data_structures.h"

#ifdef _WIN32
#include <winsock2.h>
#pragma comment(lib, "ws2_32.lib")
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
typedef int SOCKET;
#define INVALID_SOCKET -1
#define SOCKET_ERROR -1
#endif

namespace turbine_monitor {

using DataCallback = std::function<void(const RawSensorData&)>;

class UDPServer {
public:
    UDPServer(const std::string& host, uint16_t port, size_t threadPoolSize = 8);
    ~UDPServer();

    bool start();
    void stop();
    void setDataCallback(DataCallback callback);

private:
    void receiveLoop();
    void processLoop();
    bool parsePacket(const UDPDataPacket* packet, RawSensorData& data);
    void cleanup();

    std::string     host_;
    uint16_t        port_;
    SOCKET          socket_;
    std::atomic<bool> running_;

    std::thread     receiveThread_;
    std::vector<std::thread> processThreads_;

    std::queue<RawSensorData> dataQueue_;
    std::mutex                queueMutex_;
    std::condition_variable   queueCond_;

    DataCallback  dataCallback_;
    std::mutex    callbackMutex_;

    std::vector<char> receiveBuffer_;
};

}
