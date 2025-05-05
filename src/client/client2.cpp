#include "Protocol.h"
#include <opencv2/opencv.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <csignal>
#include <atomic>

class VideoClient {
public:
    VideoClient(const std::string& server_ip, int server_port) 
        : server_ip_(server_ip), server_port_(server_port) {}
    
    void sendVideo(const std::string& video_path, float target_fps = 30.0f) {
        std::cout << "Preparing to send video: " << video_path << std::endl;
        
        // 创建socket并连接
        int sock = createConnection();
        
        // 打开视频文件
        cv::VideoCapture cap(video_path);
        if (!cap.isOpened()) {
            close(sock);
            throw std::runtime_error("无法打开视频文件: " + video_path);
        }
        
        // 视频信息
        double video_fps = cap.get(cv::CAP_PROP_FPS);
        int total_frames = cap.get(cv::CAP_PROP_FRAME_COUNT);
        std::cout << "视频信息 - 分辨率: " << cap.get(cv::CAP_PROP_FRAME_WIDTH) << "x" 
                  << cap.get(cv::CAP_PROP_FRAME_HEIGHT) 
                  << " | 原始FPS: " << video_fps 
                  << " | 总帧数: " << total_frames << std::endl;
        
        // 计算帧间隔(ms)
        const int frame_delay = 1000 / target_fps;
        auto last_send_time = std::chrono::steady_clock::now();
        
        cv::Mat frame;
        uint32_t frame_id = 0;
        
        while (running_ && cap.read(frame)) {
            // 调整帧大小
            cv::resize(frame, frame, cv::Size(640, 480));
            
            std::cout << "正在发送帧 " << frame_id + 1 << "/" << total_frames << std::endl;
            
            // 发送帧数据
            sendFrame(sock, frame, frame_id++);
            
            // 接收处理结果
            receiveResult(sock);
            
            // 控制帧率
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_send_time).count();
            if (elapsed < frame_delay) {
                std::this_thread::sleep_for(std::chrono::milliseconds(frame_delay - elapsed));
            }
            last_send_time = now;
        }
        
        // 清理资源
        sendEndSignal(sock);
        cap.release();
        close(sock);
        
        std::cout << "视频发送完成" << std::endl;
    }

    void stop() { running_ = false; }

private:
    std::string server_ip_;
    int server_port_;
    std::atomic<bool> running_{true};
    
    int createConnection() {
        std::cout << "正在连接服务器 " << server_ip_ << ":" << server_port_ << std::endl;
        
        int sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0) throw std::runtime_error("创建socket失败");
        
        sockaddr_in serv_addr;
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(server_port_);
        
        if (inet_pton(AF_INET, server_ip_.c_str(), &serv_addr.sin_addr) <= 0) {
            close(sock);
            throw std::runtime_error("无效的IP地址");
        }
        
        if (connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
            close(sock);
            throw std::runtime_error("连接服务器失败");
        }
        
        std::cout << "服务器连接成功" << std::endl;
        return sock;
    }
    
    void sendFrame(int sock, const cv::Mat& frame, uint32_t frame_id) {
        auto messages = Protocol::createImageMessage(frame, frame_id);
        for (const auto& msg : messages) {
            if (send(sock, &msg, sizeof(MessageBuffer), 0) < 0) {
                throw std::runtime_error("发送数据失败");
            }
        }
    }
    
    void receiveResult(int sock) {
        MessageBuffer response;
        if (recv(sock, &response, sizeof(MessageBuffer), MSG_WAITALL) > 0) {
            if (response.MessageType == TRANSFORM) {
                double x, y, z, roll, pitch, yaw;
                Protocol::extractTransformData(response, x, y, z, roll, pitch, yaw);
                
                std::cout << "检测结果 - "
                          << "位置: (" << x << ", " << y << ", " << z << ") | "
                          << "旋转: (" << roll << ", " << pitch << ", " << yaw << ")\n";
            }
            else if (response.MessageType == STRING_MSG) {
                std::string msgStr(response.Data, response.Data + response.DataLength);
                std::cout << "服务器返回: " << msgStr << std::endl;
            }
        }
    }
    
    void sendEndSignal(int sock) {
        MessageBuffer end_msg = Protocol::createStringMessage("END", 0);
        send(sock, &end_msg, sizeof(MessageBuffer), 0);
    }
};

void signalHandler(int signum) {
    std::cout << "\n接收到中断信号 (" << signum << "), 正在停止..." << std::endl;
    VideoClient::stop();
}
//  使用示例:请加入到main函数中
int main(int argc, char** argv) {
    if (argc < 4) {
        std::cerr << "用法: " << argv[0] << " <服务器IP> <端口> <视频路径> [FPS]" << std::endl;
        std::cerr << "示例: " << argv[0] << " 127.0.0.1 8080 /home/chen/视频/armor_test.mp4 30" << std::endl;
        return 1;
    }
    
    try {
        // 注册信号处理
        signal(SIGINT, signalHandler);
        signal(SIGTERM, signalHandler);
        
        float fps = (argc > 4) ? std::stof(argv[4]) : 30.0f;
        VideoClient client(argv[1], std::stoi(argv[2]));
        
        // 启动视频传输
        client.sendVideo(argv[3], fps);
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}