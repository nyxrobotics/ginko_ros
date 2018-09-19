#ifndef SERIAL
#define SERIAL

#include <ros/ros.h>

#include <string>
#include <system_error>
#include <cstdlib>
#include <cerrno>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h> // 受信バッファに届いているデータの数を所得するために使用
#include <linux/serial.h>

class serial_port {
    private:
    std::string port_name;
    int fd;     // file descriptor
    struct termios tio, tio_backup;
    void check_error(const char*, int);

    public:
    serial_port(std::string);
    ~serial_port();
    void send_packet(void*, int);
    void receive_packet(void*, int);
    int get_fd();
};

#endif
