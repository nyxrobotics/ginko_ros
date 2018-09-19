#include "serial.hpp"

void serial_port::check_error(const char* action, int status) {
    if(status < 0) {
        ROS_ERROR("%s failed: %s(%s)", action, port_name.data(), std::strerror(errno));
        exit(-1);
    }
    return;
}

serial_port::serial_port(std::string pn) {
    port_name += pn;

    fd = open(port_name.data(), O_RDWR);
    check_error("Open", fd);

    check_error("Save", tcgetattr(fd, &tio_backup));
    // save the current port status
    
    std::memset(&tio, 0, sizeof(tio));
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 1;
    tio.c_cflag = B115200 | CS8 | CREAD | CLOCAL;
//    tio.c_cflag = B230400 | CS8 | CREAD | CLOCAL;
//    tio.c_cflag = B460800 | CS8 | CREAD | CLOCAL;
    tio.c_iflag = IGNBRK | IGNPAR;
    // init term-io

    check_error("Flash", tcflush(fd, TCIOFLUSH));
    // flush setting

    check_error("Set", tcsetattr(fd, TCSANOW, &tio));
    // change the port setting

    ROS_INFO("USB connected!");
//	struct serial_struct serial_settings;
//	ioctl(fd, TIOCGSERIAL, &serial_settings);
//	serial_settings.flags |= ASYNC_LOW_LATENCY;
//	ioctl(fd, TIOCSSERIAL, &serial_settings);
}

serial_port::~serial_port() {
    check_error("Reset", tcsetattr(fd, TCSANOW, &tio_backup));
    // restore the port setting

    close(fd);

    ROS_INFO("USB disconnected!");
}


void serial_port::send_packet(void* ptr, int size) {
    if(write(fd, ptr, size) < 0) {
        ROS_ERROR("Writing failed: %s(%s)", port_name.data(), std::strerror(errno));
    }
//    usleep(10);

    return;
}

void serial_port::receive_packet(void *buf_ptr, int size) {
    if(read(fd, buf_ptr, size) < 0) {
        ROS_ERROR("Reading failed: %s(%s)", port_name.data(), std::strerror(errno));
    }
}

int serial_port::get_fd() {
    return fd;
}
