#include "serialPort/SerialPort.h"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <unistd.h>

#if defined(__linux__)
#define termios termios_redefined
#include <asm/termbits.h> /* struct termios2 */
#include <asm/ioctls.h>
#endif

namespace {
inline void log_error(const std::string& msg) {
    std::cerr << "[ERROR] " << msg << std::endl;
}

inline void log_warning(const std::string& msg) {
    std::cerr << "[WARNING] " << msg << std::endl;
}
}  // namespace

SerialPort::SerialPort(const std::string& portName,
                       size_t recvLength,
                       uint32_t baudrate,
                       size_t timeOutUs,
                       BlockYN blockYN,
                       bytesize_t bytesize,
                       parity_t parity,
                       stopbits_t stopbits,
                       flowcontrol_t flowcontrol)
    : IOPort(blockYN, recvLength, timeOutUs),
      _portName(portName),
      _baudrate(baudrate),
      _bytesize(bytesize),
      _parity(parity),
      _stopbits(stopbits),
      _flowcontrol(flowcontrol),
      _xonxoff(false),
      _rtscts(false),
      _fd(-1) {
    _open();
    _set();
}

SerialPort::~SerialPort() {
    _close();
}

void SerialPort::resetSerial(size_t recvLength,
                             uint32_t baudrate,
                             size_t timeOutUs,
                             BlockYN blockYN,
                             bytesize_t bytesize,
                             parity_t parity,
                             stopbits_t stopbits,
                             flowcontrol_t flowcontrol) {
    _recvLength = recvLength;
    _timeoutSaved.tv_sec = 0;
    _timeoutSaved.tv_usec = static_cast<decltype(_timeoutSaved.tv_usec)>(timeOutUs);
    _timeout = _timeoutSaved;
    _blockYN = blockYN;
    _baudrate = baudrate;
    _bytesize = bytesize;
    _parity = parity;
    _stopbits = stopbits;
    _flowcontrol = flowcontrol;

    _close();
    _open();
    _set();
}

void SerialPort::_open() {
    if (_fd >= 0) {
        return;
    }

    // Open as read/write without controlling TTY.
    _fd = ::open(_portName.c_str(), O_RDWR | O_NOCTTY);
    if (_fd < 0) {
        log_error(std::string("SerialPort::_open failed for ") + _portName + ": " + strerror(errno));
        return;
    }

    // Apply blocking/non-blocking mode.
    int flags = fcntl(_fd, F_GETFL, 0);
    if (flags < 0) {
        flags = 0;
    }
    if (_blockYN == BlockYN::NO) {
        (void)fcntl(_fd, F_SETFL, flags | O_NONBLOCK);
    } else {
        (void)fcntl(_fd, F_SETFL, flags & ~O_NONBLOCK);
    }
}

void SerialPort::_close() {
    if (_fd >= 0) {
        ::close(_fd);
        _fd = -1;
    }
}

void SerialPort::_set() {
    if (_fd < 0) {
        return;
    }

#if defined(__linux__)
    // Configure raw mode and high baud rate using termios2.
    struct termios2 tio;
    memset(&tio, 0, sizeof(tio));
    if (ioctl(_fd, TCGETS2, &tio) != 0) {
        log_error(std::string("SerialPort::_set TCGETS2 failed: ") + strerror(errno));
        return;
    }

    // Input flags - turn off input processing
    tio.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    // Output flags - turn off output processing
    tio.c_oflag &= ~OPOST;
    // Local flags - turn off canonical input, echo, signals
    tio.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    // Control flags - set 8n1
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CSIZE;
    switch (_bytesize) {
        case bytesize_t::fivebits:
            tio.c_cflag |= CS5;
            break;
        case bytesize_t::sixbits:
            tio.c_cflag |= CS6;
            break;
        case bytesize_t::sevenbits:
            tio.c_cflag |= CS7;
            break;
        case bytesize_t::eightbits:
        default:
            tio.c_cflag |= CS8;
            break;
    }

    // Parity
    tio.c_cflag &= ~(PARENB | PARODD);
    if (_parity == parity_t::parity_odd) {
        tio.c_cflag |= (PARENB | PARODD);
    } else if (_parity == parity_t::parity_even) {
        tio.c_cflag |= PARENB;
    }

    // Stop bits
    if (_stopbits == stopbits_t::stopbits_two) {
        tio.c_cflag |= CSTOPB;
    } else {
        tio.c_cflag &= ~CSTOPB;
    }

    // Flow control
    _xonxoff = (_flowcontrol == flowcontrol_t::flowcontrol_software);
    _rtscts = (_flowcontrol == flowcontrol_t::flowcontrol_hardware);
    if (_xonxoff) {
        tio.c_iflag |= (IXON | IXOFF);
    } else {
        tio.c_iflag &= ~(IXON | IXOFF);
    }

    if (_rtscts) {
        tio.c_cflag |= CRTSCTS;
    } else {
        tio.c_cflag &= ~CRTSCTS;
    }

    // Read behavior: VMIN/VTIME don't matter much since we use select() for non-blocking.
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;

    // Custom baud rate
    tio.c_cflag &= ~CBAUD;
    tio.c_cflag |= BOTHER;
    tio.c_ispeed = _baudrate;
    tio.c_ospeed = _baudrate;

    if (ioctl(_fd, TCSETS2, &tio) != 0) {
        log_error(std::string("SerialPort::_set TCSETS2 failed: ") + strerror(errno));
        return;
    }

    // Best-effort: request low latency if available.
    struct serial_struct ser;
    if (ioctl(_fd, TIOCGSERIAL, &ser) == 0) {
        ser.flags |= ASYNC_LOW_LATENCY;
        (void)ioctl(_fd, TIOCSSERIAL, &ser);
    }

    // Flush any stale data.
    (void)ioctl(_fd, TCFLSH, TCIOFLUSH);
#else
    log_error("SerialPort::_set: non-Linux platform not supported in this build");
#endif
}

size_t SerialPort::send(uint8_t* sendMsg, size_t sendLength) {
    if (_fd < 0 || sendMsg == nullptr || sendLength == 0) {
        return 0;
    }

    size_t total = 0;
    while (total < sendLength) {
        const ssize_t n = ::write(_fd, sendMsg + total, sendLength - total);
        if (n > 0) {
            total += static_cast<size_t>(n);
            continue;
        }
        if (n < 0 && (errno == EINTR)) {
            continue;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            // Give the kernel a chance to flush.
            usleep(50);
            continue;
        }
        log_error(std::string("SerialPort::send failed: ") + strerror(errno));
        break;
    }

    return total;
}

size_t SerialPort::_nonBlockRecv(uint8_t* recvMsg, size_t readLen) {
    if (_fd < 0 || recvMsg == nullptr || readLen == 0) {
        return 0;
    }

    size_t total = 0;
    while (total < readLen) {
        FD_ZERO(&_rSet);
        FD_SET(_fd, &_rSet);
        _timeout = _timeoutSaved;  // select() mutates the struct

        const int ret = ::select(_fd + 1, &_rSet, nullptr, nullptr, &_timeout);
        if (ret > 0 && FD_ISSET(_fd, &_rSet)) {
            const ssize_t n = ::read(_fd, recvMsg + total, readLen - total);
            if (n > 0) {
                total += static_cast<size_t>(n);
                continue;
            }
            if (n < 0 && (errno == EINTR)) {
                continue;
            }
            if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                continue;
            }
            break;
        }

        if (ret == 0) {
            log_warning("SerialPort::recv, unblock version, wait time out");
            break;
        }

        if (ret < 0 && errno == EINTR) {
            continue;
        }

        if (ret < 0) {
            log_error(std::string("SerialPort::recv select failed: ") + strerror(errno));
            break;
        }
    }

    return total;
}

size_t SerialPort::recv(uint8_t* recvMsg, size_t recvLength) {
    if (_fd < 0 || recvMsg == nullptr || recvLength == 0) {
        return 0;
    }

    if (_blockYN == BlockYN::YES) {
        // Block until we have recvLength bytes or an error.
        size_t total = 0;
        while (total < recvLength) {
            const ssize_t n = ::read(_fd, recvMsg + total, recvLength - total);
            if (n > 0) {
                total += static_cast<size_t>(n);
                continue;
            }
            if (n < 0 && errno == EINTR) {
                continue;
            }
            if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                // In "blocking" mode, still respect timeout.
                usleep(50);
                continue;
            }
            break;
        }
        return total;
    }

    return _nonBlockRecv(recvMsg, recvLength);
}

size_t SerialPort::recv(uint8_t* recvMsg) {
    return recv(recvMsg, _recvLength);
}

bool SerialPort::sendRecv(uint8_t* sendMsg, uint8_t* recvMsg, size_t sendLength) {
    if (send(sendMsg, sendLength) != sendLength) {
        return false;
    }
    return recv(recvMsg) == _recvLength;
}

bool SerialPort::sendRecv(MotorCmd* sendMsg, MotorData* recvMsg) {
    if (sendMsg == nullptr || recvMsg == nullptr) {
        return false;
    }

    // Encode command
    sendMsg->modify_data(sendMsg);
    const size_t tx_len = static_cast<size_t>(sendMsg->hex_len);
    uint8_t* tx = sendMsg->get_motor_send_data();

    // Expected reply size
    size_t rx_len = 0;
    switch (sendMsg->motorType) {
        case MotorType::GO_M8010_6:
        case MotorType::GO_2:
            rx_len = 16;
            break;
        case MotorType::A1:
        case MotorType::B1:
        default:
            rx_len = 78;
            break;
    }

    recvMsg->motorType = sendMsg->motorType;

    // Transmit then receive
    if (send(tx, tx_len) != tx_len) {
        return false;
    }

    uint8_t* rx = recvMsg->get_motor_recv_data();
    const size_t nread = recv(rx, rx_len);
    if (nread != rx_len) {
        // Still attempt to decode what we got (but mark invalid).
        recvMsg->correct = 0;
        return false;
    }

    recvMsg->extract_data(recvMsg);
    return recvMsg->correct != 0;
}

bool SerialPort::sendRecv(std::vector<MotorCmd>& sendVec, std::vector<MotorData>& recvVec) {
    if (sendVec.empty()) {
        return false;
    }
    recvVec.resize(sendVec.size());
    bool ok = true;
    for (size_t i = 0; i < sendVec.size(); ++i) {
        ok = sendRecv(&sendVec[i], &recvVec[i]) && ok;
    }
    return ok;
}

void SerialPort::test() {
    std::cout << "SerialPort(" << _portName << ") baud=" << _baudrate
              << " recvLen=" << _recvLength << " timeoutUs=" << _timeoutSaved.tv_usec
              << std::endl;
}
