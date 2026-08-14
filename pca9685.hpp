#ifndef PCA9685_HPP
#define PCA9685_HPP

#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cmath>
#include <cstdint>

#define PCA9685_ADDR 0x40
#define MODE1 0x00
#define PRESCALE 0xFE
#define LED0_ON_L 0x06

class PCA9685 {
    private:
        int i2c_fd;

        void writeRegister(uint8_t reg, uint8_t value) {
            uint8_t buffer[2] = {reg, value};
            write(i2c_fd, buffer, 2);
        }

        uint8_t readRegister(uint8_t reg) {
            write(i2c_fd, &reg, 1);
            uint8_t value;
            read(i2c_fd, &value, 1);
            return value;
        }

    public:
        PCA9685(int bus = 1, int address = PCA9685_ADDR) {
            char filename[20];
            snprintf(filename, sizeof(filename), "/dev/i2c-%d", bus);
            i2c_fd = open(filename, O_RDWR);
            if (i2c_fd < 0) {
                std::cerr << "I2C Failed to Open Bus\n";
                exit(1);
            }
            if (ioctl(i2c_fd, I2C_SLAVE, address) < 0) {
                std::cerr << "I2C Failed to Connect to Device\n";
                exit(1);
            }
            reset();
        }

        ~PCA9685() {
            if (i2c_fd >= 0) {
                close(i2c_fd);
            }
        }

        void reset() {
            writeRegister(MODE1, 0x00);
            usleep(10000);
        }

        void setPWMFreq(float freq) {
            float prescaleval = 25000000.0f / (4096.0f * freq) - 1.0f;
            uint8_t prescale = static_cast<uint8_t>(std::floor(prescaleval + 0.5f));

            uint8_t oldmode = readRegister(MODE1);
            uint8_t newmode = (oldmode & 0x7F) | 0x10;

            writeRegister(MODE1, newmode); 
            writeRegister(PRESCALE, prescale);
            writeRegister(MODE1, oldmode);
            usleep(5000);
            
            writeRegister(MODE1, oldmode | 0xA0); 
        }

        void setPWM(uint8_t channel, uint16_t on, uint16_t off) {
            uint8_t buffer[5];
            buffer[0] = LED0_ON_L + 4 * channel;
            buffer[1] = on & 0xFF;
            buffer[2] = on >> 8;
            buffer[3] = off & 0xFF;
            buffer[4] = off >> 8;
            write(i2c_fd, buffer, 5);
        }

        void setAngle(uint8_t channel, float angle) {
            if (angle < 0.0f) angle = 0.0f;
            if (angle > 180.0f) angle = 180.0f;
            
            uint16_t pulse = 150 + static_cast<uint16_t>((angle / 180.0f) * (570 - 92));
            setPWM(channel, 0, pulse);
        }
};

#endif // PCA9685_HPP