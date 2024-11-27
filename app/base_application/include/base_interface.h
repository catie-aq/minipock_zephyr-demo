#ifndef MOTOR_INTERFACE_H
#define MOTOR_INTERFACE_H

struct base_interface_callbacks {
    void (*send_odometry_callback)(float x, float y, float theta);
};

void send_command(float linear_x,
        float linear_y,
        float linear_z,
        float angular_x,
        float angular_y,
        float angular_z);
int init_base_interface(struct base_interface_callbacks *callbacks);

#endif // MOTOR_INTERFACE_H
