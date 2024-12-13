#ifndef UPDATE_H
#define UPDATE_H

struct update_interface {
    void (*request_update_params)(void);
    void (*request_update)(void);
};

struct update_params {
    const char *version;
    size_t size;
};

#endif // UPDATE_H
