#ifndef BUFFERED_DATA_STRUCT_H
#define BUFFERED_DATA_STRUCT_H
struct buffered_data
{
    float timestamp, duty_cycle,
        reference_illuminance, measured_illuminance;
    buffered_data(float t = 0.0f, float dc = 0.0f, float r = 0.0f, float y = 0.0f)
        : timestamp{t},
          duty_cycle{dc},
          reference_illuminance{r},
          measured_illuminance{y} {};
};
#endif // CIRCULAR_BUFFER_H