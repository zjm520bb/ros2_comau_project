uint8 JOINT=0
uint8 LINEAR=1
uint8 CIRCULAR=2
uint8 SEG_VIA=3

uint8 motion_type
float64[6] target

float64 linear_speed
float64 rotational_speed
float64 segment_override
uint8 termination_type
float64 tolerance
bool segment_data

bool fly
uint8 fly_type
float64 fly_percent
float64 fly_distance_mm
uint8 fly_trajectory
float64 stress_percent

uint8 reference_index
uint8 tool_index
int32 condition_mask
int32 condition_mask_back
bool wait
