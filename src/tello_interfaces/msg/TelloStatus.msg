std_msgs/Header header
uint8 DISCONNECTED=0
uint8 LANDED=1
uint8 TAKING_OFF=2
uint8 FLYING=3
uint8 LANDING=4
uint8 EMERGENCY=5
uint8 state
string backend        # sdk | sim | fake
bool link_ok
bool hold             # /tello/hold active
float32 battery       # %, NaN if unknown
string cmd_source     # none | runner | safety
